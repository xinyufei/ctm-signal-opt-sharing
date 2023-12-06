import gurobipy as gb


def load_link_flow(network, link_flow_file):
    with open(link_flow_file, "r") as temp_file:
        all_lines = temp_file.readlines()

    for line in all_lines:
        link_id, link_flow = line[:-1].split(",")[:2]
        try:
            network.links[link_id].link_flow = float(link_flow)
        except KeyError:
            print("Link", link_id, "not included in the network.")


def check_flow_conservation(network):
    for node_id, node in network.nodes.items():
        if not node.is_intersection():
            continue
        # print(node.upstream_links, node.downstream_links)
        inflow = 0
        for upstream_link_id in node.upstream_links:
            link = network.links[upstream_link_id]
            inflow += link.link_flow
        outflow = 0
        for downstream_link_id in node.downstream_links:
            link = network.links[downstream_link_id]
            outflow += link.link_flow

        # print()
        # print(node_id)
        # print(inflow, outflow)

        if abs(outflow - inflow) > 50:
            if abs(outflow - inflow) / max([inflow, outflow, 1]) > 0.1:
                print(node_id, inflow, outflow)
                print(node.upstream_links, node.downstream_links)
                print(" ")


def compute_laneset_flow(network):
    alpha = 10
    obj_1 = 0
    obj_2 = 0
    m = gb.Model()
    link_vars = {}
    laneset_vars = {}
    connector_vars = {}
    for link_id, link in network.links.items():
        connector_vars[link_id] = {}
        laneset_vars[link_id] = {}
        link_vars[link_id] = m.addVar()

        for segment_id in link.segment_list:
            segment = network.segments[segment_id]
            for laneset_id in segment.laneset_list:
                laneset_vars[link_id][laneset_id] = m.addVar()
                laneset = network.lanesets[laneset_id]
            m.addConstr(gb.quicksum(laneset_vars[link_id][laneset_id] for laneset_id in segment.laneset_list)
                        - link_vars[link_id] == 0)

        downstream_node = network.nodes[link.downstream_node]
        if downstream_node.is_intersection():
            connector_vars[link_id] = {}
            segment_id = link.segment_list[-1]
            segment = network.segments[segment_id]
            for laneset_id in segment.laneset_list:
                # print(link_id, segment_id, laneset_id)
                connector_vars[link_id][laneset_id] = {}
                laneset = network.lanesets[laneset_id]
                for downstream_laneset_id in network.connectors[laneset.downstream_connector].downstream_lanesets:
                    connector_vars[link_id][laneset_id][downstream_laneset_id] = m.addVar()
                m.addConstr(gb.quicksum(connector_vars[link_id][laneset_id][downstream_laneset_id]
                                        for downstream_laneset_id in
                                        network.connectors[laneset.downstream_connector].downstream_lanesets) -
                            laneset_vars[link_id][laneset_id] == 0)

    for link_id, link in network.links.items():
        upstream_node = network.nodes[link.upstream_node]
        if upstream_node.is_intersection():
            segment_id = link.segment_list[0]
            segment = network.segments[segment_id]
            for laneset_id in segment.laneset_list:
                laneset = network.lanesets[laneset_id]
                upstream_lanesets = []
                for connector_id in laneset.upstream_connectors:
                    connector = network.connectors[connector_id]
                    upstream_lanesets.append(connector.upstream_laneset)
                m.addConstr(laneset_vars[link_id][laneset_id] == gb.quicksum(
                    connector_vars[network.lanesets[upstream_laneset_id].belonged_link][upstream_laneset_id][laneset_id]
                    for upstream_laneset_id in upstream_lanesets))
                obj_1 += (laneset_vars[link_id][laneset_id] / laneset.capacity) * (
                        laneset_vars[link_id][laneset_id] / laneset.capacity)

        downstream_node = network.nodes[link.downstream_node]
        if downstream_node.is_intersection():
            segment_id = link.segment_list[-1]
            segment = network.segments[segment_id]
            for laneset_id in segment.laneset_list:
                laneset = network.lanesets[laneset_id]
                obj_1 += (laneset_vars[link_id][laneset_id] / laneset.capacity) * (
                        laneset_vars[link_id][laneset_id] / laneset.capacity)

    obj_2 = gb.quicksum((link_vars[link_id] - network.links[link_id].link_flow) *
                        (link_vars[link_id] - network.links[link_id].link_flow)
                        for link_id in network.links.keys())
    m.setObjective(alpha * obj_2)
    m.Params.LogToConsole = 0
    m.optimize()

    print(obj_1.getValue(), obj_2.getValue())

    for link_id, link in network.links.items():
        # link.link_flow = link_vars[link_id].x
        print(link_id, link_vars[link_id].x, link.link_flow)
        link.link_flow = link_vars[link_id].x
        for segment_id in link.segment_list:
            segment = network.segments[segment_id]
            for laneset_id in segment.laneset_list:
                network.lanesets[laneset_id].laneset_flow = laneset_vars[link_id][laneset_id].x

    for node_id, node in network.nodes.items():
        if not node.is_intersection():
            continue
        print()
        print(node_id)
        for link_id in node.downstream_links:
            link = network.links[link_id]
            segment_id = link.segment_list[0]
            segment = network.segments[segment_id]
            for laneset_id in segment.laneset_list:
                laneset = network.lanesets[laneset_id]
                upstream_lanesets = []
                for connector_id in laneset.upstream_connectors:
                    connector = network.connectors[connector_id]
                    upstream_lanesets.append(connector.upstream_laneset)
                print(laneset_id, network.lanesets[laneset_id].laneset_flow, sum(
                    connector_vars[network.lanesets[upstream_laneset_id].belonged_link][
                        upstream_laneset_id][laneset_id].x
                    for upstream_laneset_id in upstream_lanesets))

    for connector_id, connector in network.connectors.items():
        laneset_id = connector.upstream_laneset
        total_flow = network.lanesets[laneset_id].laneset_flow
        link_id = network.lanesets[laneset_id].belonged_link
        connector.diverge_proportion = []
        if total_flow == 0 and len(connector.downstream_lanesets) > 0:
            connector.diverge_proportion = [1/len(connector.downstream_lanesets)] * len(connector.downstream_lanesets)
            continue
        if network.nodes[connector.belonged_node].type == "connector":
            for downstream_laneset_id in connector.downstream_lanesets:
                connector.diverge_proportion.append(network.lanesets[downstream_laneset_id].laneset_flow / total_flow)
        if network.nodes[connector.belonged_node].is_intersection():
            for downstream_laneset_id in connector.downstream_lanesets:
                print(connector_vars[link_id][laneset_id][downstream_laneset_id].x)
                connector.diverge_proportion.append(
                    connector_vars[link_id][laneset_id][downstream_laneset_id].x / total_flow)
        print(connector.diverge_proportion, sum(connector.diverge_proportion))

