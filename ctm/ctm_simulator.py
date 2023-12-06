import numpy as np
from scipy import stats
from tqdm import tqdm
import matplotlib.pyplot as plt
import gurobipy as gb
import ctm.ctm_classes as ctm_cls
from mtldp.tools.gps_utils import segment_gps_trace
from mtldp.apps.user_equibrium import read_od_pair, solve_user_equilibrium
from mtldp.mtlmap.flow_io import load_link_flow, check_flow_conservation, compute_laneset_flow


class CTMSimulator:
    """
    """

    def __init__(self, time_steps=300, time_interval=3, num_scenario=10):
        """
        """
        # parameter setting for the CTM simulation
        self.total_steps = time_steps
        self.time_interval = time_interval
        self.num_scenario = num_scenario

        # members
        self.cells = {}
        self.connectors = {}
        self.links = {}
        self.nodes = {}
        self.lanesets = {}
        self.conflict_points = {}
        self.signalized_nodes_id = []
        self.unsignalized_nodes_id = []
        self.intersections_id = []
        self.connectors_by_priority = []
        self.num_cells = 0
        self.num_intersections = 0
        self.num_signalized_intersections = 0
        self.num_flow = 0
        self.bounds = None

    def add_link(self, link):
        """
        add link to CTM simulator
        """
        self.links[link.link_id] = link

    def add_node(self, node):
        """
        add node to CTM simulator
        """
        if node.type in ["signalized", "unsignalized"]:
            node.node_global_id = self.num_intersections
            self.intersections_id.append(node.node_id)
            self.num_intersections += 1
            if node.type == "signalized":
                node.node_signalized_id = self.num_signalized_intersections
                node.generate_fake_signal(self.total_steps)
                self.num_signalized_intersections += 1
                self.signalized_nodes_id.append(node.node_id)
            if node.type == "unsignalized":
                self.unsignalized_nodes_id.append(node.node_id)
        self.nodes[node.node_id] = node
        # return node

    def add_conflict_point(self, conflict_point):
        """
        add conflict point to CTM simulator
        """
        self.conflict_points[conflict_point.conflict_id] = conflict_point
        node = self.nodes[conflict_point.belong_node]
        node.conflict_points_list.append(conflict_point.conflict_id)

    def add_laneset(self, laneset):
        """
        add laneset to CTM simulator
        """
        self.lanesets[laneset.laneset_id] = laneset

    def add_cell_to_network(self, cell):
        """
        add cell to CTM simulator
        """
        cell.cell_global_id = self.num_cells
        self.cells[cell.cell_id] = cell
        self.num_cells += 1
        # return cell

    def add_connector_to_network(self, connector):
        """
        add connector to CTM simulator
        """
        if connector.type == "diverge":
            connector.global_optimize_id = [self.num_flow + c for c in range(len(connector.downstream_cell))]
            self.num_flow += len(connector.downstream_cell)
        if connector.type == "straight":
            connector.global_optimize_id = [self.num_flow]
            self.num_flow += 1
        self.connectors[connector.connector_id] = connector

    def add_connector_to_priority_list(self, connector_id):
        """
        add connector to index list sorted by priority
        """
        connector = self.connectors[connector_id]
        len_connector_priority = len(self.connectors_by_priority)
        if len_connector_priority <= connector.priority_class + 1:
            for idx in range(connector.priority_class + 2 - len_connector_priority):
                self.connectors_by_priority.append([])
        self.connectors_by_priority[connector.priority_class + 1].append(connector_id)

    def add_cell_to_node(self, node_id, cell_id):
        node = self.nodes[node_id]
        cell = self.cells[cell_id]
        # update the information about node in cell class
        cell.belonged_node = node_id
        # laneset = self.lanesets[cell.belonged_laneset]
        # link = self.links[laneset.belonged_link]
        # type1 = self.nodes[link.upstream_node].type
        # type2 = self.nodes[link.downstream_node].type
        cell.cell_local_id = len(node.cells_list)
        # add cell into the cell list of node
        node.cells_list.append(cell_id)
        # if cell is destination cell, add it to destination cell list
        if cell.type == "destination":
            node.destination_cell_list.append(cell_id)

    def add_connector_to_node(self, node_id, connector_id):
        connector = self.connectors[connector_id]
        node = self.nodes[node_id]
        connector.belonged_node = node_id
        node.connectors_list.append(connector_id)
        if connector.type == "merge":
            connector.optimize_id = [node.num_flow + c for c in range(len(connector.upstream_cell))]
            node.num_flow += len(connector.upstream_cell)
        if connector.type == "diverge":
            connector.optimize_id = [node.num_flow + c for c in range(len(connector.downstream_cell))]
            node.num_flow += len(connector.downstream_cell)
        if connector.type == "straight":
            connector.optimize_id = [node.num_flow]
            node.num_flow += 1
        if connector.signalized:
            node.signalized_conector_list.append(connector_id)

    def add_outflow_boundary_connector_to_node(self, node_id, connector_id):
        node = self.nodes[node_id]
        node.outflow_connector_list.append(connector_id)
        connector = self.connectors[connector_id]
        if connector.type == "merge":
            connector.outflow_optimize_id = [node.num_outflow + c for c in range(len(connector.upstream_cell))]
            node.num_outflow += len(connector.upstream_cell)
        if connector.type == "diverge":
            connector.outflow_optimize_id = [node.num_outflow + c for c in range(len(connector.downstream_cell))]
            node.num_outflow += len(connector.downstream_cell)
        if connector.type == "straight":
            connector.outflow_optimize_id = [node.num_outflow]
            node.num_outflow += 1

    def add_inflow_boundary_connector_to_node(self, node_id, connector_id, cidx):
        node = self.nodes[node_id]
        if connector_id not in node.inflow_connector_list:
            node.inflow_connector_list.append(connector_id)
        connector = self.connectors[connector_id]
        if connector.type == "merge":
            connector.inflow_optimize_id = [node.num_inflow + c for c in range(len(connector.upstream_cell))]
            node.num_inflow += len(connector.upstream_cell)
        if connector.type == "diverge":
            # connector.inflow_optimize_id = [node.num_inflow + c for c in range(len(connector.downstream_cell))]
            connector.inflow_optimize_id.append(node.num_inflow)
            # node.num_inflow += len(connector.downstream_cell)
            node.num_inflow += 1
        if connector.type == "straight":
            connector.inflow_optimize_id = [node.num_inflow]
            node.num_inflow += 1
        if connector.downstream_cell[cidx] not in node.inflow_cell_list:
            node.inflow_cell_list.append(connector.downstream_cell[0])
            downstream_cell = self.cells[connector.downstream_cell[0]]
            downstream_cell.cell_inflow_id = len(node.inflow_cell_list) - 1

    def update_cell_params(self, cell_id):
        cell = self.cells[cell_id]
        laneset = self.lanesets[cell.belonged_laneset]
        cell.jam_density = 7
        cell.shockwave_ratio = 1 / 3
        cell.free_flow_speed = laneset.speed_limit
        cell.shockwave_speed = cell.shockwave_ratio * cell.free_flow_speed
        cell.maximum_flow = max(0.5 * self.time_interval * laneset.lane_number, 1)
        cell.maximum_veh = max(self.time_interval * cell.free_flow_speed * laneset.lane_number,
                               self.time_interval * cell.free_flow_speed) / cell.jam_density
        if cell.type == "origin":
            # self.update_cell_demand(cell_id, 1)
            cell.demand = [[0] * self.total_steps]
        if cell.type == "destination":
            cell.maximum_flow = 1e10
            cell.maximum_veh = 1e10

    def update_cell_demand(self, cell_id, num_scenario):
        cell = self.cells[cell_id]
        laneset = self.lanesets[cell.belonged_laneset]
        if cell.type == "origin":
            # pass
            # fixme: update demand by real data
            shape = 2  # use gamma distribution
            if cell.belonged_node == '665630433':
                if laneset.from_direction == 'W':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.3 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.4]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.4 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'E':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.1 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.1]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.1 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'S':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.1 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.1]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'N':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.05 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.05]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)

            if cell.belonged_node == '62490467':
                if laneset.from_direction == 'W':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.2 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.2]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.4 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'E':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.5 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.5]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.1 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'S':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.05 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.05]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'N':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.1 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.1]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)

            if cell.belonged_node == '342969885':
                if laneset.from_direction == 'W':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.1 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.1]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.4 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'E':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.2 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.2]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.1 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'S':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.15 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.15]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'N':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.15 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.15]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)

            if cell.belonged_node == '62514378':
                if laneset.from_direction == 'W':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.2 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.2]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.4 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'E':
                    np.random.seed(100)
                    sample_arr = stats.gamma(shape, 0, 0.1 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.1]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.1 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'S':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.1 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.1]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(0)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
                if laneset.from_direction == 'N':
                    np.random.seed(200)
                    sample_arr = stats.gamma(shape, 0, 0.05 / shape).rvs(num_scenario)
                    if num_scenario == 1:
                        sample_arr = [0.05]
                    demand = [None] * num_scenario
                    for xi in range(num_scenario):
                        np.random.seed(1)
                        # demand = np.random.poisson(0.15 * cell.maximum_flow, self.total_steps)
                        demand[xi] = np.random.poisson(sample_arr[xi] * cell.maximum_flow, self.total_steps)
        cell.demand = demand

    def update_demand(self, od_pairs, num_scenario, seed=False):
        # od_pairs = solve_user_equilibrium(network, demand_file)
        # for laneset_id, laneset in self.lanesets():
        #     laneset.laneset_flow = network.lanesets[laneset_id].laneset_flow
        sample = 0
        for cell_id, cell in self.cells.items():
            if cell.type == "origin":
                # self.update_cell_demand(cell_id, num_scenario)
                cell.demand = [None] * num_scenario
                mean_flow = 0
                for od_id, od_pair in od_pairs.items():
                    if od_pair.origin == cell.belonged_end_node:
                        mean_flow += od_pair.flow_rate
                if num_scenario == 1:
                    sample_arr = [mean_flow]
                else:
                    sample_arr = [0] * num_scenario
                    for xi in range(num_scenario):
                        # sample_arr = stats.norm(mean_flow, 0.7 * mean_flow).rvs(num_scenario)
                        while sample_arr[xi] <= 0:
                            if seed:
                                np.random.seed(sample)
                                sample += 1
                            sample_arr[xi] = stats.norm(mean_flow, 0.7 * mean_flow).rvs(1)[0]
                for xi in range(num_scenario):
                    cell.demand[xi] = [max(sample_arr[xi] / (3600 / self.time_interval), 0)] * self.total_steps

    def update_fake_demand(self, mean, variance, num_scenario, seed=None):
        self.num_scenario = num_scenario
        sample = seed
        for cell_id, cell in self.cells.items():
            if cell.type == "origin":
                # self.update_cell_demand(cell_id, num_scenario)
                cell.demand = [None] * num_scenario
                link = self.links[cell.belonged_link]
                if link.from_direction == "W":
                    mean_flow = mean[0]
                if link.from_direction == "E":
                    mean_flow = mean[1]
                if link.from_direction == "N":
                    mean_flow = mean[2]
                if link.from_direction == "S":
                    mean_flow = mean[3]
                # np.random.seed(sample)
                cell.mean_demand = mean_flow
                if num_scenario == 1:
                    sample_arr = [mean_flow]
                else:
                    sample_arr = [0] * num_scenario
                    # sample_arr = stats.norm(mean_flow, 0.7 * mean_flow).rvs(num_scenario)
                    for xi in range(num_scenario):
                        while sample_arr[xi] <= 0:
                            if seed:
                                np.random.seed(sample)
                                sample += 1
                            sample_arr[xi] = stats.norm(mean_flow, variance * mean_flow).rvs(1)[0]
                for xi in range(num_scenario):
                    cell.demand[xi] = [max(sample_arr[xi] / (3600 / self.time_interval), 0)] * self.total_steps
                # sample += 1

    def update_laneset_flow_ue(self, network, od_pairs):
        solve_user_equilibrium(network, od_pairs)
        for laneset_id, laneset in self.lanesets.items():
            laneset.laneset_flow = network.lanesets[laneset_id].laneset_flow

    def update_laneset_flow_turning_ratio(self, network):
        m = gb.Model()
        link_vars = {}
        laneset_vars = {}
        # todo: compute by laneset flow
        for link_id, link in network.links.items():
            link_vars[link_id] = m.addVar()
            for segment_id in link.segment_list:
                segment = network.segments[segment_id]
                for laneset_id in segment.laneset_list:
                    laneset_vars[laneset_id] = m.addVar()
                    laneset = self.lanesets[laneset_id]
                    if len(segment.laneset_list) == 1:
                        m.addConstr(laneset_vars[laneset_id] - link_vars[link_id] == 0)
                    elif len(segment.laneset_list) > 1:
                        connector = network.connectors[laneset.upstream_connectors[0]]
                        m.addConstr(laneset_vars[laneset_id] - link_vars[link_id] * connector.diverge_proportion
                        [segment.laneset_list.index(laneset_id)] == 0)
        m.update()

        for link_id, link in network.links.items():
            if network.get_node_type(link.upstream_node) == "end":
                arrival = 0
                for laneset_id in network.segments[link.segment_list[0]].laneset_list:
                    laneset = self.lanesets[laneset_id]
                    arrival += self.cells[laneset.cell_ids[0]].mean_demand
                m.addConstr(link_vars[link_id] == arrival)
            else:
                # print(network.get_node_type(link.upstream_node))
                # print(network.segments[link.segment_list[0]].laneset_list)
                # if link_id != "-101754_-101776":
                #     continue
                laneset = network.lanesets[network.segments[link.segment_list[0]].laneset_list[0]]
                laneset_id = laneset.laneset_id
                linear_expr = 0
                for upstream_connector_id in laneset.upstream_connectors:
                    connector = network.connectors[upstream_connector_id]
                    # upstream_lanesets.append(connector.upstream_laneset)
                    upstream_laneset = network.lanesets[connector.upstream_laneset]
                    linear_expr += \
                        connector.diverge_proportion[connector.downstream_lanesets.index(laneset.laneset_id)] * \
                        laneset_vars[upstream_laneset.laneset_id]
                # print(linear_expr)
                m.addConstr(laneset_vars[laneset_id] - linear_expr == 0)
                # m.addConstr(laneset_vars[laneset_id] - linear_expr <= 1)
                # m.addConstr(laneset_vars[laneset_id] - linear_expr >= -1)
        m.setObjective(0)
        m.setParam('OutputFlag', 0)
        m.optimize()

        for link_id, link in network.links.items():
            link.link_flow = link_vars[link_id].x
            self.links[link_id].link_flow = link_vars[link_id].x
            for segment_id in link.segment_list:
                segment = network.segments[segment_id]
                for laneset_id in segment.laneset_list:
                    network.lanesets[laneset_id].laneset_flow = laneset_vars[laneset_id].x
                    self.lanesets[laneset_id].laneset_flow = laneset_vars[laneset_id].x

        for node_id, node in self.nodes.items():
            if node.type != 'signalized':
                continue
            node.phase_ratio = {}
            for phase_id, phase in node.timing_plan.phases.items():
                node.phase_ratio[phase_id] = max([self.lanesets[laneset_id].laneset_flow /
                                                  self.lanesets[laneset_id].capacity
                                                  for laneset_id in phase.controlled_lanesets])
            node.v_c_ratio = sum(node.phase_ratio[phase_id] for phase_id in node.timing_plan.phases.keys())

    def update_demand_by_file(self, file_name, network):
        load_link_flow(network, file_name)
        # check_flow_conservation(network)
        compute_laneset_flow(network)

        for link_id, link in self.links.items():
            self.links[link_id].mean_link_flow = network.links[link_id].link_flow
            self.links[link_id].link_flow = network.links[link_id].link_flow

        for laneset_id, laneset in self.lanesets.items():
            self.lanesets[laneset_id].mean_laneset_flow = network.lanesets[laneset_id].laneset_flow
            self.lanesets[laneset_id].laneset_flow = network.lanesets[laneset_id].laneset_flow

        for connector_id, connector in self.connectors.items():
            if connector.network_connector_id is not None:
                network_connector = network.connectors[connector.network_connector_id]
                connector.diverge_prop_mean = network_connector.diverge_proportion
                if connector.type == "diverge":
                    num_downstream_lanesets = len(network_connector.downstream_lanesets)
                    for i in range(num_downstream_lanesets - 1):
                        connector.diverge_prop_mean[i] = min(max(network_connector.diverge_proportion[i], 0), 1)
                    connector.diverge_prop_mean[num_downstream_lanesets - 1] = \
                        max(1 - sum(connector.diverge_prop_mean[i] for i in range(num_downstream_lanesets - 1)), 0)
                    connector.diverge_prop = [connector.diverge_prop_mean]
                # connector.diverge_prop_mean = network.connectors[connector.network_connector_id].diverge_proportion

        for cell_id, cell in self.cells.items():
            if cell.type == "origin":
                link = self.links[cell.belonged_link]
                cell.mean_demand = link.mean_link_flow

    def update_demand_uncertain(self, variance, num_scenario, seed=None):
        self.num_scenario = num_scenario
        sample = seed
        for cell_id, cell in self.cells.items():
            if cell.type == "origin":
                cell.demand = [None] * num_scenario
                if num_scenario == 1:
                    sample_arr = [cell.mean_demand]
                else:
                    sample_arr = [0] * num_scenario
                    # sample_arr = stats.norm(mean_flow, 0.7 * mean_flow).rvs(num_scenario)
                    for xi in range(num_scenario):
                        while sample_arr[xi] <= 0:
                            if seed:
                                np.random.seed(sample)
                                sample += 1
                            sample_arr[xi] = stats.norm(cell.mean_demand, variance * cell.mean_demand).rvs(1)[0]
                for xi in range(num_scenario):
                    cell.demand[xi] = [max(sample_arr[xi] / (3600 / self.time_interval), 0)] * self.total_steps

    def is_exceed_capacity(self, network, node_id):
        node = self.nodes[node_id]
        max_ratio = {}
        node.phase_ratio = {}
        for phase_id, phase in node.timing_plan.phases.items():
            max_ratio[phase_id] = max([self.lanesets[laneset_id].laneset_flow / self.lanesets[laneset_id].capacity
                                       for laneset_id in phase.controlled_lanesets])
            node.phase_ratio[phase_id] = max_ratio[phase_id]
        node.v_c_ratio = sum(max_ratio[phase_id] for phase_id in node.timing_plan.phases.keys())
        network.nodes[node_id].v_c_ratio = node.v_c_ratio
        if node.v_c_ratio >= 1:
            return True
        else:
            return False

    def compute_cycle_length(self):
        total_cycle_length = 0
        all_cycle_length = []
        for node_id, node in self.nodes.items():
            if node.type == "signalized":
                node.cycle_length = int(np.ceil((len(node.timing_plan.phases) * 5 * 1.5 + 5) / (1 - node.v_c_ratio)
                                                / self.time_interval))
                total_cycle_length += node.cycle_length
                all_cycle_length.append(node.cycle_length)
        # mean_cycle_length = np.ceil(total_cycle_length / len(self.signalized_nodes_id))
        all_cycle_length.sort()
        percent_cycle_length = all_cycle_length[min(int(np.ceil(len(all_cycle_length) * 0.8)),
                                                    len(all_cycle_length) - 1)]
        for node_id, node in self.nodes.items():
            if node.type == "signalized":
                node.cycle_length = int(percent_cycle_length)

    def generate_naive_signal_plan(self, node_id):
        node = self.nodes[node_id]
        flow = {}
        all_flow = 0
        for phase_id, phase in node.timing_plan.phases.items():
            flow[phase_id] = max([self.lanesets[laneset_id].laneset_flow
                                  for laneset_id in phase.controlled_lanesets])
            all_flow += flow[phase_id]

        # cycle_length = int(30 / self.time_interval) * len(node.timing_plan.phases)
        # cycle_length = int(np.ceil(20 * 1.5 / (1 - node.v_c_ratio) / self.time_interval))
        cycle_length = node.cycle_length
        num_cycle = int(self.total_steps / cycle_length + 1)
        start_time = 0
        for phase_id, phase in node.timing_plan.phases.items():
            phase.signal_state = [0] * self.total_steps
            green_phase_length = int(np.floor(flow[phase_id] / all_flow * cycle_length))
            for time_step in range(green_phase_length):
                for j in range(num_cycle):
                    current_time_step = time_step + j * cycle_length + start_time
                    if current_time_step < self.total_steps:
                        phase.signal_state[current_time_step] = 1
            start_time += green_phase_length

    def build_link_list(self, network):
        for link_id, net_link in network.links.items():
            link = ctm_cls.CTMLink.init_from_link(net_link)
            self.add_link(link)

    def build_node_list(self, network):
        for node_id, net_node in network.nodes.items():
            if net_node.type == "signalized":
                node = ctm_cls.CTMSignalizedNode.init_from_node(net_node)
                node.cycle_length = int(30 / self.time_interval * len(net_node.timing_plan.phases))
                self.add_node(node)
            if net_node.type == "unsignalized":
                node = ctm_cls.CTMUnsignalizedNode.init_from_node(net_node)
                self.add_node(node)
            if net_node.type == "connector":
                node = ctm_cls.CTMSegmentConnectionNode.init_from_node(net_node)
                self.add_node(node)
            if net_node.type == "end":
                node = ctm_cls.CTMEndNode.init_from_node(net_node)
                self.add_node(node)

    def build_laneset_list(self, network):
        for laneset_id, net_laneset in network.lanesets.items():
            laneset = ctm_cls.CTMLaneset.init_from_laneset(net_laneset)
            self.add_laneset(laneset)

    def build_conflict_point_list(self, network):
        for conflict_point_id, net_conflict_point in network.conflict_points.items():
            conflict_point = ctm_cls.CTMConflictPoint.init_from_conflict_point(net_conflict_point)
            self.add_conflict_point(conflict_point)

    def generate_cells(self, network):
        for link_id, link in self.links.items():
            current_segment_length = 0
            for segment_id in link.segment_list:
                segment = network.segments[segment_id]
                # for segment_id, segment in network.segments.items():
                # link_id = segment.belonged_link
                for lane_id in segment.laneset_list:
                    laneset = self.lanesets[lane_id]
                    laneset.num_cells = max(int(np.ceil(laneset.length / self.time_interval / laneset.speed_limit)), 1)
                    cell_lat_list, cell_lon_list = segment_gps_trace(laneset.geometry["lat"], laneset.geometry["lon"],
                                                                     laneset.num_cells)
                    for i in range(laneset.num_cells):
                        # judge if the cell is origin or destination cell
                        # if link.upstream_node is None and cur_length == 0:
                        if network.get_node_type(laneset.upstream_node) == "end" and i == 0:
                            cell = ctm_cls.CTMOriginCell(link_id + "@" + segment_id + "@" + lane_id + "@" + str(i))
                            cell.belonged_end_node = laneset.upstream_node
                        # elif laneset.destination_pipelines is None and i == num_cell - 1:
                        elif network.get_node_type(laneset.downstream_node) == "end" and i == laneset.num_cells - 1:
                            cell = ctm_cls.CTMDestinationCell(link_id + "@" + segment_id + "@" + lane_id + "@" + str(i))
                        else:
                            cell = ctm_cls.CTMCell(link_id + "@" + segment_id + "@" + lane_id + "@" + str(i))
                        cell.geometry["lat"], cell.geometry["lon"] = cell_lat_list[i], cell_lon_list[i]
                        self.add_cell_to_network(cell)
                        # self.update_cell_params(cell.cell_id)
                        self.lanesets[lane_id].cell_ids.append(cell.cell_id)
                        # set the node cell belongs to
                        if current_segment_length + i * self.time_interval * laneset.speed_limit \
                                <= link.buffer_length / 2:
                            if network.get_node_type(link.upstream_node) != "end":
                                self.add_cell_to_node(link.upstream_node, cell.cell_id)
                            else:
                                self.add_cell_to_node(link.downstream_node, cell.cell_id)
                        else:
                            if network.get_node_type(link.downstream_node) == "end":
                                self.add_cell_to_node(link.upstream_node, cell.cell_id)
                            else:
                                self.add_cell_to_node(link.downstream_node, cell.cell_id)
                        self.update_cell_params(cell.cell_id)
                current_segment_length += segment.length

    def build_and_update_straight_connector(self, cell_id, downstream_cell_id, laneset_id):
        cell, downstream_cell = self.cells[cell_id], self.cells[downstream_cell_id]
        connector = ctm_cls.CTMStraightConnector(cell_id, downstream_cell_id)
        connector.upstream_node = cell.belonged_node
        connector.downstream_node = downstream_cell.belonged_node
        self.add_connector_to_network(connector)
        laneset = self.lanesets[laneset_id]
        laneset.connector_ids.append(connector.connector_id)
        # update cell
        cell.downstream_connector.append(connector.connector_id)
        downstream_cell.upstream_connector.append(connector.connector_id)
        # judge if the connector is boundary connector
        self.add_connector_to_node(cell.belonged_node, connector.connector_id)
        if cell.belonged_node != downstream_cell.belonged_node:
            self.add_outflow_boundary_connector_to_node(cell.belonged_node, connector.connector_id)
            self.add_inflow_boundary_connector_to_node(downstream_cell.belonged_node, connector.connector_id, 0)
        return connector.connector_id

    def build_and_update_diverge_connector(self, cell_id, downstream_cells_id, diverge_proportion, laneset_id):
        connector = ctm_cls.CTMDivergeConnector(cell_id, downstream_cells_id, diverge_proportion)
        cell = self.cells[cell_id]
        connector.upstream_node = cell.belonged_node
        # connector.downstream_node = cell.belonged_node
        connector.downstream_node = [self.cells[downstream_cell].belonged_node for downstream_cell in
                                     downstream_cells_id]
        self.add_connector_to_network(connector)
        laneset = self.lanesets[laneset_id]
        laneset.connector_ids.append(connector.connector_id)
        # update cell
        cell.downstream_connector.append(connector.connector_id)
        for downstream_cell_id in downstream_cells_id:
            self.cells[downstream_cell_id].upstream_connector.append(connector.connector_id)
        self.add_connector_to_node(cell.belonged_node, connector.connector_id)
        for downstream_cell_id in downstream_cells_id:
            downstream_cell = self.cells[downstream_cell_id]
            if cell.belonged_node != downstream_cell.belonged_node:
                self.add_outflow_boundary_connector_to_node(cell.belonged_node, connector.connector_id)
                self.add_inflow_boundary_connector_to_node(downstream_cell.belonged_node, connector.connector_id,
                                                           downstream_cells_id.index(downstream_cell_id))
        return connector.connector_id

    def update_signal_priority_connector(self, connector_id, network_connector):
        connector = self.connectors[connector_id]
        connector.priority_class = network_connector.priority_class
        connector.conflict_points = network_connector.conflict_points
        # update conflict point
        if connector.conflict_points:
            for conflict_point_id in connector.conflict_points:
                conflict_point = self.conflict_points[conflict_point_id]
                for idx in range(len(conflict_point.conflict_connectors)):
                    if network_connector.connector_id in conflict_point.conflict_connectors[idx]:
                        sub_idx = conflict_point.conflict_connectors[idx].index(network_connector.connector_id)
                        conflict_point.conflict_CTM_connectors[idx][sub_idx] = connector_id
        if network_connector.type == "signalized":
            connector.signalized = True
            connector.phase_id = network_connector.phase_id

    def update_turning_ratio(self, num_scenario, variance, seed=None):
        sample = seed
        if num_scenario == 1:
            for connector_id, connector in self.connectors.items():
                if connector.type == "diverge":
                    connector.diverge_prop = [connector.diverge_prop_mean]
            return
        for connector_id, connector in self.connectors.items():
            if connector.type != "diverge":
                continue
            mean_diverge_proportion = connector.diverge_prop_mean
            num_downstream_lanesets = len(connector.downstream_cell)
            if num_downstream_lanesets > 1:
                if num_scenario == 1:
                    sample_arr = [[mean_diverge_proportion[i]] for i in range(num_downstream_lanesets)]
                else:
                    sample_arr = []
                    trunc_a = 0
                    trunc_b = 1
                    for i in range(num_downstream_lanesets):
                        sample_arr.append([])
                        for xi in range(num_scenario):
                            if seed:
                                np.random.seed(sample)
                                sample += 1
                            mean = mean_diverge_proportion[i]
                            sd = variance * mean_diverge_proportion[i]
                            trunc_b = max(1 - sum(sample_arr[j][xi] for j in range(i)), 1e-4)
                            if mean == 0:
                                sample_arr[i].append(0)
                            else:
                                sample_arr[i].append(
                                    stats.truncnorm((trunc_a - mean) / sd, (trunc_b - mean) / sd, mean, sd).rvs(1)[0])
                            # sample_arr = [
                    # stats.truncnorm(mean_diverge_proportion[i], variance * mean_diverge_proportion[i]).rvs(num_scenario)
                    #     for i in range(num_downstream_lanesets)]
                connector.diverge_prop = [None] * num_scenario
                for xi in range(num_scenario):
                    connector.diverge_prop[xi] = [0] * num_downstream_lanesets
                    for i in range(num_downstream_lanesets - 1):
                        # if seed:
                        #     np.random.seed(sample)
                        #     sample += 1
                        # connector.diverge_prop[xi][i] = min(max(sample_arr[i][xi], 0.00001), 0.99999)
                        connector.diverge_prop[xi][i] = sample_arr[i][xi]
                    connector.diverge_prop[xi][num_downstream_lanesets - 1] = \
                        max(1 - sum(sample_arr[i][xi] for i in range(num_downstream_lanesets - 1)), 1e-4)

    def generate_connectors(self, network):
        # build connectors in laneset
        for laneset_id, laneset in self.lanesets.items():
            for c in range(laneset.num_cells - 1):
                cell_id, downstream_cell_id = laneset.cell_ids[c], laneset.cell_ids[c + 1]
                connector_id = self.build_and_update_straight_connector(cell_id, downstream_cell_id, laneset_id)
                self.add_connector_to_priority_list(connector_id)
        # travel along all nodes to build connector between lanesets
        for node_id, node in self.nodes.items():
            # connector in a link but between two different segments
            for network_connector_id in node.connector_list:
                if node.type in ["connector", "unsignalized", "signalized"]:
                    network_connector = network.connectors[network_connector_id]
                    upstream_laneset = self.lanesets[network_connector.upstream_laneset]
                    cell_id = upstream_laneset.cell_ids[upstream_laneset.num_cells - 1]
                    if len(network_connector.downstream_lanesets) == 1:
                        downstream_laneset = self.lanesets[network_connector.downstream_lanesets[0]]
                        downstream_cell_id = downstream_laneset.cell_ids[0]
                        connector_id = self.build_and_update_straight_connector(cell_id, downstream_cell_id,
                                                                                network_connector.upstream_laneset)
                        network_connector.diverge_proportion = [1]
                        self.connectors[connector_id].network_connector_id = network_connector_id
                    if len(network_connector.downstream_lanesets) > 1:
                        downstream_cells_id = []
                        for downstream_laneset_id in network_connector.downstream_lanesets:
                            downstream_laneset = self.lanesets[downstream_laneset_id]
                            downstream_cells_id.append(downstream_laneset.cell_ids[0])
                        # self.build_and_update_diverge_connector(cell_id, downstream_cells_id,
                        #                                         network_connector.diverge_proportion,
                        #                                         network_connector.upstream_laneset)
                        # fixme: update diverge proportion by real data
                        if len(downstream_cells_id) == 2:
                            upstream_link = self.links[
                                network.segments[network_connector.upstream_segment].belonged_link]
                            # if upstream_link.from_direction in ["E", "W"]:
                            #     if network_connector.direction == "s":
                            #         diverge_proportion = [0.1, 0.9]
                            #     if network_connector.direction == "rs":
                            #         diverge_proportion = [0.15, 0.85]
                            # if upstream_link.from_direction in ["S", "N"]:
                            #     if network_connector.direction == "s":
                            #         diverge_proportion = [0.2, 0.8]
                            #     if network_connector.direction == "rs":
                            #         diverge_proportion = [0.3, 0.7]
                            if network_connector.direction == "s":
                                diverge_proportion = [0.15, 0.85]
                            if network_connector.direction == "rs":
                                diverge_proportion = [0.15, 0.85]
                        elif len(downstream_cells_id) == 3:
                            diverge_proportion = [0.1, 0.7, 0.2]
                        else:
                            diverge_proportion = [1 / len(downstream_cells_id)] * len(downstream_cells_id)
                        network_connector.diverge_proportion = diverge_proportion
                        connector_id = self.build_and_update_diverge_connector(cell_id, downstream_cells_id,
                                                                               diverge_proportion,
                                                                               network_connector.upstream_laneset)
                        self.connectors[connector_id].network_connector_id = network_connector_id

                    self.update_signal_priority_connector(connector_id, network_connector)
                    self.add_connector_to_priority_list(connector_id)

    def step(self, time_step, xi):
        """
        (dynamics equation for one time step)
        move forward the simulation for one time step
        :return:
        """
        self.get_vehicle_flow(time_step, xi)
        self.update_vehicle_number(time_step, xi)  # conservation law

    def run(self, xi):
        """
        :return:
        """
        for time_step in range(self.total_steps):
            self.step(time_step, xi)

    def get_flow_of_upstream_connector(self, time_step, cell_id, connector_id):
        connector = self.connectors[connector_id]
        if connector.type == "diverge":
            idx = connector.downstream_cell.index(cell_id)
            cell_flow = connector.flow[time_step][idx]
        if connector.type == "merge":
            cell_flow = sum(connector.flow[time_step])
        if connector.type == "straight":
            cell_flow = connector.flow[time_step][0]
        return cell_flow

    def get_flow_of_downstream_connector(self, time_step, cell_id, connector_id):
        connector = self.connectors[connector_id]
        if connector.type == "merge":
            idx = connector.upstream_cell.index(cell_id)
            cell_flow = connector.flow[time_step][idx]
        if connector.type == "diverge":
            cell_flow = sum(connector.flow[time_step])
        if connector.type == "straight":
            cell_flow = connector.flow[time_step][0]
        return cell_flow

    def update_vehicle_number(self, time_step, xi):
        for cell_id, cell in self.cells.items():
            flow_in = 0
            flow_out = 0
            if cell.type == "ordinary":
                for upstream_connector_id in cell.upstream_connector:
                    flow_in += self.get_flow_of_upstream_connector(time_step, cell_id, upstream_connector_id)
                for downstream_connector_id in cell.downstream_connector:
                    flow_out += self.get_flow_of_downstream_connector(time_step, cell_id, downstream_connector_id)

            if cell.type == "origin":
                flow_in = cell.demand[xi][time_step]
                for downstream_connector_id in cell.downstream_connector:
                    flow_out += self.get_flow_of_downstream_connector(time_step, cell_id, downstream_connector_id)

            if cell.type == "destination":
                for upstream_connector_id in cell.upstream_connector:
                    flow_in += self.get_flow_of_upstream_connector(time_step, cell_id, upstream_connector_id)
                flow_out = 0

            cell.num_vehicles.append(cell.num_vehicles[time_step] + flow_in - flow_out)
            cell.occupation.append(cell.num_vehicles[time_step + 1] / cell.maximum_veh)

    def update_flow_by_connector(self, time_step, connector_id, capacity, xi):
        connector = self.connectors[connector_id]
        if connector.type == "straight":
            upstream_cell = self.cells[connector.upstream_cell]
            downstream_cell = self.cells[connector.downstream_cell[0]]
            node = self.nodes[connector.belonged_node]
            if connector.signalized and \
                    node.timing_plan.phases[connector.phase_id].signal_state[time_step] == 0:
                connector.flow.append([0])
            else:
                connector.flow.append([min(upstream_cell.num_vehicles[time_step], upstream_cell.maximum_flow,
                                           downstream_cell.maximum_flow, downstream_cell.shockwave_ratio
                                           * (downstream_cell.maximum_veh
                                              - downstream_cell.num_vehicles[time_step]), capacity)])
        if connector.type == "diverge":
            upstream_cell = self.cells[connector.upstream_cell]
            downstream_flow_list = []
            downstream_veh_list = []
            connector.flow.append([0] * len(connector.downstream_cell))
            node = self.nodes[connector.belonged_node]
            if connector.signalized and \
                    node.timing_plan.phases[connector.phase_id].signal_state[time_step] == 0:
                total_flow = 0
            else:
                for cidx in range(len(connector.downstream_cell)):
                    downstream_cell = self.cells[connector.downstream_cell[cidx]]
                    downstream_flow_list.append(downstream_cell.maximum_flow /
                                                max(connector.diverge_prop[xi][cidx], 0.001))
                    downstream_veh_list.append(downstream_cell.shockwave_ratio
                                               * (downstream_cell.maximum_veh
                                                  - downstream_cell.num_vehicles[time_step])
                                               / max(connector.diverge_prop[xi][cidx], 0.001))
                total_flow = min(upstream_cell.num_vehicles[time_step], upstream_cell.maximum_flow,
                                 min(downstream_flow_list), min(downstream_veh_list), capacity)
            for cidx in range(len(connector.diverge_prop[xi])):
                connector.flow[time_step][cidx] = total_flow * connector.diverge_prop[xi][cidx]

    def get_vehicle_flow(self, time_step, xi):
        """
        get the inflow/outflow between adjacent cells
        iteration for all the ctm connectors
        :return:
        """
        # get flow of mixed priority connectors
        for conflict_point_id, conflict_point in self.conflict_points.items():
            if conflict_point.priority == "mixed":
                num_conflict = len(conflict_point.conflict_CTM_connectors)
                current_priority = [(time_step + p) % num_conflict for p in range(num_conflict)]
                priority_index = np.argsort(current_priority)  # index sorted by priority
                for idx in range(num_conflict):
                    current_controlled_connector = []
                    for sub_idx in range(idx):
                        current_controlled_connector += conflict_point.conflict_CTM_connectors[priority_index[sub_idx]]
                    capacity = max(conflict_point.capacity - sum(sum(self.connectors[connector_id].flow[time_step])
                                                                 for connector_id in current_controlled_connector), 0)
                    pidx = priority_index[idx]
                    for connector_id in conflict_point.conflict_CTM_connectors[pidx]:
                        self.update_flow_by_connector(time_step, connector_id, capacity, xi)

        for priority_id in range(len(self.connectors_by_priority)):
            if priority_id == 1:
                for connector_id in self.connectors_by_priority[priority_id]:
                    self.update_flow_by_connector(time_step, connector_id, 1e6, xi)
            if priority_id > 1:
                for connector_id in self.connectors_by_priority[priority_id]:
                    connector = self.connectors[connector_id]
                    conflict_point = self.conflict_points[connector.conflict_points[0]]
                    current_controlled_connector = []
                    for sub_idx in range(priority_id - 1):
                        current_controlled_connector += conflict_point.conflict_CTM_connectors[sub_idx]
                    capacity = conflict_point.capacity - \
                               sum(sum(self.connectors[controlled_connector_id].flow[time_step])
                                   for controlled_connector_id in current_controlled_connector)
                    self.update_flow_by_connector(time_step, connector_id, capacity, xi)

    def reset_flow(self):
        for cell_id, cell in self.cells.items():
            cell.num_vehicles = [0]
            cell.occupation = [0]
        for connector_id, connector in self.connectors.items():
            connector.flow = []

    def evaluate(self, network, file_name, figure_folder, num_scenario):
        """
        compute the cumulative inflow and cumulative outflow for evaluation
        """
        out_put = open(file_name, "w+")
        total_travel_time = [None] * num_scenario
        total_travel_delay = [None] * num_scenario
        average_travel_time = [None] * num_scenario
        average_travel_delay = [None] * num_scenario
        total_throughput = [None] * num_scenario
        total_arrival = [None] * num_scenario
        cumulative_throughput = [None] * num_scenario

        all_total_travel_time = np.zeros(num_scenario)
        all_total_travel_delay = np.zeros(num_scenario)
        all_total_delay_ew = np.zeros(num_scenario)
        all_total_delay_ns = np.zeros(num_scenario)
        all_average_travel_time = np.zeros(num_scenario)
        all_average_travel_delay = np.zeros(num_scenario)
        all_total_throughput = np.zeros(num_scenario)
        all_cumulative_throughput = np.zeros(num_scenario)
        all_objective = np.zeros(num_scenario)

        all_total_inflow = np.zeros((num_scenario, self.total_steps))
        all_total_outflow = np.zeros((num_scenario, self.total_steps))
        all_total_freeflow = np.zeros((num_scenario, self.total_steps))
        all_total_free_time = np.zeros(num_scenario)

        origin_cell_ids = []
        destination_cell_ids = []
        for cell_id, cell in self.cells.items():
            if cell.type == "origin":
                origin_cell_ids.append(cell_id)
            if cell.type == "destination":
                destination_cell_ids.append(cell_id)

        link_ew_ids = []
        link_ns_ids = []
        for link_id, link in self.links.items():
            if link.from_direction in ["E", "W"]:
                link_ew_ids.append(link_id)
            if link.from_direction in ["N", "S"]:
                link_ns_ids.append(link_id)

        for link_id, link in self.links.items():
            capacity = 0
            link.cumulative_inflow = np.zeros(self.total_steps)
            link.cumulative_outflow = np.zeros(self.total_steps)
            link.cumulative_freeflow = np.zeros(self.total_steps)
            link.link_free_travel_time = 0
            for segment_id in link.segment_list:
                segment = network.segments[segment_id]
                link.link_free_travel_time += self.lanesets[segment.laneset_list[0]].num_cells
                for laneset_id in segment.laneset_list:
                    laneset = self.lanesets[laneset_id]
                    for cell_id in laneset.cell_ids:
                        cell = self.cells[cell_id]
                        capacity += cell.maximum_veh
            link.link_max_vehicles = capacity

        for xi in tqdm(range(num_scenario)):
            self.reset_flow()
            # # fixme: add warm-up
            # warmup_step = 100
            # for cell_id, cell in self.cells.items():
            #     cell.num_vehicles = [cell.warm_up_value[warmup_step]]
            self.run(xi)
            for link_id, link in self.links.items():
                link.cumulative_inflow = np.zeros(self.total_steps)
                link.cumulative_outflow = np.zeros(self.total_steps)
            for time_step in range(self.total_steps):
                for link_id, link in self.links.items():
                    segment = network.segments[link.segment_list[0]]
                    for laneset_id in segment.laneset_list:
                        laneset = self.lanesets[laneset_id]
                        cell = self.cells[laneset.cell_ids[0]]
                        if cell.type == "origin":
                            link.cumulative_inflow[time_step] += cell.demand[xi][time_step]
                        else:
                            for upstream_connector_id in cell.upstream_connector:
                                upstream_connector = self.connectors[upstream_connector_id]
                                link.cumulative_inflow[time_step] += upstream_connector.flow[time_step][
                                    upstream_connector.downstream_cell.index(cell.cell_id)]
                    segment = network.segments[link.segment_list[-1]]
                    for laneset_id in segment.laneset_list:
                        laneset = self.lanesets[laneset_id]
                        cell = self.cells[laneset.cell_ids[laneset.num_cells - 1]]
                        if cell.type == "destination":
                            link.cumulative_outflow[time_step] += cell.num_vehicles[time_step + 1] - \
                                                                  cell.num_vehicles[time_step]
                        else:
                            for downstream_connector_id in cell.downstream_connector:
                                downstream_connector = self.connectors[downstream_connector_id]
                                link.cumulative_outflow[time_step] += sum(downstream_connector.flow[time_step])

                    if time_step > 0:
                        link.cumulative_inflow[time_step] += link.cumulative_inflow[time_step - 1]
                        link.cumulative_outflow[time_step] += link.cumulative_outflow[time_step - 1]
                    # if time_step >= link.link_free_travel_time:
                    link.cumulative_freeflow[time_step] = link.cumulative_inflow[max(
                        time_step - link.link_free_travel_time, 0)]

            total_travel_time[xi] = {}
            average_travel_time[xi] = {}
            total_travel_delay[xi] = {}
            average_travel_delay[xi] = {}
            total_throughput[xi] = {}
            cumulative_throughput[xi] = {}
            total_arrival[xi] = {}

            arrival_node = {}
            throughput_node = {}
            for node_id in self.intersections_id:
                node = self.nodes[node_id]
                # total_travel_time[xi][node_id] = sum(
                #     sum(self.links[link_id].cumulative_inflow[time_step] - self.links[link_id].cumulative_outflow[
                #         time_step] for link_id in node.upstream_links) for time_step in range(self.total_steps))
                total_travel_time[xi][node_id] = sum(self.links[link_id].cumulative_inflow[time_step]
                                                     - self.links[link_id].cumulative_outflow[time_step]
                                                     for link_id in node.upstream_links
                                                     for time_step in range(self.total_steps))
                # fixme: update computation of average travel time
                average_travel_time[xi][node_id] = total_travel_time[xi][node_id] * 2 / max(sum(
                    self.links[link_id].cumulative_inflow[self.total_steps - 1] +
                    self.links[link_id].cumulative_outflow[self.total_steps - 1]
                    for link_id in node.upstream_links), 0.001)
                total_travel_delay[xi][node_id] = sum(
                    sum(self.links[link_id].cumulative_inflow[time_step - self.links[link_id].link_free_travel_time] -
                        self.links[link_id].cumulative_outflow[time_step] for time_step in range(
                        self.links[link_id].link_free_travel_time, self.total_steps))
                    for link_id in node.upstream_links)
                # fixme: update computation of average travel delay
                average_travel_delay[xi][node_id] = total_travel_delay[xi][node_id] * 2 / max(sum(
                    self.links[link_id].cumulative_inflow[self.total_steps - 1] +
                    self.links[link_id].cumulative_outflow[self.total_steps - 1]
                    for link_id in node.upstream_links), 0.001)
                cumulative_throughput[xi][node_id] = sum(sum(self.cells[cell_id].num_vehicles) for cell_id in
                                                         node.destination_cell_list)
                total_throughput[xi][node_id] = sum(self.cells[cell_id].num_vehicles[self.total_steps] for cell_id in
                                                    node.destination_cell_list)
                total_arrival[xi][node_id] = sum(self.links[link_id].cumulative_inflow[self.total_steps - 1]
                                                 for link_id in node.upstream_links)
                total_throughput[xi][node_id] = sum(self.links[link_id].cumulative_outflow[self.total_steps - 1]
                                                    for link_id in node.upstream_links)
                cumulative_throughput[xi][node_id] = sum(sum(self.links[link_id].cumulative_outflow[time_step]
                                                             for link_id in node.upstream_links)
                                                         for time_step in range(self.total_steps))
                if xi == num_scenario - 1:
                    plt.figure()
                    plt.plot([sum(self.links[link_id].cumulative_outflow[time_step] for link_id in node.upstream_links)
                              for time_step in range(self.total_steps)], "-", label="Departure")
                    plt.plot([sum(self.links[link_id].cumulative_inflow[time_step] for link_id in node.upstream_links)
                              for time_step in range(self.total_steps)], "-", label="Arrival")
                    plt.plot([sum(self.links[link_id].cumulative_freeflow[time_step] for link_id in node.upstream_links)
                              for time_step in range(self.total_steps)], "-", label="Free")
                    plt.xlabel("Time step")
                    plt.ylabel("Cumulative number of vehicles")
                    plt.legend()
                    plt.savefig(figure_folder + "node_" + node_id + "scenario_" + str(self.num_scenario) + ".jpg")
                    plt.close()
                    plt.figure()
                    plt.plot(
                        [sum(self.links[link_id].cumulative_inflow[time_step] - self.links[link_id].cumulative_outflow[
                            time_step] for link_id in node.upstream_links) for time_step in range(self.total_steps)],
                        "-", label="Queue")
                    plt.plot(
                        [sum(self.links[link_id].cumulative_freeflow[time_step] - self.links[link_id].cumulative_outflow
                        [time_step] for link_id in node.upstream_links) for time_step in range(self.total_steps)],
                        "-", label="Delay")
                    plt.xlabel("Time step")
                    plt.ylabel("Number of vehicles")
                    plt.legend()
                    plt.savefig(figure_folder + "delay_node_" + node_id + "scenario_" + str(self.num_scenario) + ".jpg")
                    plt.close()

                    print(node_id, file=out_put)
                    print("total travel time", total_travel_time[xi][node_id], file=out_put)
                    print("total travel delay", total_travel_delay[xi][node_id], file=out_put)
                    print("average travel time", average_travel_time[xi][node_id], file=out_put)
                    print("average travel delay", average_travel_delay[xi][node_id], file=out_put)
                    # print("total throughput", total_throughput[xi][node_id], file=out_put)
                    # print("cumulative throughput", cumulative_throughput[xi][node_id], file=out_put)

                    print("total arrival", total_arrival[xi][node_id], file=out_put)

                    print("total throughput", total_throughput[xi][node_id], file=out_put)
                    print("residual queue", total_arrival[xi][node_id] - total_throughput[xi][node_id], file=out_put)

                    print("cumulative throughput", cumulative_throughput[xi][node_id], file=out_put)

            if xi == num_scenario - 1:

                out_put = open(file_name, "a+")
                for cell_id, cell in self.cells.items():
                    if cell.type == "origin":
                        print(cell_id, cell.demand[xi][0], file=out_put)

            for time_step in range(self.total_steps):
                for cell_id in origin_cell_ids:
                    cell = self.cells[cell_id]
                    all_total_inflow[xi, time_step] += cell.demand[xi][time_step]
                for cell_id in destination_cell_ids:
                    cell = self.cells[cell_id]
                    all_total_outflow[xi, time_step] += cell.num_vehicles[time_step + 1] - cell.num_vehicles[time_step]
                if time_step > 0:
                    all_total_inflow[xi, time_step] += all_total_inflow[xi, time_step - 1]
                    all_total_outflow[xi, time_step] += all_total_outflow[xi, time_step - 1]
                # if time_step >= link.link_free_travel_time:
                #     link.cumulative_freeflow[time_step] = link.cumulative_inflow[
                #         time_step - link.link_free_travel_time]

            all_total_travel_time[xi] = sum(all_total_inflow[xi, time_step] - all_total_outflow[xi, time_step]
                                            for time_step in range(self.total_steps))
            # all_total_travel_delay[xi] = sum(total_travel_delay[xi][node_id] for node_id in self.intersections_id)
            all_average_travel_time[xi] = all_total_travel_time[xi] * 2 / max(
                all_total_inflow[xi, self.total_steps - 1] + all_total_outflow[xi, self.total_steps - 1], 0.001)
            all_total_delay_ew[xi] = sum(sum(
                self.links[link_id].cumulative_inflow[time_step] - self.links[link_id].cumulative_freeflow[time_step]
                for time_step in range(self.links[link_id].link_free_travel_time, self.total_steps))
                                         for link_id in link_ew_ids)
            all_total_delay_ns[xi] = sum(sum(
                self.links[link_id].cumulative_inflow[time_step] - self.links[link_id].cumulative_freeflow[time_step]
                for time_step in range(self.links[link_id].link_free_travel_time, self.total_steps))
                                         for link_id in link_ns_ids)
            # all_average_travel_delay[xi] = sum(average_travel_delay[xi][node_id] for node_id in self.intersections_id)
            all_total_throughput[xi] = all_total_outflow[xi, self.total_steps - 1]
            all_cumulative_throughput[xi] = sum(all_total_outflow[xi, t] for t in range(self.total_steps))
            all_objective[xi] = - sum(sum(self.cells[cell_id].num_vehicles[time_step]
                                          for time_step in range(self.total_steps))
                                      for cell_id in destination_cell_ids) - 0.0001 * sum(
                sum((self.total_steps - time_step) * (4 - connector.priority_class) * sum(connector.flow[time_step])
                    for time_step in range(self.total_steps))
                for connector in self.connectors.values())

            all_total_free_time[xi] = sum(link.link_free_travel_time *
                                          (link.cumulative_inflow[self.total_steps - 1] + link.cumulative_outflow
                                          [self.total_steps - 1]) / 2 for link in self.links.values())
            all_total_travel_delay[xi] = all_total_travel_time[xi] - all_total_free_time[xi]
            all_average_travel_delay[xi] = all_total_travel_delay[xi] / (all_total_inflow[xi, self.total_steps - 1] +
                                                                         all_total_outflow[
                                                                             xi, self.total_steps - 1]) * 2

            out_put = open(file_name, "a+")
            print("scenario", xi, file=out_put)
            print("total travel time", all_total_travel_time[xi], file=out_put)
            print("total travel delay", all_total_travel_delay[xi], file=out_put)
            print("average travel time", all_average_travel_time[xi], file=out_put)
            print("average travel delay", all_average_travel_delay[xi], file=out_put)
            print("total travel delay for E-W", all_total_delay_ew[xi], file=out_put)
            print("total travel delay for N-S", all_total_delay_ns[xi], file=out_put)
            # total_throughput[xi] = 0
            # cumulative_throughput[xi] = 0
            # for cell_id, cell in self.cells.items():
            #     if cell.type == "destination":
            #         total_throughput[xi] += cell.num_vehicles[self.total_steps]
            #         cumulative_throughput[xi] += sum(cell.num_vehicles)
            print("total arrival", all_total_inflow[xi, self.total_steps - 1], file=out_put)
            print("total throughput", all_total_throughput[xi], file=out_put)
            print("residual queue",
                  all_total_inflow[xi, self.total_steps - 1] - all_total_outflow[xi, self.total_steps - 1],
                  file=out_put)
            print("cumulative throughput", all_cumulative_throughput[xi], file=out_put)
            print("objective value", all_objective[xi], file=out_put)

        out_put = open(file_name, "a+")
        print("Average", file=out_put)
        print("total travel time", (sum(all_total_travel_time) / num_scenario), file=out_put)
        print("total travel delay", (sum(all_total_travel_delay) / num_scenario), file=out_put)
        print("average travel time", (sum(all_average_travel_time) / num_scenario), file=out_put)
        print("average travel delay", (sum(all_average_travel_delay) / num_scenario), file=out_put)
        print("total travel delay for E-W", (sum(all_total_delay_ew) / num_scenario), file=out_put)
        print("total travel delay for N-S", (sum(all_total_delay_ns) / num_scenario), file=out_put)
        print("total arrival", (sum(all_total_inflow[xi, self.total_steps - 1]
                                    for xi in range(num_scenario)) / num_scenario), file=out_put)
        print("total throughput", (sum(all_total_throughput) / num_scenario), file=out_put)
        print("residual queue", (sum(
            all_total_inflow[xi, self.total_steps - 1] - all_total_outflow[xi, self.total_steps - 1] for xi in
            range(num_scenario)) / num_scenario), file=out_put)
        print("cumulative throughput", (sum(all_cumulative_throughput) / num_scenario), file=out_put)
        print("objective value", (sum(all_objective) / num_scenario), file=out_put)
        print()

        # evaluate by intersection
        for node_id in self.intersections_id:
            print(node_id, file=out_put)
            print("total travel time",
                  (sum(total_travel_time[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("total travel delay",
                  (sum(total_travel_delay[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("average travel time",
                  (sum(average_travel_time[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("average travel delay",
                  (sum(average_travel_delay[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("total arrival", (sum(total_arrival[xi][node_id] for xi in range(num_scenario)) / num_scenario),
                  file=out_put)
            print("total throughput", (sum(total_throughput[xi][node_id] for xi in range(num_scenario)) / num_scenario),
                  file=out_put)
            print("residual queue", (sum(total_arrival[xi][node_id] - total_throughput[xi][node_id]
                                         for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("cumulative throughput", (sum(cumulative_throughput[xi][node_id]
                                                for xi in range(num_scenario)) / num_scenario), file=out_put)

    def warm_up(self, warm_up_time):
        for node_id, node in self.nodes.items():
            if node.type == "signalized":
                self.generate_naive_signal_plan(node_id)

        self.run(0)
        for cell_id, cell in self.cells.items():
            num_vehicles = cell.num_vehicles
            cell.num_vehicles = [num_vehicles[warm_up_time]]
            cell.warm_up_value = num_vehicles[0:warm_up_time + 1]
            occupancy = cell.occupation
            cell.warm_up_occ = occupancy[0:warm_up_time + 1]
        for connector_id, connector in self.connectors.items():
            flow = connector.flow
            connector.flow = [flow[warm_up_time]]
            connector.warm_up_flow = flow[0:warm_up_time + 1]

    def evaluate_warmup(self, network, file_name, figure_folder, num_scenario, warmup_step=100):
        """
        compute the cumulative inflow and cumulative outflow for evaluation
        """
        out_put = open(file_name, "w+")
        total_travel_time = [None] * num_scenario
        total_travel_delay = [None] * num_scenario
        average_travel_time = [None] * num_scenario
        average_travel_delay = [None] * num_scenario
        total_throughput = [None] * num_scenario
        total_arrival = [None] * num_scenario
        cumulative_throughput = [None] * num_scenario
        objective = [None] * num_scenario

        all_total_travel_time = np.zeros(num_scenario)
        all_total_travel_delay = np.zeros(num_scenario)
        all_total_delay_ew = np.zeros(num_scenario)
        all_total_delay_ns = np.zeros(num_scenario)
        all_average_travel_time = np.zeros(num_scenario)
        all_average_travel_delay = np.zeros(num_scenario)
        all_total_throughput = np.zeros(num_scenario)
        all_cumulative_throughput = np.zeros(num_scenario)
        all_objective = np.zeros(num_scenario)

        all_total_inflow = np.zeros((num_scenario, warmup_step + self.total_steps))
        all_total_outflow = np.zeros((num_scenario, warmup_step + self.total_steps))
        all_total_freeflow = np.zeros((num_scenario, warmup_step + self.total_steps))
        all_total_free_time = np.zeros(num_scenario)

        origin_cell_ids = []
        destination_cell_ids = []
        for cell_id, cell in self.cells.items():
            if cell.type == "origin":
                origin_cell_ids.append(cell_id)
            if cell.type == "destination":
                destination_cell_ids.append(cell_id)

        link_ew_ids = []
        link_ns_ids = []
        for link_id, link in self.links.items():
            if link.from_direction in ["E", "W"]:
                link_ew_ids.append(link_id)
            if link.from_direction in ["N", "S"]:
                link_ns_ids.append(link_id)

        for link_id, link in self.links.items():
            capacity = 0
            link.cumulative_inflow = np.zeros(self.total_steps)
            link.cumulative_outflow = np.zeros(self.total_steps)
            link.cumulative_freeflow = np.zeros(warmup_step + self.total_steps)
            link.link_free_travel_time = 0
            for segment_id in link.segment_list:
                segment = network.segments[segment_id]
                link.link_free_travel_time += self.lanesets[segment.laneset_list[0]].num_cells
                for laneset_id in segment.laneset_list:
                    laneset = self.lanesets[laneset_id]
                    for cell_id in laneset.cell_ids:
                        cell = self.cells[cell_id]
                        capacity += cell.maximum_veh
            link.link_max_vehicles = capacity

        all_num_vehicles = {}
        all_occupancy = {}
        min_num_vehicles = {}
        min_occupancy = {}
        max_num_vehicles = {}
        max_occupancy = {}
        min_delay = np.infty
        max_delay = 0
        max_arrival = 0
        min_arrival = np.infty
        for cell_id, cell in self.cells.items():
            all_occupancy[cell_id] = np.zeros(self.total_steps + len(cell.warm_up_value))
            all_num_vehicles[cell_id] = np.zeros(self.total_steps + len(cell.warm_up_value))
            min_occupancy[cell_id] = np.zeros(self.total_steps + len(cell.warm_up_value))
            min_num_vehicles[cell_id] = np.zeros(self.total_steps + len(cell.warm_up_value))
            max_occupancy[cell_id] = np.zeros(self.total_steps + len(cell.warm_up_value))
            max_num_vehicles[cell_id] = np.zeros(self.total_steps + len(cell.warm_up_value))
        for xi in tqdm(range(num_scenario)):
            self.reset_flow()
            for cell_id, cell in self.cells.items():
                cell.num_vehicles = [cell.warm_up_value[warmup_step]]
            self.run(xi)
            for cell_id, cell in self.cells.items():
                cell.num_vehicles = cell.warm_up_value[:-1] + cell.num_vehicles
            for connector_id, connector in self.connectors.items():
                connector.flow = connector.warm_up_flow[:-1] + connector.flow
            for link_id, link in self.links.items():
                link.cumulative_inflow = np.zeros(warmup_step + self.total_steps)
                link.cumulative_outflow = np.zeros(warmup_step + self.total_steps)
            for time_step in range(warmup_step + self.total_steps):
                for link_id, link in self.links.items():
                    segment = network.segments[link.segment_list[0]]
                    for laneset_id in segment.laneset_list:
                        laneset = self.lanesets[laneset_id]
                        cell = self.cells[laneset.cell_ids[0]]
                        if cell.type == "origin":
                            if time_step < warmup_step:
                                link.cumulative_inflow[time_step] += cell.mean_demand / (3600 / self.time_interval)
                            else:
                                link.cumulative_inflow[time_step] += cell.demand[xi][time_step - warmup_step]
                        else:
                            for upstream_connector_id in cell.upstream_connector:
                                upstream_connector = self.connectors[upstream_connector_id]
                                link.cumulative_inflow[time_step] += upstream_connector.flow[time_step][
                                    upstream_connector.downstream_cell.index(cell.cell_id)]
                    segment = network.segments[link.segment_list[-1]]
                    for laneset_id in segment.laneset_list:
                        laneset = self.lanesets[laneset_id]
                        cell = self.cells[laneset.cell_ids[laneset.num_cells - 1]]
                        if cell.type == "destination":
                            link.cumulative_outflow[time_step] += cell.num_vehicles[time_step + 1] - \
                                                                  cell.num_vehicles[time_step]
                        else:
                            for downstream_connector_id in cell.downstream_connector:
                                downstream_connector = self.connectors[downstream_connector_id]
                                link.cumulative_outflow[time_step] += sum(downstream_connector.flow[time_step])

                    if time_step > 0:
                        link.cumulative_inflow[time_step] += link.cumulative_inflow[time_step - 1]
                        link.cumulative_outflow[time_step] += link.cumulative_outflow[time_step - 1]
                    # if time_step >= link.link_free_travel_time:
                    link.cumulative_freeflow[time_step] = link.cumulative_inflow[max(
                        time_step - link.link_free_travel_time, 0)]

            total_travel_time[xi] = {}
            average_travel_time[xi] = {}
            total_travel_delay[xi] = {}
            average_travel_delay[xi] = {}
            total_throughput[xi] = {}
            cumulative_throughput[xi] = {}
            objective[xi] = {}
            total_travel_time[xi] = {}
            average_travel_time[xi] = {}
            total_travel_delay[xi] = {}
            average_travel_delay[xi] = {}
            total_throughput[xi] = {}
            cumulative_throughput[xi] = {}
            total_arrival[xi] = {}

            arrival_node = {}
            throughput_node = {}

            for node_id in self.intersections_id:
                node = self.nodes[node_id]
                # total_travel_time[xi][node_id] = sum(
                #     sum(self.links[link_id].cumulative_inflow[time_step] - self.links[link_id].cumulative_outflow[
                #         time_step] for link_id in node.upstream_links) for time_step in range(self.total_steps))
                total_travel_time[xi][node_id] = sum(self.links[link_id].cumulative_inflow[time_step]
                                                     - self.links[link_id].cumulative_outflow[time_step]
                                                     for link_id in node.upstream_links
                                                     for time_step in
                                                     range(warmup_step, self.total_steps + warmup_step))
                # fixme: update computation of average travel time
                average_travel_time[xi][node_id] = total_travel_time[xi][node_id] * 2 / max(sum(
                    self.links[link_id].cumulative_inflow[self.total_steps + warmup_step - 1] +
                    self.links[link_id].cumulative_outflow[self.total_steps + warmup_step - 1] -
                    self.links[link_id].cumulative_inflow[warmup_step] -
                    self.links[link_id].cumulative_outflow[warmup_step]
                    for link_id in node.upstream_links), 0.001)
                total_travel_delay[xi][node_id] = sum(
                    sum(self.links[link_id].cumulative_inflow[time_step - self.links[link_id].link_free_travel_time] -
                        self.links[link_id].cumulative_outflow[time_step] for time_step in range(
                        self.links[link_id].link_free_travel_time + warmup_step, self.total_steps + warmup_step))
                    for link_id in node.upstream_links)
                # fixme: update computation of average travel delay
                average_travel_delay[xi][node_id] = total_travel_delay[xi][node_id] * 2 / max(sum(
                    self.links[link_id].cumulative_inflow[self.total_steps + warmup_step - 1] +
                    self.links[link_id].cumulative_outflow[self.total_steps + warmup_step - 1] -
                    self.links[link_id].cumulative_inflow[warmup_step] -
                    self.links[link_id].cumulative_outflow[warmup_step]
                    for link_id in node.upstream_links), 0.001)
                # cumulative_throughput[xi][node_id] = sum(sum(self.cells[cell_id].num_vehicles[time_step]
                #                                              for time_step in range(warmup_step, self.total_steps))
                #                                          for cell_id in node.destination_cell_list)
                # total_throughput[xi][node_id] = sum(self.cells[cell_id].num_vehicles[warmup_step + self.total_steps - 1]
                #                                     - self.cells[cell_id].num_vehicles[warmup_step]
                #                                     for cell_id in node.destination_cell_list)
                total_arrival[xi][node_id] = sum(
                    self.links[link_id].cumulative_inflow[warmup_step + self.total_steps - 1]
                    - self.links[link_id].cumulative_outflow[warmup_step]
                    for link_id in node.upstream_links)
                total_throughput[xi][node_id] = sum(
                    self.links[link_id].cumulative_outflow[warmup_step + self.total_steps - 1]
                    - self.links[link_id].cumulative_outflow[warmup_step]
                    for link_id in node.upstream_links)
                cumulative_throughput[xi][node_id] = sum(
                    sum(self.links[link_id].cumulative_outflow[time_step] for link_id in node.upstream_links)
                    for time_step in range(warmup_step, warmup_step + self.total_steps))
                objective[xi][node_id] = - sum(sum(self.cells[cell_id].num_vehicles[time_step]
                                                   for time_step in range(self.total_steps))
                                               for cell_id in destination_cell_ids) \
                                         - 0.0001 * sum(sum((self.total_steps - time_step) * (
                        4 - self.connectors[connector_id].priority_class) *
                                                            sum(self.connectors[connector_id].flow[time_step])
                                                            for time_step in
                                                            range(warmup_step, warmup_step + self.total_steps))
                                                        for connector_id in node.connectors_list)
                if xi == num_scenario - 1:
                    if figure_folder is not None:
                        plt.figure()
                        plt.plot([sum(self.links[link_id].cumulative_outflow[time_step] for link_id in node.upstream_links)
                                  for time_step in range(warmup_step + self.total_steps)], "-",
                                 label="Departure")
                        plt.plot([sum(self.links[link_id].cumulative_inflow[time_step] for link_id in node.upstream_links)
                                  for time_step in range(warmup_step + self.total_steps)], "-",
                                 label="Arrival")
                        plt.plot([sum(self.links[link_id].cumulative_freeflow[time_step] for link_id in node.upstream_links)
                                  for time_step in range(warmup_step + self.total_steps)], "-", label="Free")
                        plt.xlabel("Time step")
                        plt.ylabel("Cumulative number of vehicles")
                        plt.legend()
                        plt.savefig(figure_folder + "node_" + node_id + "scenario_" + str(self.num_scenario) + ".jpg")
                        plt.close()
                        plt.figure()
                        plt.plot(
                            [sum(self.links[link_id].cumulative_inflow[time_step] - self.links[link_id].cumulative_outflow[
                                time_step] for link_id in node.upstream_links)
                             for time_step in range(warmup_step + self.total_steps)], "-", label="Queue")
                        plt.plot(
                            [sum(self.links[link_id].cumulative_freeflow[time_step] - self.links[link_id].cumulative_outflow
                            [time_step] for link_id in node.upstream_links)
                             for time_step in range(warmup_step + self.total_steps)], "-", label="Delay")
                        plt.xlabel("Time step")
                        plt.ylabel("Number of vehicles")
                        plt.legend()
                        plt.savefig(figure_folder + "delay_node_" + node_id + "scenario_" + str(self.num_scenario) + ".jpg")
                        plt.close()

                    print(node_id, file=out_put)
                    print("total travel time", total_travel_time[xi][node_id], file=out_put)
                    print("total travel delay", total_travel_delay[xi][node_id], file=out_put)
                    print("average travel time", average_travel_time[xi][node_id], file=out_put)
                    print("average travel delay", average_travel_delay[xi][node_id], file=out_put)
                    # print("total throughput", total_throughput[xi][node_id], file=out_put)
                    # print("cumulative throughput", cumulative_throughput[xi][node_id], file=out_put)

                    print("total arrival", total_arrival[xi][node_id], file=out_put)

                    print("total throughput", total_throughput[xi][node_id], file=out_put)
                    print("residual queue", total_arrival[xi][node_id] - total_throughput[xi][node_id], file=out_put)

                    print("cumulative throughput", cumulative_throughput[xi][node_id], file=out_put)

            if xi == num_scenario - 1:

                out_put = open(file_name, "a+")
                for cell_id, cell in self.cells.items():
                    if cell.type == "origin":
                        print(cell_id, cell.demand[xi][0], file=out_put)

            for time_step in range(warmup_step + self.total_steps):
                for cell_id in origin_cell_ids:
                    cell = self.cells[cell_id]
                    if time_step >= warmup_step:
                        all_total_inflow[xi, time_step] += cell.demand[xi][time_step - warmup_step]
                    else:
                        all_total_inflow[xi, time_step] += cell.mean_demand / (3600 / self.time_interval)
                for cell_id in destination_cell_ids:
                    cell = self.cells[cell_id]
                    all_total_outflow[xi, time_step] += cell.num_vehicles[time_step + 1] - cell.num_vehicles[time_step]
                if time_step > 0:
                    all_total_inflow[xi, time_step] += all_total_inflow[xi, time_step - 1]
                    all_total_outflow[xi, time_step] += all_total_outflow[xi, time_step - 1]
                # if time_step >= link.link_free_travel_time:
                #     link.cumulative_freeflow[time_step] = link.cumulative_inflow[
                #         time_step - link.link_free_travel_time]

            all_total_travel_time[xi] = sum(all_total_inflow[xi, time_step] - all_total_outflow[xi, time_step]
                                            for time_step in range(warmup_step, warmup_step + self.total_steps))
            # all_total_travel_delay[xi] = sum(total_travel_delay[xi][node_id] for node_id in self.intersections_id)
            all_average_travel_time[xi] = all_total_travel_time[xi] * 2 / max(
                all_total_inflow[xi, warmup_step + self.total_steps - 1]
                + all_total_outflow[xi, warmup_step + self.total_steps - 1]
                - all_total_inflow[xi, warmup_step] - all_total_outflow[xi, warmup_step], 0.001)
            all_total_delay_ew[xi] = sum(sum(
                self.links[link_id].cumulative_freeflow[time_step] - self.links[link_id].cumulative_outflow[time_step]
                for time_step in range(
                    self.links[link_id].link_free_travel_time + warmup_step, warmup_step + self.total_steps))
                                         for link_id in link_ew_ids)
            all_total_delay_ns[xi] = sum(sum(
                self.links[link_id].cumulative_freeflow[time_step] - self.links[link_id].cumulative_outflow[time_step]
                for time_step in range(
                    self.links[link_id].link_free_travel_time + warmup_step, warmup_step + self.total_steps))
                                         for link_id in link_ns_ids)
            # all_average_travel_delay[xi] = sum(average_travel_delay[xi][node_id] for node_id in self.intersections_id)
            all_total_throughput[xi] = all_total_outflow[xi, warmup_step + self.total_steps - 1] \
                                       - all_total_outflow[xi, warmup_step]
            all_cumulative_throughput[xi] = sum(all_total_outflow[xi, t]
                                                for t in range(warmup_step, warmup_step + self.total_steps))
            all_objective[xi] = - sum(sum(self.cells[cell_id].num_vehicles[time_step]
                                          for time_step in range(warmup_step, warmup_step + self.total_steps))
                                      for cell_id in destination_cell_ids) - 0.0001 * sum(
                sum((self.total_steps - time_step) * (4 - connector.priority_class) * sum(connector.flow[time_step])
                    for time_step in range(warmup_step, warmup_step + self.total_steps))
                for connector in self.connectors.values())

            all_total_free_time[xi] = sum(link.link_free_travel_time *
                                          (link.cumulative_inflow[self.total_steps + warmup_step - 1] +
                                           link.cumulative_outflow[self.total_steps + warmup_step - 1] -
                                           link.cumulative_inflow[warmup_step] - link.cumulative_outflow[
                                               warmup_step]) / 2
                                          for link in self.links.values())
            all_total_travel_delay[xi] = all_total_travel_time[xi] - all_total_free_time[xi]
            all_average_travel_delay[xi] = all_total_travel_delay[xi] * 2 / max(
                all_total_inflow[xi, warmup_step + self.total_steps - 1]
                + all_total_outflow[xi, warmup_step + self.total_steps - 1]
                - all_total_inflow[xi, warmup_step] - all_total_outflow[xi, warmup_step], 0.001)

            out_put = open(file_name, "a+")
            print("scenario", xi, file=out_put)
            print("total travel time", all_total_travel_time[xi], file=out_put)
            print("total travel delay", all_total_travel_delay[xi], file=out_put)
            print("average travel time", all_average_travel_time[xi], file=out_put)
            print("average travel delay", all_average_travel_delay[xi], file=out_put)
            print("total travel delay for E-W", all_total_delay_ew[xi], file=out_put)
            print("total travel delay for N-S", all_total_delay_ns[xi], file=out_put)
            # total_throughput[xi] = 0
            # cumulative_throughput[xi] = 0
            # for cell_id, cell in self.cells.items():
            #     if cell.type == "destination":
            #         total_throughput[xi] += cell.num_vehicles[self.total_steps]
            #         cumulative_throughput[xi] += sum(cell.num_vehicles)
            print("total arrival", all_total_inflow[xi, warmup_step + self.total_steps - 1]
                  - all_total_outflow[xi, warmup_step], file=out_put)
            print("total throughput", all_total_throughput[xi], file=out_put)
            print("residual queue", all_total_inflow[xi, warmup_step + self.total_steps - 1]
                  - all_total_outflow[xi, warmup_step + self.total_steps - 1],
                  file=out_put)
            print("cumulative throughput", all_cumulative_throughput[xi], file=out_put)
            print("objective value", all_objective[xi], file=out_put)
            print("CTM term", all_objective[xi] + all_cumulative_throughput[xi], file=out_put)

            arrival = all_total_inflow[xi, warmup_step + self.total_steps - 1] - all_total_outflow[xi, warmup_step]

            # if min_delay <= all_average_travel_delay[xi] <= max_delay:
            #     for cell_id, cell in self.cells.items():
            #         all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         all_num_vehicles[cell_id] += np.array(cell.num_vehicles)
            #
            # if all_average_travel_delay[xi] < min_delay and all_average_travel_delay[xi] <= max_delay:
            #     for cell_id, cell in self.cells.items():
            #         all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         all_num_vehicles[cell_id] += np.array(cell.num_vehicles)
            #         min_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         min_num_vehicles[cell_id] = np.array(cell.num_vehicles)
            #         min_delay = all_average_travel_delay[xi]
            #
            # if all_average_travel_delay[xi] > max_delay and all_average_travel_delay[xi] >= min_delay:
            #     for cell_id, cell in self.cells.items():
            #         all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         all_num_vehicles[cell_id] += np.array(cell.num_vehicles)
            #         max_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         max_num_vehicles[cell_id] = np.array(cell.num_vehicles)
            #         max_delay = all_average_travel_delay[xi]
            #
            # if min_delay > all_average_travel_delay[xi] > max_delay:
            #     for cell_id, cell in self.cells.items():
            #         all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         all_num_vehicles[cell_id] += np.array(cell.num_vehicles)
            #         min_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         min_num_vehicles[cell_id] = np.array(cell.num_vehicles)
            #         max_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
            #         max_num_vehicles[cell_id] = np.array(cell.num_vehicles)
            #         min_delay = all_average_travel_delay[xi]
            #         max_delay = all_average_travel_delay[xi]

            if min_arrival <= arrival <= max_arrival:
                for cell_id, cell in self.cells.items():
                    all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    all_num_vehicles[cell_id] += np.array(cell.num_vehicles)

            if arrival < min_arrival and arrival <= max_arrival:
                for cell_id, cell in self.cells.items():
                    all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    all_num_vehicles[cell_id] += np.array(cell.num_vehicles)
                    min_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    min_num_vehicles[cell_id] = np.array(cell.num_vehicles)
                    min_arrival = arrival

            if arrival > max_arrival and arrival >= min_arrival:
                for cell_id, cell in self.cells.items():
                    all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    all_num_vehicles[cell_id] += np.array(cell.num_vehicles)
                    max_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    max_num_vehicles[cell_id] = np.array(cell.num_vehicles)
                    max_arrival = arrival

            if min_arrival > arrival > max_arrival:
                for cell_id, cell in self.cells.items():
                    all_occupancy[cell_id] += np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    all_num_vehicles[cell_id] += np.array(cell.num_vehicles)
                    min_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    min_num_vehicles[cell_id] = np.array(cell.num_vehicles)
                    max_occupancy[cell_id] = np.array(cell.warm_up_occ[:-1] + cell.occupation)
                    max_num_vehicles[cell_id] = np.array(cell.num_vehicles)
                    min_arrival = arrival
                    max_arrival = arrival

        out_put = open(file_name, "a+")
        print("Average", file=out_put)
        print("total travel time", (sum(all_total_travel_time) / num_scenario), file=out_put)
        print("total travel delay", (sum(all_total_travel_delay) / num_scenario), file=out_put)
        print("average travel time", (sum(all_average_travel_time) / num_scenario), file=out_put)
        print("average travel delay", (sum(all_average_travel_delay) / num_scenario), file=out_put)
        print("total travel delay for E-W", (sum(all_total_delay_ew) / num_scenario), file=out_put)
        print("total travel delay for N-S", (sum(all_total_delay_ns) / num_scenario), file=out_put)
        print("total arrival", (sum(all_total_inflow[xi, warmup_step + self.total_steps - 1]
                                    - all_total_outflow[xi, warmup_step] for xi in range(num_scenario)) / num_scenario),
              file=out_put)
        print("total throughput", (sum(all_total_throughput) / num_scenario), file=out_put)
        print("residual queue", (sum(
            all_total_inflow[xi, warmup_step + self.total_steps - 1]
            - all_total_outflow[xi, warmup_step + self.total_steps - 1] for xi in range(num_scenario)) / num_scenario),
              file=out_put)
        print("cumulative throughput", (sum(all_cumulative_throughput) / num_scenario), file=out_put)
        print("objective value", (sum(all_objective) / num_scenario), file=out_put)
        print("CTM term", (sum(all_cumulative_throughput) + sum(all_objective)) / num_scenario, file=out_put)
        # print("best and worst delay", min_delay, max_delay, file=out_put)
        print("best and worst scenario", min_arrival, max_arrival, file=out_put)
        print()

        # evaluate by intersection
        for node_id in self.intersections_id:
            print(node_id, file=out_put)
            print("total travel time",
                  (sum(total_travel_time[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("total travel delay",
                  (sum(total_travel_delay[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("average travel time",
                  (sum(average_travel_time[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("average travel delay",
                  (sum(average_travel_delay[xi][node_id] for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("total arrival", (sum(total_arrival[xi][node_id] for xi in range(num_scenario)) / num_scenario),
                  file=out_put)
            print("total throughput", (sum(total_throughput[xi][node_id] for xi in range(num_scenario)) / num_scenario),
                  file=out_put)
            print("residual queue", (sum(total_arrival[xi][node_id] - total_throughput[xi][node_id]
                                         for xi in range(num_scenario)) / num_scenario), file=out_put)
            print("cumulative throughput", (sum(cumulative_throughput[xi][node_id]
                                                for xi in range(num_scenario)) / num_scenario), file=out_put)
        return all_num_vehicles, all_occupancy, min_num_vehicles, min_occupancy, min_arrival, max_num_vehicles, max_occupancy, max_arrival

    def __len__(self):
        """
        return cell number
        :return:
        """
        return self.num_cells
