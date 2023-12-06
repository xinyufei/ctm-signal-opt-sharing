"""
This file is to build the connectors of the network
"""
from ..tools import logger


class Connector(object):
    """
    Connector corresponds to a upstream laneset and downstream laneset(s)
    only diverge exists, do not record converge
    """
    def __init__(self):
        self.connector_id = None
        self.upstream_laneset = None
        self.downstream_lanesets = []

        # type of the connector: "signalized", "unsignalized", "ordinary", "origin", "destination"
        # fixme: I am not sure
        self.type = None

        # # priority class: 0 > 1 > 2 > ...
        self.priority_class = 0
        # todo: this is not added yet
        self.conflict_points = []

        self.upstream_segment = None
        self.downstream_segments = []

        self.diverge_proportion = []
        self.belonged_node = None

        self.direction = ""
        self.upstream_origin = False
        self.downstream_destination = False

        self.controlled_node = None
        self.phase_id = None


def generate_network_connectors(network):
    """
    build all connectors

    :param network:
    :return:
    """
    for node_id, node in network.nodes.items():
        if node.is_intersection():
            upstream_segments = node.upstream_segments
            for up_seg in upstream_segments:
                segment = network.segments[up_seg]
                downstream_dirs = segment.downstream_directions_info

                lanesets_list = segment.laneset_list
                for laneset_id in lanesets_list:
                    laneset = network.lanesets[laneset_id]

                    directions = laneset.turning_direction

                    direction_list = []
                    segment_list = []
                    for direction in directions:
                        if not (direction in downstream_dirs.keys()):
                            logger.map_logger.warning("downstream_dirs does not have direction " + direction +
                                                      " in segment " + segment.segment_id)
                            continue

                        if downstream_dirs[direction] is None:
                            logger.map_logger.warning("downstream direction " + direction +
                                                      " not found of laneset " + laneset_id)
                        else:
                            direction_list.append(direction)
                            segment_list.append(downstream_dirs[direction])
                    build_connector(network, laneset_id, segment_list, direction_list, node.type)
        elif node.type == "connector":
            if len(node.upstream_segments) == 1:
                # oneway scenario
                build_segment_siso_connector(network, node.upstream_segments[0], node.downstream_segments[0])
            elif len(node.upstream_segments) == 2:
                upstream_segments = node.upstream_segments
                up_seg_way_id = []
                for up_seg in upstream_segments:
                    segment = network.segments[up_seg]
                    up_seg_way_id.append(segment.osm_way_id)

                downstream_segments = node.downstream_segments
                down_seg_way_id = []
                for down_seg in downstream_segments:
                    segment = network.segments[down_seg]
                    down_seg_way_id.append(segment.osm_way_id)

                if len(up_seg_way_id) != 2:
                    logger.map_logger.error("# of upstream segments of segment connector node "
                                            + node_id + "does not equal 2")
                    continue

                if len(down_seg_way_id) != 2:
                    logger.map_logger.error("# of downstream segments of segment connector node "
                                            + node_id + " does not equal 2")
                    continue

                indicator_list = [up_seg_way_id[ii] == down_seg_way_id[jj]
                                  for ii in range(2) for jj in range(2)]

                if sum(indicator_list) != 2:
                    logger.map_logger.error("indicator list not correct")

                if indicator_list[0]:
                    build_segment_siso_connector(network, upstream_segments[0], downstream_segments[1])
                    build_segment_siso_connector(network, upstream_segments[1], downstream_segments[0])
                else:
                    build_segment_siso_connector(network, upstream_segments[0], downstream_segments[1])
                    build_segment_siso_connector(network, upstream_segments[1], downstream_segments[0])
            else:
                logger.map_logger.error("ordinary node has more than 2 in segments ")
        elif node.type == "end":
            upstream_segments = node.upstream_segments
            downstream_segments = node.downstream_segments
            build_destination_connector(network, upstream_segments)
            build_origin_connector(network, downstream_segments)
    return network


def build_connector(network, in_laneset, downstream_segments, direction_list, c_type="ordinary"):
    laneset = network.lanesets[in_laneset]

    upstream_segment = network.segments[laneset.belonged_segment]
    connector = Connector()
    connector.connector_id = laneset.laneset_id
    laneset.downstream_connector = connector.connector_id
    connector.upstream_laneset = laneset.laneset_id
    connector.upstream_segment = laneset.belonged_segment
    connector.belonged_node = laneset.downstream_node

    connector.type = c_type
    network.nodes[laneset.downstream_node].add_connector(connector.connector_id)
    upstream_segment.downstream_connectors.append(connector.connector_id)

    for idx in range(len(downstream_segments)):
        segment_id = downstream_segments[idx]
        segment = network.segments[segment_id]

        # add downstream segments to the upstream segment
        # upstream_segment.add_downstream_segment(segment_id)
        # segment.add_upstream_segment(laneset.belonged_segment)

        # add downstream info to connector
        connector.downstream_segments.append(segment_id)
        connector.downstream_lanesets += segment.laneset_list

        # add upstream connector to the laneset
        for laneset_id in segment.laneset_list:
            network.lanesets[laneset_id].upstream_connectors.append(connector.connector_id)

        connector.direction += direction_list[idx]

        # add connector to segment
        segment.upstream_connectors.append(connector.connector_id)
    laneset.downstream_lanesets = connector.downstream_lanesets
    network.add_connector(connector)


def build_destination_connector(network, upstream_segments):
    """

    @param network:
    @param upstream_segments:
    """
    for segment_id in upstream_segments:
        segment = network.segments[segment_id]
        for laneset_id in segment.laneset_list:
            laneset = network.lanesets[laneset_id]

            connector = Connector()
            connector.connector_id = laneset_id + ">>destination"
            connector.upstream_segment = laneset.belonged_segment
            connector.direction = "d"
            connector.type = "destination"
            connector.downstream_destination = True
            connector.belonged_node = laneset.downstream_node

            network.nodes[laneset.downstream_node].add_connector(connector.connector_id)
            connector.upstream_laneset = laneset.laneset_id
            laneset.downstream_connector = connector.connector_id
            network.add_connector(connector)


def build_origin_connector(network, downstream_segments):
    for segment_id in downstream_segments:
        segment = network.segments[segment_id]
        for laneset_id in segment.laneset_list:
            laneset = network.lanesets[laneset_id]

            connector = Connector()
            connector.connector_id = laneset_id + "<<origin"
            connector.upstream_segment = "origin"
            connector.direction = "o"
            connector.upstream_origin = True
            connector.type = "origin"
            connector.belonged_node = laneset.upstream_node

            network.nodes[laneset.upstream_node].add_connector(connector.connector_id)
            connector.upstream_laneset = laneset.laneset_id
            laneset.upstream_connectors.append(connector.connector_id)
            network.add_connector(connector)


def build_segment_siso_connector(network, in_segment_id, out_segment_id):
    laneset_list = network.segments[in_segment_id].laneset_list
    for laneset_id in laneset_list:
        build_connector(network, laneset_id, [out_segment_id], ["s"])