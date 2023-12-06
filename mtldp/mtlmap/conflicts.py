import numpy as np
from . import nodes_classes as nd
from ..tools import logger


class ConflictPoint(object):
    def __init__(self):
        self.conflict_id = None
        self.belong_node = None
        self.conflict_connectors = []

        # equivalent lane number: 1800 veh / (lane*sec)
        # Therefore, you need to * 0.5 * time_step to get the veh/s
        self.capacity = None

        # type of the priority: "fixed", "mixed"
        # "mixed" priority for the stop sign
        self.priority = None


def build_mixed_conflict(network, node_id, segment_list, direction_info):
    conflict_point = ConflictPoint()
    conflict_point.priority = "mixed"
    conflict_point.belong_node = node_id
    conflict_point.conflict_id = node_id + "_stopSign" + "_" + direction_info

    connector_details = []
    lane_number_list = [1]
    for up_seg in segment_list:
        segment = network.segments[up_seg]
        laneset_list = segment.laneset_list
        lane_number_list.append(segment.lane_number)
        connector_list = []
        for laneset_id in laneset_list:
            laneset = network.lanesets[laneset_id]
            downstream_connector = laneset.downstream_connector
            connector_list.append(downstream_connector)

            # fetch the connector
            connector = network.connectors[downstream_connector]
            connector.conflict_points.append(conflict_point.conflict_id)
            connector.priority_class = -1
        connector_details.append(connector_list)

    conflict_point.capacity = np.average(lane_number_list) * 2
    conflict_point.conflict_connectors = connector_details
    network.add_conflict_point(conflict_point)


def build_fixed_conflict(network, node_id, laneset_list, direction_str):
    conflict_point = ConflictPoint()
    conflict_point.priority = "fixed"
    conflict_point.belong_node = node_id
    conflict_point.conflict_id = node_id + "_left_" + direction_str

    # fixme: negative left capacity
    conflict_point.capacity = 1

    overall_conflict_connectors = []
    for idx in range(len(laneset_list)):
        connector_list = []
        for laneset_id in laneset_list[idx]:
            laneset = network.lanesets[laneset_id]
            connector_list.append(laneset.downstream_connector)

            # fetch the connector
            connector = network.connectors[laneset.downstream_connector]
            connector.priority_class = idx
            connector.conflict_points.append(conflict_point.conflict_id)
        overall_conflict_connectors.append(connector_list)
    conflict_point.conflict_connectors = overall_conflict_connectors
    network.add_conflict_point(conflict_point)


def build_node_conflict(network, node_id, direction="all"):
    node = network.nodes[node_id]
    segment_list = node.upstream_segments

    if direction == "all":
        network.build_mixed_conflict(node_id, segment_list, direction)
    else:
        segment_list = []
        for segment_id in node.upstream_segments:
            segment = network.segments[segment_id]
            if segment.from_direction in direction:
                segment_list.append(segment_id)
        if len(segment_list) <= 1:
            pass
        elif len(segment_list) == 2:
            mixed_number = 0
            # segment
            for sdx in range(2):
                segment = network.segments[segment_list[sdx]]
                lanesets = segment.laneset_list
                if len(lanesets) < 2:
                    mixed_number += 1
                else:
                    # get the dedicated left turn segment
                    left_turn_laneset_id = None
                    for laneset_id in lanesets:
                        laneset = network.lanesets[laneset_id]
                        if laneset.turning_direction == "l":
                            left_turn_laneset_id = laneset_id
                            break
                    if left_turn_laneset_id is None:
                        pass
                    else:
                        # start to build the conflict points
                        # # get the inverse direction segment
                        if sdx == 0:
                            segment = network.segments[segment_list[1]]
                        else:
                            segment = network.segments[segment_list[0]]

                        # get all the non-dedicated left turn
                        for laneset_id in segment.laneset_list:
                            laneset = network.lanesets[laneset_id]
                            if laneset.turning_direction != "l":
                                build_fixed_conflict(network, node_id,
                                                     [[laneset_id], [left_turn_laneset_id]],
                                                     direction[sdx])
            if mixed_number == 2:
                build_mixed_conflict(network, node_id, segment_list, direction)
        else:
            logger.map_logger.warning("conflict points of " + direction + " has more than two segments")
