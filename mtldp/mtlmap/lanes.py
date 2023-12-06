import json
import numpy as np

from ..tools import constants
from ..tools import logger
from ..tools.gps_utils import shift_geometry, get_closest_angle


class LaneSet(object):
    """
    LaneSet is a set of lanes that has the same downstream direction (e.g. through movement, left turn, right turn).
     It can be used to build CTM or LTM model.

    **Main attributes**
        - ``.laneset_id`` a integer for laneset ID. The lane set index within the segment is added to the segment ID.
          For example, if the segment ID is 1768090910, and the laneset index is 0, this value should be "1768090910_0"
        - ``.type`` type of the road. Could be "internal", "source", or "destination"
        - ``.belonged_segment`` the segment ID that this lane set belongs to
        - ``.belonged_link`` the link ID that this lane set belongs to
        - ``.turning_direction`` the movement of this lane set. For example, 's' means through movement (straight).
          'r' means right turn movement, and 'l' means left turn movement. The value can also be the combination of
          's', 'r' and 'l'
        - ``.length`` length of the lane set in meters
        - ``.speed_limit`` speed limit of the lane set in m/s
        - ``.lane_number`` number of lanes of the lane sets
        - ``.heading`` the heading angle of the lane sets (range: (-180,180]))
        - ``.from_direction`` the direction from which the lane set originates. For example, if the lane set originates
          from south, this value is "S".
        - ``.geometry`` the GPS coordinates along the lane set
        - ``.downstream_lanesets`` the downstream lane sets that it connects to
        - ``.turning_ratio_list`` the list of turning ratio information. The value is None if unavailable
        - ``.upstream_node`` the upstream node of the lane set
        - ``.downstream_node`` the downstream node of the lane set
        - ``.phase_id`` the ID of the phase associated with the lane set

    """

    def __init__(self):
        self.laneset_id = None
        self.type = None  # "internal", "source", "destination"

        # segment id of this pipeline
        self.belonged_segment = None
        self.belonged_link = None

        self.turning_direction = None  # "l", "r", "s", or their combination
        self.length = None  # unit: meters
        self.speed_limit = None  # unit: meters / sec
        self.lane_number = None

        self.heading = None
        self.from_direction = None

        self.downstream_connector = None
        self.upstream_connectors = []
        self.geometry = None

        # offset: 0 - through   1 - left   -1  -  right
        self.insegment_offset = None  # fixme

        self.downstream_lanesets = []  # downstream pipeline id list
        self.turning_ratio_list = None  # turning ratio list

        self.upstream_node = None
        self.downstream_node = None

        # traffic signal attributes
        self.phase_id = None
        self.movement_list = []  # fixme: what is the difference between movement_list and turning_direction

        self.travel_time = 0
        self.free_travel_time = 0
        self.capacity = 0
        self.belonged_path = []
        self.laneset_flow = 0

        # user equilibrium
        self.laneset_previous_flow = 0

    @classmethod
    def init_from_segment(cls, segment, direction, lane_number, insegment_offset):
        """

        :param segment:
        :param direction:
        :param lane_number:
        :param insegment_offset:
        :return:
        """
        laneset = cls()
        laneset.lane_number = lane_number
        laneset.turning_direction = direction
        laneset.laneset_id = segment.segment_id + "_" + str(insegment_offset)
        laneset.belonged_segment = segment.segment_id
        laneset.belonged_link = segment.belonged_link
        laneset.speed_limit = segment.speed_limit
        laneset.upstream_node = segment.upstream_node
        laneset.from_direction = segment.from_direction
        laneset.heading = segment.heading
        laneset.downstream_node = segment.downstream_node

        if insegment_offset > 0:
            laneset.geometry = shift_geometry(segment.geometry,
                                              shift_distance=constants.DISPLAY_LANE_INTERVAL,
                                              shift_direction="left")
        elif insegment_offset == 0:
            laneset.geometry = segment.geometry
        else:
            laneset.geometry = shift_geometry(segment.geometry,
                                              shift_distance=constants.DISPLAY_LANE_INTERVAL,
                                              shift_direction="right")
        laneset.length = segment.length
        laneset.insegment_offset = insegment_offset
        laneset.free_travel_time = laneset.length / laneset.speed_limit
        laneset.capacity = laneset.lane_number * 1800
        return laneset

    def compute_travel_time(self):
        """
        compute the travel time according to the link performance function
        :return:
        """
        self.travel_time = self.free_travel_time * (1 + 0.15 * pow(self.laneset_flow / self.capacity, 4))

    def compute_cumulative_travel_time(self, alpha):
        """
        objective function of the BPR link performance function

        :param alpha:
        :return:
        """
        current_flow = self.laneset_previous_flow + alpha * (self.laneset_flow - self.laneset_previous_flow)
        objective_function = self.free_travel_time * current_flow + 0.15 / 5 * self.free_travel_time * \
                             pow(current_flow / self.capacity, 5) * self.capacity
        return objective_function

    def update_current_flow(self, flow):
        self.laneset_flow = flow

    def update_previous_flow(self):
        self.laneset_previous_flow = self.laneset_flow

    def update_belonged_paths(self, path_id):
        if path_id not in self.belonged_path:
            self.belonged_path.append(path_id)

    def __str__(self):
        pass


def generate_network_lanesets(network):
    """

    :param network:
    :return:
    """
    for node_id, node in network.nodes.items():
        if node.type == "ordinary":
            continue
        elif node.type == "connector" or node.type == "end":
            upstream_segments = node.upstream_segments
            for upstream_seg in upstream_segments:
                segment = network.segments[upstream_seg]
                lanesets = segment.generate_lanesets()
                for laneset in lanesets:
                    network.add_laneset(laneset)

        elif node.is_intersection():
            upstream_segments = node.upstream_segments
            for up_seg in upstream_segments:
                segment = network.segments[up_seg]
                downstream_directed_segments = segment.downstream_directions_info
                overall_directions = ""
                for dir_k, dir_v in downstream_directed_segments.items():
                    if dir_v is not None:
                        overall_directions += dir_k
                segment_lane_number = int(segment.lane_number)
                if segment.lane_assignment == "null":
                    assignments = None
                    if segment_lane_number == 0:
                        logger.map_logger.warning("segment " + segment.segment_id + " lane number equals 0.")
                    elif segment_lane_number == 1:
                        assignments = [{"dir": overall_directions, "num": segment_lane_number, "shift": 0}]
                    else:
                        if "s" in overall_directions:
                            left_lanes = segment_lane_number
                            assignments = []
                            if "l" in overall_directions:
                                assignments.append({"dir": "l", "num": 1, "shift": 1})
                                left_lanes -= 1

                            if "r" in overall_directions:
                                if left_lanes > 2:
                                    assignments.append({"dir": "r", "num": 1, "shift": -1})
                                    assignments.append({"dir": "s", "num": left_lanes - 1, "shift": 0})
                                else:
                                    assignments.append({"dir": "rs", "num": left_lanes, "shift": 0})
                            else:
                                assignments.append({"dir": "s", "num": left_lanes, "shift": 0})
                        else:
                            if len(overall_directions) == 1:
                                assignments = [{"dir": overall_directions, "num": segment_lane_number, "shift": 0}]
                            else:
                                left_lanes = int(np.ceil(segment_lane_number / 2))
                                assignments = [{"dir": "l", "num": left_lanes, "shift": 1},
                                               {"dir": "r", "num": segment_lane_number - left_lanes, "shift": 0}]
                    lanesets = segment.generate_lanesets(assignments)
                    for laneset in lanesets:
                        network.add_laneset(laneset)
                else:
                    if "left" in segment.lane_assignment:
                        if not ("l" in overall_directions):
                            logger.map_logger.error("left downstream of segment " +
                                                    segment.segment_id + " segment not detected. ")
                    if "right" in segment.lane_assignment:
                        if not ("r" in overall_directions):
                            logger.map_logger.error("right downstream of segment " +
                                                    segment.segment_id + " segment not detected. ")
                            logger.map_logger.error("lane assignment " + segment.lane_assignment +
                                                    "detected direction " +
                                                    json.dumps(segment.downstream_directions_info))
                    if not ("left|right" in segment.lane_assignment):
                        if not ("left" == segment.lane_assignment):
                            if not ("right" == segment.lane_assignment):
                                if not ("s" in overall_directions):
                                    logger.map_logger.error("straight downstream of segment " +
                                                            segment.segment_id + " segment not detected")
                    lanesets = segment.generate_lanesets()
                    for laneset in lanesets:
                        network.add_laneset(laneset)

    # update the upstream/downstream lanesets
    for laneset_id, laneset in network.lanesets.items():
        upstream_node = laneset.upstream_node
        node = network.nodes[upstream_node]
        node.downstream_lanesets.append(laneset_id)
        downstream_node = laneset.downstream_node
        node = network.nodes[downstream_node]
        node.upstream_lanesets.append(laneset_id)
    return network
