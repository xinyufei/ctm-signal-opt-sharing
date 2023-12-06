import json

from copy import deepcopy
from .lanes import LaneSet
from .nodes_classes import SegmentConnectionNode

from ..tools import logger
from ..tools import constants as mapping
from ..tools.gps_utils import shift_geometry, get_closest_angle


class Segment(object):
    """
    A segment is a proportion of a link that share share the same number of lanes.

    **Main attributes**
        - ``.segment_id`` a integer for segment ID. 0 or 1 (denotes the direction ) is added at the end of the
          ``.osm_way_id`` as the ``.segment_id``
        - ``.osm_way_id`` a integer for the original OSM way ID
        - ``.osm_tags`` a dictionary contains all the tags in the original osm data.
        - ``.osm_attrib`` a dictionary contains all the attributes in the original osm data.
        - ``.belonged_link`` the link ID that the segment belongs to
        - ``.laneset_list`` the list of lane sets that belong to the segment
        - ``.laneset_num`` the number of lane sets that belong to the segment
        - ``.speed_limit`` speed limit of the segment in m/s
        - ``.length`` length of the segment in meters
        - ``.geometry`` the GPS coordinates along the segment
        - ``.lane_number`` number of lanes of the segment
        - ``.lane_assignment`` the assignment of the lanes of the segment. For example, "all_through" means all lanes on
          the segment are through movements. "left|through;right" means the segments include both left turn movement
          through (right turn) movement. If unavailable, this value is null.
        - ``.heading`` the heading angle of the segment (range: (-180,180]))
        - ``.from_direction`` the direction from which the segment originates. For example, if the segment originates
          from south, this value is "S".
        - ``.node_list`` a list of nodes on the segment
        - ``.upstream_node`` the upstream node of the segment
        - ``.downstream_node`` the downstream node of the segment
        - ``.upstream_segment`` a list of the upstream segment ID of this segment
        - ``.downstream_segment`` a list of the downstream segment ID of this segment
        - ``.downstream_direction_info`` a dictionary that represents the direction of the downstream segments.
          For example, ``{'l': '4116329441', 'r': '4126838890', 's': '87279680'}`` means left turn downstream segment is
          ``4116329441``. Through movement downstream segment is ``87279680``, and right turn downstream segment is
          ``4126838890``

    """

    def __init__(self):
        self.segment_id = None
        self.osm_way_id = None

        self.osm_tags = None
        self.osm_attrib = None

        self.belonged_link = None

        self.laneset_list = []
        self.laneset_num = None

        self.osm_direction_flag = None  # "backward" or "forward"

        self.speed_limit = None
        self.length = None  # unit: meters
        self.geometry = None  # {"lat": [], "lon": []}
        self.lane_number = None
        self.lane_assignment = None

        self.heading = None
        # self.weighted_heading = None
        self.from_direction = None

        self.node_list = None
        # self.downstream_node_type = None            # "end", "ordinary", "signalized", "unsignalized"
        # self.upstream_node_type = None

        # network topology
        self.upstream_node = None
        self.downstream_node = None

        self.downstream_connectors = []
        self.upstream_connectors = []

        self.upstream_segments = []
        self.downstream_segments = []
        self.downstream_directions_info = {}

    @classmethod
    def init_from_way(cls, osmway, direction):
        """
        initiate a segment using the osm way

        :param osmway:
        :param direction: "backward" or "forward"
        :return:
        """
        segment = cls()
        if direction == "forward":
            segment.segment_id = osmway.way_id + "0"
        else:
            segment.segment_id = osmway.way_id + "1"
        segment.osm_way_id = osmway.way_id
        segment.length = osmway.length
        segment.speed_limit = osmway.speed_limit

        # get the osm tag and attributes
        segment.osm_tags = deepcopy(osmway.osm_tags)
        segment.osm_attrib = deepcopy(osmway.osm_attrib)

        # change the osm tags and attributes
        segment.osm_tags["oneway"] = "yes"
        if "action" in osmway.osm_attrib:
            segment.osm_attrib["action"] += "-directed"
        segment.osm_attrib["id"] = segment.segment_id

        # todo: reverse the direction, I am not sure whether it will be used but
        way_geometry = osmway.geometry
        way_node_list = osmway.node_list
        if direction == "backward":
            # reverse the sequence
            segment.geometry = {"lat": way_geometry["lat"][::-1], "lon": way_geometry["lon"][::-1]}

            segment.node_list = way_node_list[::-1]
            segment.lane_number = osmway.backward_lanes
            segment.lane_assignment = osmway.backward_lane_assignment
            segment.heading = osmway.backward_heading

            # reverse the backward and forward
            backward_tags = []
            forward_tags = []
            for tag in segment.osm_tags.keys():
                if "backward" in tag and "forward" in tag:
                    logger.map_logger.warning("strange osm tags " + json.dumps(tag))
                    continue
                if "backward" in tag:
                    backward_tags.append(tag)
                if "forward" in tag:
                    forward_tags.append(tag)

            new_osm_tags = deepcopy(segment.osm_tags)
            for tag in forward_tags:
                del new_osm_tags[tag]

            for tag in backward_tags:
                new_osm_tags[tag.replace(":backward", "")] = segment.osm_tags[tag]
                del new_osm_tags[tag]
            segment.osm_tags = new_osm_tags
        else:
            segment.geometry = way_geometry
            segment.node_list = way_node_list
            segment.lane_number = osmway.forward_lanes
            segment.lane_assignment = osmway.forward_lane_assignment
            segment.heading = osmway.forward_heading

            # delete the tags that related to the "backward"
            backward_tags = []
            forward_tags = []
            for tag in segment.osm_tags.keys():
                if "backward" in tag and "forward" in tag:
                    logger.map_logger.warning("strange osm tags " + json.dumps(tag))
                    continue
                if "backward" in tag:
                    backward_tags.append(tag)
                if "forward" in tag:
                    forward_tags.append(tag)

            new_osm_tags = deepcopy(segment.osm_tags)
            for tag in backward_tags:
                del new_osm_tags[tag]
            for tag in forward_tags:
                new_osm_tags[tag.replace(":forward", "")] = segment.osm_tags[tag]
                del new_osm_tags[tag]
            segment.osm_tags = new_osm_tags

        segment.osm_tags["lanes"] = str(int(segment.lane_number))
        # shift segment geometry
        if not osmway.directed:
            segment.geometry = \
                shift_geometry(segment.geometry,
                               shift_distance=mapping.DISPLAY_LANE_INTERVAL * mapping.SEGMENT_SHIFT,
                               shift_direction="right")

        segment.upstream_node = segment.node_list[0]
        segment.downstream_node = segment.node_list[-1]
        segment.from_direction = mapping.generate_geo_heading_direction(segment.heading)
        return segment

    def add_downstream_segment(self, seg_id):
        if not (seg_id in self.downstream_segments):
            self.downstream_segments.append(seg_id)

    def add_upstream_segment(self, seg_id):
        if not (seg_id in self.upstream_segments):
            self.upstream_segments.append(seg_id)

    def generate_lanesets(self, assignments=None):
        """
        initiate the lane sets of the segment

        :return:
        """
        laneset_list = []

        # easy case: only the through lanes
        if assignments is None:
            if self.lane_assignment == "all_through":
                lanset = LaneSet.init_from_segment(self, "s", self.lane_number, 0)
                # self.add_lanset(lanset)
                self.laneset_list.append(lanset.laneset_id)
                laneset_list.append(lanset)
            elif self.lane_assignment == "null":
                lanset = LaneSet.init_from_segment(self, "a", self.lane_number, 0)
                self.laneset_list.append(lanset.laneset_id)
                laneset_list.append(lanset)
            else:
                direction_list = self.lane_assignment.split("|")
                left_lanes = 0
                right_lanes = 0
                through_lanes = 0
                for direction in direction_list:
                    sub_direction_list = direction.split(";")
                    sub_nums = len(sub_direction_list)
                    sub_left = sum([val == "left" for val in sub_direction_list])
                    sub_right = sum([val == "right" for val in sub_direction_list])
                    sub_through = sum([val == "" or val == "through" for val in sub_direction_list])
                    left_lanes += sub_left / sub_nums
                    right_lanes += sub_right / sub_nums
                    through_lanes += sub_through / sub_nums
                # dedicated left lane
                dedicated_left_lane, dedicated_right_lane = 0, 0
                if left_lanes >= 1:
                    dedicated_left_lane = int(left_lanes)
                    lanset = LaneSet.init_from_segment(self, "l", dedicated_left_lane, 1)
                    self.laneset_list.append(lanset.laneset_id)
                    laneset_list.append(lanset)
                if right_lanes >= 1:
                    dedicated_right_lane = int(right_lanes)
                    lanset = LaneSet.init_from_segment(self, "r", dedicated_right_lane, -1)
                    self.laneset_list.append(lanset.laneset_id)
                    laneset_list.append(lanset)
                through_lane_number = self.lane_number - dedicated_left_lane - dedicated_right_lane
                if through_lane_number > 0:
                    direction = "s"

                    if dedicated_left_lane == 0:
                        if left_lanes > 0:
                            direction += "l"
                    if dedicated_right_lane == 0:
                        if right_lanes > 0:
                            direction += "r"
                    lanset = LaneSet.init_from_segment(self, direction, through_lane_number, 0)
                    self.laneset_list.append(lanset.laneset_id)
                    laneset_list.append(lanset)
        else:
            for assignment in assignments:
                lanset = LaneSet.init_from_segment(self, assignment["dir"],
                                                   assignment["num"], assignment["shift"])
                self.laneset_list.append(lanset.laneset_id)
                laneset_list.append(lanset)
        self.laneset_num = len(self.laneset_list)
        return laneset_list


def generate_network_segments(network):
    """
    initiate the network segment given the osm way each segment is a directed OSM way

    :param network:
    :return:
    """
    # create the segment
    for way_id, way in network.ways.items():
        # deal with the back ward and forward separately
        if way.backward_lanes is None:
            # logger.map_logger.warning(way.way_id + " backward direction not initialized correctly!")
            pass
        else:
            # fixme: if the lane number is zero, there must be something wrong with the map data
            #  or the processing, if the lane number is -1, this means that this is a oneway,
            #  the same with the forward direction
            if way.backward_lanes >= 0:
                if way.backward_lanes == 0:
                    logger.map_logger.warning(way.way_id, "backward direction lane number equals to 0!")
                backward_segment = Segment.init_from_way(way, "backward")
                network.add_segment(backward_segment)

        if way.forward_lanes is None:
            # logger.map_logger.warning(way.way_id + "forward direction not initialized correctly!")
            pass
        else:
            if way.forward_lanes == 0:
                # logger.map_logger.warning(way.way_id + "forward direction lane number equals to 0!")
                pass
            forward_segment = Segment.init_from_way(way, "forward")
            network.add_segment(forward_segment)

    for segment_id, segment in network.segments.items():
        if segment.lane_assignment is None:
            node = network.nodes[segment.downstream_node]
            if not node.is_intersection():
                segment.lane_assignment = "all_through"
            else:
                segment.lane_assignment = "null"

        # add the node upstream and downstream segment
        network.nodes[segment.upstream_node].downstream_segments.append(segment_id)
        network.nodes[segment.downstream_node].upstream_segments.append(segment_id)

    # get a special node ---- segment connector
    for node_id, node in network.nodes.items():
        if node.type == "ordinary":
            if len(node.upstream_segments) > 0:
                # change this node to connector node
                new_node = SegmentConnectionNode.init_from_node(node)
                network.nodes[node_id] = new_node
    return network


def generate_segments_connections(network):
    for node_id, node in network.nodes.items():
        if node.type == "ordinary":
            continue
        elif node.type == "connector":
            # set the lane assignment to all_through
            upstream_segments = node.upstream_segments
            for upstream_seg in upstream_segments:
                segment = network.segments[upstream_seg]
                if segment.lane_assignment != "all_through":
                    segment.lane_assignment = "all_through"

            if len(node.upstream_segments) == 1:
                upstream_segment = node.upstream_segments[0]
                downstream_segments = node.downstream_segments
                if len(downstream_segments) < 1:
                    logger.map_logger.error("single segment in node " + node_id + ": downstream segments "
                                            + ",".join([str(val) for val in downstream_segments]))
                    continue
                if len(downstream_segments) != 1:
                    logger.map_logger.warning("single segment in node " + node_id + ": downstream segments "
                                              + ",".join([str(val) for val in downstream_segments]))
                # # add the turning info
                segment = network.segments[upstream_segment]
                segment.downstream_directions_info = {"s": downstream_segments[0]}

                network.add_segment_connection(upstream_segment, downstream_segments[0])
            elif len(node.upstream_segments) == 2:
                upstream_segments = node.upstream_segments
                downstream_segments = node.downstream_segments
                if len(downstream_segments) != 2:
                    logger.map_logger.error("# of downstream segments of segment connector node "
                                            + node_id + "does not equal 2")
                    continue
                up_seg_way_id = []
                for up_seg in upstream_segments:
                    segment = network.segments[up_seg]
                    up_seg_way_id.append(segment.osm_way_id)
                down_seg_way_id = []
                for down_seg in downstream_segments:
                    segment = network.segments[down_seg]
                    down_seg_way_id.append(segment.osm_way_id)
                indicator_list = [up_seg_way_id[ii] == down_seg_way_id[jj]
                                  for ii in range(2) for jj in range(2)]

                if indicator_list[0]:
                    network.add_segment_connection(upstream_segments[0], downstream_segments[1])
                    network.add_segment_connection(upstream_segments[1], downstream_segments[0])
                else:
                    # fixme: there might be something wrong here!
                    network.add_segment_connection(upstream_segments[0], downstream_segments[0])
                    network.add_segment_connection(upstream_segments[1], downstream_segments[1])
            else:
                logger.map_logger.error("ordinary node has more than 2 upstream segments ")
        elif node.type == "end":
            upstream_segments = node.upstream_segments
            for upstream_seg in upstream_segments:
                segment = network.segments[upstream_seg]
                if segment.lane_assignment != "all_through":
                    segment.lane_assignment = "all_through"
        elif node.is_intersection():
            upstream_segments = node.upstream_segments
            downstream_segments = node.downstream_segments
            downstream_heading_list = [network.segments[sd].heading for sd in downstream_segments]
            for up_seg in upstream_segments:
                segment = network.segments[up_seg]
                segment_heading = segment.heading
                left_heading = segment_heading + 90
                right_heading = segment_heading - 90
                straight_heading = segment_heading
                if len(downstream_heading_list) == 0:
                    logger.map_logger.warning("there is not downstream segments in intersection " + node_id)
                    continue
                left_seg_index, d_left = get_closest_angle(left_heading, downstream_heading_list)
                right_seg_index, d_right = get_closest_angle(right_heading, downstream_heading_list)
                straight_seg_index, d_straight = get_closest_angle(straight_heading, downstream_heading_list)
                tolerance_angle_difference = 45
                if d_left < tolerance_angle_difference:
                    left_seg = downstream_segments[left_seg_index]
                else:
                    left_seg = None
                if d_right < tolerance_angle_difference:
                    right_seg = downstream_segments[right_seg_index]
                else:
                    right_seg = None
                if d_straight < tolerance_angle_difference:
                    straight_seg = downstream_segments[straight_seg_index]
                else:
                    straight_seg = None
                downstream_directed_segments = {"l": left_seg, "r": right_seg, "s": straight_seg}
                segment.downstream_directions_info = downstream_directed_segments

                connected_downstream_segments = downstream_directed_segments.values()
                for downstream_seg in connected_downstream_segments:
                    if downstream_seg is None:
                        continue
                    network.add_segment_connection(segment.segment_id, downstream_seg)
    return network
