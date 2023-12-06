import numpy as np
from ..tools import constants
from ..tools import logger


class Link(object):
    """
    A link connects two signalized/unsignalized/origin/destination nodes. It might contain multiple segments


    **Main attributes**
        - ``.link_id`` a integer for link ID. It has the format: number1_number2. The first number is the original node
          of the link. The second number is the destination node of the link.
        - ``.segment_list`` the list of segments that belong to the link
        - ``.geometry`` the GPS coordinates along the link
        - ``.node_list`` a list of nodes on the link
        - ``.upstream_node`` the upstream node of the link
        - ``.downstream_node`` the downstream node of the link
        - ``.heading`` the heading angle of the link (range: (-180,180]))
        - ``.from_direction`` the direction from which the segment originates. For example, if the segment originates
          from south, this value is "S".
        - ``.length`` length of the link in meters

    """

    def __init__(self):
        self.link_id = None
        self.segment_list = []
        self.geometry = None
        self.node_list = []

        self.upstream_node = None
        self.downstream_node = None

        self.heading = None
        self.from_direction = None

        self.length = None

        # buffer segments: segments before the buffer (only 1 laneset)
        self.buffer_segments = []  # fixme
        self.buffer_length = None  # fixme

        # user equilibrium


def generate_network_links(network):
    """
    generate network links

        combine the segment to get the link, start from each intersection node and end node
            connects the segment until encounter next end node or intersection node
    :param network:
    :return:
    """
    for node_id, node in network.nodes.items():
        if node.type == "ordinary" or node.type == "connector":
            continue
        downstream_segments = node.downstream_segments
        for segment_id in downstream_segments:
            segment_list = [segment_id]
            segment = network.segments[segment_id]

            maximum_loops = 20
            loop_count = 0
            while True:
                loop_count += 1
                if loop_count > maximum_loops:
                    break

                downstream_node_id = segment.downstream_node
                if network.get_node_type(downstream_node_id) == "ordinary":
                    logger.map_logger.error("something wrong here, "
                                            "downstream node of a segment is "
                                            "ordinary node")
                    logger.map_logger.error("segment id" + segment.segment_id +
                                            "downstream node" + segment.downstream_node)

                # jump out of the loop if the downstream node is not an ordinary node
                if network.get_node_type(downstream_node_id) != "connector":
                    break
                local_down_segs = segment.downstream_segments
                if len(local_down_segs) != 1:
                    logger.map_logger.error("the # of downstream segments at the ordinary connector"
                                            " is not 1")
                    logger.map_logger.error("segment id " + segment.segment_id +
                                            " downstream segs: " +
                                            " ".join([str(val) for val in segment.downstream_segments]))
                    continue

                down_seg_id = local_down_segs[0]
                segment = network.segments[down_seg_id]
                segment_list.append(segment.segment_id)

            # detect repeat
            if len(set(segment_list)) != len(segment_list):
                logger.map_logger.error("circle segments in link generation")
                logger.map_logger.error("details: " + ",".join([str(val) for val in segment_list]))
            generate_link_from_segments(network, segment_list)

    # update laneset info
    for laneset_id, laneset in network.lanesets.items():
        segment_id = laneset.belonged_segment
        segment = network.segments[segment_id]

        laneset.belonged_link = segment.belonged_link
        laneset.speed_limit = segment.speed_limit
        laneset.upstream_node = segment.upstream_node
        laneset.downstream_node = segment.downstream_node
    return network


def generate_link_details(network):
    """

    :param network:
    :return:
    """
    for link_id, link in network.links.items():
        segment_list = link.segment_list
        lanset_num_list = [network.segments[segment_id].laneset_num for segment_id in segment_list]
        invalid_flag = [lanset_num_list[idx + 1] < lanset_num_list[idx] for idx in range(len(lanset_num_list) - 1)]
        if sum(invalid_flag):
            logger.map_logger.error("lane set number decreases in a link")
            logger.map_logger.error("link id " + link.link_id + "segment list: " +
                                    ",".join([str(val) for val in link.segment_list]) +
                                    " # of laneset: " + ",".join([str(val) for val in lanset_num_list]))

        head_list = lanset_num_list[:-1]
        invalid_flag = sum([val != 1 for val in head_list])
        if invalid_flag:
            logger.map_logger.error("need to fix the following segments:")
            logger.map_logger.error("link id " + link.link_id + "segment list: " +
                                    ",".join([str(val) for val in link.segment_list]) +
                                    " # of laneset: " + ",".join([str(val) for val in lanset_num_list]))

        segment_indicator_list = [val == 1 for val in lanset_num_list]

        buffer_segments = []
        total_length = 0
        buffer_length = 0
        for bdx in range(len(segment_list)):
            segment_length = network.segments[segment_list[bdx]].length
            total_length += segment_length
            if segment_indicator_list[bdx]:
                buffer_segments.append(segment_list[bdx])
                buffer_length += segment_length
        link.buffer_length = buffer_length
        link.length = total_length
        link.buffer_segments = buffer_segments

    return network


def generate_link_from_segments(network, segment_list):
    """
    Generate the link and add the link to the network given a series of segments

    :param network:
    :param segment_list: the list of the segment id
    """
    link = Link()
    link.segment_list = segment_list
    segment = network.segments[segment_list[0]]
    upstream_node = segment.upstream_node
    link.upstream_node = upstream_node

    lat_list = [segment.geometry["lat"][0]]
    lon_list = [segment.geometry["lon"][0]]

    segment = network.segments[segment_list[-1]]
    downstream_node = segment.downstream_node
    link.downstream_node = downstream_node

    link.link_id = upstream_node + "_" + downstream_node
    network.nodes[upstream_node].downstream_links.append(link.link_id)
    network.nodes[downstream_node].upstream_links.append(link.link_id)

    # add link id to the segment
    node_list = [upstream_node]

    heading_list = []
    for segment_id in segment_list:
        segment.belonged_link = link.link_id
        segment = network.segments[segment_id]

        heading_list.append(segment.heading)
        segment_geometry = segment.geometry
        segment_nodes = segment.node_list
        node_list += segment_nodes[1:]
        lat_list += segment_geometry["lat"][1:]
        lon_list += segment_geometry["lon"][1:]
    link.node_list = node_list
    link.geometry = {"lat": lat_list, "lon": lon_list}
    link.heading = np.average(heading_list)
    link.from_direction = constants.generate_geo_heading_direction(link.heading)
    network.add_link(link)



