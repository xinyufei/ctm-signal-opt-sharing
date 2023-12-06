"""

"""


class Node(object):
    """
    Node corresponds to the node in osm (xml) data,

    Node is also the father class for the following classes:
        - :class:`MTTTrajectoryData.mimap.SignalizedNode`
        - :class:`MTTTrajectoryData.mimap.SignalizedNode`
        - :class:`MTTTrajectoryData.mimap.UnSignalizedNode`
        - :class:`MTTTrajectoryData.mimap.EndNode`

    **Main attributes**
        - ``.node_id`` unique id of the node.
        - ``.osm_attrib`` attributes of the node in the original osm data (dict)
        - ``.osm_tags`` tags of the node in the original osm data (dict)
        - ``.type`` type of the node ("ordinary", "connector", "signalized", "unsignalized", "end")
        - ``.latitude`` and ``.longitude`` node GPS coordinate
        - ``.upstream_segments`` upstream segments of this node (list of str)
        - ``.downstream_segments`` downstream segments of this node (list of str)

    """
    def __init__(self, node_id=None, osm_attrib=None, osm_tags=None):
        # original osm data (essential components)
        self.node_id = node_id
        self.osm_attrib = osm_attrib
        self.osm_tags = osm_tags

        self.type = "ordinary"

        self.latitude = None
        self.longitude = None

        self.connector_list = []

        # upstream and downstream segments
        self.upstream_segments = []
        self.downstream_segments = []

        # upstream and downstream links
        self.upstream_links = []
        self.downstream_links = []

        # upstream and downstream lanesets
        self.upstream_lanesets = []
        self.downstream_lanesets = []
        self.movement_id_list = []

        # osm way id that starts/ends at this node
        self.od_ways = []
        # in original osm data, some segments might directly traverse the node, this is
        # invalid, we need to filter this condition out by splitting the traversing segments
        self.traverse_ways = []

        # volume/capacity ratio
        self.v_c_ratio = 1

        if self.node_id is not None and self.osm_attrib is not None and self.osm_tags is not None:
            self.generate_basic_info()

    @classmethod
    def init_from_node(cls, node):
        new_node = cls()
        for k, v in node.__dict__.items():
            setattr(new_node, k, v)
        return new_node

    def is_intersection(self) -> bool:
        """

        :return: True if this node is an intersection
        """
        intersection_flag = (self.type == "signalized") or (self.type == "unsignalized")
        return intersection_flag

    def is_ordinary_node(self) -> bool:
        """

        :return: True if this node is an ordinary node
        """
        return self.type == "ordinary"

    def generate_basic_info(self):
        self.latitude = float(self.osm_attrib["lat"])
        self.longitude = float(self.osm_attrib["lon"])

    def add_connector(self, connector_id):
        if not (connector_id in self.connector_list):
            self.connector_list.append(connector_id)

    def add_movement(self, movement_id):
        if movement_id not in self.movement_id_list:
            self.movement_id_list.append(movement_id)

    def __str__(self):
        output_string = f"node id: {self.node_id}\n"
        for k, v in self.__dict__.items():
            output_string += f"\t {k}: {v}\n"
        output_string = output_string[:-1]
        return output_string


class SegmentConnectionNode(Node):
    """
    The node that connects the segments (all through, no left/right turn)
    """
    def __init__(self):
        super().__init__()
        self.type = "connector"

    @classmethod
    def init_from_node(cls, node):
        segment_connector = cls()
        for k, v in node.__dict__.items():
            setattr(segment_connector, k, v)
        segment_connector.type = "connector"
        return segment_connector


class SignalizedNode(Node):
    """
    Class for signalized intersection

    Inherit from :py:class:`MTTTrajectoryData.mimap.Node`

    **Additional Attributes**

        - ``.timing_plan`` the signal controller of this node, :py:class:`mimap.SignalTimingPlan`.
    """
    def __init__(self):
        super().__init__()
        self.type = "signalized"
        self.timing_plan = None

    @classmethod
    def init_from_node(cls, node):
        signalized_node = cls()
        for k, v in node.__dict__.items():
            setattr(signalized_node, k, v)
        signalized_node.type = "signalized"
        return signalized_node


class UnSignalizedNode(Node):
    """
    Class for unsignalized node

    Inherit from :py:class:`MTTTrajectoryData.mimap.Node`

    **Additional Attributes**
    """
    def __init__(self):
        super().__init__()
        self.type = "unsignalized"

    @classmethod
    def init_from_node(cls, node):
        unsignalized_node = cls()
        for k, v in node.__dict__.items():
            setattr(unsignalized_node, k, v)
        unsignalized_node.type = "unsignalized"
        return unsignalized_node


class EndNode(Node):
    def __init__(self):
        super().__init__()
        self.type = "end"

    @classmethod
    def init_from_node(cls, node):
        end_node = cls()
        for k, v in node.__dict__.items():
            setattr(end_node, k, v)
        end_node.type = "end"
        return end_node


def node_differentiation(network):
    """
    differentiate the node into:
        ordinary node, signalized intersection and unsignalized intersection

    :param network:
    :return:
    """
    signalized_nodes = []
    end_nodes = []
    unsignalized_nodes = []

    for node_id, node in network.nodes.items():
        undirected_degree = 2 * len(node.traverse_ways) + len(node.od_ways)
        if undirected_degree == 1:
            end_nodes.append(node_id)

            # create new node
            new_node = EndNode.init_from_node(node)

            # replace the original node
            network.nodes[node_id] = new_node
        elif undirected_degree == 2:
            pass
        else:
            node_tags = node.osm_tags
            signalized_flag = False
            if "highway" in node_tags.keys():
                if node_tags["highway"] == "traffic_signals":
                    signalized_flag = True

            if signalized_flag:
                signalized_nodes.append(node_id)

                # replace the original node
                new_node = SignalizedNode.init_from_node(node)
                network.nodes[node_id] = new_node
            else:
                unsignalized_nodes.append(node_id)

                # replace the original node
                new_node = UnSignalizedNode.init_from_node(node)
                network.nodes[node_id] = new_node

    # save node list to network
    network.unsignalized_node_list = unsignalized_nodes
    network.end_node_list = end_nodes
    network.signalized_node_list = signalized_nodes
    return network

