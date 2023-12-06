"""
This file contains the classes and load/save of the OSM (XML) data
    see more information for the OSM (XML) data: https://wiki.openstreetmap.org/wiki/OSM_XML
    highly recommended map edit software: JSOM | https://josm.openstreetmap.de/

"""
import numpy as np
import networkx as nx

from .map_modes import GraphMode


class Network(object):
    """
    Class for the general network

    **Main Attributes**
        Classes for the roads
            - ``.ways``: a dictionary contains all the OpenStreetMap ways
              (:class:`mtldp.mtlmap.OsmWay`) in the network.
            - ``.links`` a dictionary contains all the links
              (:class:`mtldp.mtlmap.Link`) in the network.
            - ``.segments`` a dictionary contains all the segments
              (:class:`mtldp.mtlmap.Segment`) in the network.
            - ``.lanesets`` a dictionary contains all the lanesets
              (:class:`mtldp.mtlmap.LaneSet`) in the network.
        Classes for the nodes
            - ``.nodes`` a dictionary contains all the nodes
              (:py:class:`mtldp.mtlmap.Node`) in the network
        Others
            - ``.bounds`` the bounding box of the network
            - ``.networkx_graph`` networkx graph
    """

    def __init__(self):
        # basic members
        self.segments = {}
        self.ways = {}
        self.nodes = {}
        self.links = {}
        self.movements = {}

        # the following content is for network modeling
        self.lanesets = {}
        self.connectors = {}
        self.conflict_points = {}

        self.signalized_node_list = []
        self.unsignalized_node_list = []
        self.end_node_list = []

        self.networkx_mode = GraphMode.SEGMENT
        self.networkx_graph = None
        self.bounds = None

    def shortest_path_between_nodes(self, source_node: str, end_node: str,
                                    weight_attrib: str = "length"):
        """
        Calculate the shortest path between **unordinary nodes** (the source node and end node
        should not be an ordinary node). This implementation is based on NetworkX.

        Attention:
            Please pay attention to the NetworkX graph mode, there are different modes for building the networkX graph
            and the shortest path function also differs under different mode. See :class:`mtldp.mtlmap.GraphMode` for
            more information.

        :param source_node: source node id
        :param end_node: end node id
        :param weight_attrib: the chosen weight to calculate the shortest path
        :return: ``{"nodes": [str], "weight": float, "edges": [str]}``, ``"weight"`` is ``None`` if no connected path.
        """
        if self.networkx_graph is None:
            self.build_networkx_graph()

        graph = self.networkx_graph

        # dump the weight to the directed graph
        for edge, edge_attribs in graph.edges.items():
            edge_id = edge_attribs["id"]

            if self.networkx_mode == GraphMode.LINK:
                if not (weight_attrib in self.links[edge_id].__dict__.keys()):
                    raise ValueError(weight_attrib + " not in the link " + edge_id)
                graph.edges[edge]["weight"] = getattr(self.links[edge_id], weight_attrib)
            elif self.networkx_mode == GraphMode.SEGMENT:
                if not (weight_attrib in self.segments[edge_id].__dict__.keys()):
                    raise ValueError(weight_attrib + " not in the segment " + edge_id)
                graph.edges[edge]["weight"] = getattr(self.segments[edge_id], weight_attrib)
            elif self.networkx_mode == GraphMode.LANESET:
                if not (weight_attrib in self.lanesets[edge_id].__dict__.keys()):
                    raise ValueError(weight_attrib + " not in the laneset " + edge_id)
                graph.edges[edge]["weight"] = getattr(self.lanesets[edge_id], weight_attrib)

        if self.networkx_mode == GraphMode.SEGMENT or self.networkx_mode == GraphMode.LINK:
            node_list = nx.shortest_path(graph, source_node, end_node, "weight")
            total_weight = 0
            edge_list = []
            for idx in range(len(node_list) - 1):
                source_n = node_list[idx]
                end_n = node_list[idx + 1]
                total_weight += graph.edges[source_n, end_n, 0]["weight"]
                edge_list.append(graph.edges[source_n, end_n, 0]["id"])
            output_dict = {"weight": total_weight, "nodes": node_list, "edges": edge_list}
            return output_dict
        else:
            node = self.nodes[source_node]
            source_lanesets = node.downstream_lanesets
            node = self.nodes[end_node]
            end_lanesets = node.upstream_lanesets
            total_weight_list = []
            output_dict_list = []
            for source_laneset in source_lanesets:
                for end_laneset in end_lanesets:
                    try:
                        laneset_list = nx.shortest_path(graph, source_laneset, end_laneset, "weight")
                        total_weight = 0
                        node_list = []
                        for idx, laneset_id in enumerate(laneset_list):
                            laneset = self.lanesets[laneset_id]
                            total_weight += getattr(laneset, weight_attrib)
                            node_list.append(self.lanesets[laneset_id].downstream_node)
                        output_dict = {"weight": total_weight, "nodes": node_list, "edges": laneset_list}
                        total_weight_list.append(total_weight)
                        output_dict_list.append(output_dict)
                    except nx.exception.NetworkXNoPath:
                        # could not find shortest path by networkx
                        total_weight = 1e8
                        output_dict = {"weight": total_weight, "nodes": [], "edges": []}
                        total_weight_list.append(total_weight)
                        output_dict_list.append(output_dict)

                    # print(output_dict)
            chosen_index = int(np.argmin(total_weight_list))
            weight_ans = total_weight_list[chosen_index]
            if weight_ans >= 1e8 - 2:
                return {"weight": None, "nodes": [], "edges": []}
            else:
                return output_dict_list[chosen_index]

    def build_networkx_graph(self, graph_mode: "mtldp.mtlmap.GraphMode" = GraphMode.SEGMENT,
                             networkx_type: int = 0):
        """
        Build the self to a NetworkX graph object

        See reference for networkx: https://networkx.org/, this package will allow you
        to apply different types of algorithms based on network including shortest path, etc.

        :param networkx_type: graph type, 0: MultiDiGraph, 1: DiGraph
        :param graph_mode: the chosen graph mode, see :class:`mtldp.mtlmap.GraphMode`
        """
        if networkx_type == 0:
            graph = nx.MultiDiGraph()
        else:
            graph = nx.DiGraph()

        if graph_mode == GraphMode.SEGMENT:
            self.networkx_mode = GraphMode.SEGMENT
            # segment level graph, the ordinary nodes will be ignored
            for node_id, node in self.nodes.items():
                if node.is_ordinary_node():
                    continue
                graph.add_node(node_id, pos=(node.longitude, node.latitude))
            for segment_id, segment in self.segments.items():
                graph.add_edge(segment.upstream_node, segment.downstream_node,
                               id=segment.segment_id)
        elif graph_mode == GraphMode.LINK:
            print("This is not recommended unless you have special need to generate"
                  " the networkX object based on link level segmentation")
            self.networkx_mode = GraphMode.LINK
            # link level graph, only the intersection node will be considered
            for node_id, node in self.nodes.items():
                if node.is_intersection() or node.type == "end":
                    graph.add_node(node_id, pos=(node.longitude, node.latitude))
            for link_id, link in self.links.items():
                graph.add_edge(link.upstream_node, link.downstream_node,
                               id=link.link_id)
        elif graph_mode == GraphMode.LANESET:
            self.networkx_mode = GraphMode.LANESET
            for laneset_id, laneset in self.lanesets.items():
                node = self.nodes[laneset.upstream_node]
                graph.add_node(laneset_id, pos=(node.longitude, node.latitude), weight=laneset.length)
            for laneset_id, laneset in self.lanesets.items():
                for downstream_laneset in laneset.downstream_lanesets:
                    graph.add_edge(laneset_id, downstream_laneset, id=downstream_laneset)
        else:
            raise ValueError("Input mode not correct for building networkx graph.")

        self.networkx_graph = graph

    def reset_bound(self):
        lat_list = []
        lon_list = []
        for node_id, node in self.nodes.items():
            lat_list.append(node.latitude)
            lon_list.append(node.longitude)
        self.bounds = {"minlat": str(np.round(np.min(lat_list), 5)),
                       "minlon": str(np.round(np.min(lon_list), 5)),
                       "maxlat": str(np.round(np.max(lat_list), 5)),
                       "maxlon": str(np.round(np.max(lon_list), 5)),
                       "origin": "xingminw"}

    def add_segment_connection(self, upstream_segment_id, downstream_segment_id):
        segment = self.segments[upstream_segment_id]
        segment.add_downstream_segment(downstream_segment_id)
        segment = self.segments[downstream_segment_id]
        segment.add_upstream_segment(upstream_segment_id)

    def add_laneset(self, laneset):
        self.lanesets[laneset.laneset_id] = laneset

    def add_way(self, osm_way):
        self.ways[osm_way.way_id] = osm_way

    def add_connector(self, connector):
        self.connectors[connector.connector_id] = connector

    def add_node(self, node):
        self.nodes[node.node_id] = node

    def add_segment(self, segment):
        self.segments[segment.segment_id] = segment

    def add_link(self, link):
        self.links[link.link_id] = link

    def add_movement(self, movement):
        self.movements[movement.movement_id] = movement

    def add_conflict_point(self, conflict_point):
        self.conflict_points[conflict_point.conflict_id] = conflict_point

    def get_link_id(self):
        return list(self.links.keys())

    def get_node_type(self, node_id: str) -> str:
        """
        Get the node type

        :param node_id: node id
        :return: the type of the node
        """
        return self.nodes[node_id].type


