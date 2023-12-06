"""
This file is to process the map data and construct the network class

The map data processing includes the following procedure:

"""

import os
from typing import Union

from .map_modes import MapMode
from .map_xml import load_xml_map
from .map_process import split_traverse_intersection_way
from .nodes_classes import node_differentiation
from .osm_ways import parse_osm_ways
from .segments import generate_network_segments, generate_segments_connections
from .links import generate_network_links, generate_link_details
from .connections import generate_network_connectors
from .signals import generate_intersections_and_conflicts
from .movements import generate_network_movements
from .lanes import generate_network_lanesets

from ..tools import logger


def build_network_from_xml(file_name: str, logger_path: str = "output",
                           mode: "mtldp.mtlmap.MapMode" = MapMode.ACCURATE,
                           build_networkx: bool = True):
    """
    Build the network class in :py:class:`mtldp.mtlmap.Network` from OpenStreetMap data

    :param file_name: input map file name (`.osm` or `.xml`)
    :param build_networkx: whether build the networkx graph object (Default: True)
    :param logger_path: output logger path, name as `/map.log`
    :param mode: mode selection for the network layers

    :return: Static network class :py:class:`mtldp.mtlmap.Network`,
             see :ref:`reference <static_core>` for the list of the static network classes
    """
    if not os.path.exists(logger_path):
        os.makedirs(logger_path)

    logger_file = logger_path + "/map.log"
    if os.path.exists(logger_file):
        os.remove(logger_file)

    # create the logger to record the map data processing process
    logger.map_logger = logger.setup_logger("map-data", logger.map_logger_formatter, logger_file)

    logger.map_logger.info("Loading the map data from " + file_name + " ...")
    logger.map_logger.info("Process the map data using " + mode.name + " mode...")
    network = load_xml_map(file_name)

    # parse the node and way in the original osm data
    logger.map_logger.info("Parsing the original osm data...")
    network = parse_osm_ways(network)
    network.reset_bound()

    # node differentiation
    logger.map_logger.info("Differentiate the node...")
    network = node_differentiation(network)

    # split the way that traverses the intersections
    logger.map_logger.info("Split osm ways that traverse the intersections...")
    network = split_traverse_intersection_way(network)

    # create network segment
    logger.map_logger.info("Generating the segments...")
    network = generate_network_segments(network)

    if build_networkx:
        logger.map_logger.info("Build networkx graph...")
        network.build_networkx_graph()

    if mode == MapMode.MAP_MATCHING:
        logger.map_logger.info("Map data processing done. (MAP_MATCHING MODE)")
        return network

    logger.map_logger.info("Generate segment connections...")
    network = generate_segments_connections(network)

    # create network links
    logger.map_logger.info("Generating the network links...")
    network = generate_network_links(network)

    logger.map_logger.info("Generating the network movements...")
    network = generate_network_movements(network)

    if mode == MapMode.ROUGH:
        logger.map_logger.info("Map data processing done. (ROUGH_MAP MODE)")
        return network

    # generate the connection of the lane sets and segments at intersections
    logger.map_logger.info("Generating the lanesets...")
    network = generate_network_lanesets(network)

    logger.map_logger.info("Generate link detailes...")
    network = generate_link_details(network)

    logger.map_logger.info("Generating the connectors...")
    network = generate_network_connectors(network)

    logger.map_logger.info("Initiate the intersection configurations...")
    network = generate_intersections_and_conflicts(network)

    logger.map_logger.info("Map data processing done. (ACCURATE MODE)")
    logger.map_logger = None
    return network

