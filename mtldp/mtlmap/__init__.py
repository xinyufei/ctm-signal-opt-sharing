# general classes for the network
from .osm_ways import OsmWay
from .links import Link
from .segments import Segment
from .lanes import LaneSet
from .nodes_classes import Node, SignalizedNode, UnSignalizedNode,\
    SegmentConnectionNode, EndNode

from .static_net import Network
from .movements import Movement
from .signals import SignalTimingPlan
from .build_network import build_network_from_xml

from .map_xml import save_network_to_xml
from .map_modes import GraphMode, MapMode

from .osm_filter import osm_way_filter
from .map_json import output_static_geometry_json

try:
    from .converter import convert_osm_to_shp
except ImportError:
    print("osmnx not installed correctly, you cannot use the osm->shapefile converter,"
          " run the following cmd to install:")
    print("\t conda config --prepend channels conda-forge")
    print("\t conda install --strict-channel-priority osmnx")