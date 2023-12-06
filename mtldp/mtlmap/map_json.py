"""
visualize the map data
"""

from copy import deepcopy


import json
import numpy as np
from ..tools import constants as utils


def output_static_geometry_json(network, output_file=None,
                                lat_ahead=True):
    """
    Input the network, output the json file to display. The output ``.json`` is used to display the network
    in the web-based visualization tool: `[map web] <https://xingminw.github.io/mtldp/map.html>`_.

    :param network:
    :param output_file: if ``None``, this will return the dict, else save to the file
    :param lat_ahead: latitude ahead or longitude ahead
    :return: ``None`` if the ``output_file`` is not empty, otherwise this function will return the ``dict``.
    """
    output_dict = {"segments": {}, "nodes": {}, "links": {}, "lanesets": {}}
    for link_id, link in network.links.items():
        geometry_list = get_geometry_list(link.geometry, lat_ahead)
        link_tag = "<p> link id: " + link.link_id
        link_tag += "</p>"
        output_dict["links"][link_id] = {"geometry": geometry_list, "type": "arrow", "tag": link_tag}

    for laneset_id, laneset in network.lanesets.items():
        geometry_list = get_geometry_list(laneset.geometry, lat_ahead)
        laneset_tag = "<p> laneset id: " + laneset_id + "<br/>"
        laneset_tag += "speed limit: " + str(int(laneset.speed_limit / utils.MPH_TO_METERS_PER_SEC)) + " mph <br/>"
        laneset_tag += "lane number: " + str(int(laneset.lane_number)) + "<br/>"
        laneset_tag += "direction: " + laneset.from_direction + "-" + laneset.turning_direction + "<br/>"
        laneset_tag += "travel time: " + str(np.round(laneset.travel_time, 3)) + "s <br/>"
        laneset_tag += "flow: " + str(np.round(laneset.laneset_flow)) + "vph <br/>"
        laneset_tag += "movements:" + "(" + ",".join([str(val) for val in laneset.movement_list]) + ")"
        laneset_tag += "</p>"
        # color indicating for flow
        color = get_color(min(laneset.laneset_flow / laneset.capacity, 1), 0.3)
        output_dict["lanesets"][laneset_id] = {"geometry": geometry_list, "type": "arrow", "color": color,
                                               "tag": laneset_tag}

    for segment_id, segment in network.segments.items():
        geometry_list = get_geometry_list(segment.geometry, lat_ahead)
        segment_tag = "<p> segment id: " + segment_id + " <br/> "
        segment_tag += "lane number: " + str(int(segment.lane_number)) + "<br/>"
        segment_tag += "speed limit: " + str(int(segment.speed_limit / utils.MPH_TO_METERS_PER_SEC)) + "mph <br/>"
        segment_tag += "upstream node: " + segment.upstream_node + " <br/> "
        segment_tag += "downstream node: " + segment.downstream_node + " <br/> "

        segment_tag += "</p>"
        output_dict["segments"][segment_id] = {"geometry": geometry_list, "type": "arrow", "tag": segment_tag}

    for node_id, node in network.nodes.items():
        node_tag = "<p> node id: " + node.node_id + "<br/>"
        node_tag += "type: " + node.type + "<br/>"
        node_tag += "v/c ratio: " + str(np.round(node.v_c_ratio, 3))

        phase_details = {}
        # add button for phase and movement
        if node.type == "signalized":
            timing_plan = node.timing_plan
            if timing_plan is None:
                continue
            for phase_id, phase in timing_plan.phases.items():
                if len(phase.controlled_lanesets) == 0:
                    continue

                phase_name = "phase" + str(phase.phase_id)
                if not (phase_name in phase_details.keys()):
                    phase_details[phase_name] = {"geometry": []}

                for laneset_id in phase.controlled_lanesets:
                    laneset = network.lanesets[laneset_id]
                    geometry = laneset.geometry
                    geometry_list = get_geometry_list(geometry, lat_ahead)

                    phase_details[phase_name]["geometry"].append(geometry_list)

                node_tag += "<br /> <button onclick= \"clickPhaseButton(\'phase" + str(phase.phase_id) + \
                            "\', \'" + str(node_id) + "\')\"> Phase" \
                            + str(phase.phase_id) + " </button>:  "
                for movement_id in phase.movement_list:
                    movement = timing_plan.movements[movement_id]

        node_tag += "</p>"
        if lat_ahead:
            node_geometry = [node.latitude, node.longitude]
        else:
            node_geometry = [node.longitude, node.latitude]
        output_dict["nodes"][node_id] = \
            {"geometry": node_geometry, "tag": node_tag, "type": node.type,
             "color": get_color(min(node.v_c_ratio, 1), 0.3),
             "phases": phase_details}

    if lat_ahead:
        output_dict["bounds"] = [[network.bounds["minlat"], network.bounds["minlon"]],
                                 [network.bounds["maxlat"], network.bounds["maxlon"]]]
    else:
        output_dict["bounds"] = [[network.bounds["minlon"], network.bounds["minlat"]],
                                 [network.bounds["maxlon"], network.bounds["maxlat"]]]

    if output_file is None:
        return output_dict
    else:
        output_text = json.dumps(output_dict, indent=2)
        with open(output_file, "w") as temp_file:
            temp_file.write(output_text)
        return None


def get_geometry_list(geometry, lat_ahead=True):
    lat_list = geometry["lat"]
    lon_list = geometry["lon"]
    geometry_list = []
    for idx in range(len(lat_list)):
        if not lat_ahead:
            geometry_list.append([lon_list[idx], lat_list[idx]])
        else:
            geometry_list.append([lat_list[idx], lon_list[idx]])
    return geometry_list


def get_color(val, marker=0.5):
    hex_color_converter = '#%02x%02x%02x'
    color_1 = np.array([0, 255, 0])
    color_2 = np.array([255, 255, 0])
    color_3 = np.array([255, 0, 0])
    if val < 0:
        val = 0
    if val > 1:
        val = 1
    if val <= marker:
        color = val / marker * (color_2 - color_1) + color_1
    else:
        color = (val - marker) / (1 - marker) * (color_3 - color_2) + color_2
    return hex_color_converter % tuple([int(v) for v in color])
