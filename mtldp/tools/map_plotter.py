"""
visualize the map data
"""

from copy import deepcopy
from math import cos, pi, sqrt

import json
import numpy as np
import networkx as nx
import matplotlib.pyplot as plt
from mtldp.tools import mapping as utils


def networkx_plotter_demonstration(network, output_file=None):
    """
    This is a demonstration that uses the networkx drawing functions

    Since we have built the network model to a networkx graph object, you can use any networkx drawing functions

    Reference

        - NetworkX: https://networkx.org/
        - NetworkX drawing: https://networkx.org/documentation/latest/reference/drawing.html

    :param network: :py:class:`mimap.Network`
    :param output_file: output file, show the figure if None
    """
    plt.figure(figsize=[10, 10])
    graph = network.networkx_graph
    node_pos = nx.get_node_attributes(graph, "pos")
    nx.draw(graph, node_pos)
    plt.tight_layout()
    if output_file is None:
        plt.show()
    else:
        plt.savefig(output_file, dpi=200)
    plt.close()


def general_map_plotter(network, customized_components=None, output_file=None):
    """
    General plotter for the map data and the related components (mainly based on the )

    :param network: :py:class:`mimap.Network`
    :param customized_components: a dictionary contains all the customized components except the see example later
    :param output_file: the location of the output file (display the plot if None)

    **Example**

    Here is an example for the ``customized_components``:

    .. code-block:: python

        {
            "node": [{"node_list": [], "color": "r", "alpha": 0.5, "type": "*", "label": "signalized"}, ...],
            "way": [{"way_list": [], "color": "r", "alpha": 0.5, "label": "XXX"}],
            "trajectories":
                {
                    "veh_id": {"lat": [], "lon": [], "style": {"color": "k", "alpha": 0.5, "line": "--"}
                }
        }


    """
    network_boundary = network.bounds
    legend_flag = False

    # adjust the fig width/height ratio and fig size
    figsize = 11

    figure_height = 1   # any positive scalar works here
    figure_width = figure_height * abs(
        float(network_boundary["maxlon"]) - float(network_boundary["minlon"])) / abs(
        float(network_boundary["maxlat"]) - float(network_boundary["minlat"])) * cos(
        float(network_boundary["minlat"]) / 180 * pi)

    figsize_vec = [figure_width, figure_height]
    figsize_vec_length = sqrt(pow(figure_height, 2) + pow(figure_width, 2))
    scale_up = figsize / figsize_vec_length
    new_figsize_vec = [val * scale_up for val in figsize_vec]

    node_latitudes = []
    node_longitudes = []
    for node_id in network.nodes.keys():
        node = network.nodes[node_id]
        node_latitudes.append(node.latitude)
        node_longitudes.append(node.longitude)

    plt.figure(figsize=new_figsize_vec)
    plt.plot(node_longitudes, node_latitudes, "k.", alpha=0.1)

    for way_id, way in network.ways.items():
        geometry = way.geometry
        plt.plot(geometry["lon"], geometry["lat"], "k.-", alpha=0.5)

    # plot the customized components
    if customized_components is not None:
        if "trajectories" in customized_components.keys():
            legend_flag = True

            for traj_id, trajs in customized_components["trajectories"].items():
                if "style" in trajs.keys():
                    plt.plot(trajs["lon"], trajs["lat"], trajs["style"]["line"],
                             color=trajs["style"]["color"],
                             alpha=trajs["style"]["alpha"])
                else:
                    plt.plot(trajs["lon"], trajs["lat"], ".--", label=traj_id)
                if "mlat" in trajs.keys():
                    plt.plot(trajs["mlon"], trajs["mlat"], "-", color=plt.gca().lines[-1].get_color())

        if "title" in customized_components.keys():
            plt.title(customized_components["title"])
        if "way" in customized_components.keys():
            for way_details in customized_components["way"]:
                way_list = way_details["way_list"]
                if "color" in way_details.keys():
                    color = way_details["color"]
                else:
                    color = "k"
                if "alpha" in way_details.keys():
                    alpha = way_details["alpha"]
                else:
                    alpha = 1.0
                if "label" in way_details.keys():
                    label = way_details["label"]
                else:
                    label = None

                for way_id in way_list:
                    way = network.ways[way_id]
                    geometry = way.geometry
                    if label is not None:
                        legend_flag = True
                        plt.plot(geometry["lon"], geometry["lat"], color=color, alpha=alpha, label=label)
                        label = None
                    else:
                        plt.plot(geometry["lon"], geometry["lat"], color=color, alpha=alpha)
        if "node" in customized_components.keys():
            for node_details in customized_components["node"]:
                node_list = node_details["node_list"]
                if "color" in node_details.keys():
                    color = node_details["color"]
                else:
                    color = "k"
                if "alpha" in node_details.keys():
                    alpha = node_details["alpha"]
                else:
                    alpha = 1.0
                if "type" in node_details.keys():
                    dot_type = node_details["type"]
                else:
                    dot_type = "."
                if "label" in node_details.keys():
                    label = node_details["label"]
                else:
                    label = None

                node_latitudes = []
                node_longitudes = []
                for node_id in node_list:
                    if not (node_id in network.nodes.keys()):
                        continue
                    node = network.nodes[node_id]
                    node_latitudes.append(node.latitude)
                    node_longitudes.append(node.longitude)

                if label is None:
                    plt.plot(node_longitudes, node_latitudes, dot_type, color=color, alpha=alpha)
                else:
                    legend_flag = True
                    plt.plot(node_longitudes, node_latitudes, dot_type, color=color, alpha=alpha, label=label)

    plt.xlabel("Longitude")
    plt.ylabel("Latitude")
    network.reset_bound()
    plt.ylim([float(val) for val in [network.bounds["minlat"], network.bounds["maxlat"]]])
    plt.xlim([float(val) for val in [network.bounds["minlon"], network.bounds["maxlon"]]])
    if legend_flag:
        plt.legend()
    plt.tight_layout()

    if output_file is None:
        plt.show()
    else:
        plt.savefig(output_file, dpi=200)
    plt.close()


def plot_map_and_trajectory_json(network, gps_file, mr_file, output_file=None):
    """
    jlfkjklajglkjazkl
    :param network:
    :param gps_file:
    :param mr_file:
    :param output_file:
    :return:
    """
    with open(gps_file, "r") as temp_file:
        all_lines = temp_file.readlines()
    trajs_dict = {}
    for single_line in all_lines[1:]:
        vehicle_id, lon, lat, timestamp = single_line[:-1].split(";")
        if not (vehicle_id in trajs_dict.keys()):
            trajs_dict[vehicle_id] = {"lat": [], "lon": [], "time": []}
        trajs_dict[vehicle_id]["lat"].append(float(lat))
        trajs_dict[vehicle_id]["lon"].append(float(lon))
        trajs_dict[vehicle_id]["time"].append(float(timestamp))

    matched_trajs_dict = {}
    with open(mr_file, "r") as temp_file:
        all_lines = temp_file.readlines()

    for single_line in all_lines[1:]:
        # print(single_line)
        split_info = single_line[:-1].split(";")
        vehicle_id = split_info[0]
        fmm_geometry = split_info[3]

        if vehicle_id == "0":
            continue
        fmm_geometry = fmm_geometry.split("(")[1].split(")")[0].split(",")
        lat_list = [float(val.split(" ")[1]) for val in fmm_geometry]
        lon_list = [float(val.split(" ")[0]) for val in fmm_geometry]

        matched_trajs_dict[vehicle_id] = deepcopy(trajs_dict[vehicle_id])
        matched_trajs_dict[vehicle_id]["mlat"] = lat_list
        matched_trajs_dict[vehicle_id]["mlon"] = lon_list

    # for traj_id in trajs_dict.keys():
    #     if not (traj_id in matched_trajs_dict.keys()):
    #         matched_trajs_dict[traj_id] = deepcopy(trajs_dict[traj_id])
    #         matched_trajs_dict[traj_id]["style"] = {"color": "k", "alpha": 0.3, "line": "--"}
    general_map_plotter(network, customized_components={"trajectories": matched_trajs_dict}, output_file=output_file)


def output_static_geometry_json(network, output_file=None,
                                lat_ahead=True):
    """
    Input the network, output the json file to display
    file_name: if None, this will return the dict, else save to the file
    lat ahead: latitude ahead or longitude ahead

    :param network:
    :param output_file:
    :param lat_ahead:
    :return:
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
        node_tag += "type: " + node.type

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


def plot_node_differentiation(network, output_file=None):
    """

    :param network:
    :param output_file:
    :return:
    """

    customized_components = \
        {"node": [{"node_list": network.signalized_node_list, "color": "r", "type": "*", "label": "Signalized"},
                  {"node_list": network.unsignalized_node_list, "color": "g", "type": "*", "label": "Unsignalized"},
                  {"node_list": network.end_node_list, "color": "b", "type": "*", "label": "End"}]}
    general_map_plotter(network, customized_components, output_file)


def plot_segment_list_and_node_list(network, node_list, segment_list, output_file=None):
    """

    :param network:
    :param node_list:
    :param segment_list:
    :param output_file:
    :return:
    """
    way_list = []
    for segment_id in segment_list:
        segment = network.segments[segment_id]
        way_list.append(segment.osm_way_id)
    customized_components = \
        {"node": [{"node_list": node_list, "color": "r", "type": "*", "label": "Nodes"}],
         "way": [{"way_list": way_list, "color": "b"}]}
    general_map_plotter(network, customized_components, output_file)


def check_segment_heading(network):
    """
    check the heading of the segment
    :param network:
    :return:
    """
    for segment_id, segment in network.segments.items():
        osm_way_id = segment.osm_way_id
        node_list = segment.node_list

        customized_components = \
            {"node": [{"node_list": [node_list[-1]], "color": "r", "type": "*", "label": "Nodes"}],
             "way": [{"way_list": [osm_way_id], "color": "b"}],
             "title": "heading: " + str(int(segment.heading)) + " | " + str(int(segment.weighted_heading))}
        general_map_plotter(network, customized_components, None)


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
