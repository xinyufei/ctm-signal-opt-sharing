import json
import numpy as np
import matplotlib.pyplot as plt

from tqdm import tqdm
# from mtldp.tools.gps_utils import segment_gps_trace


def generate_ctm_leaflet_json(ctm_simulator, file_name="test_data/ctm_test.json",
                              truncate_steps=None, debug=False):
    """

    input the CTM simulator instance
    color: cell occupation
    output the json file including the CTM model:

    :param ctm_simulator: ctm simulator
    :param file_name: path of the output file
    :param truncate_steps:
    :param debug: debug mode
    :return:
    """
    if truncate_steps is not None:
        total_steps = min(truncate_steps, ctm_simulator.total_steps)
    else:
        total_steps = ctm_simulator.total_steps

    output_dict = {}
    cell_static_tag_list = {}
    cell_geometry_list = {}

    static_information = {}
    for cell_id, cell in ctm_simulator.cells.items():

        cell_static_tag = "<p> cell type: " + cell.type + "<br/>"

        cell_static_tag += "belonged node: " + cell.belonged_node + "<br/>"
        if debug > 1:
            cell_static_tag += "cell id: " + cell.cell_id + "<br/>"
            cell_static_tag += "local cell id: " + str(cell.cell_local_id) + "<br/>"
            cell_static_tag += "upstream connector: "
            for upstream_connector in cell.upstream_connector:
                cell_static_tag += upstream_connector
            cell_static_tag += "<br/>"
            cell_static_tag += "downstream connector: "
            for downstream_connector in cell.downstream_connector:
                cell_static_tag += downstream_connector

            cell_static_tag += "<br/>"
        cell_static_tag_list[cell_id] = cell_static_tag

        geometry = cell.geometry
        lat_list, lon_list = geometry["lat"], geometry["lon"]
        geometry_list = []
        for idx in range(len(lat_list)):
            geometry_list.append([lat_list[idx], lon_list[idx]])
        cell_geometry_list[cell_id] = geometry_list

        if debug:
            static_information[cell_id] = {"geometry": geometry_list, "tag": cell_static_tag}
        else:
            static_information[cell_id] = {"geometry": geometry_list}

    dynamic_information = {"total_steps": total_steps, "cells": {}}
    print("Generating cell transmission model output json file...")
    for time_step in tqdm(range(total_steps)):
        for cell_id, cell in ctm_simulator.cells.items():
            if not (cell_id in dynamic_information["cells"].keys()):
                if debug:
                    dynamic_information["cells"][cell_id] = {"tag": [], "color": [], "gray": []}
                else:
                    dynamic_information["cells"][cell_id] = {"color": [], "gray": []}

            color = get_color(cell.occupation[time_step], 0.3)
            gray_color = get_color_gray(cell.occupation[time_step])
            dynamic_information["cells"][cell_id]["color"].append(color)
            dynamic_information["cells"][cell_id]["gray"].append(gray_color)
            if debug:
                cell_tag = cell_static_tag_list[cell_id] + "number of vehicles: " +\
                           str(np.round(cell.num_vehicles[time_step], 2)) + "<br/>"
                cell_tag += "occupation: " + str(np.round(cell.occupation[time_step], 3)) + "<br/>"
                cell_tag += "</p>"
                dynamic_information["cells"][cell_id]["tag"].append(cell_tag)

    # output_dict["ctm"] = all_output
    output_dict["static"] = static_information
    output_dict["dynamic"] = dynamic_information
    output_dict["bounds"] = [[ctm_simulator.bounds["minlat"], ctm_simulator.bounds["minlon"]],
                             [ctm_simulator.bounds["maxlat"], ctm_simulator.bounds["maxlon"]]]

    # output_text = json.dumps(output_dict, indent=2)
    output_text = json.dumps(output_dict)
    with open(file_name, "w") as temp_file:
        temp_file.write(output_text)


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


def get_color_gray(val):
    hex_color_converter = '#%02x%02x%02x'
    if val < 0:
        gray_color = 255
    if val > 1:
        gray_color = 0
    if 0 <= val <= 1:
        gray_color = (1 - val) * 255
    color = [gray_color] * 3
    return hex_color_converter % tuple([int(v) for v in color])


def draw_vehicles(ctm_simulator, fig_file_name, data_file_name):
    plt.figure()
    total_num_vehicles = np.zeros(ctm_simulator.total_steps)
    for time_step in tqdm(range(ctm_simulator.total_steps)):
        for cell_id, cell in ctm_simulator.cells.items():
            if cell.type == "destination":
                continue
            total_num_vehicles[time_step] += cell.num_vehicles[time_step]
    plt.plot([time_step for time_step in range(ctm_simulator.total_steps)], total_num_vehicles)
    plt.xlabel("Time steps")
    plt.ylabel("Number of vehicles")
    plt.savefig(fig_file_name)
    f = open(data_file_name, "w+")
    for time_step in tqdm(range(ctm_simulator.total_steps)):
        print(total_num_vehicles[time_step], file=f)
