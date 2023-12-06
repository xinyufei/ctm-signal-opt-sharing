import os
import argparse
import numpy as np
from opt import ADMM_optimizer, Decentralize_optimizer
from mtldp.mtlmap import build_network_from_xml, output_static_geometry_json
from ctm.build_ctm import generate_ctm_simulator_from_network
from ctm.ctm_visualization import generate_ctm_leaflet_json
from apps.evluate_signal import set_signal_to_simulator

parser = argparse.ArgumentParser(description='parameters')
# Required positional argument
parser.add_argument('--mapfile', type=str, help='file path of osm map file', default="map_data/fake_instance/1-8.osm")
parser.add_argument('--scenario', type=int, help='number of scenarios', default=10)
# Optional positional argument
parser.add_argument('--numadmm', type=int, help='number of iteration in ADMM', default=10)
parser.add_argument('--numbenders', type=int, help='number of iteration in Benders', default=4)
args = parser.parse_args()
if args.mapfile is None:
    raise ValueError("The map file cannot be None.")

fake_instance = build_network_from_xml(args.mapfile)
# you can change time_interval and time_steps here.
fake_instance_simulator = generate_ctm_simulator_from_network(
    fake_instance, time_interval=3, time_steps=600)
output_static_geometry_json(fake_instance, args.mapfile.replace('.osm', '.json'))
mean = [400, 400, 100, 100]
variance = 3
num_scenario = 1
fake_instance_simulator.update_fake_demand(mean, variance, num_scenario)
fake_instance_simulator.update_laneset_flow_turning_ratio(fake_instance)
num = 0
v_c_ratio_list = []
for node_id, node in fake_instance_simulator.nodes.items():
    if node.type == "signalized":
        v_c_ratio_list.append(node.v_c_ratio)
fake_instance_simulator.compute_cycle_length()
for node_id, node in fake_instance_simulator.nodes.items():
    if node.type == "signalized":
        mean_cl = node.cycle_length
fake_instance_simulator.warm_up(100)
group_list = []
global_list = []
for obj_id, obj in fake_instance_simulator.nodes.items():
    if obj.type in ["signalized", "unsignalized"]:
        group_list.append([obj_id])
        global_list.append(obj_id)
num_scenario = args.scenario
fake_instance_simulator.update_fake_demand(mean, variance, num_scenario)
fake_instance_simulator.update_turning_ratio(num_scenario, 0.3)
folder_name = f"log_fake/{args.mapfile.split('/')[-1].strip('.osm')}/bite{args.numbenders}_admmite{args.numadmm}/"
print('log folder', folder_name)
if not os.path.exists(folder_name):
    os.makedirs(folder_name)
fake_instance_optimizer = ADMM_optimizer.ADMMOptimizer(fake_instance_simulator, num_scenario, group_list,
                                                       folder_name, folder_name + "figures/", args.numadmm,
                                                       args.numbenders)
fake_instance_optimizer.run()
fake_instance_optimizer.draw_signal()
fake_instance_simulator = fake_instance_optimizer.retrieve_signal_plan()
# fake_instance_simulator.update_laneset_flow_turning_ratio(fake_instance)
# for node_id, node in fake_instance_simulator.nodes.items():
#     if node.type == "signalized":
#         fake_instance_simulator.generate_naive_signal_plan(node_id)
# do the evaluation
eval_scenario = 100
for xi_eval in range(5):
    fake_instance_simulator.update_fake_demand(mean, variance, eval_scenario, xi_eval * 1000 + 1)
    fake_instance_simulator.update_turning_ratio(eval_scenario, 0.3, xi_eval * 1000 + 1)
    # fake_instance_simulator.run(100)
    fake_instance_simulator.num_scenario = num_scenario
    file_name = folder_name + "evaluate_" + str(xi_eval) + '_scenario_' + str(num_scenario) + '.log'
    fake_instance_simulator.evaluate_warmup(fake_instance, file_name, folder_name, eval_scenario)