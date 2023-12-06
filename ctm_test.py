import numpy as np
from opt import ADMM_optimizer, Decentralize_optimizer
from mtldp.mtlmap import build_network_from_xml, output_static_geometry_json
from ctm.build_ctm import generate_ctm_simulator_from_network
from ctm.ctm_visualization import generate_ctm_leaflet_json
from apps.evluate_signal import set_signal_to_simulator

if __name__ == '__main__':
    fake_instance = build_network_from_xml("map_data/fake_instance/1-8.osm")
    fake_instance_simulator = generate_ctm_simulator_from_network(
        fake_instance, time_interval=3, time_steps=600)
    output_static_geometry_json(fake_instance, "output/static.json")
    mean = [1100, 1100, 300, 300]
    # mean = [400, 400, 200, 200]
    variance = 1.5
    num_scenario = 1
    fake_instance_simulator.update_fake_demand(mean, variance, num_scenario)
    fake_instance_simulator.update_laneset_flow_turning_ratio(fake_instance)
    fake_instance_simulator.compute_cycle_length()
    output_static_geometry_json(fake_instance, "output/static_with_demand.json")

    fake_instance_simulator.warm_up(100)
    fake_instance_simulator.update_laneset_flow_turning_ratio(fake_instance)
    for node_id, node in fake_instance_simulator.nodes.items():
        if node.type == "signalized":
            fake_instance_simulator.generate_naive_signal_plan(node_id)
    eval_scenario = 1
    fake_instance_simulator.update_fake_demand(mean, variance, eval_scenario)
    fake_instance_simulator.update_turning_ratio(eval_scenario, 0.3)
    fake_instance_simulator.num_scenario = num_scenario
    file_name = 'output/evaluation.log'
    fake_instance_simulator.evaluate(fake_instance, file_name, "output/", eval_scenario)
    generate_ctm_leaflet_json(fake_instance_simulator, "output/evaluate.json", truncate_steps=600)

