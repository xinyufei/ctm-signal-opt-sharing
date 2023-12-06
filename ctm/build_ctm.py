import ctm.ctm_simulator as ctm_sim


def generate_ctm_simulator_from_network(network, time_interval=2, time_steps=900):
    ctm_simulator = ctm_sim.CTMSimulator(time_steps, time_interval)

    ctm_simulator = inherit_property_from_network(ctm_simulator, network)
    ctm_simulator.generate_cells(network)
    ctm_simulator.generate_connectors(network)
    return ctm_simulator


def inherit_property_from_network(ctm_simulator, network):
    ctm_simulator.bounds = network.bounds
    ctm_simulator.build_link_list(network)
    ctm_simulator.build_node_list(network)
    ctm_simulator.build_laneset_list(network)
    ctm_simulator.build_conflict_point_list(network)
    return ctm_simulator
