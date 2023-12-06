import csv
import numpy as np
from ..mtlmap.flow_classes import OdPair, Path
from ..mtlmap.map_modes import GraphMode


def read_od_pair(file_name):
    """
    read od pairs from csv file
    """
    od_pairs = {}
    with open(file_name, mode='r') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        for row in csv_reader:
            od_pair = OdPair()
            od_pair.build_od_pair(row['origin'], row['destination'], float(row['flow']))
            od_pairs[od_pair.OD_id] = od_pair
    return od_pairs


def solve_user_equilibrium(network, od_pairs, epsilon=0.001):
    """
    Solve the user equilibrium for the network using Frank-Wolfe method.
    See example at: https://xingminw.github.io/mtldp/build/html/example/ue.html
    
    :param network:
    :param od_pairs:
    :param epsilon:
    :return:
    """
    network.build_networkx_graph(GraphMode.LANESET)

    # od_pairs = read_od_pair(od_flow_file)
    # initialize laneset attributes
    for laneset_id, laneset in network.lanesets.items():
        laneset.laneset_flow = 0
        laneset.laneset_previous_flow = 0
        laneset.belonged_path = []

    # initiate path flow
    initialization_flag = False
    while True:
        paths = {}
        # update laneset travel time
        for laneset_id, laneset in network.lanesets.items():
            laneset.compute_travel_time()

        # find search direction
        # initialize the path information of lanesets
        for laneset_id, laneset in network.lanesets.items():
            laneset.belonged_path = []

        # find the shortest path according the new travel time
        for OD_pair_id, OD_pair in od_pairs.items():
            shortest_path_dict = \
                network.shortest_path_between_nodes(OD_pair.origin, OD_pair.destination, 'travel_time')
            path = Path()
            path.build_path(OD_pair_id, len(OD_pair), shortest_path_dict['edges'])
            new_path_id = OD_pair.add_path(path)
            paths[new_path_id] = path
            for laneset_id in shortest_path_dict['edges']:
                network.lanesets[laneset_id].belonged_path.append(new_path_id)
            # paths[new_path_id].flow = OD_pair.flow_rate

        # update laneset flow
        for laneset_id, laneset in network.lanesets.items():
            laneset.update_previous_flow()
            laneset.update_current_flow(sum(od_pairs[paths[path_id].belonged_OD].flow_rate
                                            for path_id in laneset.belonged_path))

        if not initialization_flag:
            # set path flow to od flow
            for od_id, od_pair in od_pairs.items():
                for path_id, path in od_pair.path_dict.items():
                    path.flow = od_pairs[paths[path_id].belonged_OD].flow_rate
            initialization_flag = True
            continue

        # find step by golden search
        # reference: https://en.wikipedia.org/wiki/Golden-section_search
        gr = (np.sqrt(5) - 1) / 2
        terminate_error = 0.000001
        lb, ub = 0, 1
        temp1 = ub - gr * (ub - lb)
        temp2 = lb + gr * (ub - lb)
        while True:
            f_lb = sum(laneset.compute_cumulative_travel_time(temp1) for laneset in network.lanesets.values())
            f_ub = sum(laneset.compute_cumulative_travel_time(temp2) for laneset in network.lanesets.values())
            if f_lb - f_ub < 0:
                ub = temp2
            else:
                lb = temp1
            temp1 = ub - gr * (ub - lb)
            temp2 = lb + gr * (ub - lb)
            if abs(f_lb - f_ub) < terminate_error:
                break
        proportion_ans = (temp1 + temp2) / 2

        # update laneset flow
        for laneset_id, laneset in network.lanesets.items():
            flow = proportion_ans * (laneset.laneset_flow - laneset.laneset_previous_flow) + \
                   laneset.laneset_previous_flow
            laneset.update_current_flow(flow)

        # update path flow and path travel time
        for od_id, od_pair in od_pairs.items():
            for path_id, path in od_pair.path_dict.items():
                if path.path_id in paths.keys():
                    path.flow = path.flow + \
                                (od_pairs[paths[path_id].belonged_OD].flow_rate - path.flow) * proportion_ans
                else:
                    path.flow = path.flow + (0 - path.flow) * proportion_ans
                travel_time = sum(network.lanesets[laneset_id].travel_time for laneset_id in path.edge_list)
                path.travel_time = travel_time

        # compute terminate criterion
        terminate = np.sqrt(sum(np.square(laneset.laneset_flow - laneset.laneset_previous_flow)
                                for (lanset_id, laneset) in network.lanesets.items())) / sum(
            laneset.laneset_previous_flow for (lanset_id, laneset) in network.lanesets.items())

        if terminate < epsilon:
            break
    return od_pairs
