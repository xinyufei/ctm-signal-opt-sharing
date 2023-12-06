import numpy as np
import matplotlib.pyplot as plt


def retrieve_signal_by_file(file_name):
    file = open(file_name, 'r')
    Lines = file.readlines()
    dataset = []
    for line in Lines:
        line = line.strip('\n')
        line = line.split(' ')
        if line[0] != '':
            dataset.append(line)
            print(line)

    z_1 = {}
    z_2 = {}
    line_id = 0
    while line_id < len(dataset):
        cycle_length = len(dataset[line_id + 3][0].split(';')) - 1
        print(dataset[line_id + 3][0].split(';'))
        node_id = dataset[line_id][1]
        z_1[node_id] = np.zeros((4, 2, cycle_length))
        z_2[node_id] = np.zeros((4, 2, cycle_length))
        for p in range(4):
            for cy in range(2):
                line = dataset[line_id + p * 5 + cy * 2 + 3][0].split(';')
                for t in range(cycle_length):
                    line_t = line[t].split(',')
                    z_1[node_id][p, cy, t] = line_t[0]
                    z_2[node_id][p, cy, t] = line_t[1]
        line_id += 21
    return z_1, z_2


def set_signal_to_simulator(file_name, simulator):
    z_1, z_2 = retrieve_signal_by_file(file_name)
    for node_id, node in simulator.nodes.items():
        if node.type != "signalized":
            continue
        node.cycle_length = z_1[node_id].shape[2]
        for phase_id, phase in node.timing_plan.phases.items():
            phase.signal_state = [0] * simulator.total_steps
            for time_step in range(simulator.total_steps):
                t = int(time_step - np.floor(time_step / node.cycle_length) * node.cycle_length)
                green = sum(z_1[node_id][int(phase_id) - 1, cy, t] + z_2[node_id][int(phase_id) - 1, cy, t] - 1
                            for cy in range(2))
                if abs(green) > 0:
                    phase.signal_state[time_step] = 1
    return simulator


if __name__ == '__main__':
    exit()