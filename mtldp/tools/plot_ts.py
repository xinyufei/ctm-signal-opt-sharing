import os
import numpy as np
import matplotlib.pyplot as plt


def plot_times_space(network, output_folder=None):
    for node_id, node in network.nodes.items():
        if not node.is_intersection():
            continue

        movement_list = node.movement_id_list
        movement_num = len(movement_list)
        rows = int(np.ceil(movement_num / 4))
        height_dict = {1: 5, 2: 9, 3: 14}
        plt.figure(figsize=[15, height_dict[rows]])

        # sort the movement id
        movement_index_list = [network.movements[mid].index for mid in movement_list]
        sequence = np.argsort(movement_index_list)

        for idx in range(len(sequence)):
            movement = network.movements[movement_list[sequence[idx]]]
            trajectories = movement.trajectories

            plt.subplot(rows, 4, idx + 1)
            plt.title(f"Movement {movement.index}", fontsize=10)

            for traj_id, trajectory in trajectories.items():
                if "green_start" in trajectory.__dict__.keys():
                    green_start = trajectory.green_start
                    times, distances = trajectory.get_point_attributes("timestamp", "relative_dis")
                    if green_start is None:
                        green_start = times[0]
                    times = [time - green_start for time in times]
                    plt.plot(times, distances, "k-", alpha=0.3)
                else:
                    times, distances = trajectory.get_point_attributes("timestamp", "relative_dis")
                    times = [time - times[0] for time in times]
                    plt.plot(times, distances, "k-", alpha=0.3)

        plt.suptitle("Intersection " + node_id, fontsize=12)

        if output_folder is None:
            plt.show()
        else:
            if not os.path.exists(output_folder):
                os.mkdir(output_folder)
            plt.savefig(output_folder + "/" + node_id + "_ts.png", dpi=200)
        plt.close()
