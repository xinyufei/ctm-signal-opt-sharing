import os
import numpy as np
import matplotlib.pyplot as plt


def plot_trajectory_split(network, output_folder=None):
    for node_id, node in network.nodes.items():
        if not node.is_intersection():
            continue

        movement_list = node.movement_id_list
        movement_num = len(movement_list)
        rows = int(np.ceil(movement_num / 4))
        height_dict = {1: 4, 2: 6, 3: 9}
        plt.figure(figsize=[10, height_dict[rows]])

        # sort the movement id
        movement_index_list = [network.movements[mid].index for mid in movement_list]
        sequence = np.argsort(movement_index_list)

        for idx in range(len(sequence)):
            movement = network.movements[movement_list[sequence[idx]]]
            trajectories = movement.trajectories

            plt.subplot(rows, 4, idx + 1)
            plt.xlabel("longitude", fontsize=8)
            plt.ylabel("latitude", fontsize=8)
            plt.title(f"Movement {movement.index}", fontsize=10)
            plt.xticks([])
            plt.yticks([])
            # plt.axis("equal")

            for link_id in (node.upstream_links + node.downstream_links):
                link = network.links[link_id]
                plt.plot(link.geometry["lon"], link.geometry["lat"], "b.-")

            for traj_id, trajectory in trajectories.items():
                traj_lats = []
                traj_lons = []

                point_list = trajectory.point_list
                for point in point_list:
                    traj_lats.append(point.latitude)
                    traj_lons.append(point.longitude)

                plt.plot(traj_lons, traj_lats, "k-", alpha=0.5)

        plt.suptitle("Intersection " + node_id, fontsize=12)

        if output_folder is None:
            plt.show()
        else:
            if not os.path.exists(output_folder):
                os.mkdir(output_folder)
            plt.savefig(output_folder + "/" + node_id + ".png", dpi=200)
        plt.close()
