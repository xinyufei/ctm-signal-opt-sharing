class Movement(object):
    """
     A movement reflects the user perspective and is defined by the user type and the action that is taken (e.g.
     through, right turn, left turn)
    """
    def __init__(self):
        self.index = None
        self.movement_id = None
        self.upstream_link_id = None
        self.downstream_link_id = None
        self.signal_status = None
        self.node_id = None

    def set_basic_info(self, upstream_link_id, downstream_link_id, node_id):
        self.upstream_link_id = upstream_link_id
        self.downstream_link_id = downstream_link_id
        self.node_id = node_id

    def calculate_movement_id(self, upstream_link, downstream_link):
        # fixme: no turn around
        movement_id = -1  # not match
        if upstream_link.from_direction == "N":
            if downstream_link.from_direction == "W":
                movement_id = 3
            elif downstream_link.from_direction == "E":
                movement_id = 12
            elif downstream_link.from_direction == "N":
                movement_id = 8
            else:
                movement_id = -1
        elif upstream_link.from_direction == "W":
            if downstream_link.from_direction == "W":
                movement_id = 6
            elif downstream_link.from_direction == "S":
                movement_id = 1
            elif downstream_link.from_direction == "N":
                movement_id = 11
            else:
                movement_id = -1
        elif upstream_link.from_direction == "S":
            if downstream_link.from_direction == "W":
                movement_id = 10
            elif downstream_link.from_direction == "S":
                movement_id = 4
            elif downstream_link.from_direction == "E":
                movement_id = 7
            else:
                movement_id = -1
        elif upstream_link.from_direction == "E":
            if downstream_link.from_direction == "E":
                movement_id = 2
            elif downstream_link.from_direction == "S":
                movement_id = 9
            elif downstream_link.from_direction == "N":
                movement_id = 5
            else:
                movement_id = -1

        self.index = movement_id
        self.movement_id = str(self.node_id) + "_" + str(movement_id)
        return movement_id
    
    def __str__(self):
        output_string = f"Movement id: {self.movement_id}\n"
        for k, v in output_string.__dict__.items():
            output_string += f"\t{k}: {v}\n"
        output_string = output_string[:-1]
        return output_string


def generate_network_movements(network):
    """
    generate network movements

        Use node information in the network to generate network movements
    :param network:
    :return:
    """
    for node_id, node in network.nodes.items():
        if node.is_intersection():
            for i in range(len(node.upstream_links)):
                for j in range(len(node.downstream_links)):
                    movement = Movement()
                    movement.set_basic_info(node.upstream_links[i], node.downstream_links[j], node.node_id)
                    movement_id = movement.calculate_movement_id(network.links[node.upstream_links[i]],
                                                                 network.links[node.downstream_links[j]])
                    # movement.set_signal_status(test_current_time)
                    if movement_id != -1:
                        node.add_movement(movement.movement_id)
                        network.add_movement(movement)
    return network
