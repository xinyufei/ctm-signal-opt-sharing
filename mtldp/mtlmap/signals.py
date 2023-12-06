from ..tools import constants
from ..tools import logger
from .conflicts import build_node_conflict


class Phase(object):
    """
    A phase is defined as a signal state time series
    """
    def __init__(self):
        self.phase_id = None
        self.movement_list = []
        self.signal_state = []

        self.controlled_lanesets = []
        self.controlled_connectors = []
        self.upstream_segment = None
        self.upstream_link = None


class SignalTimingPlan(object):
    """
    each signal timing plan corresponds to a signalized intersection
    """
    def __init__(self):
        self.node_id = None
        self.type = None
        self.phases = {}
        self.movement_id_list = []

    def add_phases(self, phase):
        self.phases[phase.phase_id] = phase


def generate_intersections_and_conflicts(network):
    """

    :param network:
    :return:
    """
    for node_id, node in network.nodes.items():
        if not node.is_intersection():
            continue
        if node.type == "unsignalized":
            build_node_conflict(network, node_id, node.upstream_segments)
        else:
            east_west_mixed = False
            north_south_mixed = False
            upstream_segments = node.upstream_segments

            overall_movement_list = []

            # get the phase id and mixed condition
            for up_seg in upstream_segments:
                segment = network.segments[up_seg]
                laneset_list = segment.laneset_list
                for laneset_id in laneset_list:
                    laneset = network.lanesets[laneset_id]
                    movement_list = constants.get_movement_id(laneset.from_direction, laneset.turning_direction)
                    overall_movement_list += movement_list

                    if len(movement_list) == 0:
                        logger.map_logger.warning("laneset " + laneset.laneset_id + " movement list not recoginized!")

                    if len(movement_list) >= 2:
                        if 1 in movement_list or 5 in movement_list:
                            east_west_mixed = True
                            phase_id = 1
                        elif 3 in movement_list or 7 in movement_list:
                            north_south_mixed = True
                            phase_id = 3
                        elif 2 in movement_list or 6 in movement_list:
                            phase_id = 2
                        elif 4 in movement_list or 8 in movement_list:
                            phase_id = 4
                        else:
                            phase_id = -1
                    else:
                        if len(movement_list) != 1:
                            logger.map_logger.error("# of movement_list not equal 1")
                            continue
                        movement_id = movement_list[0]
                        if movement_id in [1, 5]:
                            phase_id = 1
                        elif movement_id in [2, 6, 9, 11]:
                            phase_id = 2
                        elif movement_id in [3, 7]:
                            phase_id = 3
                        else:
                            phase_id = 4
                    laneset.movement_list = movement_list
                    laneset.phase_id = phase_id

            # build conflict points accordingly
            if east_west_mixed:
                build_node_conflict(network, node_id, "EW")
            if north_south_mixed:
                build_node_conflict(network, node_id, "NS")

            overall_movement_set = set(overall_movement_list)
            if not ({1, 2}.issubset(overall_movement_set) or {5, 6}.issubset(overall_movement_set)):
                east_west_mixed = True
            if not ({3, 4}.issubset(overall_movement_set) or {7, 8}.issubset(overall_movement_set)):
                north_south_mixed = True

            # build the signal phase and timing data accordingly
            controlled_connectors_by_phase = [[], [], [], []]
            for up_seg in upstream_segments:
                segment = network.segments[up_seg]
                laneset_list = segment.laneset_list
                for laneset_id in laneset_list:
                    laneset = network.lanesets[laneset_id]
                    if laneset.phase_id is None:
                        logger.map_logger.warning("laneset " + laneset_id + " does not have phase id...")
                        continue
                    if laneset.phase_id < 2.5:
                        if east_west_mixed:
                            laneset.phase_id = 1
                    if laneset.phase_id > 2.5:
                        if north_south_mixed:
                            laneset.phase_id = 3

                    connector = network.connectors[laneset.downstream_connector]
                    connector.controlled_node = node_id
                    connector.phase_id = laneset.phase_id
                    controlled_connectors_by_phase[laneset.phase_id - 1].append(connector.connector_id)

            signal_plan = build_traffic_signal(network, node_id, controlled_connectors_by_phase)
            node.timing_plan = signal_plan
            # build traffic signal timing plan
    return network


def build_traffic_signal(network, node_id, controlled_connectors):
    signal_timing = SignalTimingPlan()
    signal_timing.node_id = node_id
    for idx in range(len(controlled_connectors)):
        phase_id = idx + 1
        local_connectors = controlled_connectors[idx]
        phase = Phase()
        phase.phase_id = phase_id
        phase.controlled_connectors = local_connectors

        for connector_id in local_connectors:
            connector = network.connectors[connector_id]
            laneset = network.lanesets[connector.upstream_laneset]
            phase.controlled_lanesets.append(laneset.laneset_id)
            phase.upstream_segment = laneset.belonged_segment
            phase.upstream_link = laneset.belonged_link

        if len(phase.controlled_connectors) > 0:
            signal_timing.add_phases(phase)
    return signal_timing
