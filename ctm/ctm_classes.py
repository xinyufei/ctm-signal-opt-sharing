"""
Class file for CTM
"""

# from map import net_classes as net
from mtldp.mtlmap import nodes_classes as nd
from mtldp.mtlmap import Link, LaneSet
from mtldp.mtlmap.conflicts import ConflictPoint


class CTMLink(Link):
    """
    a link in CTM is defined as a series of ordered cells
    .. math::
        \\alpha + \\beta = 1
    """

    def __init__(self):
        super().__init__()
        self.link_max_vehicles = 0
        self.link_free_travel_time = 0
        self.cumulative_vehicles_in = None
        self.cumulative_vehicles_out = None
        self.cumulative_vehicles_free = None

    @classmethod
    def init_from_link(cls, link):
        """
        Initialize a CTM link from a link in network data.
        :param link:
        :return:
        """
        ctm_link = cls()
        ctm_link.link_max_vehicles = 0
        ctm_link.link_free_travel_time = 0
        ctm_link.cumulative_vehicles_in = None
        ctm_link.cumulative_vehicles_out = None
        ctm_link.cumulative_vehicles_free = None

        for k, v in link.__dict__.items():
            setattr(ctm_link, k, v)
        return ctm_link


class CTMLaneset(LaneSet):
    """
    laneset with index of cells and connectors
    """

    def __init__(self):
        super().__init__()

    @classmethod
    def init_from_laneset(cls, laneset):
        ctm_laneset = cls()
        ctm_laneset.cell_ids = []
        ctm_laneset.connector_ids = []
        ctm_laneset.num_cells = 0

        for k, v in laneset.__dict__.items():
            setattr(ctm_laneset, k, v)
        return ctm_laneset


class CTMSegmentConnectionNode(nd.SegmentConnectionNode):
    """
    Node connecting laneset
    """

    def __init__(self):
        super().__init__()

    @classmethod
    def init_from_node(cls, node):
        ctm_node = cls()

        for k, v in node.__dict__.items():
            setattr(ctm_node, k, v)
        return ctm_node


class CTMEndNode(nd.EndNode):
    """
    Node of origin and destination
    """

    def __init__(self):
        super().__init__()

    @classmethod
    def init_from_node(cls, node):
        ctm_node = cls()

        for k, v in node.__dict__.items():
            setattr(ctm_node, k, v)
        return ctm_node


class CTMUnsignalizedNode(nd.UnSignalizedNode):
    """
    Node without signal
    """

    def __init__(self):
        super().__init__()
        self.node_global_id = None
        self.group_id = None
        self.outflow_connector_list = []  # store the index of connectors of outflow
        self.inflow_connector_list = []
        self.destination_cell_list = []
        self.inflow_cell_list = []
        self.cells_list = []  # store the index of cells and connectors
        self.connectors_list = []
        self.conflict_points_list = []
        self.num_flow = 0
        self.num_inflow = 0
        self.num_outflow = 0

    @classmethod
    def init_from_node(cls, node):
        ctm_node = cls()
        ctm_node.node_global_id = None
        ctm_node.group_id = None
        ctm_node.outflow_connector_list = []
        ctm_node.inflow_connector_list = []
        ctm_node.inflow_cell_list = []
        ctm_node.destination_cell_list = []
        ctm_node.cells_list = []
        ctm_node.connectors_list = []
        ctm_node.conflict_points_list = []
        ctm_node.num_flow = 0
        ctm_node.num_inflow = 0
        ctm_node.num_outflow = 0

        for k, v in node.__dict__.items():
            setattr(ctm_node, k, v)
        return ctm_node


class CTMSignalizedNode(nd.SignalizedNode):
    """
    Node with signal
    """

    def __init__(self):
        super().__init__()
        self.node_global_id = None
        self.node_signalized_id = None
        self.group_id = None
        self.outflow_connector_list = []
        self.inflow_connector_list = []
        self.signalized_connector_list = []
        self.inflow_cell_list = []
        self.destination_cell_list = []
        self.cells_list = []
        self.connectors_list = []
        self.conflict_points_list = []
        self.num_flow = 0
        self.num_inflow = 0
        self.num_outflow = 0
        self.cycle_length = 60
        # self.cycle_length = None

    @classmethod
    def init_from_node(cls, node):
        ctm_node = cls()
        ctm_node.node_global_id = None
        ctm_node.node_signalized_id = None
        ctm_node.group_id = None
        ctm_node.outflow_connector_list = []
        ctm_node.inflow_connector_list = []
        ctm_node.inflow_cell_list = []
        ctm_node.signalized_connector_list = []
        ctm_node.destination_cell_list = []
        ctm_node.cells_list = []
        ctm_node.connectors_list = []
        ctm_node.conflict_points_list = []
        ctm_node.num_flow = 0
        ctm_node.num_inflow = 0
        ctm_node.num_outflow = 0

        for k, v in node.__dict__.items():
            setattr(ctm_node, k, v)

        ctm_node.cycle_length = 15 * len(ctm_node.timing_plan.phases)
        return ctm_node

    def generate_fake_signal(self, total_step):
        num_phases = len(self.timing_plan.phases)
        green_phase_length = 15
        cycle_length = green_phase_length * num_phases
        num_cycle = int(green_phase_length * num_phases + 1)
        idx = 0
        for phase_id, phase in self.timing_plan.phases.items():
            phase.signal_state = [0] * total_step
            for time_step in range(green_phase_length):
                for j in range(num_cycle):
                    current_time_step = time_step + j * cycle_length + idx * green_phase_length
                    if current_time_step < total_step:
                        phase.signal_state[current_time_step] = 1
            idx += 1


class CTMConnector(object):
    """
    a connector is associated with a specific flow between two adjacent cells
    """

    def __init__(self, connector_id):
        self.connector_id = connector_id
        self.network_connector_id = None
        self.type = None
        self.flow = 0
        self.belonged_node = None
        self.optimize_id = None
        self.global_optimize_id = None
        self.upstream_node = None
        self.downstream_node = None
        self.inflow_optimize_id = []
        self.outflow_optimize_id = []

        self.priority_class = 0
        self.conflict_points = []

        # self.controlled_node = None
        self.signalized = False         # the flow is controlled by signal or not
        self.phase_id = None            # if the flow is controlled by signal, it indicates the controlled phase


class CTMStraightConnector(CTMConnector):
    """
    straight connector
    """

    def __init__(self, upstream_cell_id, downstream_cell_id):
        super().__init__(upstream_cell_id + "," + downstream_cell_id)
        self.type = "straight"
        self.upstream_cell = upstream_cell_id
        self.downstream_cell = [downstream_cell_id]
        self.flow = []


class CTMDivergeConnector(CTMConnector):
    """
    diverge connector
    """

    def __init__(self, upstream_cell_id, downstream_cell_id, diverge_prop):
        super().__init__(upstream_cell_id + "," + "diverge")
        self.type = "diverge"
        self.upstream_cell = upstream_cell_id
        self.downstream_cell = downstream_cell_id
        self.diverge_prop_mean = diverge_prop
        self.diverge_prop = [diverge_prop]
        self.flow = []


class CTMMergeConnector(CTMConnector):
    """
    merge connector
    """

    def __init__(self, upstream_cell_id, downstream_cell_id):
        super().__init__("merge" + "," + downstream_cell_id)
        self.type = "merge"
        self.upstream_cell = upstream_cell_id
        self.downstream_cell = downstream_cell_id
        self.priority_prop = [p for p in range(len(upstream_cell_id))]
        self.flow = []


class CTMCell(object):
    """
    ctm cell
    """

    def __init__(self, cell_id):
        # self.cell_unique_id = None
        self.cell_id = cell_id
        self.cell_local_id = None
        self.cell_global_id = None
        self.cell_inflow_id = None
        self.geometry = {}
        self.type = "ordinary"  # ordinary cells
        [link_id, _, lane_id, _] = cell_id.split("@")
        self.belonged_link = link_id
        self.belonged_node = None
        self.belonged_laneset = lane_id

        self.maximum_veh = None
        self.maximum_flow = None
        self.free_flow_speed = None
        self.jam_density = None
        self.shockwave_speed = None
        self.shockwave_ratio = None

        self.upstream_connector = []
        self.downstream_connector = []

        self.num_vehicles = [0]
        self.occupation = [0]


class CTMOriginCell(CTMCell):
    """
    diverge cell
    """

    def __init__(self, cell_id):
        super().__init__(cell_id)
        self.demand = None
        self.mean_demand = None
        self.type = "origin"  # origin cells
        self.belonged_end_node = None


class CTMDestinationCell(CTMCell):
    def __init__(self, cell_id):
        super().__init__(cell_id)
        self.type = "destination"  # destination cells


class CTMConflictPoint(ConflictPoint):
    def __init__(self):
        super().__init__()
        self.conflict_CTM_connectors = []

    @classmethod
    def init_from_conflict_point(cls, conflict_point):
        ctm_conflict_point = cls()

        for k, v in conflict_point.__dict__.items():
            setattr(ctm_conflict_point, k, v)

        for idx in range(len(conflict_point.conflict_connectors)):
            ctm_conflict_point.conflict_CTM_connectors.append([None] * len(conflict_point.conflict_connectors[idx]))
        return ctm_conflict_point
