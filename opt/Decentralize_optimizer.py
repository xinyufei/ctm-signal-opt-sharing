import os
import sys
import time
import numpy as np
import gurobipy as gb
import matplotlib.pyplot as plt

from ctm.build_ctm import generate_ctm_simulator_from_network


class DecentralizeOptimizer(object):
    def __init__(self, network_simulator, num_scenario, group_list):

        self.simulator = network_simulator
        self.group_list = group_list
        self.num_groups = len(group_list)

        self.num_scenario = num_scenario
        self.type = "decentralize"

        # update parameters
        self.num_priority = 4
        self.U = 2 * network_simulator.total_steps
        self.epsilon = 0.001
        self.alpha = 0.0001
        self.num_cycle = 2
        self.total_num_ite = 1

        # generate boundary flow for groups
        self.group_inflow = [None] * self.num_groups
        self.group_outflow = [None] * self.num_groups

        # build model list
        self.master_model = [None] * self.num_groups
        self.sub_model = [None] * self.num_scenario
        self.aux_sub_model = [None] * self.num_scenario
        for scenario in range(self.num_scenario):
            self.sub_model[scenario] = [None] * self.num_groups

        # build variables for master problem
        self.z1 = [None] * self.num_groups
        self.z2 = [None] * self.num_groups
        self.b = [None] * self.num_groups
        self.e = [None] * self.num_groups
        self.g = [None] * self.num_groups
        self.o = [None] * self.num_groups
        self.length = [None] * self.num_groups
        self.theta = [None] * self.num_groups

        # build variables for sub problem
        self.y = [None] * self.num_scenario
        self.n = [None] * self.num_scenario
        # self.sc = [None] * self.num_scenario
        for scenario in range(self.num_scenario):
            self.y[scenario] = [None] * self.num_groups
            self.n[scenario] = [None] * self.num_groups

        # values of variables in master problem
        self.z1_tilde = [None] * self.num_groups
        self.z2_tilde = [None] * self.num_groups
        self.b_tilde = [None] * self.num_groups
        self.e_tilde = [None] * self.num_groups
        self.g_tilde = [None] * self.num_groups
        self.o_tilde = [None] * self.num_groups
        self.length_tilde = [None] * self.num_groups
        self.theta_tilde = [None] * self.num_groups

        self.z1_opt = [None] * self.num_groups
        self.z2_opt = [None] * self.num_groups
        self.b_opt = [None] * self.num_groups
        self.e_opt = [None] * self.num_groups
        self.g_opt = [None] * self.num_groups
        self.o_opt = [None] * self.num_groups
        self.length_opt = [None] * self.num_groups
        self.theta_opt = [None] * self.num_groups

        # values of variables in sub problem
        self.y_value = [None] * self.num_scenario
        self.n_value = [None] * self.num_scenario

        # time
        self.master_build = 0
        self.sub_build = 0
        self.start_ite = 0
        self.start_global = 0
        self.start_sub = 0
        self.start_master = 0
        self.end_master = 0
        self.end_sub = 0
        self.end_global = 0

        # auxiliary variables for building model
        self.signalized_node = [None] * self.num_groups  # store the id of signalized nodes of each group
        self.conflict_points_node = [None] * self.num_groups
        self.signal_cons = [None] * self.num_scenario
        self.unsignal_cons = [None] * self.num_scenario
        self.downQ_cons = [None] * self.num_scenario
        self.downN_cons = [None] * self.num_scenario
        self.init_cons = [None] * self.num_scenario
        self.origin_cons = [None] * self.num_scenario
        self.capacity_cons = [None] * self.num_scenario

        self.ub = np.infty
        self.lb = -np.infty
        self.ub_array = []
        self.lb_array = []
        self.obj_array = []
        self.gap_array = []

        self.log_folder = 'log_aa/OP/decentralize' + '/' + str(self.total_num_ite) + '/heuristic'
        self.figure_folder = 'log_aa/OP/decentralize' + '/' + str(self.total_num_ite) + '/heuristic'
        if not os.path.exists(self.log_folder):
            os.makedirs(self.log_folder)
        if not os.path.exists(self.figure_folder):
            os.makedirs(self.figure_folder)

    # def build_group_boundary_connectors(self):
    #     for gid in range(self.num_groups):
    #         group_intersection = self.group_list[gid]
    #         num_inter = len(group_intersection)
    #         for iid in range(num_inter):
    #

    def update_node_group(self):
        for gid in range(len(self.group_list)):
            group_intersection = self.group_list[gid]
            self.signalized_node[gid] = []
            for inter_idx in range(len(group_intersection)):
                inter = self.simulator.nodes[group_intersection[inter_idx]]
                inter.group_id = gid
                if inter.type == "signalized":
                    self.signalized_node[gid].append(group_intersection[inter_idx])
            num_signalized_node = len(self.signalized_node[gid])
            self.z1_tilde[gid] = [None] * num_signalized_node
            self.z2_tilde[gid] = [None] * num_signalized_node
            self.b_tilde[gid] = [None] * num_signalized_node
            self.e_tilde[gid] = [None] * num_signalized_node
            self.g_tilde[gid] = [None] * num_signalized_node
            self.o_tilde[gid] = [None] * num_signalized_node
            self.length_tilde[gid] = [None] * num_signalized_node

            self.z1_opt[gid] = [None] * num_signalized_node
            self.z2_opt[gid] = [None] * num_signalized_node
            self.b_opt[gid] = [None] * num_signalized_node
            self.e_opt[gid] = [None] * num_signalized_node
            self.g_opt[gid] = [None] * num_signalized_node
            self.o_opt[gid] = [None] * num_signalized_node
            self.length_opt[gid] = [None] * num_signalized_node
            for iid in range(num_signalized_node):
                inter = self.simulator.nodes[self.signalized_node[gid][iid]]
                self.g_opt[gid][iid] = np.zeros(4)
                self.z1_opt[gid][iid] = np.zeros((4, self.num_cycle, inter.cycle_length))
                self.z2_opt[gid][iid] = np.zeros((4, self.num_cycle, inter.cycle_length))
                self.b_opt[gid][iid] = np.zeros((4, self.num_cycle))
                self.e_opt[gid][iid] = np.zeros((4, self.num_cycle))

    def build_master_problem(self):
        """
        build master problem model
        """
        for group_idx in range(self.num_groups):
            group_intersection = self.group_list[group_idx]
            num_signalized_node = len(self.signalized_node[group_idx])
            if num_signalized_node == 0:
                continue
            self.master_model[group_idx] = gb.Model()
            self.z1[group_idx] = [None] * num_signalized_node
            self.z2[group_idx] = [None] * num_signalized_node
            self.b[group_idx] = [None] * num_signalized_node
            self.e[group_idx] = [None] * num_signalized_node
            self.g[group_idx] = [None] * num_signalized_node
            self.o[group_idx] = [None] * num_signalized_node
            self.theta[group_idx] = self.master_model[group_idx].addVars(self.num_scenario, lb=0)
            self.master_model[group_idx].setObjective(
                1 / self.num_scenario * gb.quicksum(self.theta[group_idx][xi] for xi in range(self.num_scenario)),
                gb.GRB.MINIMIZE)
            for signalized_idx in range(num_signalized_node):
                inter = self.simulator.nodes[self.signalized_node[group_idx][signalized_idx]]
                self.z1[group_idx][signalized_idx] = self.master_model[group_idx].addVars(4, self.num_cycle,
                                                                                          inter.cycle_length,
                                                                                          vtype=gb.GRB.BINARY)
                self.z2[group_idx][signalized_idx] = self.master_model[group_idx].addVars(4, self.num_cycle,
                                                                                          inter.cycle_length,
                                                                                          vtype=gb.GRB.BINARY)
                self.b[group_idx][signalized_idx] = self.master_model[group_idx].addVars(4, self.num_cycle)
                for p in range(4):
                    self.b[group_idx][signalized_idx][p, 0].lb = -np.infty
                self.e[group_idx][signalized_idx] = self.master_model[group_idx].addVars(4, self.num_cycle)
                self.g[group_idx][signalized_idx] = self.master_model[group_idx].addVars(4, lb=0, ub=25, vtype=gb.GRB.INTEGER)
                self.o[group_idx][signalized_idx] = self.master_model[group_idx].addVar(vtype=gb.GRB.INTEGER)

                for t in range(inter.cycle_length):
                    self.master_model[group_idx].addConstrs(
                        -self.U * self.z1[group_idx][signalized_idx][p, cy, t] + self.epsilon <= t -
                        self.e[group_idx][signalized_idx][p, cy] for p in range(4) for cy in range(2))
                    self.master_model[group_idx].addConstrs(
                        -self.U * self.z2[group_idx][signalized_idx][p, cy, t] <=
                        self.b[group_idx][signalized_idx][p, cy] - t for p in range(4) for cy in range(2))
                    self.master_model[group_idx].addConstrs(
                        t - self.e[group_idx][signalized_idx][p, cy] <= self.U * (
                                1 - self.z1[group_idx][signalized_idx][p, cy, t])
                        for p in range(4) for cy in range(2))
                    self.master_model[group_idx].addConstrs(
                        self.b[group_idx][signalized_idx][p, cy] - t <= self.U * (
                                1 - self.z2[group_idx][signalized_idx][p, cy, t]) - self.epsilon
                        for p in range(4) for cy in range(2))
                    self.master_model[group_idx].addConstrs(self.z1[group_idx][signalized_idx][p, cy, t]
                                                            + self.z2[group_idx][signalized_idx][
                                                                p, cy, t]
                                                            >= 1 for p in range(4) for cy in range(2))
                    self.master_model[group_idx].addConstrs(
                        gb.quicksum(self.z1[group_idx][signalized_idx]
                                    [p, cy, t]
                                    + self.z2[group_idx][signalized_idx][p, cy, t] for p in range(4)) <= 5 for cy in
                        range(2))
                self.master_model[group_idx].addConstr(self.o[group_idx][signalized_idx] <= inter.cycle_length)

                self.master_model[group_idx].addConstrs(self.b[group_idx][signalized_idx][0, cy]
                                                        == inter.cycle_length * cy - self.o[group_idx][
                                                            signalized_idx] for cy in range(2))
                self.master_model[group_idx].addConstrs(
                    self.e[group_idx][signalized_idx][0, cy] == self.b[group_idx][signalized_idx][0, cy] +
                    self.g[group_idx][signalized_idx][0] for cy in range(2))
                self.master_model[group_idx].addConstrs(
                    self.b[group_idx][signalized_idx][1, cy] == self.e[group_idx][signalized_idx][0, cy] for cy in
                    range(2))
                self.master_model[group_idx].addConstrs(
                    self.e[group_idx][signalized_idx][1, cy] == self.b[group_idx][signalized_idx][1, cy] +
                    self.g[group_idx][signalized_idx][1] for cy in range(2))
                self.master_model[group_idx].addConstrs(
                    self.b[group_idx][signalized_idx][2, cy] == self.e[group_idx][signalized_idx][1, cy] for cy in
                    range(2))
                self.master_model[group_idx].addConstrs(
                    self.e[group_idx][signalized_idx][2, cy] == self.b[group_idx][signalized_idx][2, cy] +
                    self.g[group_idx][signalized_idx][2] for cy in range(2))
                self.master_model[group_idx].addConstrs(
                    self.b[group_idx][signalized_idx][3, cy] == self.e[group_idx][signalized_idx][2, cy] for cy in
                    range(2))
                self.master_model[group_idx].addConstrs(
                    self.e[group_idx][signalized_idx][3, cy] == self.b[group_idx][signalized_idx][3, cy] +
                    self.g[group_idx][signalized_idx][3] for cy in range(2))
                self.master_model[group_idx].addConstr(
                    gb.quicksum(self.g[group_idx][signalized_idx][p] for p in range(4)) == inter.cycle_length)
                phase_key_list = [int(phase) - 1 for phase in inter.timing_plan.phases.keys()]
                for p in range(4):
                    if p not in phase_key_list:
                        self.g[group_idx][signalized_idx][p].lb = 0
                        self.master_model[group_idx].addConstr(self.g[group_idx][signalized_idx][p] == 0)
                    else:
                        self.master_model[group_idx].addConstr(
                            self.g[group_idx][signalized_idx][p] >= inter.phase_ratio[p + 1] / inter.v_c_ratio
                            * inter.cycle_length * 0.6)

            self.master_model[group_idx].Params.LogToConsole = 0
            self.master_model[group_idx].Params.TimeLimit = 7200
            self.master_model[group_idx].Params.LogFile = self.log_folder + '/T' + str(self.simulator.total_steps) + \
                                                          '_S' + str(self.num_scenario) + '_master.log'
            self.master_model[group_idx].Params.LazyConstraints = 1
            self.master_model[group_idx].update()

    def build_global_sub_problem(self):
        """
        build sub problem model and solve it globally
        """
        for xi in range(self.num_scenario):
            self.sub_model[xi] = gb.Model()
            self.y[xi] = self.sub_model[xi].addVars(sum(len(connector.global_optimize_id) for connector in
                                                        self.simulator.connectors.values()), self.simulator.total_steps)
            self.n[xi] = self.sub_model[xi].addVars(len(self.simulator.cells), self.simulator.total_steps + 1)
            self.signal_cons[xi] = {}
            self.unsignal_cons[xi] = {}
            self.downQ_cons[xi] = {}
            self.downN_cons[xi] = {}
            self.init_cons[xi] = {}
            self.origin_cons[xi] = {}
            self.capacity_cons[xi] = {}
            for connector_id, connector in self.simulator.connectors.items():
                upstream_cell = self.simulator.cells[connector.upstream_cell]
                downstream_cell_list = connector.downstream_cell
                node = self.simulator.nodes[connector.belonged_node]
                if len(downstream_cell_list) > 1:
                    for cidx in range(len(downstream_cell_list) - 1):
                        self.sub_model[xi].addConstrs(
                            self.y[xi][connector.global_optimize_id[cidx], t] * connector.diverge_prop[xi][cidx + 1] -
                            self.y[xi][connector.global_optimize_id[cidx + 1], t] * connector.diverge_prop[xi][cidx] == 0
                            for t in range(self.simulator.total_steps))
                for cidx in range(len(downstream_cell_list)):
                    downstream_cell = self.simulator.cells[connector.downstream_cell[cidx]]
                    optimize_id = connector.global_optimize_id[cidx]
                    self.sub_model[xi].addConstrs(self.y[xi][optimize_id, t] - self.n[xi][
                        upstream_cell.cell_global_id, t] <= 0 for t in range(self.simulator.total_steps))
                    # if the connector is controlled by signal
                    if connector.signalized:
                        signalized_idx = self.signalized_node[node.group_id].index(node.node_id)
                        self.signal_cons[xi][connector_id + "->" + connector.downstream_cell[cidx]] = None
                    else:
                        self.unsignal_cons[xi][connector_id + "->" + str(optimize_id)] = \
                            self.sub_model[xi].addConstrs(self.y[xi][optimize_id, t] <= upstream_cell.maximum_flow
                                                          for t in range(self.simulator.total_steps))
                    self.downQ_cons[xi][connector_id + "->" + str(optimize_id)] = \
                        self.sub_model[xi].addConstrs(self.y[xi][optimize_id, t] <= downstream_cell.maximum_flow
                                                      for t in range(self.simulator.total_steps))
                    self.downN_cons[xi][connector_id + "->" + str(optimize_id)] = \
                        self.sub_model[xi].addConstrs(self.y[xi][optimize_id, t] - downstream_cell.shockwave_ratio * (
                                downstream_cell.maximum_veh - self.n[xi][downstream_cell.cell_global_id, t]) <= 0
                                                      for t in range(self.simulator.total_steps))

            for cell_id, cell in self.simulator.cells.items():
                self.init_cons[xi][cell_id] = \
                    self.sub_model[xi].addConstr(self.n[xi][cell.cell_global_id, 0] == cell.num_vehicles[0])
                if cell.type == "origin":
                    self.origin_cons[xi][cell_id] = \
                        self.sub_model[xi].addConstrs(self.n[xi][cell.cell_global_id, t + 1] - self.n[xi][
                            cell.cell_global_id, t] + gb.quicksum(gb.quicksum(self.y[xi][opt_id, t] for opt_id in
                                                                              self.simulator.connectors[
                                                                                  connector_id].global_optimize_id) for
                                                                  connector_id in cell.downstream_connector) -
                                                      cell.demand[xi][
                                                          t] == 0 for t in range(self.simulator.total_steps))
                    # fixme: update after getting uncertain demand
                if cell.type == "ordinary":
                    # if the cell is not on bound
                    inflow_opt_id = [self.simulator.connectors[upstream_connector_id].global_optimize_id[
                                         self.simulator.connectors[upstream_connector_id].downstream_cell.index(
                                             cell_id)] for upstream_connector_id in cell.upstream_connector]

                    self.sub_model[xi].addConstrs(
                        self.n[xi][cell.cell_global_id, t + 1] - self.n[xi][
                            cell.cell_global_id, t] + gb.quicksum(gb.quicksum(self.y[xi][opt_id, t] for opt_id in
                                                                              self.simulator.connectors[
                                                                                  connector_id].global_optimize_id)
                                                                  for connector_id in cell.downstream_connector) -
                        gb.quicksum(self.y[xi][inflow_opt_id[tempid], t] for tempid
                                    in range(len(inflow_opt_id))) == 0 for t in range(self.simulator.total_steps))
                if cell.type == "destination":
                    inflow_opt_id = [self.simulator.connectors[upstream_connector_id].global_optimize_id[
                                         self.simulator.connectors[upstream_connector_id].downstream_cell.index(
                                             cell_id)] for upstream_connector_id in cell.upstream_connector]
                    self.sub_model[xi].addConstrs(self.n[xi][cell.cell_global_id, t + 1] - self.n[xi][
                        cell.cell_global_id, t] - gb.quicksum(self.y[xi][inflow_opt_id[tempid], t]
                                                              for tempid in range(len(inflow_opt_id))) == 0
                                                  for t in range(self.simulator.total_steps))

            # add constraints for conflict points, notice that we did not consider when the connector is inflow
            for conflict_point_id, conflict_point in self.simulator.conflict_points.items():
                self.capacity_cons[xi][conflict_point_id] = self.sub_model[xi].addConstrs(
                    gb.quicksum(gb.quicksum(self.y[xi][opt_id, t] for opt_id in
                                            self.simulator.connectors[connector_id[0]].global_optimize_id)
                                for connector_id in conflict_point.conflict_CTM_connectors) <= conflict_point.capacity
                    for t in range(self.simulator.total_steps))

        self.sub_model[xi].Params.LogToConsole = 1
        self.sub_model[xi].Params.LogFile = self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(
            self.num_scenario) + '_scenario' + str(xi) + '_sub.log'
        self.sub_model[xi].update()

    def build_sub_problem_obj(self):
        """
        update objective function of sub problem
        """
        for xi in range(self.num_scenario):
            num_inter = len(self.simulator.intersections_id)
            obj = [None] * num_inter
            for iid in range(num_inter):
                inter = self.simulator.nodes[self.simulator.intersections_id[iid]]
                obj[iid] = 0
                for conflict_point_id in inter.conflict_points_list:
                    conflict_point = self.simulator.conflict_points[conflict_point_id]
                    if conflict_point.priority == "mixed":
                        num_conflict = len(conflict_point.conflict_CTM_connectors)
                        print([self.simulator.connectors[conflict_point.conflict_CTM_connectors[iii][0]].priority_class
                               for iii in range(num_conflict)])
                        priority_time = [None] * self.simulator.total_steps
                        for time_step in range(self.simulator.total_steps):
                            priority_time[time_step] = [(time_step + p) % num_conflict for p in range(num_conflict)]
                        obj[iid] += self.alpha * gb.quicksum(gb.quicksum((self.simulator.total_steps - t) *
                                                                         (priority_time[t][cidx] + 1) * gb.quicksum(
                            self.y[xi][opt_id, t] for opt_id in
                            self.simulator.connectors[
                                conflict_point.conflict_CTM_connectors[cidx][0]].global_optimize_id)
                                                                         for cidx in range(num_conflict)) for t in
                                                             range(self.simulator.total_steps))

                obj[iid] += gb.quicksum(gb.quicksum(-self.n[xi][self.simulator.cells[dcell_id].cell_global_id, t] for
                                                    dcell_id in inter.destination_cell_list)
                                        for t in range(self.simulator.total_steps))
                obj[iid] += (-self.alpha * gb.quicksum(gb.quicksum((self.simulator.total_steps - t) *
                                                                   (self.num_priority - self.simulator.connectors[
                                                                       connector_id].priority_class) * gb.quicksum(
                    self.y[xi][opt_id, t] for opt_id in self.simulator.connectors[connector_id].global_optimize_id) for
                                                                   connector_id in inter.connectors_list) for t in
                                                       range(self.simulator.total_steps)))

            self.sub_model[xi].setObjective(gb.quicksum(obj[iid] for iid in range(num_inter)))
            self.sub_model[xi].update()

    def update_sub_problem_cons(self, xi):
        """
        update signal constraints of sub problem
        """
        # add signal constrains
        for cons_id, cons in self.signal_cons[xi].items():
            if cons:
                self.sub_model[xi].remove(cons)
            [connector_id, downstream_cell_id] = cons_id.split("->")
            connector = self.simulator.connectors[connector_id]
            upstream_cell = self.simulator.cells[connector.upstream_cell]
            optimize_id = connector.global_optimize_id[connector.downstream_cell.index(downstream_cell_id)]
            node = self.simulator.nodes[connector.belonged_node]
            gid = node.group_id
            signalized_idx = self.signalized_node[gid].index(node.node_id)
            p = int(connector.phase_id) - 1
            self.signal_cons[xi][connector_id + "->" + downstream_cell_id] = \
                self.sub_model[xi].addConstrs(
                    self.y[xi][optimize_id, t] <= gb.quicksum(
                        self.z1_tilde[gid][signalized_idx][p, cy, int(t - np.floor(t / node.cycle_length) *
                                                                      node.cycle_length)] +
                        self.z2_tilde[gid][signalized_idx][p, cy, int(t - np.floor(t / node.cycle_length) *
                                                                      node.cycle_length)] - 1 for cy in
                        range(2)) * upstream_cell.maximum_flow for t in range(self.simulator.total_steps))
        self.sub_model[xi].update()

    def add_cut(self, xi, gid):
        group_intersection = self.group_list[gid]
        num_inter = len(group_intersection)
        master_cons = [None] * num_inter
        constant = [None] * num_inter
        for iid in range(num_inter):
            master_cons[iid] = 0
            constant[iid] = 0
            inter = self.simulator.nodes[group_intersection[iid]]
            for connector_id in inter.connectors_list:
                connector = self.simulator.connectors[connector_id]
                node = self.simulator.nodes[connector.belonged_node]
                upstream_cell = self.simulator.cells[connector.upstream_cell]
                for cidx in range(len(connector.downstream_cell)):
                    downstream_cell = self.simulator.cells[connector.downstream_cell[cidx]]
                    optimize_id = connector.global_optimize_id[cidx]
                    if connector.signalized:
                        signalized_idx = self.signalized_node[gid].index(node.node_id)
                        p = int(connector.phase_id) - 1
                        master_cons[iid] += gb.quicksum(
                            gb.quicksum(-self.z1[gid][signalized_idx][p, cy, int(t - np.floor(t / node.cycle_length) *
                                                                                 node.cycle_length)] -
                                        self.z2[gid][signalized_idx][p, cy, int(t - np.floor(t / node.cycle_length) *
                                                                                node.cycle_length)]
                                        for cy in range(self.num_cycle)) * upstream_cell.maximum_flow *
                            self.signal_cons[xi][connector_id + "->" + downstream_cell.cell_id][t].Pi for t in
                            range(self.simulator.total_steps))
                        constant[iid] += gb.quicksum(
                            -self.signal_cons[xi][connector_id + "->" + downstream_cell.cell_id][t].Pi *
                            upstream_cell.maximum_flow * 2 +
                            self.downQ_cons[xi][connector_id + "->" + str(optimize_id)][t].Pi *
                            downstream_cell.maximum_flow + self.downN_cons[xi][
                                connector_id + "->" + str(optimize_id)][t].Pi *
                            downstream_cell.shockwave_ratio * downstream_cell.maximum_veh
                            for t in range(self.simulator.total_steps))
                    else:
                        constant[iid] += gb.quicksum(
                            self.unsignal_cons[xi][connector_id + "->" + str(optimize_id)][t].Pi *
                            upstream_cell.maximum_flow + self.downQ_cons[xi][
                                connector_id + "->" + str(optimize_id)][t].Pi *
                            downstream_cell.maximum_flow + self.downN_cons[xi][
                                connector_id + "->" + str(optimize_id)][t].Pi *
                            downstream_cell.shockwave_ratio * downstream_cell.maximum_veh
                            for t in range(self.simulator.total_steps))

            for cell_id in inter.cells_list:
                cell = self.simulator.cells[cell_id]
                constant[iid] += self.init_cons[xi][cell_id].Pi * cell.num_vehicles[0]
                if cell.type == "origin":  # fixme: update after receiving stochastic demand
                    constant[iid] += gb.quicksum(
                        self.origin_cons[xi][cell_id][t].Pi * cell.demand[xi][t] for t in range(self.simulator.total_steps))

            for conflict_point_id in inter.conflict_points_list:
                conflict_point = self.simulator.conflict_points[conflict_point_id]
                constant[iid] += gb.quicksum(self.capacity_cons[xi][conflict_point_id][t].Pi * conflict_point.capacity
                                             for t in range(self.simulator.total_steps))
        add_cons = self.master_model[gid].addConstr(
            self.theta[gid][xi] >= gb.quicksum(constant[iid] - master_cons[iid] for iid in range(num_inter)))
        add_cons.Lazy = 1

    def get_local_optimal_value(self, xi, gid):
        group_intersection = self.group_list[gid]
        num_inter = len(group_intersection)
        opt_val = [None] * num_inter
        throughput = [None] * num_inter
        for iid in range(num_inter):
            opt_val[iid] = 0
            throughput[iid] = 0
            inter = self.simulator.nodes[group_intersection[iid]]
            for conflict_point_id in inter.conflict_points_list:
                conflict_point = self.simulator.conflict_points[conflict_point_id]
                if conflict_point.priority == "mixed":
                    num_conflict = len(conflict_point.conflict_CTM_connectors)
                    print([self.simulator.connectors[conflict_point.conflict_CTM_connectors[iii][0]].priority_class
                           for iii in range(num_conflict)])
                    priority_time = [None] * self.simulator.total_steps
                    for time_step in range(self.simulator.total_steps):
                        priority_time[time_step] = [(time_step + p) % num_conflict for p in range(num_conflict)]
                    opt_val[iid] += self.alpha * sum(sum((self.simulator.total_steps - t) * (priority_time[t][cidx] + 1)
                                                         * sum(self.y_value[xi][opt_id, t] for opt_id in
                                                               self.simulator.connectors[
                                                                   conflict_point.conflict_CTM_connectors[cidx][
                                                                       0]].global_optimize_id)
                                                         for cidx in range(num_conflict)) for t in
                                                     range(self.simulator.total_steps))

            throughput_iid = sum(sum(-self.n_value[xi][self.simulator.cells[dcell_id].cell_global_id, t] for
                                     dcell_id in inter.destination_cell_list) for t in
                                 range(self.simulator.total_steps))
            throughput[iid] += throughput_iid
            opt_val[iid] += throughput_iid
            opt_val[iid] += (-self.alpha * sum(sum((self.simulator.total_steps - t) *
                                                   (self.num_priority - self.simulator.connectors[
                                                       connector_id].priority_class) * sum(
                self.y_value[xi][opt_id, t] for opt_id in self.simulator.connectors[connector_id].global_optimize_id)
                                                   for connector_id in inter.connectors_list) for t in
                                               range(self.simulator.total_steps)))
        return [sum(opt_val[iid] for iid in range(num_inter)), sum(throughput[iid] for iid in range(num_inter))]

    def benders_step(self, num_ite):
        """
        step function in Benders algorithm
        """
        throughput_sub = np.zeros((self.num_groups, self.num_scenario))
        ctm_sub = np.zeros(self.num_scenario)
        opt_sub = np.zeros((self.num_groups, self.num_scenario))
        ub_sub = np.zeros(self.num_scenario)

        self.start_ite = time.time()
        self.start_master = time.time()
        for gid in range(self.num_groups):
            if self.master_model[gid]:
                self.master_model[gid].optimize()
                if self.master_model[gid].status == 2:
                    for iid in range(len(self.signalized_node[gid])):
                        self.z1_tilde[gid][iid] = self.master_model[gid].getAttr('X', self.z1[gid][iid])
                        self.z2_tilde[gid][iid] = self.master_model[gid].getAttr('X', self.z2[gid][iid])
                        self.g_tilde[gid][iid] = self.master_model[gid].getAttr('X', self.g[gid][iid])
                        self.o_tilde[gid][iid] = self.o[gid][iid].x
                        self.b_tilde[gid][iid] = self.master_model[gid].getAttr('X', self.b[gid][iid])
                        self.e_tilde[gid][iid] = self.master_model[gid].getAttr('X', self.e[gid][iid])
                        self.theta_tilde[gid] = self.master_model[gid].getAttr('X', self.theta[gid])
                        self.length_tilde[gid][iid] = self.simulator.nodes[self.signalized_node[gid][iid]].cycle_length
        if num_ite == 1:
            for gid in range(self.num_groups):
                if self.master_model[gid]:
                    for xi in range(self.num_scenario):
                        self.theta[gid][xi].lb = -np.infty
                        self.theta_tilde[gid][xi] = -np.infty
            master_obj = -np.infty
        if num_ite > 1:
            master_obj = 0
            for gid in range(self.num_groups):
                if self.master_model[gid]:
                    master_obj += self.master_model[gid].objval
        self.end_master = time.time()

        self.start_sub = time.time()
        for xi in range(self.num_scenario):
            self.update_sub_problem_cons(xi)
            self.sub_model[xi].optimize()
            self.n_value[xi] = self.sub_model[xi].getAttr('X', self.n[xi])
            self.y_value[xi] = self.sub_model[xi].getAttr('X', self.y[xi])
            for gid in range(self.num_groups):
                [opt_sub[gid, xi], throughput_sub[gid, xi]] = self.get_local_optimal_value(xi, gid)
            ub_sub[xi] = sum(opt_sub[gid, xi] for gid in range(self.num_groups))
            ctm_sub[xi] = ub_sub[xi] - sum(throughput_sub[gid, xi] for gid in range(self.num_groups))

            for gid in range(self.num_groups):
                if self.master_model[gid]:
                    if self.theta_tilde[gid][xi] < opt_sub[gid, xi]:
                        self.add_cut(xi, gid)
        self.end_sub = time.time()

        if self.lb < master_obj:
            self.lb = master_obj
        if num_ite > 0:
            self.lb_array.append(master_obj)
        if self.ub > sum(ub_sub) / self.num_scenario:
            self.ub = sum(ub_sub) / self.num_scenario
            for gid in range(self.num_groups):
                for iid in range(len(self.signalized_node[gid])):
                    self.o_opt[gid][iid] = self.o_tilde[gid][iid]
                    self.length_opt[gid][iid] = self.length_tilde[gid][iid]
                    inter = self.simulator.nodes[self.signalized_node[gid][iid]]
                    for p in range(4):
                        self.g_opt[gid][iid][p] = self.g_tilde[gid][iid][p]
                        for cy in range(self.num_cycle):
                            self.b_opt[gid][iid][p, cy] = self.b_tilde[gid][iid][p, cy]
                            self.e_opt[gid][iid][p, cy] = self.e_tilde[gid][iid][p, cy]
                            for t in range(inter.cycle_length):
                                self.z1_opt[gid][iid][p, cy, t] = self.z1_tilde[gid][iid][p, cy, t]
                                self.z2_opt[gid][iid][p, cy, t] = self.z2_tilde[gid][iid][p, cy, t]
        self.end_global = time.time()

        self.gap_array.append((self.ub - self.lb) / abs(self.lb))
        self.obj_array.append(sum(ub_sub) / self.num_scenario)
        self.ub_array.append(self.ub)

        f = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                 '_LP_bound_fixed_cycle_length_noadmm.log', 'a+')
        print("iteration " + str(num_ite), file=f)
        print("upper bound is ", self.ub, file=f)
        print("lower bound is ", self.lb, file=f)
        print("gap is ", ((self.ub - self.lb) / abs(self.lb)), file=f)
        print("throughput term is ", (sum(sum(throughput_sub)) / self.num_scenario), file=f)
        print("ctm term is ", (sum(ctm_sub) / self.num_scenario), file=f)
        print("time to solve master problem is ", (self.end_master - self.start_master), file=f)
        print("time to solve sub problem is ", (self.end_sub - self.start_sub), file=f)
        print("time for this iteration is ", (self.end_global - self.start_ite), file=f)
        print("all time to solve problem is ", (self.end_global - self.start_global), file=f)
        f.close()
        f = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                 '_signal_fixed_cycle_length_noadmm.log', 'a+')
        print("iteration " + str(num_ite), file=f)
        for gid in range(self.num_groups):
            for iid in range(len(self.signalized_node[gid])):
                print("intersection", self.signalized_node[gid][iid], file=f)
                for p in range(4):
                    print(self.g_tilde[gid][iid][p], end=",", file=f)
                print("cycle length", self.length_tilde[gid], file=f)
                print("offset", self.o_tilde[gid], file=f)
                print("\n", file=f)
        f.close()

    def run(self):
        """
        run Benders algorithm
        """
        f = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                 '_LP_bound_fixed_cycle_length_noadmm.log', 'w+')
        f.close()
        f = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                 '_signal_fixed_cycle_length_noadmm.log', 'w+')
        f.close()
        f = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                 '_optimal_signal_fixed_cycle_length_noadmm.log', 'w+')
        f.close()

        self.update_node_group()
        self.build_master_problem()
        self.build_global_sub_problem()
        self.build_sub_problem_obj()

        self.start_global = time.time()
        num_ite = 0
        while 1:
            num_ite += 1
            self.benders_step(num_ite)
            if num_ite == self.total_num_ite:
                break

        f = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                 '_optimal_signal_fixed_cycle_length_noadmm.log', 'w+')
        for gid in range(self.num_groups):
            for iid in range(len(self.signalized_node[gid])):
                print("intersection", self.signalized_node[gid][iid], file=f)
                for p in range(4):
                    print(self.g_opt[gid][iid][p], end=",", file=f)
                print("cycle length", self.length_opt[gid], file=f)
                print("offset", self.o_opt[gid], file=f)
                print("start", self.b_opt[gid][iid][0, 0], ",", self.b_opt[gid][iid][1, 0], ",",
                      self.b_opt[gid][iid][2, 0], ",", self.b_opt[gid][iid][3, 0], ",", file=f)
                print("end", self.e_opt[gid][iid][0, 0], ",", self.e_opt[gid][iid][1, 0], ",",
                      self.e_opt[gid][iid][2, 0], ",", self.e_opt[gid][iid][3, 0], ",", file=f)
                print("\n", file=f)

        f = open(self.log_folder + "/T" + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                 '_integer_signal_fixed_cycle_length_no_admm.log', 'w+')
        for gid in range(self.num_groups):
            for iid in range(len(self.signalized_node[gid])):
                print("intersection", self.signalized_node[gid][iid], file=f)
                inter = self.simulator.nodes[self.signalized_node[gid][iid]]
                for p in range(4):
                    print('phase', p, file=f)
                    for cy in range(self.num_cycle):
                        print('cycle', cy, file=f)
                        for t in range(inter.cycle_length):
                            print(self.z1_opt[gid][iid][p, cy, t], end=',', file=f)
                            print(self.z2_opt[gid][iid][p, cy, t], end=';', file=f)
                        print('\n', file=f)
        f.close()

        plt.figure()
        plt.plot(self.obj_array, label='objective value')
        plt.xlabel('iterations')
        plt.ylabel('objective value')
        plt.legend()
        plt.savefig(self.figure_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                    '_obj_fixed_length_noadmm.png')

        plt.figure()
        plt.plot(self.ub_array, label='throughput')
        plt.xlabel('iterations')
        plt.ylabel('upper bound')
        plt.legend()
        plt.savefig(self.figure_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                    '_ub_fixed_length_noadmm.png')

        plt.figure()
        plt.plot(self.lb_array, label='lb')
        plt.xlabel('iterations')
        plt.ylabel('lower bound')
        plt.legend()
        plt.savefig(self.figure_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                    '_lb_fixed_length_noadmm.png')

        f1 = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                  '_obj_fixed_length_noadmm.log', 'w+')
        f2 = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                  '_ub_fixed_length_noadmm.log', 'w+')
        f3 = open(self.log_folder + '/T' + str(self.simulator.total_steps) + '_S' + str(self.num_scenario) +
                  '_lb_fixed_length_noadmm.log', 'w+')
        for num_ite in range(len(self.obj_array)):
            print(str(self.obj_array[num_ite]), file=f1)
        for num_ite in range(len(self.ub_array)):
            print(str(self.ub_array[num_ite]), file=f2)
        for num_ite in range(len(self.lb_array)):
            print(str(self.lb_array[num_ite]), file=f3)

    def retrieve_signal_plan(self):
        """
        retrieve signal plan from optimal solution
        """
        for gid in range(self.num_groups):
            signalized_intersection = self.signalized_node[gid]
            for iid in range(len(signalized_intersection)):
                inter = self.simulator.nodes[signalized_intersection[iid]]
                for phase_id, phase in inter.timing_plan.phases.items():
                    phase.signal_state = [0] * self.simulator.total_steps
                    for time_step in range(self.simulator.total_steps):
                        t = int(time_step - np.floor(time_step / inter.cycle_length) * inter.cycle_length)
                        green = sum(self.z1_opt[gid][iid][int(phase_id) - 1, cy, t] +
                                    self.z2_opt[gid][iid][int(phase_id) - 1, cy, t] - 1 for cy in range(self.num_cycle))
                        if abs(green) > 0:
                            phase.signal_state[time_step] = 1
        return self.simulator

    def draw_signal(self):
        for gid in range(self.num_groups):
            signalized_intersection = self.signalized_node[gid]
            for iid in range(len(signalized_intersection)):
                inter = self.simulator.nodes[signalized_intersection[iid]]
                plt.figure()
                part_1 = np.zeros(4)
                part_2 = np.zeros(4)
                part_3 = np.zeros(4)
                for phase in range(4):
                    if self.b_opt[gid][iid][phase, 0] < 0:
                        if self.e_opt[gid][iid][phase, 0] > 0:
                            part_1[phase] = self.e_opt[gid][iid][phase, 0]
                            part_2[phase] = self.length_opt[gid][iid] - self.e_opt[gid][iid][phase, 0]
                            part_3[phase] = self.g_opt[gid][iid][phase] - self.e_opt[gid][iid][phase, 0]
                            rect_1 = plt.barh(
                                range(phase, phase + 1), part_1[phase], height=0.2, color='green', alpha=0.8)
                            rect_2 = plt.barh(range(phase, phase + 1), part_2[phase], left=part_1[phase], height=0.2,
                                              color='red', alpha=0.8)
                            rect_3 = plt.barh(range(phase, phase + 1), part_3[phase],
                                              left=(part_1[phase] + part_2[phase]),
                                              height=0.2, color='green', alpha=0.8)
                        else:
                            part_1[phase] = self.b_opt[gid][iid][phase, 1]
                            part_2[phase] = self.g_opt[gid][iid][phase]
                            part_3[phase] = self.length_opt[gid][iid] - self.e_opt[gid][iid][phase, 1]
                            rect_1 = plt.barh(
                                range(phase, phase + 1), part_1[phase], height=0.2, color='red', alpha=0.8)
                            rect_2 = plt.barh(range(phase, phase + 1), part_2[phase], left=part_1[phase], height=0.2,
                                              color='green', alpha=0.8)
                            rect_3 = plt.barh(range(phase, phase + 1), part_3[phase],
                                              left=(part_1[phase] + part_2[phase]),
                                              height=0.2, color='red', alpha=0.8)
                    else:
                        part_1[phase] = self.b_opt[gid][iid][phase, 0]
                        part_2[phase] = self.g_opt[gid][iid][phase]
                        part_3[phase] = self.length_opt[gid][iid] - self.e_opt[gid][iid][phase, 0]
                        rect_1 = plt.barh(
                            range(phase, phase + 1), part_1[phase], height=0.2, color='red', alpha=0.8)
                        rect_2 = plt.barh(range(phase, phase + 1), part_2[phase], left=part_1[phase], height=0.2,
                                          color='green', alpha=0.8)
                        rect_3 = plt.barh(range(phase, phase + 1), part_3[phase], left=(part_1[phase] + part_2[phase]),
                                          height=0.2, color='red', alpha=0.8)

                plt.xlim(0, self.length_opt[gid][iid] + 5)
                plt.yticks(range(4), ['phase 1', 'phase 2', 'phase 3', 'phase 4'])
                plt.xlabel("Time steps")
                # plt.ylabel("Phases")
                plt.savefig(self.figure_folder + "/T" + str(self.simulator.total_steps) + '_S' + str(self.num_scenario)
                            + '_signal_' + inter.node_id + '.png')
                plt.close()

