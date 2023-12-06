import gurobipy as gb
import numpy as np


class TestOptimizer(object):
    def __init__(self):
        self.model = None
        self.x = None

    def build_model(self):
        self.model = gb.Model()
        self.x = self.model.addVars(10)
        self.model.setObjective(gb.quicksum(self.x[i] for i in range(10)))

    def add_constr(self, i):
        self.model.addConstr(self.x[0] + self.x[i] >= 1)

    def update_obj(self, val):
        self.model.setObjective(val + gb.quicksum(self.x[i] for i in range(10)))

    def optimize(self):
        self.model.optimize()
        print("optimal value", self.model.objval)
