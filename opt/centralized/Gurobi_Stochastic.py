import numpy as np
import gurobipy as gb
from data import Network
import matplotlib.pyplot as plt
from data_global import *
from warmup import Warmup

T = 600
sample_size = 10
network_data = Network(N_edge, True, sample_size= sample_size, T=T)
n_init_all = Warmup(False)
Demand_ALL = network_data.Demand[0]
for i in range(1,N):
    Demand_ALL = Demand_ALL + [de for de in network_data.Demand[i]]
beta = network_data.beta
num_cycle = network_data.num_cycle
U = network_data.U
Demand = network_data.Demand
alpha = network_data.alpha
num_cycle = int(np.ceil(T/15))
# sample_size = 10
m = gb.Model()
y = [None]*sample_size
n = [None]*sample_size
for xi in range(sample_size):
    y[xi] = m.addVars(len(C_ALL), T, lb=0, vtype=gb.GRB.CONTINUOUS)
    n[xi] = m.addVars(len(C_ALL), T+1, lb=0, vtype=gb.GRB.CONTINUOUS)
z1 = m.addVars(N,4,num_cycle,T, vtype=gb.GRB.BINARY)
z2 = m.addVars(N,4,num_cycle,T, vtype=gb.GRB.BINARY)
# z = m.addVars(4, T, vtype=gb.GRB.BINARY)
e = m.addVars(N,4,num_cycle)
b = m.addVars(N,4, num_cycle)
for i in range(N):
    b[i,0,0].lb = -np.infty
g = m.addVars(N,4, lb = 2, ub = 25)
l  = m.addVars(N)
o = m.addVars(N)
m.update()
for i in range(N):
    for t in range(T):
        m.addConstrs(-U*z1[i,p,cy,t]+epsilon <= t-e[i,p,cy] for p in range(4) for cy in range(num_cycle))
        m.addConstrs(-U*z2[i,p,cy,t] <= b[i,p,cy]-t for p in range(4) for cy in range(num_cycle))
        m.addConstrs(t-e[i,p,cy] <= U*(1-z1[i,p,cy,t]) for p in range(4) for cy in range(num_cycle))
        m.addConstrs(b[i,p,cy]-t <= U*(1-z2[i,p,cy,t]) for p in range(4) for cy in range(num_cycle))
        m.addConstrs(z1[i,p,cy,t]+z2[i,p,cy,t] >= 1 for p in range(4) for cy in range(num_cycle))
        m.addConstrs(gb.quicksum(z1[i,p,cy,t]+z2[i,p,cy,t] for p in range(4)) <= 5 for cy in range(num_cycle))
    m.addConstr(o[i] <= l[i])
    m.addConstrs(b[i,0,cy] == l[i]*cy - o[i] for cy in range(num_cycle))
    m.addConstrs(e[i,0,cy] == b[i,0,cy] + g[i,0] for cy in range(num_cycle))
    m.addConstrs(b[i,1,cy] == e[i,0,cy] for cy in range(num_cycle))
    m.addConstrs(e[i,1,cy] == b[i,1,cy] + g[i,1] for cy in range(num_cycle))
    m.addConstrs(b[i,2,cy] == e[i,1,cy] for cy in range(num_cycle))
    m.addConstrs(e[i,2,cy] == b[i,2,cy] + g[i,2] for cy in range(num_cycle))
    m.addConstrs(b[i,3,cy] == e[i,2,cy] for cy in range(num_cycle))
    m.addConstrs(e[i,3,cy] == b[i,3,cy] + g[i,3] for cy in range(num_cycle))
    m.addConstr(gb.quicksum(g[i,p] for p in range(4)) == l[i])
m.setObjective(1/sample_size * (-gb.quicksum(gb.quicksum(gb.quicksum(n[xi][c,t] for c in D_ALL) for t in range(T)) for xi in range(sample_size))
        - alpha * gb.quicksum(gb.quicksum(gb.quicksum(t* y[xi][c,t] for c in list(set(C_ALL)-set(D_ALL))) for t in range(T)) for xi in range(sample_size))), gb.GRB.MINIMIZE)


# maximize the y for cell transmission model, minimize total number of vehicles in the network
for xi in range(sample_size):
    m.addConstrs(y[xi][c,t] <= gb.quicksum(z1[i,p,cy,t]+z2[i,p,cy,t]-1 for cy in range(num_cycle))*Q_ALL[c] for i in range(N) for p in range(4) for c in I[i][p] for t in range(T))
    m.addConstrs(y[xi][c,t]-n[xi][c,t]<=0 for c in C_ALL for t in range(T))
    m.addConstrs(y[xi][c+add[i],t]<=Q[i][0] for i in range(N) for c in list(set(C[i])-set(I1[i])-set(I2[i])-set(I3[i])-set(I4[i])-set(D[i])) for t in range(T))
    m.addConstrs(y[xi][c+add[i],t]<=Q[i][c+1]/beta[i][xi][t][0,0] for c in V[i] for t in range(T))
    m.addConstrs(y[xi][c+add[i],t]<=Q[i][c+2]/beta[i][xi][t][1,0] for c in V[i] for t in range(T))
    m.addConstrs(y[xi][c+add[i],t]<=Q[i][c+3]/beta[i][xi][t][2,0] for c in V[i] for t in range(T))
    m.addConstrs(y[xi][c+add[i],t]+W*n[xi][proc_all[c+add[i]],t]<=W*Jam_N_ALL[int(proc_all[int(c+add[i])])] for i in range(N) for c in list(set(C[i])-set(D[i])-set(V[i])) for t in range(T))
    m.addConstrs(beta[i][xi][t][0,0]*y[xi][c+add[i],t]+W*n[xi][c+add[i]+1,t]<=W*Jam_N_ALL[int(c+add[i]+1)] for c in V[i] for t in range(T))
    m.addConstrs(beta[i][xi][t][1,0]*y[xi][c+add[i],t]+W*n[xi][c+add[i]+2,t]<=W*Jam_N_ALL[int(c+add[i]+2)] for c in V[i] for t in range(T))
    m.addConstrs(beta[i][xi][t][2,0]*y[xi][c+add[i],t]+W*n[xi][c+add[i]+3,t]<=W*Jam_N_ALL[int(c+add[i]+3)] for c in V[i] for t in range(T))
    m.addConstrs(n[xi][c+add[i],t+1] - n[xi][c+add[i],t] - beta[i][xi][t][c-pred[i][c]-1,0]*y[xi][pred[i][c]+add[i],t] + y[xi][c+add[i],t] == 0 for c in I1[i]+I2[i]+I3[i]+I4[i] for t in range(T))
    m.addConstrs(n[xi][c,t+1] - n[xi][c,t] - y[xi][pred_all[c],t] + y[xi][c,t] == 0 for c in list(set(C_ALL)-set(O_ALL)-set(I1_ALL)-set(I2_ALL)-set(I3_ALL)-set(I4_ALL)-set(M_ALL)) for t in range(T))
    m.addConstrs(n[xi][c,t+1] - n[xi][c,t] - gb.quicksum(y[xi][d,t] for d in pred_all[c]) + y[xi][c,t] == 0 for c in M_ALL for t in range(T))
    m.addConstrs(n[xi][int(O[i][c]+add[i]),t+1] - n[xi][int(O[i][c]+add[i]),t] + y[xi][int(O[i][c]+add[i]),t] == Demand[i][c][xi][t] for i in range(N) for c in range(len(O[i])) for t in range(T))
    m.addConstrs((n[xi][c,0] == n_init_all[c] for c in C_ALL), name = 'n0')
    m.addConstrs(y[xi][c,t] == 0 for c in D_ALL for t in range(T))
# m.addConstrs(n[xi][c,T]==0 for c in C_ALL)
# m.addConstrs(n[xi][c,0] == 1 for c in C_ALL)
m.Params.TimeLimit=7200*2
m.Params.LogFile = 'log/MIP_Grid_Gurobi_T' + str(T) + 'size' + str(N_edge[0]) + '.log'
m.Params.LogToConsole = 1
m.update()
m.optimize()