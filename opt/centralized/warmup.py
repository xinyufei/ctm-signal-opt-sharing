from data import *
from data_global import *
import gurobipy as gb

def Warmup(decompose = False):
    # define signal plan
    T_step = 30
    T_total = 5*T_step
    w = np.zeros((N, 4, T_total))
    for g in range(5):
        """ w[:,0,g*T_step:g*T_step+16] = 1
        w[:,1,g*T_step+16:g*T_step+92] = 1
        w[:,2,g*T_step+92:g*T_step+100] = 1
        w[:,3,g*T_step+100:g*T_step+120] = 1 """
        w[:,0,g*T_step:g*T_step+4] = 1
        w[:,1,g*T_step+4:g*T_step+23] = 1
        w[:,2,g*T_step+23:g*T_step+25] = 1
        w[:,3,g*T_step+25:g*T_step+30] = 1
    # solve for n
    m_warmup = gb.Model()
    y = m_warmup.addVars(len(C_ALL), T_total, lb=0, vtype=gb.GRB.CONTINUOUS)
    n = m_warmup.addVars(len(C_ALL), T_total+1, lb=0, vtype=gb.GRB.CONTINUOUS)
    m_warmup.setObjective(-gb.quicksum(gb.quicksum((T_total - t) * y[c,t] for c in C_ALL) for t in range(T_total)), gb.GRB.MINIMIZE)
    cons1 = m_warmup.addConstrs(y[c,t]-w[i,j,t]*Q_ALL[c]<=0 for i in range(N) for j in range(4) for c in I[i][j] for t in range(T_total))
    cons2 = m_warmup.addConstrs(y[c,t]-n[c,t]<=0 for c in C_ALL for t in range(T_total))
    cons3 = m_warmup.addConstrs(y[c,t]<=Q[0][0] for c in list(set(C_ALL)-set(I1_ALL)-set(I2_ALL)-set(I3_ALL)-set(I4_ALL)-set(D_ALL)) for t in range(T_total))
    cons4 = [None]*3*N
    for i in range(N):
        cons4[i*3] = m_warmup.addConstrs(y[c+add[i],t]<=Q[i][c+1]/beta[i][0,0] for c in V[i] for t in range(T_total))
        cons4[i*3+1] = m_warmup.addConstrs(y[c+add[i],t]<=Q[i][c+2]/beta[i][1,0] for c in V[i] for t in range(T_total))
        cons4[i*3+2] = m_warmup.addConstrs(y[c+add[i],t]<=Q[i][c+3]/beta[i][2,0] for c in V[i] for t in range(T_total))
    cons5 = m_warmup.addConstrs(y[c,t]+W*n[proc_all[c],t]<=W*Jam_N_ALL[int(proc_all[c])] for c in list(set(C_ALL)-set(D_ALL)-set(V_ALL)) for t in range(T_total))
    cons6 = [None]*4*N
    for i in range(N):
        cons6[i*4] = m_warmup.addConstrs(beta[i][0,0]*y[c+add[i],t]+W*n[c+add[i]+1,t]<=W*Jam_N_ALL[int(c+add[i]+1)] for c in V[i] for t in range(T_total))
        cons6[i*4+1] = m_warmup.addConstrs(beta[i][1,0]*y[c+add[i],t]+W*n[c+add[i]+2,t]<=W*Jam_N_ALL[int(c+add[i]+2)] for c in V[i] for t in range(T_total))
        cons6[i*4+2] = m_warmup.addConstrs(beta[i][2,0]*y[c+add[i],t]+W*n[c+add[i]+3,t]<=W*Jam_N_ALL[int(c+add[i]+3)] for c in V[i] for t in range(T_total))
        cons6[i*4+3] = m_warmup.addConstrs(n[c+add[i],t+1] - n[c+add[i],t] - beta[i][c-pred[i][c]-1,0]*y[pred[i][c]+add[i],t] + y[c+add[i],t] == 0 for c in I1[i]+I2[i]+I3[i]+I4[i] for t in range(T_total))
    cons7 = m_warmup.addConstrs(n[c,t+1] - n[c,t] - y[pred_all[c],t] + y[c,t] == 0 for c in list(set(C_ALL)-set(O_ALL)-set(I1_ALL)-set(I2_ALL)-set(I3_ALL)-set(I4_ALL)-set(M_ALL)) for t in range(T_total))
    cons8 = m_warmup.addConstrs(n[c,t+1] - n[c,t] - gb.quicksum(y[d,t] for d in pred_all[c]) + y[c,t] == 0 for c in M_ALL for t in range(T_total))
    cons9 = m_warmup.addConstrs(n[O_ALL[i],t+1] - n[O_ALL[i],t] + y[O_ALL[i],t] == Demand_ALL[i][0] for i in range(len(O_ALL)) for t in range(T_total))
    cons10 = m_warmup.addConstrs((n[c,0] == n_init_all[c] for c in C_ALL), name = 'n0')
    cons11 = m_warmup.addConstrs(y[c,t] == 0 for c in D_ALL for t in range(T))
    m_warmup.Params.LogToConsole = 1
    m_warmup.Params.TimeLimit=7200
    m_warmup.update()
    m_warmup.optimize()

    # get initialized value of n
    n_tilde = m_warmup.getAttr('X',n)
    n_init_value_all = np.zeros(len(C_ALL))
    for c in range(len(C_ALL)):
        n_init_value_all[c] = n_tilde[c,T_total]
    
    if decompose == False:
        return n_init_value_all
    if decompose == True:
        n_init_value = [None]*N
        n_init_value[0] = n_init_value_all[0:len(C[0])]
        for i in range(N-1):
            n_init_value[i] = n_init_value_all[add[i]:add[i+1]]
        n_init_value[N-1] = n_init_value_all[add[N-1]:]
        return n_init_value



