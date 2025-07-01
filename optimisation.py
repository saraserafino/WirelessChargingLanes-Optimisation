from gurobipy import *
import gurobipy as gb
import numpy as np
import vehicles

# Chose some parameters:
scalability = True # for grid networks, otherwise it is Braess network
budget = 30000


WCL = gb.Model()
WCL.modelSense = gb.GRB.MAXIMIZE
WCL.setParam('OutputFlag', 0) # this is used to quite the xpress outputs (no printing)

M = ["EV", "IVC"] # class of vehicles

# The following lists are taken with respect to problem scalability
# seguendo le figure del paper
# in un file apposito scriviamo il dizionario e i paths
link_length = {1: 0, 2: 900, 3: 1800, 4: 900, 5: 3600, 6: 900, 7: 0}
paths = [(1,2,4,6,7), (1,2,5,7), (1,3,6,7)] # lista dei percorsi
timestep = 0.25 # time steps che dipende da scalabilità
T = 60

link = list(link_length.keys())
link_source = link[0]
link_sink = link [-1]
length = list(link_length.values())

x = WCL.addVars(link, vtype=gb.GRB.BINARY) # 1 if link a has WCL
y = WCL.addVars(M, len(paths), vtype=gb.GRB.BINARY) # 1 if path p is feasible for vehicle m
B = WCL.addVars(link, len(paths), lb=0.0, ub=vehicles.Bmax) # state of energy, bounded between being positive and max capacity

n = WCL.addVars(len(link), M, T, lb=0) # number of vehicles M on link 𝑎 at time t
u = WCL.addVars(len(link), M, T, lb=0) # incoming traffic flow of vehicle M to link 𝑎 at time t
v = WCL.addVars(len(link), M, T, lb=0) # outgoing traffic flow of vehicle M to link 𝑎 at time t
f = WCL.addVars(len(link), len(link), M, T, lb=0) # upstream traffic of vehicle M at link b, coming from downstream traffic at link a
# Already defined with the constraint of non-negativity (formula 24)

# Feasibility of path
WCL.addConstr(gb.quicksum(length[a] * x[a+1] for a in range(len(link))) <= budget) # formula 3: budget
# State of energy after travelling on link 𝑎 is no greater than the battery capacity
for p in range(len(paths)):
    # energia iniziale serve rimetterla?? tanto è già da EV. In caso:
    # a0 = (paths[p][0], paths[p][1])
    # WCL.addConstr(B[a0,p] == B0)
    for i in range(1,len(paths[p])):
        b = paths[p][i-1]
        a = paths[p][i] if i+1<length[p] else b
        # formula 5: state of energy after traversing link 𝑎 on path 𝑝
        # t0[a] travel time of link a at velocity Va (=length[a]/vehicles.Va)
        WCL.addConstr(B[a,p] <= B[b,p] -vehicles.epsilon * length[a-1] + vehicles.omega * length[a-1]/vehicles.Va * x[a])
    for a in link: # formula 6: feasibility of path. M chosen to be 1000
        WCL.addConstr(B[a,p] >= 1000 * (y["EV",p]) - 1)
# Exactly one path must be chosen
WCL.addConstr(gb.quicksum(y["EV",p] for p in range(len(paths))) == 1)

# Flow capacity

# Compute Kronecher's delta for constraint 19
deltaKron = {}
for p_idx, path in enumerate(paths):
    # Check if link a is in the path
    arc_list = list(zip(path[:-1], path[1:]))
    for a in link:
        deltaKron[p_idx,a] = 1 if a in arc_list else 0

for a in link:
    for t in np.arange(0, T, timestep):
        # Formula 12: conservation of vehicle numbers
        WCL.addConstr(n[a,"EV",t] == gb.quicksum(u[a,"EV",k] - v[a,"EV",k] for k in np.arange(t+1)))
        WCL.addConstr(n[a,"IVC",t] == gb.quicksum(u[a,"IVC",k] - v[a,"IVC",k] for k in np.arange(t+1)))
        # formula 13: upstream capacity
        t_in = max(0, t - int(length[a]/vehicles.Va))
        WCL.addConstr(gb.quicksum(u[a,m,k] for k in range(t_in,t+1)) <= n[a,m,t])
        # formula 14: downstream capacity
        t_out = max(0, t - int(length[a]/vehicles.Wa))
        WCL.addConstr(n[a,"EV",t] + gb.quicksum(v[a,"EV",k] for k in range(t_out,t+1)) <= vehicles.Ka * length[a])
        WCL.addConstr(n[a,"IVC",t] + gb.quicksum(v[a,"IVC",k] for k in range(t_out,t+1)) <= vehicles.Ka * length[a])
        # formula 16: flux conservation for incoming vehicles
        in_nb = [e for e in link if e[1] == a[0]] # neighbour nodes
        WCL.addConstr(u[a,"EV",t] == gb.quicksum(f[b,a,"EV",t] for b in in_nb))
        WCL.addConstr(u[a,"IVC",t] == gb.quicksum(f[b,a,"IVC",t] for b in in_nb))
        # formula 17:  flux conservation for outgoing vehicles
        out_nb = [e for e in link if e[0] == a[1]] # neighbour nodes
        WCL.addConstr(v[a,"EV",t] == gb.quicksum(f[b,a,"EV",t] for b in out_nb))
        WCL.addConstr(v[a,"IVC",t] == gb.quicksum(f[b,a,"IVC",t] for b in out_nb))

        if a not in link_source:
            # formula 19: flow capacity of EV on links
            WCL.addConstr(gb.quicksum(f[b,a,"EV",t] for b in not(link_source)) <= vehicles.Qa * gb.quicksum(deltaKron[a,p] * y["EV",p] for p in paths))
            WCL.addConstr(gb.quicksum(f[b,a,"IVC",t] for b in not(link_source)) <= vehicles.Qa * gb.quicksum(deltaKron[a,p] * y["IVC",p] for p in paths))
# formula 15: source link constraint is the demand rate of vehicle M at time step t
WCL.addConstr(u[a,"EV",t] == vehicles.Da)
WCL.addConstr(u[a,"IVC",t] == vehicles.Da)
# formula 18: sink link constraint
WCL.addConstr(v[a,"EV",t] == 0)
WCL.addConstr(v[a,"IVC",t] == 0)

# Supply and demand at node
S = {}
D = {}
for a in link:
    for t in range(0, T, timestep):
        # formula 20
        S[a,t] = min(vehicles.Qa, vehicles.Ka * length[a] + gb.quicksum(v[a,m,k] for m in M for k in np.arange(0, t - int(length[a]/vehicles.Wa))) - gb.quicksum(u[a,m,k] for m in M for k in np.arange(0,t)))
        # formula 23: 
        WCL.addConstr(u[a,"EV",t] <= S[a,t])
        WCL.addConstr(u[a,"IVC",t] <= S[a,t])
        # formula 21:
        D[a,t] = min(vehicles.Qa, gb.quicksum(u[a,m,k] for m in M for k in np.arange(0, t - int(length[a]/vehicles.Wa))) - gb.quicksum(v[a,m,k] for m in M for k in np.arange(0,t)))
        # formula 24: 
        WCL.addConstr(u[a,"EV",t] <= D[a,t])
        WCL.addConstr(u[a,"IVC",t] <= D[a,t])

WCL.setObjective(gb.quicksum((len(np.arange(0, T, timestep)) + 1 - i) * f[a, b, i, m] for m in M for i in T for a in link_sink for b in link[1:]))
WCL.optimize()