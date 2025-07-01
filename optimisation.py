from gurobipy import *
import gurobipy as gb
import numpy as np
import vehicles

# Chose some parameters:
scalability = False # True for Braess networks, True for grid networks
budget = 30000

# The following lists are taken with respect to problem scalability
### seguendo le figure del paper in un file apposito scriviamo il dizionario e i paths
######### per ora Braess network
link_length = {1: 0, 2: 900, 3: 1800, 4: 900, 5: 3600, 6: 900, 7: 0}
paths = [(1,2,4,6,7), (1,2,5,7), (1,3,6,7)]
timestep = 0.25 # time steps (in minutes) #####che dipende da scalabilità
T = 60 # maximum time (in minutes)
########## fine Braess cose

# Extract info about the network
link = list(link_length.keys())
link_source = link[0] # initial link (it is unique)
link_sink = link [-1] # final link (it is unique)
length = list(link_length.values())


# Define optimisation model

WCL = gb.Model()
WCL.modelSense = gb.GRB.MAXIMIZE
WCL.setParam('OutputFlag', 0) # for no prints

M = ["EV", "ICV"] # class of vehicles: Electric or Internal Combustion

x = WCL.addVars(link, vtype=gb.GRB.BINARY) # = 1 if link a has WCL
y = WCL.addVars(M, len(paths), vtype=gb.GRB.BINARY) # = 1 if path p is feasible for vehicle M
B = WCL.addVars(link, len(paths), lb=0.0, ub=vehicles.Bmax) # state of energy, bounded between being positive and max capacity

n = WCL.addVars(link, M, np.arange(0, T + timestep, timestep), lb=0) # number of vehicles M on link 𝑎 at time t
u = WCL.addVars(link, M, np.arange(0, T + timestep, timestep), lb=0) # incoming traffic flow of vehicle M to link 𝑎 at time t
v = WCL.addVars(link, M, np.arange(0, T + timestep, timestep), lb=0) # outgoing traffic flow of vehicle M to link 𝑎 at time t
f = WCL.addVars(link, link, M, np.arange(0, T + timestep, timestep), lb=0) # upstream traffic of vehicle M at link b, coming from downstream traffic at link a
# These variables are already defined with the constraint of non-negativity (formula 24)

# Feasibility of path

WCL.addConstr(gb.quicksum(length[a] * x[a+1] for a in range(len(link))) <= budget) # formula 3: budget
# State of energy after travelling on link 𝑎 is no greater than the battery capacity
for p in range(len(paths)):
    #### energia iniziale serve rimetterla?? tanto è già da EV. In caso:
    # a0 = paths[p][0]
    # WCL.addConstr(B[a0,p] == B0)
    for i in range(1,len(paths[p])):
        b = paths[p][i-1]
        a = paths[p][i]
        # formula 5: state of energy after traversing link 𝑎 on path 𝑝
        # travel time of link a at velocity Va is t0[a]=length[a]/vehicles.Va
        WCL.addConstr(B[a,p] <= B[b,p] - vehicles.epsilon * length[a-1] + vehicles.omega * length[a-1]/vehicles.Va * x[a])
    for a in link: # formula 6: feasibility of path where M is arbitrary chosen to be 1000  ###### poi cerchiamo se ha senso
        WCL.addConstr(B[a,p] >= 1000 * (y["EV",p]) - 1)
# Exactly one path must be chosen ##### nel ppt diciamo che questo constraint lo abbiamo aggiunto noi, nel paper non c'era
WCL.addConstr(gb.quicksum(y["EV",p] for p in range(len(paths))) == 1)

# Flow capacity

# Compute Kronecker's delta for formula 19
deltaKron = {}
for p_idx in range(len(paths)):
    for a in link:
        deltaKron[p_idx,a] = 1 if a in paths[p_idx] else 0

for a in link:
    for t in np.arange(0, T+timestep, timestep):
        # Formula 12: conservation of vehicle numbers
        WCL.addConstr(n[a,"EV",t] == gb.quicksum(u[a,"EV",k] - v[a,"EV",k] for k in np.arange(t+1)))
        WCL.addConstr(n[a,"ICV",t] == gb.quicksum(u[a,"ICV",k] - v[a,"ICV",k] for k in np.arange(t+1)))
        # formula 13: upstream capacity
        t_in = max(0, t - int(length[a-1]/vehicles.Va))
        WCL.addConstr(gb.quicksum(u[a,"EV",k] for k in np.arange(t_in,t+1)) <= n[a,"EV",t])
        WCL.addConstr(gb.quicksum(u[a,"ICV",k] for k in np.arange(t_in,t+1)) <= n[a,"ICV",t])
        # formula 14: downstream capacity
        t_out = max(0, t - int(length[a-1]/vehicles.Wa))
        WCL.addConstr(n[a,"EV",t] + gb.quicksum(v[a,"EV",k] for k in range(t_out,t+1)) <= vehicles.Ka * length[a-1])
        WCL.addConstr(n[a,"ICV",t] + gb.quicksum(v[a,"ICV",k] for k in range(t_out,t+1)) <= vehicles.Ka * length[a-1])
        if a != link[0]: # no source link
            # formula 16: flux conservation for incoming vehicles
            WCL.addConstr(u[a,"EV",t] == gb.quicksum(f[b,a,"EV",t] for b in link[1:]))
            WCL.addConstr(u[a,"ICV",t] == gb.quicksum(f[b,a,"ICV",t] for b in link[1:]))
        if a != link[-1]: # no sink link
            # formula 17: flux conservation for outgoing vehicles
            WCL.addConstr(v[a,"EV",t] == gb.quicksum(f[b,a,"EV",t] for b in link[:-1]))
            WCL.addConstr(v[a,"ICV",t] == gb.quicksum(f[b,a,"ICV",t] for b in link[:-1]))
            if a != link[0]: # neither source link so a is normal link
                # formula 19: flow capacity of EV on links
                WCL.addConstr(gb.quicksum(f[b,a,"EV",t] for b in link[1:-1]) <= vehicles.Qa * gb.quicksum(deltaKron[p_idx,a] * y["EV",p_idx] for p_idx in range(len(paths))))
                WCL.addConstr(gb.quicksum(f[b,a,"ICV",t] for b in link[1:-1]) <= vehicles.Qa * gb.quicksum(deltaKron[p_idx,a] * y["ICV",p_idx] for p_idx in range(len(paths))))

# formula 15: source link constraint is the demand rate of vehicle M at time step t
WCL.addConstr(u[a,"EV",t] == vehicles.Da)
WCL.addConstr(u[a,"ICV",t] == vehicles.Da)
# formula 18: sink link constraint
WCL.addConstr(v[a,"EV",t] == 0)
WCL.addConstr(v[a,"ICV",t] == 0)

# Supply and demand at node

# Since the minimum between an int and a linear expression of Gurobi cannot be done,
# define supply and demand as variables with constraint to be smaller them
S = WCL.addVars(link, np.arange(0, T + timestep, timestep), lb=0)
D = WCL.addVars(link, np.arange(0, T + timestep, timestep), lb=0)
for a in link:
    for t in np.arange(0, T+timestep, timestep):
        ######### poi check se supply e demand sum sono uguali per EV e ICV e quindi basta calcolarle una volta
        # formula 20
        inflow_s = gb.quicksum(u[a,"EV",k] for k in np.arange(0,t))
        outflow_s = gb.quicksum(v[a,"EV",k] for k in np.arange(0, t - int(length[a-1]/vehicles.Va)))
        supply_sum = vehicles.Ka * length[a-1] + outflow_s - inflow_s
        WCL.addConstr(S[a,t] <= vehicles.Qa)
        WCL.addConstr(S[a,t] <= supply_sum)
        # formula 22: 
        WCL.addConstr(u[a,"EV",t] <= S[a,t])
        
        # Idem for ICV
        inflow_s = gb.quicksum(u[a,"ICV",k] for k in np.arange(0,t))
        outflow_s = gb.quicksum(v[a,"ICV",k] for k in np.arange(0, t - int(length[a-1]/vehicles.Va)))
        supply_sum = vehicles.Ka * length[a-1] + outflow_s - inflow_s
        WCL.addConstr(S[a,t] <= vehicles.Qa)
        WCL.addConstr(S[a,t] <= supply_sum)
        WCL.addConstr(u[a,"ICV",t] <= S[a,t])
                             
        # formula 21
        inflow_d = gb.quicksum(v[a,"EV",k] for k in np.arange(0,t))
        outflow_d = gb.quicksum(u[a,"EV",k] for k in np.arange(0, t - int(length[a-1]/vehicles.Wa)))
        demand_sum = outflow_d - inflow_d
        WCL.addConstr(D[a,t] <= vehicles.Qa)
        WCL.addConstr(D[a,t] <= demand_sum)
        # formula 23: 
        WCL.addConstr(u[a,"EV",t] <= D[a,t])
        
        # Idem for ICV
        inflow_d = gb.quicksum(v[a,"ICV",k] for k in np.arange(0,t))
        outflow_d = gb.quicksum(u[a,"ICV",k] for k in np.arange(0, t - int(length[a-1]/vehicles.Wa)))
        demand_sum = outflow_d - inflow_d
        WCL.addConstr(D[a,t] <= vehicles.Qa)
        WCL.addConstr(D[a,t] <= demand_sum)
        WCL.addConstr(u[a,"ICV",t] <= D[a,t])


#WCL.setObjective(gb.quicksum((len(np.arange(0, T, timestep)) + 1 - i) * f[a, b, i, m] for m in M for i in T for a in link_sink for b in link[1:]))
WCL.setObjective(gb.quicksum((len(np.arange(0, T+timestep, timestep)) + 1 - t) * f[b, link[-1], m, t] for m in M for t in np.arange(0, T+timestep, timestep) for b in link[1:]))
WCL.optimize()