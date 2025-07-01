from gurobipy import *
import gurobipy as gb
import vehicles

WCL = gb.Model()
WCL.modelSense = gb.GRB.MAXIMIZE
WCL.setParam('OutputFlag', 0) # this is used to quite the xpress outputs (no printing)

M = ["EV", "IVC"] # class of vehicles
# questi dati li prendiamo a seconda della scalability del problema, seguendo le figure del paper
nodes # lista dei nodi es 1, 2,3 
link # lista dei link es. (1,2),(1,3)
# dizionario delle lunghezze relative
paths # lista dei percorsi
T = # time steps che dipende da scalabilità

x = WCL.addMVar(link, vtype=gb.GRB.BINARY) # 1 if link a has WCL
y = WCL.addMVar(M, paths, vtype=gb.GRB.BINARY) # 1 if path p is feasible for vehicle m
B = WCL.addMVar(link, paths, lb=0.0, ub=vehicles.Bmax) # state of energy, bounded between being positive and max capacity

n = WCL.addMVar(link, M, T, lb=0) # number of vehicles class 𝑚 on link 𝑎 at time 𝑖
u = WCL.addMVar(link, M, T, lb=0) # incoming traffic flow of vehicle M to link 𝑎 at time 𝑖
v = WCL.addMVar(link, M, T, lb=0) # outgoing traffic flow of vehicle M to link 𝑎 at time 𝑖
f = WCL.addMVar(link, link, M, T, lb=0) # upstream traffic of vehicle M at link b, coming from downstream traffic at link a

# Feasibility of path
WCL.addConstr((gb.quicksum(length[a] * x) for a in link) <= budget) # formula 3: budget
# State of energy after travelling on link 𝑎 is no greater than the battery capacity
for p in range(len(paths)):
    # energia iniziale serve rimetterla?? tanto è già da EV. In caso:
    # a0 = (paths[p][0], paths[p][1])
    # WCL.addConstr(B[a0,p] == B0)
    for i in range(1,len(paths[p])):
        b = (paths[p][i-1], paths[p][i])
        a = (paths[p][i], paths[p][i+1]) if i+1<length(paths[p]) else b
        # formula 5: state of energy after traversing link 𝑎 on path 𝑝
        WCL.addConstr(B[a,p] <= B[b,p]  -vehicles.epsilon * length[a] + vehicles.omega * t0[a] * x[a])
    for a in links: # formula 6: feasibility of path. M chosen to be 1000
        WCL.addConstr(B[a,p] >= 1000 * (y["EV"][p]) - 1)
# Exactly one path must be chosen
WCL.addConstr(gb.quicksum(y["EV"][p]) for p in range(len(paths)) == 1)

# Flow capacity
for a in link:
    for t in range(T):
        for m in M:
            # Formula 12: conservation of vehicle numbers
            WCL.addConstr(n[a,m,t] == gb.quicksum(u[a,m,k] - v[a,m,k] for k in range(t+1)))
            # formula 13: upstream capacity
            t_in = max(0, t - int(lenght[a]/vehicles.Va))
            WCL.addConstr(gb.quicksum(u[a,m,k] for k in range(t_in,t+1)) <= n[a,m,t])
            # formula 14: downstream capacity
            t_out = max(0, t - int(lenght[a]/vehicles.Wa))
            WCL.addConstr(n[a,m,t] + gb.quicksum(v[a,m,k] for k in range(t_out,t+1)) <= vehicles.Ka * length[a])
            # formula 16: flux conservation for incoming vehicles
            in_nb = [e for e in link if e[1] == a[0]] # neighbour nodes
            WCL.addConstr(u[a,m,t] == gb.quicksum(f[b,a,m,t] for b in in_nb))
            # formula 17:  flux conservation for outgoing vehicles
            out_nb = [e for e in link if e[0] == a[1]] # neighbour nodes
            WCL.addConstr(v[a,m,t] == gb.quicksum(f[b,a,m,t] for b in out_nb))
# formula 15: incoming flux


WCL.setObjective(gb.quicksum((len(T) + 1 - i) * f[a, b, i, m] for m in M for i in T for a in link_sink for b in link-link_source))
WCL.optimize()