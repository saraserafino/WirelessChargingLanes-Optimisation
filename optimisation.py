from gurobipy import *
import gurobipy as gb
import vehicles

WCL = gb.Model()
WCL.modelSense = gb.GRB.MAXIMIZE
WCL.setParam('OutputFlag', 0) # this is used to quite the xpress outputs (no printing)

M = ["EV", "IVC"] # class of vehicles
# questi dati li prendiamo a seconda della scalability del problema, seguendo le figure del paper
link # lista dei link
paths # lista dei percorsi
nodes # lista dei nodi
T = # time steps che dipende da scalabilità

x = WCL.addMVar(link, vtype=gb.GRB.BINARY) # 1 if link a has WCL
y = WCL.addMVar(M, paths, vtype=gb.GRB.BINARY) # 1 if path p is feasible for vehicle m
B = WCL.addMVar(link, paths, lb=0.0, ub=vehicles.Bmax) # state of energy, bounded between being positive and max capacity

n = WCL.addMVar(link, M, T, lb=0) # number of vehicles class 𝑚 on link 𝑎 at time 𝑖
u = WCL.addMVar(link, M, T, lb=0) # incoming traffic flow of vehicle M to link 𝑎 at time 𝑖
v = WCL.addMVar(link, M, T, lb=0) # outgoing traffic flow of vehicle M to link 𝑎 at time 𝑖
f = WCL.addMVar(link, link, T, M, lb=0) # upstream traffic of vehicle M at link b, coming from downstream traffic at link a


# Path feasible??? Questo va messo proprio dai constraints
        # constraint 3
    
        # State of energy after travelling on link 𝑎 is no greater than the battery capacity
       # if self.energy >= 0:
       #     self.energy = min(self.energy, self.Bmax) # formula 5
       # else:
       #     print("Path not feasible") # fai in modo che non ci vada proprio
        # Relationship between the path feasibility of an EV and its state of energy on this path
       #     <= self.energy # formula 6

WCL.setObjective(gb.quicksum((len(T) + 1 - i) * f[a, b, i, m] for m in M for i in T for a in link_sink for b in link-link_source))
WCL.optimize()