from gurobipy import *
import gurobipy as gb
import numpy as np
import vehicles

def optimisation_model(link_length, paths, T, timestep, scalability, budget):
    # Extract info about the network
    link = list(link_length.keys())
    link_source = link[0] # initial link (it is unique)
    link_sink = link [-1] # final link (it is unique)
    length = list(link_length.values())

    # Create instances for EV and ICV
    ev = vehicles.EV(scalability)
    icv = vehicles.ICV()
    # Actually we do not need the icv instance since it has the same attributes of ev, but for the sake of coherence with inheritance we'll use it

    # Define optimisation model

    WCL = gb.Model()
    WCL.modelSense = gb.GRB.MAXIMIZE
    WCL.setParam('OutputFlag', 0) # for no prints

    M = ["EV", "ICV"] # class of vehicle: Electric or Internal Combustion

    x = WCL.addVars(link, vtype=gb.GRB.BINARY) # = 1 if link a has WCL
    y = WCL.addVars(M, len(paths), vtype=gb.GRB.BINARY) # = 1 if path p is feasible for vehicle M
    B = WCL.addVars(link, len(paths), ub=ev.Bmax) # state of energy, bounded with its max capacity

    ## 3600 di n è la max lumghezza del path quindi poi sistema genericamente
    n = WCL.addVars(link, M, np.arange(0, T + timestep, timestep), lb=0, ub=ev.Ka * 3600) # number of vehicle M on link 𝑎 at time t
    u = WCL.addVars(link, M, np.arange(0, T + timestep, timestep), lb=0, ub=ev.Qa) # incoming traffic flow of vehicle M to link 𝑎 at time t
    v = WCL.addVars(link, M, np.arange(0, T + timestep, timestep), lb=0, ub=ev.Qa) # outgoing traffic flow of vehicle M to link 𝑎 at time t
    f = WCL.addVars(link, link, M, np.arange(0, T + timestep, timestep), lb=0, ub=ev.Qa) # upstream traffic of vehicle M at link b, coming from downstream traffic at link a
    # These variables are already defined with the constraint of non-negativity (formula 24)

    # Feasibility of path

    WCL.addConstr(gb.quicksum(length[a] * x[a] for a in link[1:-1]) <= budget) # formula 3: budget

    # Add new variable for writing formula 5 (minimum between constant and expression)
    G = WCL.addVars(link, len(paths), vtype=gb.GRB.BINARY)
    # State of energy after travelling on link a is no greater than the battery capacity
    for p in range(len(paths)):
        # formula 4: initial state of energy
        a0 = paths[p][0]
        WCL.addConstr(B[a0,p] == ev.B0)
        for i in range(1,len(paths[p])-1):
            b = paths[p][i-1]
            a = paths[p][i]
            # formula 5: state of energy after traversing link 𝑎 on path 𝑝
            # length[a-1] instead of length[a] as in paper, because a starts from 1, but arrays do not
            ## dobbiamo mettere constr == min(0.5, altra expr)
            energy = B[b,p] - ev.epsilon * length[a-1] + ev.omega * length[a-1]/ev.Va * x[a]
            # travel time of link a at velocity Va is t0[a]=length[a]/ev.Va
            WCL.addConstr(B[a,p] <= ev.Bmax)
            WCL.addConstr(B[a,p] <= energy)
            WCL.addConstr(B[a,p] >= ev.Bmax - 1000 * G[a,p])
            WCL.addConstr(B[a,p] >= energy - 1000 * (1-G[a,p]))
        for a in link[1:-1]:
            if a in paths[p]:
                # formula 6: feasibility of path. M can be whatever because if y["EV",p]=1, then B[a,p]>=0, if =0 B[a,p]>=-something
                WCL.addConstr(B[a,p] >= 10 * (y["EV",p] - 1))
    # Exactly one path must be chosen ##### nel ppt diciamo che questo constraint lo abbiamo aggiunto noi, nel paper non c'era
    #WCL.addConstr(gb.quicksum(y["EV",p] for p in range(len(paths))) == 1)

    # Flow capacity

    # Compute Kronecker's delta for formula 19
    deltaKron = {}
    for p in range(len(paths)):
        for a in link:
            deltaKron[p,a] = 1 if a in paths[p] else 0

    # alpha aggregate link-based share factor of vehicle class 𝑚 of formula 14, must be defined as a variable
    # because it depends on u and otherwise it cannot be computed
    ## spiega nel ppt
    alpha = WCL.addVars([link_source], M, np.arange(0, T + timestep, timestep), lb=0.0, ub=1.0)

    # Compute alpha of formula 14: aggregate link-based share factor of vehicle class 𝑚
    WCL.addConstr(gb.quicksum(alpha[link_source,m,t] for m in M for t in np.arange(0, T + timestep, timestep)) == 1)
    alpha_EV = gb.quicksum(u[link_source,"EV",t] for t in np.arange(0, T + timestep, timestep))
    alpha_ICV = gb.quicksum(u[link_source,"ICV",t] for t in np.arange(0, T + timestep, timestep))
    den_alpha = alpha_EV + alpha_ICV
    WCL.addConstr(gb.quicksum(alpha[link_source,"EV",t] for t in np.arange(0, T + timestep, timestep)) * den_alpha == alpha_EV)
    WCL.addConstr(gb.quicksum(alpha[link_source,"ICV",t] for t in np.arange(0, T + timestep, timestep)) * den_alpha == alpha_ICV)

    for a in link:
        for t in np.arange(0, T+timestep, timestep):
            # Formula 12: conservation of vehicle numbers
            WCL.addConstr(n[a,"EV",t] == gb.quicksum(u[a,"EV",k] - v[a,"EV",k] for k in np.arange(t+1)))
            WCL.addConstr(n[a,"ICV",t] == gb.quicksum(u[a,"ICV",k] - v[a,"ICV",k] for k in np.arange(t+1)))
            if a != link[0] and a!= link[-1]:
                # formula 13: upstream capacity
                t_in = max(0, round(t - length[a-1]/ev.Va + 1))
                WCL.addConstr(gb.quicksum(u[a,"EV",k] for k in np.arange(t_in,t+1)) <= n[a,"EV",t])
                WCL.addConstr(gb.quicksum(u[a,"ICV",k] for k in np.arange(t_in,t+1)) <= n[a,"ICV",t])
                # formula 14: downstream capacity
                t_out = max(0, round(t - length[a-1]/ev.Wa + 1))
                WCL.addConstr(n[a,"EV",t] + gb.quicksum(v[a,"EV",k] for k in np.arange(t_out,t+1)) <= ev.Ka * length[a-1] * alpha[link_source,"EV",t])
                WCL.addConstr(n[a,"ICV",t] + gb.quicksum(v[a,"ICV",k] for k in np.arange(t_out,t+1)) <= icv.Ka * length[a-1] * alpha[link_source,"ICV",t])
                # formula 19: flow capacity of EV on links
                WCL.addConstr(gb.quicksum(f[b,a,"EV",t] for b in link[:-1]) <= ev.Qa * gb.quicksum(deltaKron[p,a] * y["EV",p] for p in range(len(paths))))
                WCL.addConstr(gb.quicksum(f[b,a,"ICV",t] for b in link[:-1]) <= icv.Qa * gb.quicksum(deltaKron[p,a] * y["ICV",p] for p in range(len(paths))))
            if a != link[0]: # no source link
                # formula 16: flux conservation for incoming vehicle
                WCL.addConstr(u[a,"EV",t] == gb.quicksum(f[b,a,"EV",t] for b in link[:-1]))
                WCL.addConstr(u[a,"ICV",t] == gb.quicksum(f[b,a,"ICV",t] for b in link[:-1]))
            if a != link[-1]: # no sink link
                # formula 17: flux conservation for outgoing vehicle
                WCL.addConstr(v[a,"EV",t] == gb.quicksum(f[a,b,"EV",t] for b in link[1:]))
                WCL.addConstr(v[a,"ICV",t] == gb.quicksum(f[a,b,"ICV",t] for b in link[1:]))

    for t in np.arange(0, T + timestep, timestep):
        # formula 15: source link constraint is the demand rate of vehicle M at time step t
        WCL.addConstr(u[link_source,"EV",t] == ev.Da)
        WCL.addConstr(u[link_source,"ICV",t] == icv.Da)
        # formula 18: sink link constraint
        WCL.addConstr(v[link_sink,"EV",t] == 0)
        WCL.addConstr(v[link_sink,"ICV",t] == 0)

    # Supply and demand at node
    ## spiega nel ppt
    # Since the minimum between an int and a linear expression of Gurobi cannot be done,
    # define supply and demand as variables with constraint to be smaller them
    S = WCL.addVars(link, np.arange(0, T + timestep, timestep), lb=0)
    D = WCL.addVars(link, np.arange(0, T + timestep, timestep), lb=0)
    for a in link[1:-1]:
        for t in np.arange(0, T+timestep, timestep):
            ######### poi check se supply e demand sum sono uguali per EV e ICV e quindi basta calcolarle una volta
            # formula 20
            inflow_s = gb.quicksum(u[a,m,k] for k in np.arange(0,t) for m in M)
            outflow_s = gb.quicksum(v[a,m,k] for k in np.arange(0, round(t - length[a-1]/ev.Va)) for m in M)
            supply_sum = ev.Ka * length[a-1] + outflow_s - inflow_s
            WCL.addConstr(S[a,t] <= ev.Qa)
            WCL.addConstr(S[a,t] <= supply_sum)
            # formula 22: 
            WCL.addConstr(gb.quicksum(u[a,m,t] for m in M) <= S[a,t])
                                
            # formula 21
            inflow_d = gb.quicksum(v[a,m,k] for k in np.arange(0,t) for m in M)
            outflow_d = gb.quicksum(u[a,m,k] for k in np.arange(0, round(t - length[a-1]/ev.Wa)) for m in M)
            demand_sum = outflow_d - inflow_d
            WCL.addConstr(D[a,t] <= ev.Qa)
            WCL.addConstr(D[a,t] <= demand_sum)
            # formula 23: 
            WCL.addConstr(gb.quicksum(v[a,m,t] for m in M)  <= D[a,t])

    WCL.setObjective(gb.quicksum((len(np.arange(0, T+timestep, timestep)) + 1 - t) * f[b, link[-1], m, t] for m in M for t in np.arange(0, T+timestep, timestep) for b in link[:-1]))
    WCL.optimize()

    if WCL.status == gb.GRB.OPTIMAL or WCL.status == gb.GRB.SUBOPTIMAL:
        for a in link:
            if x[a].x > 0.5:
                print(f"Link {a} ha WCL installato.")
    else:
        print(f"⚠️ Il modello non è stato risolto correttamente. Status: {WCL.status}")

    return WCL, x, B