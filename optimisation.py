import gurobipy as gp
import numpy as np
import vehicles
import networkx as nx # for generating paths from edges without going crazy

def optimisation_model(length, graph, T, timestep, scalability, budget):
    # Extract info about the network
    E = list(length.keys()) # links
    N = int(T / timestep) # cardinality of time given timesteps

    # Construct network topology

    E_R = E[0] # source link (it is unique)
    E_S = E[-1] # sink link (it is unique)
    E_A = E[1:-1] # normal links i.e. no source and no sink
    # Generate paths from the graph
    paths = list(nx.all_simple_paths(graph, source=E_R, target=E_S))
    # Remove source and sink link from paths because they are unique and have no length
    paths = {i: path[1:-1] for i, path in enumerate(paths)}

    # Generate incoming links and outgoing links relative to link a
    incoming_links = {a: list(graph.predecessors(a)) for a in graph.nodes}
    ## ex {1: [], 2: [1], 3: [1], 4: [2], 5: [2], 6: [3, 4], 7: [5, 6]}
    outgoing_links = {a: list(graph.successors(a)) for a in graph.nodes}
    ## ex {1: [2, 3], 2: [4, 5], 3: [6], 4: [6], 5: [7], 6: [7], 7: []}

    # Create instances for EV and ICV
    ev = vehicles.EV(scalability)
    icv = vehicles.ICV()
    # Actually we do not need the icv instance since it has the same attributes of ev, but for the sake of coherence with inheritance we'll use it

    # Define optimisation model and its variables

    model = gp.Model()
    model.setParam('OutputFlag', 0) # for no prints

    M = ["EV", "ICV"] # class of vehicle: Electric or Internal Combustion

    x = model.addVars(E_A, vtype=gp.GRB.BINARY, name="x") # = 1 if link a has WCL
    y = model.addVars(len(paths), vtype=gp.GRB.BINARY, name="y") # = 1 if path p is feasible for vehicle M
    # y does is only for EV models since for ICV all paths are feasible
    B = {(a, p): model.addVar(ub=ev.Bmax, name=f"B({a},{p})") for p in paths for a in paths[p]}  # state of energy, bounded with its max capacity

    # The following variables are already defined with the constraint of non-negativity of formula 24
    n = model.addVars(M, E, range(N+1), lb=0) # number of vehicle M on link 𝑎 at time t
    u = model.addVars(M, E, range(N+1), lb=0) # incoming traffic flow of vehicle M to link 𝑎 at time t
    v = model.addVars(M, E, range(N+1), lb=0) # outgoing traffic flow of vehicle M to link 𝑎 at time t
    f = {}
    for m in M:
        for a in E:
            for b in incoming_links[a]:
                for i in range(N+1): # upstream traffic of vehicle M at link b, coming from downstream traffic at link a
                    f[m, b, a, i] = model.addVar(lb=0.0, name=f"f[{m},{b},{a},{i}]")

    # alpha aggregate link-based share factor of vehicle class 𝑚 of formula 14.
    # Must be defined as a variable because it depends on u, otherwise it cannot be computed
    ## spiega nel ppt!!!!!!
    alpha = model.addVars(E_A, M, range(1, N+1), lb=0.0, ub=1.0, name="alpha")
    for a in E_A:
        for i in range(1, N+1):
            model.addConstr(gp.quicksum(alpha[a, m, i] for m in M) == 1.0)

    # Feasibility of path
                    
    # Formula 3: budget constraint
    model.addConstr(gp.quicksum(length[a] * x[a] for a in E_A) <= budget)

    # State of energy after travelling on link a is no greater than the battery capacity
    Big_M = ev.Bmax + ev.omega * (3600 / ev.Va) # 3600 is the maximum length for every link observed
    for p, link_list in paths.items():
        for idx, a in enumerate(link_list):
            # Formula 4: initial state of energy or previous one
            prevB = ev.B0 if idx == 0 else B[(link_list[idx - 1], p)]
            # Formula 2: state of energy after traversing link 𝑎 on path 𝑝
            charge = ev.omega * length[a] / ev.Va
            # where length[a]/ev.Va=t0[a] travel time of link a at velocity Va
            consume = ev.epsilon * length[a]
            # Formula 5: B[a,p] == min(Bmax, energy)
            model.addConstr(B[(a, p)] <= ev.Bmax)
            model.addConstr(B[(a, p)] <= prevB - consume + charge * x[a] + Big_M * (1 - y[p]))
            model.addConstr(B[(a, p)] >= prevB - consume + charge * x[a] - Big_M * (1 - y[p]))
            # Formula 6: feasibility of path (if feasible, B[a,p]>=0)
            model.addConstr(Big_M * (y[p]-1) <= B[(a, p)])

    # Flow capacity
    
    # Compute travel time for formula 13, 14, 20, 21
    # /timestep so instead of time of travel we have the needed steps for travelling
    travelV = {a: int((length[a]/ev.Va)/timestep + 1e-9) for a in length}
    travelW = {a: int((length[a]/ev.Wa)/timestep + 1e-9) for a in length}
    # Kronecker's delta for EV paths
    delta = {(a, p): int(a in paths[p]) for p in paths for a in E_A}

    for i in range(N+1):
        for m in M:
            for a in E:
                # Formula 12: conservation of vehicle numbers
                ##inflow = gp.quicksum(u[m, a, k] for k in range(i+1)) ## li ho uniti sotto
                ##outflow = gp.quicksum(v[m, a, k] for k in range(i+1))
                model.addConstr(n[m, a, i] - gp.quicksum(u[m, a, k] - v[m, a, k] for k in range(i+1)) == 0)
                if a in E_A: # i.e. no source or sink link
                    if i >= travelV[a]: # formula 13: upstream capacity
                        model.addConstr(gp.quicksum(u[m, a, k] for k in range(max(0,i - travelV[a] + 1), i+1)) - n[m, a, i] <= 0)
                    if i >= travelW[a]: # Formula 14: downstream capacity 
                        model.addConstr(n[m, a, i] + gp.quicksum(v[m, a, k] for k in range(max(0,i - travelW[a] + 1), i+1)) <= ev.Ka * length[a] * alpha[a, m, i])
                    # Formula 19: flow capacity on links
                    inflow = gp.quicksum(f[m, b, a, i] for b in incoming_links[a])
                    if m == "EV":
                        model.addConstr(inflow <= ev.Qa * timestep * gp.quicksum(delta[(a,p)] * y[p] for p in paths))
                    else: # Also upstream traffic of ICV vehicles must be lower than capacity
                        model.addConstr(inflow <= icv.Qa * timestep)
                if a != E_R: # i.e. no source link
                    # Formula 16: flux conservation for incoming vehicle
                    model.addConstr(u[m, a, i] - gp.quicksum(f[m, b, a, i] for b in incoming_links[a]) == 0)
                if a != E_S: # i.e. no sink link
                    # Formula 17: flux conservation for outgoing vehicle
                    model.addConstr(v[m, a, i] - gp.quicksum(f[m, a, b, i] for b in outgoing_links[a]) == 0)

        # Formula 15: source link constraint is the demand rate of vehicle
        model.addConstr(u["EV", E_R, i] == ev.Da)
        model.addConstr(u["ICV", E_R, i] == icv.Da)
        # Formula 18: sink link constraint i.e. every vehicle has already exited
        model.addConstr(v["EV", E_S, i] == 0)
        model.addConstr(v["ICV", E_S, i] == 0)

    # Supply and demand at node
    
    for a in E_A:
        for i in range(N+1):
            # Formula 20 and 22: supply and corresponding entry flow of the link
            inflow_s = gp.quicksum(u[m, a, k] for m in M for k in range(i))
            outflow_s = gp.quicksum(v[m, a, k] for m in M for k in range(i - travelW[a] + 1)) if i > travelW[a] else 0
            supply = ev.Ka * length[a] + outflow_s - inflow_s
            model.addConstr(gp.quicksum(u[m, a, i] for m in M) <= ev.Qa * timestep)
            model.addConstr(gp.quicksum(u[m, a, i] for m in M) <= supply)
                                
            # Formula 21 and 23: demand and corresponding exit flow of the link
            inflow_d = gp.quicksum(v[m, a, k] for m in M for k in range(i))
            outflow_d = gp.quicksum(u[m, a, k] for m in M for k in range(i - travelV[a] + 1)) if i > travelV[a] else 0
            demand = outflow_d - inflow_d
            model.addConstr(gp.quicksum(v[m, a, i] for m in M) <= ev.Qa * timestep)
            model.addConstr(gp.quicksum(v[m, a, i] for m in M) <= demand)

    # Extra

    # Over alpha for controlling which links are used depending on y and the maximum flow for each link
    for i in range(1, N+1):
        for a in E_A:
            # For alpha[m] = 0 EV flows cannot enter; for alpha=1 they can reach the maximum capacity Qa
            model.addConstr(gp.quicksum(f['EV', b, a, i] for b in incoming_links[a]) <= ev.Qa * timestep * alpha[a, 'EV', i],name=f"alpha1")
            model.addConstr(gp.quicksum(f['ICV', b, a, i] for b in incoming_links[a]) <= ev.Qa * timestep * alpha[a, 'ICV', i],name=f"alpha2")
            # If no feasible path goes in a, then alpha[EV] = 0, thus also EV flows = 0
            model.addConstr(alpha[a, 'EV', i] <= gp.quicksum(delta[(a, p)] * y[p] for p in paths),name=f"alpha3")
            # If no path is active for link a, then alpha[EV] = 0 (tighter bound than previous one)
            model.addConstr(alpha[a, "EV", i] <= gp.quicksum(y[p] for p in paths if delta[(a,p)]==1),name="alpha4")

    # If a path is not feasible, no EV flow can be on its links
    for p in paths:
        for idx, a in enumerate(paths[p]):
            for i in range(N+1):
                for b in incoming_links[a]:
                    model.addConstr(f['EV', b, a, i] <= ev.Qa * timestep * y[p],name=f"const")


    # Set and optimize the objective function
    #model.setObjective(gp.quicksum((N - i + 1) * f[m, b, E_S, i] for m in M for b in incoming_links[E_S] for i in range(1, N+1)), gp.GRB.MAXIMIZE)
    model.setObjective(gp.quicksum((T - (i - 1)*timestep) *f[(m, b, E_S, i)] for m in M for b in incoming_links[E_S] for i in range(1, N+1)), gp.GRB.MAXIMIZE)
    model.optimize()

    return model, x, y, B, n, u, v, f


def print_optimal_solution(length, graph, model, x, y, B, n, u, v, f):
    # Extract and compute info about the network
    E = list(length.keys()) # links
    E_R = E[0] # source link (it is unique)
    E_S = E[-1] # sink link (it is unique)
    E_A = E[1:-1] # normal links i.e. no source and no sink
    paths = list(nx.all_simple_paths(graph, source=E_R, target=E_S))
    # Remove source and sink link from paths because they are unique and have no length
    paths = {i: path[1:-1] for i, path in enumerate(paths)}


    # === Output: WCL installation and EV path choices ===
    if model.status == gp.GRB.OPTIMAL or model.status == gp.GRB.SUBOPTIMAL:
        print(f"🎯 Valore funzione obiettivo (outflow pesato): {model.ObjVal:.2f}")
        for p in paths:
            for a in paths[p]:
                if B[(a,p)].x > 1e-4:
                    print(f"energia B[({a},{p})]={B[(a,p)]}")
        
        print("🔌 Link selezionati per l'installazione delle WCL:")
        for a in E_A:
            if x[a].x > 0.5:
                print(f"  - Link {a} (lunghezza: {length[a]} m)")

        print("🚗 Percorsi selezionati per EV:")
        for p in paths:
            if y[p].x > 0.5:
                print(f"  - Path {p}: 1 -> " + " -> ".join(str(a) for a in paths[p]) + " -> 7")
    else:
        print(f"⚠️ Il modello non è stato risolto correttamente. Status: {model.status}")