from optimisation import *
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx # for automatical generation of paths from edges

# Script for testing the code on a small instance, following Braess Network of section 4.1 of the paper.

# Set parameters
link_length = {1: 0, 2: 900, 3: 1800, 4: 900, 5: 3600, 6: 900, 7: 0}
edges = [(1, 2), (1, 3), (2, 4), (2, 5), (3, 6), (4, 6), (5, 7), (6, 7)]
T = 60 # maximum time (in minutes)
timestep = 0.25 # time steps (in minutes)
scalability = False
# Budget/cost per length of path
I_budget = 1e6/500

# Generate paths from the edges
graph = nx.DiGraph()
graph.add_edges_from(edges)

test_model, x, y, u, v, t_exec = optimisation_model(link_length, graph, T, timestep, scalability, I_budget)
print_optimal_solution(link_length, graph, test_model, x, y)

# Inflow-outflow profiles of each vehicle class as in figure 3 of paper
N = int(T / timestep)
x_time = [i * timestep for i in range(N+1)]

fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(20, 10), sharex=True, sharey=True)
axes = axes.flatten()

for idx, a in enumerate(list(link_length.keys())[:-1]):
    ax = axes[idx]

    EV_inflow = [u[("EV", a, i)].X for i in range(N+1)]
    EV_outflow = [v[("EV", a, i)].X for i in range(N+1)]
    ICV_inflow = [u[("ICV", a, i)].X for i in range(N+1)]
    ICV_outflow = [v[("ICV", a, i)].X for i in range(N+1)]

    ax.plot(x_time, np.cumsum(EV_inflow), label="EV inflow", linestyle=(0,(3,5,1,5)), color='blue')
    ax.plot(x_time, np.cumsum(EV_outflow), label="EV outflow", linestyle='dotted', color='green')
    ax.plot(x_time, np.cumsum(ICV_inflow), label="ICV inflow", linestyle='dashdot', color='red')
    ax.plot(x_time, np.cumsum(ICV_outflow), label="ICV outflow", linestyle='solid', color='orange')

    ax.set_title(f"Link {a}")
    ax.set_xlabel("Time (min)")
    ax.set_ylabel("Flow accumulation (vehicles)")
    ax.grid(True)

# Common legend
handles, labels = ax.get_legend_handles_labels()
fig.legend(handles, labels, ncol=2, loc='upper left')
fig.tight_layout(rect=[0, 0, 1, 0.95])
plt.savefig("Inflow-outflow profiles of Braess network")