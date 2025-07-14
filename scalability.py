from optimisation import *
import json
import networkx as nx # for generating paths from edges without going crazy
import matplotlib.pyplot as plt

# Script for testing the code on bigger instances for scalability purposes, following Grid Networks of section 4.2 of the paper.

# Set parameters
T = 30 # maximum time (in minutes)
timestep = 0.5 # time steps (in minutes)
scalability = True
# Budget/cost per length of path
I_budget = 2.7e6/500

# Import data of Grid-42
with open("data/grid42.json", "r") as f:
    data = json.load(f)

# Note: keys of JSON dictionary are strings, so convert them to integers
link_length42 = {int(k): v for k, v in data["link_length"].items()}
edges42 = [tuple(edge) for edge in data["edges"]]

# Generate paths from the edges
G42 = nx.DiGraph()
G42.add_edges_from(edges42)
scalability_model42, x42, y42, B42, n42, u42, v42, f42, t_exec42 = optimisation_model(link_length42, G42, T, timestep, scalability, I_budget)
print_optimal_solution(link_length42, G42, scalability_model42, x42, y42)


# Inflow-outflow profiles of each vehicle class as in figure 3 of paper
N = int(T / timestep)
x_time = [i * timestep for i in range(N+1)]

fig, axes = plt.subplots(nrows=2, ncols=3, figsize=(20, 10), sharex=True, sharey=True)
axes = axes.flatten()

for idx, a in enumerate(list(link_length42.keys())[:7]):
    ax = axes[idx]

    EV_inflow = [u42[("EV", a, i)].X for i in range(N+1)]
    EV_outflow = [v42[("EV", a, i)].X for i in range(N+1)]
    ICV_inflow = [u42[("ICV", a, i)].X for i in range(N+1)]
    ICV_outflow = [v42[("ICV", a, i)].X for i in range(N+1)]

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
plt.savefig("Inflow-outflow profiles of Grid 42 network")


# Import data of Grid-82
#with open("data/grid82.json", "r") as f:
#    data = json.load(f)

#link_length82 = {int(k): v for k, v in data["link_length"].items()}
#edges82 = [tuple(edge) for edge in data["edges"]]

#G82 = nx.DiGraph()
#G82.add_edges_from(edges82)
#paths82 = list(nx.all_simple_paths(G82, source=1, target=82))

#scalability_model82, x82, y82, B82, n82, u82, v82, f82, t_exec82 = optimisation_model(link_length82, G82, T, timestep, scalability, I_budget)
#print_optimal_solution(link_length82, G82, scalability_model82, x82, y82)