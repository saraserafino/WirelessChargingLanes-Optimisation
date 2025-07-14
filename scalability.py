from optimisation import *
import json
import networkx as nx # for generating paths from edges without going crazy
import matplotlib.pyplot as plt

# Script performing the scalability analysis, following Grid Networks of section 4.2 of the paper.

## Nota per noi: to produce that/several plot in which you perform scalability analysis and the actual experiments of the test.
## The code must take less than one night to run.

# Set parameters
T = 30 # maximum time (in minutes)
timestep = 0.5 # time steps (in minutes)
scalability = True
# Budget/cost per length of path
I_budget = 1e6/500

# Import data of Grid-42
with open("data/grid42.json", "r") as f:
    data = json.load(f)

# Note: keys of JSOn dictionary are strings, so convert them to integers
link_length42 = {int(k): v for k, v in data["link_length"].items()}
edges42 = [tuple(edge) for edge in data["edges"]]

# Generate paths from the edges
G42 = nx.DiGraph()
G42.add_edges_from(edges42)
#paths42 = list(nx.all_simple_paths(G42, source=1, target=42))
#nx.draw(G42)
#plt.savefig("paths42.png")
scalability_model42, x42, y42, B42, n42, u42, v42, f42, t_exec42 = optimisation_model(link_length42, G42, T, timestep, scalability, I_budget)
print_optimal_solution(link_length42, G42, scalability_model42, x42, y42, B42, n42, u42, v42, f42)


# Import data of Grid-82
with open("data/grid82.json", "r") as f:
    data = json.load(f)

link_length82 = {int(k): v for k, v in data["link_length"].items()}
edges82 = [tuple(edge) for edge in data["edges"]]

G82 = nx.DiGraph()
G82.add_edges_from(edges82)
#paths82 = list(nx.all_simple_paths(G82, source=1, target=82))

scalability_model82, x82, y82, B82, n82, u82, v82, f82, t_exec82 = optimisation_model(link_length82, G82, T, timestep, scalability, I_budget)
print_optimal_solution(link_length82, G82, scalability_model82, x82, y82, B82, n82, u82, v82, f82)