from optimisation import *
import matplotlib.pyplot as plt
import networkx as nx # for generating paths from edges without going crazy

# Script for testing the code on a small instance, following Braess Network of section 4.1 of the paper.

# Set parameters
link_length = {1: 0, 2: 900, 3: 1800, 4: 900, 5: 3600, 6: 900, 7: 0}
edges = [(1, 2), (1, 3), (2, 4), (2, 5), (3, 6), (4, 6), (5, 7), (6, 7)]
T = 60 # maximum time (in minutes)
timestep = 0.25 # time steps (in minutes)
scalability = False
budget = 2800

# Generate paths from the edges
graph = nx.DiGraph()
graph.add_edges_from(edges)

test_model, x, y, B, n, u, v, f = optimisation_model(link_length, graph, T, timestep, scalability, budget)
print_optimal_solution(link_length, graph, test_model, x, y, B, n, u, v, f)