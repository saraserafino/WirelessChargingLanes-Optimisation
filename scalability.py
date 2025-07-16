from optimisation import *
import json # for loading grid data
import networkx as nx # for automatical generation of paths from edges
import argparse # for choosing the grid from command line

# Script for testing the code on bigger instances for scalability purposes, following Grid Networks of section 4.2 of the paper.

# Set parameters
T = 30 # maximum time (in minutes)
timestep = 0.5 # time steps (in minutes)
scalability = True
# Budget/cost per length of path
I_budget = 2.7e6/500

# From terminal you can choose the size of grid to test scalability on
parser = argparse.ArgumentParser(description="Size of grid chosen by command line")
parser.add_argument('--grid-size', type=int, default=42, metavar='STR',
                      help='size of grid to test (chosen among 10, 28, 42, 51, 82)')
args = parser.parse_args()

# Import data of chosen grid
with open(f"data/grid{args.grid_size}.json", "r") as f:
    data = json.load(f)

link_length = {int(k): v for k, v in data["link_length"].items()}
edges = [tuple(edge) for edge in data["edges"]]

G = nx.DiGraph()
G.add_edges_from(edges)
paths82 = list(nx.all_simple_paths(G, source=1, target=args.grid_size))

scalability_model, x, y, u, v, t_exec = optimisation_model(link_length, G, T, timestep, scalability, I_budget)
print_optimal_solution(link_length, G, scalability_model, x, y)