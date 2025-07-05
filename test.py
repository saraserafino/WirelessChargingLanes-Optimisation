from optimisation import optimisation_model
import matplotlib.pyplot as plt

# Script for testing the code on a small instance, following Braess Network of section 4.1 of the paper.

# Set parameters
link_length = {1: 0, 2: 900, 3: 1800, 4: 900, 5: 3600, 6: 900, 7: 0}
paths = [(1,2,4,6,7), (1,2,5,7), (1,3,6,7)]
T = 60 # maximum time (in minutes)
timestep = 0.25 # time steps (in minutes)
scalability = False
budget = 2700

test_model, x, B = optimisation_model(link_length, paths, T, timestep, scalability, budget)