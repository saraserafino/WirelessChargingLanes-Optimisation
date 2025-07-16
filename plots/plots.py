import matplotlib.pyplot as plt

# Scalability plot
scalability = {10: 1.3, 28: 6.5, 42: 39.8, 51: 128.1, 82: 510.7}
plt.figure(figsize=(8, 5))
plt.plot(scalability.keys(), scalability.values(), marker='o', linestyle='--', color='royalblue')
for x, y in zip(scalability.keys(), scalability.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
plt.title("Model scalability")
plt.xlabel("Dimensionality")
plt.ylabel("Time (s)")
plt.savefig("Model scalability")

# Parameters value's effect on execution time

Ka = {0.05: 20.7, 0.08: 19.2, 0.10: 15.9, 0.12: 39.8, 0.15: 36.6, 0.20: 30.7}
plt.figure(figsize=(8, 5))
plt.plot(Ka.keys(), Ka.values(), marker='o', linestyle='--', color='royalblue')
for x, y in zip(Ka.keys(), Ka.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
plt.title("Jam density for each link a")
plt.xlabel("Ka (vehicles/m)")
plt.ylabel("Time (s)")
plt.savefig("Jam density for each link")

Da = {16: 21.2, 26: 29.3, 36: 39.8, 46: 43.3, 56: 47.9, 66: 72.5, 76: 54.0}
plt.figure(figsize=(8, 5))
plt.plot(Da.keys(), Da.values(), marker='o', linestyle='--', color='royalblue')
for x, y in zip(Da.keys(), Da.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
plt.title("Demand rate")
plt.xlabel("Da (vehicles/timestep)")
plt.ylabel("Time (s)")
plt.savefig("Demand rate")

VW = {0.5: 13.9, 0.7: 13.8, 1: 39.8, 1.5: 37.1, 2: 30.4}
plt.figure(figsize=(8, 5))
plt.plot(VW.keys(), VW.values(), marker='o', linestyle='--', color='royalblue')
for x, y in zip(VW.keys(), VW.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
plt.title("Free-flow and backward speed")
plt.xlabel("Factor of proportionality for V and W  (m/min)")
plt.ylabel("Time (s)")
plt.savefig("Free-flow and backward speed")

Thorizon = {15: 3.9, 20: 16.2, 25: 25, 30: 39.8, 60: 44.4, 75: 84.5, 90: 127.5}
plt.figure(figsize=(8, 5))
plt.plot(Thorizon.keys(), Thorizon.values(), marker='o', linestyle='--', color='royalblue')
for x, y in zip(Thorizon.keys(), Thorizon.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
plt.title("Time horizon")
plt.xlabel("Time horizon (min)")
plt.ylabel("Time (s)")
plt.savefig("Time horizon")

timestep = {0.15: 223.4, 0.25: 48.8, 0.5: 39.8, 0.75: 9.9, 1: 6.1}
plt.figure(figsize=(8, 5))
plt.plot(timestep.keys(), timestep.values(), marker='o', linestyle='--', color='royalblue')
for x, y in zip(timestep.keys(), timestep.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
plt.title("Timestep")
plt.xlabel("Timestep (min)")
plt.ylabel("Time (s)")
plt.savefig("Timestep")

# Number of variables and constraints versus dimensionality
variables = {10: 5276, 28: 15381, 42: 24094, 51: 29979, 82: 49384}
constraints = {10: 8458, 28: 26040, 42: 41447, 51: 52439, 82: 92693}
plt.figure(figsize=(8, 5))
plt.plot(variables.keys(), variables.values(), label="variables", marker='o', linestyle='--', color='royalblue')
plt.plot(constraints.keys(), constraints.values(), label="constraints", marker='o', linestyle='--', color='deeppink')
for x, y in zip(variables.keys(), variables.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
for x, y in zip(constraints.keys(), constraints.values()):
    plt.text(x, y, f'({x}, {y})', fontsize=9, ha='left', va='bottom')
plt.title("Number of variables and constraints versus dimensionality")
plt.xlabel("Dimensionality")
plt.ylabel("Number of variables/constraints")
plt.legend()
plt.savefig("Number of variables and constraints versus dimensionality")