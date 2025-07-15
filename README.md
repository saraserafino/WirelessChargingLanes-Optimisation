# Optimisation Model for Wireless Charging Lanes
This repository contains the project for the course of "Mathematical Optimisation" @ UniTS, consisting in the realisation of the paper [Dynamic wireless charging lanes location model in urban networks considering route choices](https://www.sciencedirect.com/science/article/pii/S0968090X2200095X) by Tran et al. (2022).<br>

In particular, it contains:
- a script `vehicles.py` defining vehicles' parameters
- a file `test.py` where the proposed algorithm is executed on a small instance, in this case the Braess Network
- a file `scalability.py` to be used for the scalability analysis of the algorithm
- a presentation explaining the project and some of our changes, also performing scalability and parameters analysis through plots (of which code can be found in `plots.py`).

## Abstract of the paper
Wireless charging technologies have now made it possible to charge while driving, which offers the opportunity to stimulate the market penetration of electric vehicles. This paper aims to support the system planner in optimally deploying the wireless charging lanes on the network, considering traffic dynamics and congestion under multiple vehicle classes. The overall objective is to maximise network performance while providing insights into traffic propagation patterns over the network. A multi-class dynamic system optimal model is adopted to compute an approximate representation of the dynamic traffic flow. As a result, the problem is formulated as a mixed-integer linear program by integrating the dynamic routing behaviour into the charging location problem. Finally, the proposed framework has been tested on different sized test-bed networks to examine the solution quality and illustrate the model’s efficacy.

## How to compile
The scalability analysis is performed on grid networks composed of 10, 28, 42, 51 and 82 links. Therefore, the user can provide the size of the grid as parsed parameter; for example:
```bash
python scalability.py --grid-size 42
```

## Requirements
The algorithm is performed using a Gurobi solver, thus an active [Gurobi license](https://support.gurobi.com/hc/en-us/articles/12684663118993-How-do-I-obtain-a-Gurobi-license) is needed. <br>
The following Python packages - installable with `pip install [names] --user` - are used:
- MatPlotLib for the plots in `test.py` and `plots.py`
- NumPy for performing the cumulative sum in `test.py`
- networkx for automatical generation of paths from edges
- json for creating and importing the grid networks
- time for the time wrapper returning the time of compilation
- argparse for defining the parser in `scalability.py`.