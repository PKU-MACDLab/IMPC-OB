# IMPC-OB：Infinite-horizon Model Predictive Control in Obtacle-dense environment
code for article: Multi-robot Trajectory Generation with Feasibility Gurantee and Deadlock Resolution： An obstacle-dense Environment

IMPC-OB is a online replanning method for multi-robot which can achieve the recursive feasibility when solving optimization problems during replanning in addition to its capibility that resolving deadlock problem in obstacle-dense enviroment.

Here we provide a python3-based program which can simulate the trasition for multi-robot systems. 

## General Setup

We have tested our code with:
`ubuntu 20.0.4`
in python3

This APP need following dependencies:
numpy          1.21.2
scipy          1.6.3
cvxopt         1.3.0
matplotlib     3.4.3



## Running Program

python3 test.py