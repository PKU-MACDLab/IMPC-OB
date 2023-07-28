# IMPC-OB: MPC-Based Multi-robot Trajectory planning in An Obtacle-Dense Environment
code for article: Multi-robot Trajectory Generation with Feasibility Gurantee and Deadlock Resolution： An obstacle-dense Environment

It is a online replanning method for multi-robot which can achieve the recursive feasibility when solving optimization problems during replanning in addition to its capibility that resolving deadlock problem in obstacle-dense enviroment.

Here we provide a python3-based program which can simulate the trasition for multi-robot systems. 

## 0. General Setup

We have tested our code with:
`ubuntu 20.0.4`
in python3

This APP need following dependencies:
numpy          1.21.2
scipy          1.6.3
cvxopt         1.3.0
matplotlib     3.4.3

## 1. Essential file 'opengjkc.so' Setup
IMPC-OB depends on the package [openGJK](https://github.com/MattiaMontanari/openGJK#getting-started) to apply polytope-related calculation. As openGJK is wrriten in C++, a conversion from C++ to python is needed. This can be done by generating a Python-Binding file called `opengjkc.so`.

PS: this file must be generated specifically for each different running environment, the one provided in IMPC-OB source code is generated based on python 3.9, thus it might not be available for importing in your environment. Be sure to manually generate it yourself!

The process is shown below:
### 1.1 Copy the source code  
    git clone https://github.com/MattiaMontanari/openGJK  
### 1.2 Build the file  
    cmake -E make_directory build  
    cmake -E chdir build cmake -DCMAKE_BUILD_TYPE=Release ..   
    cmake --build build  
    cmake -E chdir build/examples/c ./example_lib_opengjk_ce  
### 1.3 File location adjustment  
Move the 'openGJK.c' file from the root directory to the 'src' directory, then open it and delete all keywords named `restrict`.  
### 1.4 File generation  
cd to the 'examples/python' directory and run ```source build.sh```.  
This should generate a file called `opengjkc.cpython-38-x86_64-linux-gnu.so`.  
Rename it as `opengjkc.so` and move it to the 'IMPC-OB/src/opengjk' directory to replace the existing opengjkc.so file.  


## Running Program
$ cd src
$ python3 test.py
