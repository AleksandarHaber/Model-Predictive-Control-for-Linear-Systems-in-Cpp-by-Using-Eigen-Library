# -*- coding: utf-8 -*-
"""
/*
Model Predictive Control implementation in C++
Author:Aleksandar Haber 
Date: September 2023 

We implemented a Model Predictive Control (MPC) algorithm in C++ by using the Eigen C++ Matrix library 
The description of the MPC algorithm is given in this tutorial:

https://aleksandarhaber.com/model-predictive-control-mpc-tutorial-1-unconstrained-formulation-derivation-and-implementation-in-python-from-scratch/

This is the Python file used to visualize the results
and to save the graphs

This file will load the saved matrices and vectors.
After that, it will generate the graphs
*/
"""

import numpy as np
import matplotlib.pyplot as plt

# Load the matrices and vectors from csv files 
# computed control inputs
cppInputs = np.loadtxt("computedInputs.csv", delimiter=",")
# controlled state trajectory
cppStates = np.loadtxt("states.csv", delimiter=",")
# computed outputs
cppOutputs= np.loadtxt("outputs.csv", delimiter=",")
# reference (desired) trajectory
cppTrajectory= np.loadtxt("trajectory.csv", delimiter=",")
# lifted O matrix 
Omatrix=np.loadtxt("Omatrix.csv", delimiter=",")
# lifted M matrix
Mmatrix=np.loadtxt("Mmatrix.csv", delimiter=",")

# plot the results
    
plt.figure(figsize=(6,6))
plt.plot(cppOutputs,linewidth=5, label='Controlled trajectory')
plt.plot(cppTrajectory,'r', linewidth=3, label='Desired trajectory')
plt.xlabel('time steps')
plt.ylabel('Outputs')
plt.legend()
plt.savefig('controlledOutputsPulseCpp.png',dpi=600)
plt.show()


plt.figure(figsize=(6,6))
plt.plot(cppInputs,linewidth=4, label='Computed inputs')
plt.xlabel('time steps')
plt.ylabel('Input')
plt.legend()
plt.savefig('inputsPulseCpp.png',dpi=600)
plt.show()