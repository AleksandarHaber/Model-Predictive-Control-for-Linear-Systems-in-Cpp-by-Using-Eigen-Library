# Model-Predictive-Control-for-Linear-Systems-in-Cpp-by-Using-Eigen-Library

IMPORTANT NOTE: The code files are released under "Creative Commons Attribution-NonCommercial-NoDerivatives 4.0
International Public License (CC BY-NC-ND 4.0.)"  

**Brief explanation of the license (read the complete license):** 
**Attribution (BY):** You must give appropriate credit and reference to the creator and code (citation). You need to provide a link to the license and link to the code files. 
**NonCommercial (NC):** You may not use the material for commercial purposes. 
**NoDerivatives (ND):** You cannot remix, transform, or build upon the material, meaning you can only share the original work without any adaptations. If you plan to use the code for commercial purposes, contact the author at ml.mecheng@gmail.com



This repository contains C++ files that explain how to implement the Model Predictive Control (MPC) algorithm for linear systems in C++ by using the Eigen C++ matrix library.

The implemented MPC algorithm is derived and explained in this webpage tutorial:

https://aleksandarhaber.com/model-predictive-control-mpc-tutorial-2-unconstrained-solution-for-linear-systems-and-implementation-in-c-from-scratch-by-using-eigen-c-library/

The YouTube tutorial explaining the derivation and implementation is given below:

https://www.youtube.com/watch?v=fgNz1RE2DG4

TO DO AND TO IMPROVE: PASS BY CONST REFERENCE FOR SPEED.

Explanation of the posted files:

- "ModelPredictiveController.h" is the header file of the class that implements the MPC algorithm
- "ModelPredictiveController.cpp" is the implementation file of the class that implements the MPC algorithm
- "driver_code.cpp" is the driver code for the class that implements the MPC algorithm. You should run this file to generate the results. The results are saved
  in the comma-separated-value files (csv files). The files store the controlled state trajectory, controlled outputs, computed inputs, and desired trajectory. Also, these
  files store the lifted system matrices O and M.
- "visualizeResultsPython.py" is the Python file used to plot the results. This file reads the saved csv files and it plots the controlled output trajectory, desired trajectory, and applied control inputs. One of the generated graphs is shown below.

  ![My Image](controlledOutputsPulseCpp.png)
