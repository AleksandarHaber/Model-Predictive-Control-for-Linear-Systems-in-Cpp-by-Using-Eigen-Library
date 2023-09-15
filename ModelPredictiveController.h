/*
Model Predictive Control implementation in C++
Author:Aleksandar Haber 
Date: September 2023 

We implemented a Model Predictive Control (MPC) algorithm in C++ by using the Eigen C++ Matrix library 
The description of the MPC algorithm is given in this tutorial:

https://aleksandarhaber.com/model-predictive-control-mpc-tutorial-1-unconstrained-formulation-derivation-and-implementation-in-python-from-scratch/

This is the header file of the Model Predictive Controller class
*/
#ifndef MODELPREDICTIVECONTROLLER_H
#define MODELPREDICTIVECONTROLLER_H

#include<string>
#include<Eigen/Dense>
using namespace Eigen;
using namespace std;


class ModelPredictiveController{
    public:
        // default constructor - edit this later
        ModelPredictiveController();

        /* 
        This constructor will initialize all the private variables, 
        and it will construct the lifted system matrices O and M,
        as well as the control gain matrix necessary to compute the control inputs
        */
        // this is the constructor that we use
        // Ainput, Binput, Cinput - A,B,C system matrices 
        // fInput - prediction horizon
        // vInput - control horizon
        // W3input, W4input - weighting matrices, see the tutorial 
        // x0Input - x0 initial state of the system 
        // desiredControlTrajectoryTotalInput - total desired input trajectory
        
        ModelPredictiveController(MatrixXd Ainput, MatrixXd Binput, MatrixXd Cinput, 
                     unsigned int fInput, unsigned int vInput, 
                     MatrixXd W3input, MatrixXd W4input,
                     MatrixXd x0Input, MatrixXd desiredControlTrajectoryTotalInput);
        
        // this function is used to propagate the dynamics and to compute the solution of the MPC problem
        void computeControlInputs();

        // this function is used to save the variables as "csv" files
        // desiredControlTrajectoryTotalFile - name of the file used to store the trajectory
        // inputsFile  - name of the file used to store the computed inputs
        // statesFile  - name of the file used to store the state trajectory
        // outputsFile  - name of the file used to store the computed outputs - this is the controlled output
        // Ofile        - name of the file used to store the O lifted matrix - you can use this for diagonostics
        // Mfile        - name of the file used to store the M lifted matrix - you can use this for diagonostics
        void ModelPredictiveController::saveData(string desiredControlTrajectoryTotalFile, string inputsFile, 
							string statesFile, string outputsFile,string OFile, string MFile) const;

    private:


        // this variable is used to track the current time step k of the controller
        // it is incremented in the function computeControlInputs()
        unsigned int k;

        // m - input dimension, n- state dimension, r-output dimension 
        unsigned int m,n,r; 

        // MatrixXd is an Eigen typdef for Matrix<double, Dynamic, Dynamic>
	    MatrixXd A,B,C,Q; // system matrices
        MatrixXd W3,W4;   // weighting matrices
	    MatrixXd x0;      // initial state
        MatrixXd desiredControlTrajectoryTotal; // total desired trajectory
        unsigned int f,v; // f- prediction horizon, v - control horizon
        
                   
                
        // we store the state vectors of the controlled state trajectory
        MatrixXd states;
                
        // we store the computed inputs 
        MatrixXd inputs;
        
        // we store the output vectors of the controlled state trajectory
        MatrixXd outputs;

        // lifted system matrices O and M, computed by the constructor 
        MatrixXd O;
        MatrixXd M;

        // control gain matrix, computed by constructor
        MatrixXd gainMatrix;

};

#endif