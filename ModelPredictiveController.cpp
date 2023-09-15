/*
Model Predictive Control implementation in C++
Author:Aleksandar Haber 
Date: September 2023 

We implemented a Model Predictive Control (MPC) algorithm in C++ by using the Eigen C++ Matrix library 
The description of the MPC algorithm is given in this tutorial:

https://aleksandarhaber.com/model-predictive-control-mpc-tutorial-1-unconstrained-formulation-derivation-and-implementation-in-python-from-scratch/

This is the implementation file of the Model Predictive Controller class
*/

#include <iostream>
#include<tuple>
#include<string>
#include<fstream>
#include<vector>
#include<Eigen/Dense>
#include "ModelPredictiveController.h"

using namespace Eigen;
using namespace std;

// edit this default constructor later
ModelPredictiveController::ModelPredictiveController()
{    
}


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

ModelPredictiveController::ModelPredictiveController(
    MatrixXd Ainput, MatrixXd Binput, MatrixXd Cinput, 
    unsigned int fInput, unsigned int vInput, 
    MatrixXd W3input, MatrixXd W4input, 
    MatrixXd x0Input, MatrixXd desiredControlTrajectoryTotalInput)
{
   
    A=Ainput; B=Binput; C=Cinput;
    f=fInput; v=vInput;
    W3=W3input; W4=W4input;
    x0=x0Input; desiredControlTrajectoryTotal=desiredControlTrajectoryTotalInput;
   
    // m - input dimension, n- state dimension, r-output dimension 
    // extract the appropriate dimensions
    n = A.rows();   m = B.cols(); r = C.rows();
    
    // this variable is used to track the current time step k of the controller
    k=0;

    // the trajectory is a column matrix
    unsigned int maxSimulationSamples = desiredControlTrajectoryTotal.rows() - f;
    
    // we store the state vectors of the controlled state trajectory
    states.resize(n, maxSimulationSamples); 
    states.setZero();
    states.col(0)=x0;

    // we store the computed inputs 
    inputs.resize(m, maxSimulationSamples-1); 
    inputs.setZero();
   
    // we store the output vectors of the controlled state trajectory
    outputs.resize(r, maxSimulationSamples-1); 
    outputs.setZero();


    // we form the lifted system matrices O and M and the gain control matrix

    // form the lifted O matrix
    O.resize(f*r,n);
    O.setZero();

    // this matrix is used to store the powers of the matrix A for forming the O matrix
    MatrixXd powA;
    powA.resize(n,n);
    
    for (int i=0; i<f;i++)
    {
        if (i == 0)
        {
            powA=A;
        }
        else
        {
            powA=powA*A;
        }        

            O(seq(i*r,(i+1)*r-1),all)=C*powA;
    }
    
    // form the lifted M matrix
    M.resize(f*r,v*m);
    M.setZero();

    MatrixXd In;
    In= MatrixXd::Identity(n,n);

    MatrixXd sumLast;
    sumLast.resize(n,n);
    sumLast.setZero();

    for (int i=0; i<f;i++)
    {
        // until the control horizon
        if(i<v)
        {
            for (int j=0; j<i+1;j++)
            {
                if (j==0)
                {
                    powA=In;
                }
                else
                {
                    powA=powA*A;
                   
                    
                }
                M(seq(i*r,(i+1)*r-1),seq((i-j)*m,(i-j+1)*m-1))=C*powA*B;
            }
        }
        // from the control horizon until the prediction horizon
        else
        {
            for(int j=0;j<v;j++)
            {
                if (j==0)
                {
                        sumLast.setZero();
                        for (int s=0;s<i+v+2;s++)
                        {
                            if (s == 0)
                            {
                                powA=In;
                            }
                            else
                            {
                                powA=powA*A;
                            }
                            sumLast=sumLast+powA;
                        }
                        M(seq(i*r,(i+1)*r-1),seq((v-1)*m,(v)*m-1))=C*sumLast*B;


                }
                else
                {

                    powA=powA*A;
                    M(seq(i*r,(i+1)*r-1),seq((v-1-j)*m,(v-j)*m-1))=C*powA*B;
        

                }

            }

        }
    }

    MatrixXd tmp;
    tmp.resize(v*m,v*m);

    tmp=M.transpose()*W4*M+W3;
    gainMatrix=(tmp.inverse())*(M.transpose())*W4;
        
}

    
// this function propagates the dynamics
// and computes the control inputs by solving the model predictive control problem
void ModelPredictiveController::computeControlInputs()
{

    // # extract the segment of the desired control trajectory
    MatrixXd desiredControlTrajectory;
    desiredControlTrajectory=desiredControlTrajectoryTotal(seq(k,k+f-1),all);

    //# compute the vector s
    MatrixXd vectorS;
    vectorS=desiredControlTrajectory-O*states.col(k);
 
    //# compute the control sequence
    MatrixXd inputSequenceComputed;
    inputSequenceComputed=gainMatrix*vectorS;
    // extract the first entry that is applied to the system
    inputs.col(k)=inputSequenceComputed(seq(0,m-1),all);

    // propagate the dynamics
    states.col(k+1)=A*states.col(k)+B*inputs.col(k);
    outputs.col(k)=C*states.col(k);

    //increment the index
    k=k+1;

}



  // this function is used to save the variables as "csv" files
        // desiredControlTrajectoryTotalFile - name of the file used to store the trajectory
        // inputsFile  - name of the file used to store the computed inputs
        // statesFile  - name of the file used to store the state trajectory
        // outputsFile  - name of the file used to store the computed outputs - this is the controlled output
        // Ofile        - name of the file used to store the O lifted matrix - you can use this for diagonostics
        // Mfile        - name of the file used to store the M lifted matrix - you can use this for diagonostics

void ModelPredictiveController::saveData(string desiredControlTrajectoryTotalFile, string inputsFile, 
							string statesFile, string outputsFile, string OFile, string MFile) const
{
	const static IOFormat CSVFormat(FullPrecision, DontAlignCols, ", ", "\n");
	
	ofstream file1(desiredControlTrajectoryTotalFile);
	if (file1.is_open())
	{
		file1 << desiredControlTrajectoryTotal.format(CSVFormat);
		
		file1.close();
	}

	ofstream file2(inputsFile);
	if (file2.is_open())
	{
		file2 << inputs.format(CSVFormat);
		file2.close();
	}
	
	ofstream file3(statesFile);
	if (file3.is_open())
	{
		file3 << states.format(CSVFormat);
		file3.close();
	}

	ofstream file4(outputsFile);
	if (file4.is_open())
	{
		file4 << outputs.format(CSVFormat);
		file4.close();
	}

    ofstream file5(OFile);
	if (file5.is_open())
	{
		file5 << O.format(CSVFormat);
		file5.close();
	}


ofstream file6(MFile);
	if (file6.is_open())
	{
		file6 << M.format(CSVFormat);
		file6.close();
	}


	
}

  