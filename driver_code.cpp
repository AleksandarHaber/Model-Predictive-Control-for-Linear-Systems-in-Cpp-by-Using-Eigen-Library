/*
Model Predictive Control implementation in C++
Author:Aleksandar Haber 
Date: September 2023 

We implemented a Model Predictive Control (MPC) algorithm in C++ by using the Eigen C++ Matrix library 
The description of the MPC algorithm is given in this tutorial:

https://aleksandarhaber.com/model-predictive-control-mpc-tutorial-1-unconstrained-formulation-derivation-and-implementation-in-python-from-scratch/

This is the driver code that explains how to use the Model Predictive Controller class
and that solves the MPC problem
*/

#include <iostream>
#include<Eigen/Dense>

#include "ModelPredictiveController.h"
#include "ModelPredictiveController.cpp"

using namespace Eigen;
using namespace std;


int main()
{

//###############################################################################
//#  Define the MPC algorithm parameters
//###############################################################################

// prediction horizon
unsigned int f=20;
// control horizon
unsigned int v=18;

//###############################################################################
//# end of MPC parameter definitions
//###############################################################################


//###############################################################################
//# Define the model - continious time
//###############################################################################

//# masses, spring and damper constants
double m1=2  ; double m2=2   ; double k1=100  ; double k2=200 ; double d1=1  ; double d2=5; 
//# define the continuous-time system matrices and initial condition

    Matrix <double,4,4> Ac {{0, 1, 0, 0},
                            {-(k1+k2)/m1 ,  -(d1+d2)/m1 , k2/m1 , d2/m1},
                            {0 , 0 ,  0 , 1},
                            {k2/m2,  d2/m2, -k2/m2, -d2/m2}};
    Matrix <double,4,1> Bc {{0},{0},{0},{1/m2}}; 
    Matrix <double,1,4> Cc {{1,0,0,0}};

    Matrix <double,4,1> x0 {{0},{0},{0},{0}}; 

//  m- number of inputs
//  r - number of outputs
//  n - state dimension
  unsigned int n = Ac.rows();  unsigned int m = Bc.cols(); unsigned int r = Cc.rows();


//###############################################################################
//# end of model definition
//###############################################################################

//###############################################################################
//# discretize the model
//###############################################################################

//# discretization constant
double sampling=0.05;

// # model discretization
// identity matrix
MatrixXd In;
In= MatrixXd::Identity(n,n);

MatrixXd A;
MatrixXd B;
MatrixXd C;
A.resize(n,n);
B.resize(n,m);
C.resize(r,n);
A=(In-sampling*Ac).inverse();
B=A*sampling*Bc;
C=Cc;

//###############################################################################
//# end of discretize the model
//###############################################################################

//###############################################################################
//# form the weighting matrices
//###############################################################################

//# W1 matrix
MatrixXd W1;
W1.resize(v*m,v*m);
W1.setZero();

MatrixXd Im;
Im= MatrixXd::Identity(m,m);

for (int i=0; i<v;i++)
{
  if (i==0)
  {
     W1(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Im;
  }
  else
  {
     W1(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Im;
     W1(seq(i*m,(i+1)*m-1),seq((i-1)*m,(i)*m-1))=-Im;
  }

}



//# W2 matrix
double Q0=0.0000000011;
double Qother=0.0001;

MatrixXd W2;
W2.resize(v*m,v*m);
W2.setZero();

for (int i=0; i<v; i++)
{
  if (i==0)
  {
    // this is for multivariable
    //W2(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Q0;

    W2(i*m,i*m)=Q0;
  }
  else
  {
    // this is for multivariable
    //W2(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Qother;
    W2(i*m,i*m)=Qother;

  }
        


}

MatrixXd W3;
W3=(W1.transpose())*W2*W1;


MatrixXd W4;
W4.resize(f*r,f*r);
W4.setZero();

// # in the general case, this constant should be a matrix
double predWeight=10;

for (int i=0; i<f;i++)
{
   //this is for multivariable
  //W4(seq(i*r,(i+1)*r-1),seq(i*r,(i+1)*r-1))=predWeight;
  W4(i*r,i*r)=predWeight;
}

  
//###############################################################################
//# end of form the weighting matrices
//###############################################################################

//###############################################################################
//# Define the reference trajectory 
//###############################################################################

unsigned int timeSteps=300;

//# pulse trajectory

MatrixXd desiredTrajectory;
desiredTrajectory.resize(timeSteps,1);
desiredTrajectory.setZero();

MatrixXd tmp1;
tmp1=MatrixXd::Ones(100,1);

desiredTrajectory(seq(0,100-1),all)=tmp1;

desiredTrajectory(seq(200,timeSteps-1),all)=tmp1;

//###############################################################################
//# end of definition of the reference trajectory 
//###############################################################################

//###############################################################################
//# Run the MPC algorithm
//###############################################################################


// create the MPC object 
ModelPredictiveController  mpc(A, B, C, 
    f, v,W3,W4,x0,desiredTrajectory);

// this is the main control loop
for (int index1=0; index1<timeSteps-f-1; index1++)
{
  mpc.computeControlInputs();    
}
    
// save the computed vectors and matrices
// saveData(string desiredControlTrajectoryTotalFile, string inputsFile, 
// 							string statesFile, string outputsFile)
mpc.saveData("trajectory.csv", "computedInputs.csv", 
                            "states.csv", "outputs.csv","Omatrix.csv","Mmatrix.csv");

cout<<"MPC simulation completed and data saved!"<<endl;
return 0;

}