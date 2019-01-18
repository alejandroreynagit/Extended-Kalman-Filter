#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // TODO: YOUR CODE HERE
  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  // check division by zero
  if (estimations.size() == 0) {
    cout << "Vector shape Zero" << endl;
    return rmse;
  }
  
  if (estimations.size() != ground_truth.size()) {
    cout << "Vector should be the same size" << endl;
    return rmse;
  }
  
  // TODO: accumulate squared residuals
  for (int i=0; i < estimations.size(); ++i) {
    // ... your code here
    VectorXd residual = estimations[i] - ground_truth[i];
    
    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    //cout << "residual: " << residual << endl;
    // residual is a 4D vector, we sum each element wise!
    rmse += residual;
  }
  
  // calculate the mean
  rmse = rmse/estimations.size(); 
  
  // calculate the squared root
  rmse = rmse.array().sqrt();
  
  return rmse;
  
  
  
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];

  // If rho == 0 (something right ahead), skip the update step to avoid division by zero.
  if ((px*px+py*py) < 0.0001) 
    {
       // statements
       cout << "Division by zero_= " ;
       return Hj;
    }
  Hj_ << px/pow((pow(px,2)+pow(py,2)),0.5), py/pow((pow(px,2)+pow(py,2)),0.5), 0, 0,
       -py/(pow(px,2)+pow(py,2)), px/(pow(px,2)+pow(py,2)), 0, 0,
       py*(vx*py - vy*px)/pow((pow(px,2)+pow(py,2)),1.5), px*(vy*px-vx*py)/pow((pow(px,2)+pow(py,2)),1.5), px/pow((pow(px,2)+pow(py,2)),0.5),        py/pow((pow(px,2)+pow(py,2)),0.5);
  
  return Hj;
}
