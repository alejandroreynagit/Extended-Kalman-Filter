#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
#define PI 3.14159265
/* 
 * Please note that the Eigen library does not initialize 
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in; // Measurement matrix
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
  
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_[0];
  float py = x_[1];
  float vx = x_[2];
  float vy = x_[3];
  float rho = sqrt( px*px + py*py );
  
  // If rho == 0 (something right ahead), skip the update step to avoid division by zero.
  //Avoid Division by 0
  if (fabs(rho) < 0.0001) {
    return; // Is this a valid assumption??????
  } 

  // compute the Jacobian matrix
  Hj_ = tools.CalculateJacobian(x_);
  
  //  Lidar Measurement that will map the state vector x into Polar Coordinates,instead of H we have h(x)
  VectorXd hx(3);
  hx << rho, atan2( py, px ), ( px*vx + py*vy )/rho;

  // Update the state using the Extended Kalman Filter equations
  VectorXd y = z - hx;
  
  // Validate +- 2pi enclosing, results are 'centred'
  if( y[1] > PI )
    y[1] -= 2.f*PI;
  if( y[1] < -PI )
    y[1] += 2.f*PI;
  
  // Comput new estimate (new state)
  MatrixXd Hjt = Hj_.transpose();
  MatrixXd S = Hj_*P_*Hjt + R_;
  MatrixXd Si = S.inverse();
  MatrixXd K =  P_*Hjt*Si;
  x_ = x_ + ( K*y );
  P_ = ( I_ - K*Hj_ )*P_;
}
