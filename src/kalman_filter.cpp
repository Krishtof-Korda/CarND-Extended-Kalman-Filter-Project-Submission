#include "kalman_filter.h"
#include <iostream>


using Eigen::MatrixXd;
using Eigen::VectorXd;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
  TODO:
    DONE: * predict the state
  */
  
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::NormalizeAngle(double& phi){
  //Normalize phi between -pi and pi
  phi = atan2(sin(phi), cos(phi));
}

void KalmanFilter::Estimator(const VectorXd& y){
  
  //Performs the common matrix algebra for both the EKF and KF
  MatrixXd PHt = P_ * H_.transpose();
  MatrixXd S = H_ * PHt + R_;
  MatrixXd K = PHt * S.inverse();
    
  //new estimate
  x_ = x_ + (K * y);
  P_ -= K * H_ * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    DONE: * update the state by using Kalman Filter equations
  */
  
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  
  //Call Estimator function to complete the update
  Estimator(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    DONE: * update the state by using Extended Kalman Filter equations
  */
  
  //recover state parameters
  const double px = x_(0);
  const double py = x_(1);
  const double vx = x_(2);
  const double vy = x_(3);
  
  //Precalculate reused quantities for error calculation
  const double px_2 = px*px;
  const double py_2 = py*py;
  const double sumsq = px_2+py_2;
  const double range = sqrt(sumsq);
  
  //check division by zero
  if(fabs(sumsq) < 0.0001){
    std::cout << "UpdateEKF () - Error - Divide by zero,  skipping current RADAR update calculation.\n\n";
    return;
  }
  
  VectorXd h_x = VectorXd(3);
  h_x << range, atan2(py, px), (px*vx + py*vy)/range;
 
  VectorXd y = z - h_x;
  std::cout << "\n\n Phi = " << y(1) << std::endl;

  //Adjust phi value to be between -pi and pi.
  NormalizeAngle(y(1));
  
  //Call Estimator function to complete the update
  Estimator(y);
  
}
