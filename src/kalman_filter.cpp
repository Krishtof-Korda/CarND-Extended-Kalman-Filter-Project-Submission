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
  Estimator(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    DONE: * update the state by using Extended Kalman Filter equations
  */
  
  //recover state parameters
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  
  //Precalculate reused quantities for error calculation
  float px_2 = px*px;
  float py_2 = py*py;
  float sumsq = px_2+py_2;
  float range = sqrt(sumsq);
  
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
  void NormalizeAngle(double& phi){
    phi = atan2(sin(phi), cos(phi));
  }
  
  NormalizeAngle(y(1));
  Estimator(y);
  
  /* My orginal code for normalizing angle phi. I used the recommended code from the Udacity reviewer instead.
  if (y(1) > M_PI){
    while(y(1) > M_PI){
      y(1) = y(1) - 2*M_PI;
    }
    std::cout << "\n\n Phi after reduction = " << y(1) << std::endl;
  }

  if (y(1) < -M_PI){
    while(y(1) < -M_PI){
      y(1) = y(1) + 2*M_PI;
    }
    std::cout << "\n\n Phi after increase = " << y(1) << std::endl;
  }
  */

  /*
  //Update precalculations
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;
  
  //new estimate
  x_ = x_ + (K * y);
  P_ -= K * H_ * P_;
  //long x_size = x_.size();
  //MatrixXd I = MatrixXd::Identity(x_size, x_size);
  //P_ = (I - K * H_) * P_;
  */
  
}
