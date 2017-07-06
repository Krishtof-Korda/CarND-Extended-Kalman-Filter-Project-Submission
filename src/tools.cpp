#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
  TODO:
    DONE: * Calculate the RMSE here.
  */
  
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  int n = estimations.size();
  int g = ground_truth.size();
  //cout << "number of estimations = " << n << endl;
  //cout << "number of ground truths = " << g << endl;
  
  if(n==0 || n!=g) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }
  
  //accumulate squared residuals
  for(int i=0; i < n; ++i) {
    
      //cout << "i = " << i << endl;
      VectorXd residuals = estimations[i]-ground_truth[i];
      residuals = residuals.cwiseProduct(residuals);
      rmse += residuals;
      //cout << "residuals = " << residuals << endl << "sum = " << rmse << endl;
  }
  
  //calculate the mean
  rmse = rmse/n;
  //cout << "mean = " << rmse << endl;
  
  //calculate the squared root
  rmse = rmse.array().sqrt();
  //cout << "RMSE = " << rmse << endl;
  
  //return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
  TODO:
    DONE: * Calculate a Jacobian here.
  */
  
  MatrixXd Hj(3,4);
  //recover state parameters
  const double px = x_state(0);
  const double py = x_state(1);
  const double vx = x_state(2);
  const double vy = x_state(3);
  
  //Precalculate parameters
  const double px_2 = px*px;
  const double py_2 = py*py;
  double sumsq = px_2+py_2;
  const double range = sqrt(sumsq);
  const double range_3 = range*sumsq;
  
  //check division by zero
  const double eps = 0.001;
  sumsq = std::max(eps, sumsq);
  /*
  if(fabs(sumsq) < 0.0001){
    cout << "CalculateJacobian () - Error - Divide by zero, returning initialized Hj matrix without calculation.\n\n";
    return Hj;
  }
  */
  //compute the Jacobian matrix
  Hj << (px/range),                 (py/range),                 0,        0,
        -(py/sumsq),                (px/sumsq),                 0,        0,
        py*(vx*py - vy*px)/range_3, px*(vy*px - vx*py)/range_3, px/range, py/range;
  
  return Hj;
}
