#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);
  Q_ = MatrixXd(4, 4);
  P_ = MatrixXd(4,4);
  x_ = VectorXd(4);
  F_ = MatrixXd(4,4);
 
 //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
            0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
            0, 0.0009, 0,
            0, 0, 0.09;

  
  /**
   TODO:
   DONE: * Finish initializing the FusionEKF.
   DONE: * Set the process and measurement noises
   */
  
  //initialize state matrix with zeros
  //x_ << 0, 0, 0, 0;
  
  //measurment matrix for laser
  H_laser_ << 1, 0, 0, 0,
             0, 1, 0, 0;
  
  //initialize transition matrix
  F_ << 1, 0, 1, 0,
        0, 1, 0, 1,
        0, 0, 1, 0,
        0, 0, 0, 1;
  
  ekf_.Init(x_, P_, F_, H_laser_, R_laser_, Q_);
  
  //set the acceleration noise components
  noise_ax = 9;
  noise_ay = 9;
}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      DONE: * Initialize the state ekf_.x_ with the first measurement.
      DONE: * Create the covariance matrix.
      DONE: * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    
    cout << "EKF: " << endl;
    
    // Initialize measurement covariance
    P_ << 1, 0, 0, 0,
          0, 1, 0, 0,
          0, 0, 1000, 0,
          0, 0, 0, 1000;
    ekf_.P_ = P_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      DONE: Convert radar from polar to cartesian coordinates and initialize state.
      */
      
      cout << "Kalman Filter Initialization using RADAR " << endl;

      const float px = measurement_pack.raw_measurements_[0] * cos(measurement_pack.raw_measurements_[1]);
      const float py = measurement_pack.raw_measurements_[0] * sin(measurement_pack.raw_measurements_[1]);
      const float vx = measurement_pack.raw_measurements_[2] * cos(measurement_pack.raw_measurements_[1]);
      const float vy = measurement_pack.raw_measurements_[2] * sin(measurement_pack.raw_measurements_[1]);
      
      ekf_.x_ << px, py, vx, vy;
      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;
      return;
    }
    
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
       DONE: Initialize state.
        */
      
      cout << "Kalman Filter Initialization using LASER " << endl;
      
      //set the state with the initial location and zero velocity
      ekf_.x_ << measurement_pack.raw_measurements_[0], measurement_pack.raw_measurements_[1], 0, 0;
      
      previous_timestamp_ = measurement_pack.timestamp_;
      is_initialized_ = true;
      return;
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     DONE:* Update the state transition matrix F according to the new elapsed time.- Time is measured in seconds.
     DONE:* Update the process noise covariance matrix.
     DONE:* Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  
  //compute the time elapsed between the current and previous measurements
  const float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;	//dt - expressed in seconds
  previous_timestamp_ = measurement_pack.timestamp_;
  
  const float dt_2 = dt * dt;
  const float dt_3 = dt_2 * dt;
  const float dt_4 = dt_3 * dt;
  const float c_3x = dt_3/2*noise_ax;
  const float c_3y = dt_3/2*noise_ay;
  
  //Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt;
  
  //set the process covariance matrix Q
  ekf_.Q_ <<  dt_4/4*noise_ax, 0, c_3x, 0,
  0, dt_4/4*noise_ay, 0, c_3y,
  c_3x, 0, dt_2*noise_ax, 0,
  0, c_3y, 0, dt_2*noise_ay;

  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     DONE: * Use the sensor type to perform the update step.
     DONE: * Update the state and covariance matrices.
   */
//*
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    // Create Jacobian matrix for RADAR update
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  
  }
//*/
  if (measurement_pack.sensor_type_ == MeasurementPackage::LASER){
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
    
  }
  
  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
