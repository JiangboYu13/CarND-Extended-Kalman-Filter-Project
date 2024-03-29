#include "FusionEKF.h"
#include <iostream>
#include "Eigen/Dense"
#include "tools.h"
#include <cmath>
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::cout;
using std::endl;
using std::vector;

/**
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
   * TODO: Finish initializing the FusionEKF.
   * TODO: Set the process and measurement noises
   */
  H_laser_ << 1, 0, 0, 0,
          0, 1, 0, 0;

  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ <<  1, 0, 0, 0,
          	0, 1, 0, 0,
          	0, 0, 1000, 0,
          	0, 0, 0, 1000;
  noise_ax = 9.;
  noise_ay = 9.;
}

/**
 * Destructor.
 */
FusionEKF::~FusionEKF() {}
MatrixXd FusionEKF::obtainF(float dt)
{
  MatrixXd F(4,4);
  F<< 1, 0, dt, 0 ,
     0, 1, 0 , dt,
     0, 0, 1 , 0 ,
     0, 0, 0 , 1 ;
  return F;
}
MatrixXd FusionEKF::obtainQ(float dt)
{
  float dt2 = pow(dt, 2);
  float dt3 = pow(dt, 3);
  float dt4 = pow(dt, 4);
  MatrixXd Q = MatrixXd(4, 4);
  Q <<  dt4/4*noise_ax, 0,              dt3/2*noise_ax, 0,
       0,              dt4/4*noise_ay, 0,              dt3/2*noise_ay,
       dt3/2*noise_ax, 0,              dt2*noise_ax,   0,
       0,              dt3/2*noise_ay, 0,              dt2*noise_ay;
  return Q;

}
void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {
  /**
   * Initialization
   */
  cout<<"Measurement: ";
  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) cout<<"Radar";
  else cout<<"Lidar";
  cout<<endl;
  cout << "raw measurement = " << measurement_pack.raw_measurements_ << endl;
  if (!is_initialized_) {
    previous_timestamp_ = measurement_pack.timestamp_;

    /**
     * TODO: Initialize the state ekf_.x_ with the first measurement.
     * TODO: Create the covariance matrix.
     * You'll need to convert radar from polar to cartesian coordinates.
     */

    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 0, 0;
    
    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      cout << "Radar Initialization" << endl;
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      float rhodot = measurement_pack.raw_measurements_[2];
      ekf_.x_(0)= rho*cos(phi);
      ekf_.x_(1)= rho*sin(phi);
      
      // TODO: Convert radar from polar to cartesian coordinates 
      //         and initialize state.
     
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // TODO: Initialize state.
      cout << "Lidar Initialization" << endl;
      ekf_.x_(0) = measurement_pack.raw_measurements_[0];
      ekf_.x_(1) = measurement_pack.raw_measurements_[1];

    
    }
    cout << "x_ = " << ekf_.x_ << endl;
    cout << "P_ = " << ekf_.P_ << endl;
    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /**
   * Prediction
   */

  /**
   * TODO: Update the state transition matrix F according to the new elapsed time.
   * Time is measured in seconds.
   * TODO: Update the process noise covariance matrix.
   * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  cout<<"Predict"<<endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = measurement_pack.timestamp_;
  cout<<"dt="<<dt<<endl;
  if (dt>1e-6){
    ekf_.F_ = obtainF(dt);
    ekf_.Q_ = obtainQ(dt);
    ekf_.Predict();
  }
  /**
   * Update
   */
  
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
  /**
   * TODO:
   * - Use the sensor type to perform the update step.
   * - Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // TODO: Radar updates
    cout << "Radar Update" << endl;
    ekf_.R_ = R_radar_;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);

  } else {
    
    // TODO: Laser updates
    cout << "Lidar Update" << endl;
    ekf_.R_ = R_laser_;
    ekf_.H_ = H_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);

  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
