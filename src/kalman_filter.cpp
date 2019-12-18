#include "kalman_filter.h"
#include <cmath>
#include <iostream>
using std::cout;
using std::endl;
using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  /**
   * TODO: predict the state
   */
  cout<<"Predict Step:"<<endl;
  cout<<"Before Predict: "<<endl;
  cout<<"x_: "<<x_<<endl;
  cout<<"F_: "<<F_<<endl;
  cout<<"P_: "<<P_<<endl;
  cout<<"Q_: "<<Q_<<endl;
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose()+Q_;
  cout<<"After Predict"<<endl;
  cout<<"x_: "<<x_<<endl;
  cout<<"P_: "<<P_<<endl;
}
void KalmanFilter::UpdateCommon(const VectorXd &z, const VectorXd& y) {
 
  cout<<"Common Update Function"<<endl;
  cout<<"R_: "<<R_<<endl;
  MatrixXd S = H_*P_*H_.transpose()+R_;
  cout<<"S: "<<S<<endl;
  MatrixXd K = P_*H_.transpose()*S.inverse();
  cout<<"k: "<<K<<endl;
  MatrixXd I = MatrixXd::Identity(x_.size(), x_.size());
  x_ = x_ + K*y;
  cout<<"After Update"<<endl;
  cout<<"x_: "<<x_<<endl;
  P_ = (I - K*H_)*P_;
  cout<<"P_: "<<P_<<endl;
}
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  cout<<"Update Function"<<endl;
  cout<<"x_: "<<x_<<endl;
  cout<<"H_: "<<H_<<endl;
  cout<<"z_: "<<z<<endl;
  VectorXd y = z - H_*x_;
  cout<<"y: "<<y<<endl;
  UpdateCommon(z, y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equationsS
   */
  cout<<"EKF Update Function"<<endl;
  cout<<"x_: "<<x_<<endl;
  cout<<"H_: "<<H_<<endl;
  cout<<"z: "<<z<<endl;
  float px = x_(0)+1e-6;
  float py = x_(1)+1e-6;
  float vx = x_(2)+1e-6;
  float vy = x_(3)+1e-6;
  VectorXd h_x(3);
  h_x<< sqrt(px*px+py*py), atan2(py, px), (px*vx+py*vy)/sqrt(px*px+py*py);
  cout<<"h_x: "<<h_x<<endl;
  VectorXd y = z - h_x;
  cout<<"y: "<<y<<endl;
  y(1) = radianRound(y(1));
  cout<<"round y: "<<y<<endl;
  // H_ = Tools::CalculateJacobian(x_);
  UpdateCommon(z, y);
}
