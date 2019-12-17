#include "kalman_filter.h"
#include <cmath>
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
  x_ = F_*x_;
  P_ = F_*P_*F_.transpose()+Q_;
}
void KalmanFilter::UpdateCommon(const VectorXd &z, const VectorXd y) {
 
  VectorXd S = H_*P_*H_.transpose()+R_;
  VectorXd K = P_*H_.transpose()*S_.inverse();
  VectorXd I = MatrixXd::Identity(x_.size(), x_.size());
  x_ = x_ + K*y;
  P_ = (I - K*H_)*P_;
}
void KalmanFilter::Update(const VectorXd &z) {
  /**
   * TODO: update the state by using Kalman Filter equations
   */
  VectorXd y = z - H_*x_;
  UpdateCommon(z, y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
   * TODO: update the state by using Extended Kalman Filter equations
   */
  float px = x_(0)+1e6;
  float py = x_(1)+1e6;
  float vx = x_(2)+1e6;
  float vy = x_(3)+1e6;
  VectorXd h_x(3);
  h_x<< sqrt(px*px+py*py), atan2(py, px), (px*vx+py*vy)/sqrt(px*px+py*py);
  VectorXd y = z - h_x;
  // H_ = Tools::CalculateJacobian(x_);
  UpdateCommon(z, y);
}
