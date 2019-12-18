#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;
using std::cout;
using std::endl;
float radianRound(float rad)
{

  while(rad > PI)
    rad -= 2*PI;

  while(rad < -PI)
    rad += 2*PI;

  return rad;
}
Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  VectorXd rmse(4);
  rmse << 0,0,0,0;

  // check the validity of the following inputs:
  //  * the estimation vector size should not be zero
  //  * the estimation vector size should equal ground truth vector size
  if (estimations.size() != ground_truth.size()
      || estimations.size() == 0) {
    cout << "Invalid estimation or ground_truth data" << endl;
    return rmse;
  }

  // accumulate squared residuals
  for (unsigned int i=0; i < estimations.size(); ++i) {

    VectorXd residual = estimations[i] - ground_truth[i];

    // coefficient-wise multiplication
    residual = residual.array()*residual.array();
    rmse += residual;
  }

  // calculate the mean
  rmse = rmse/estimations.size();

  // calculate the squared root
  rmse = rmse.array().sqrt();

  // return the result
  return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  MatrixXd Hj(3,4);
  float px = x_state(0)+1e6;
  float py = x_state(1)+1e6;
  float vx = x_state(2)+1e6;
  float vy = x_state(3)+1e6;
  float sq_p = px*px+py*py;
  float sqrt_p = sqrt(px*px+py*py);
  // compute the Jacobian matrix

  Hj << px/sqrt_p, py/sqrt_p, 0, 0,
        -py/sq_p, px/sq_p, 0, 0,
        py*(vx*py-vy*px)/pow(sq_p, -2/3.0),px*(vy*px-vx*py)/pow(sq_p, -2/3.0),px/sqrt_p, py/sqrt_p;
  return Hj;
}
