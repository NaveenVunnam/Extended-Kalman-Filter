#include "kalman_filter.h"
#define PI 3.14156712
using Eigen::MatrixXd;
using Eigen::VectorXd;


// Please note that the Eigen library does not initialize
// VectorXd or MatrixXd objects with zeros upon creation.

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
    * predict the state
  */
  x_ = F_*x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_*P_*Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z-H_*x_;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;
  // Update the state
  x_ = x_+(K*y);
  long xsize = x_.size();
  MatrixXd I = MatrixXd::Identity(xsize, xsize);
  P_ = (I-K*H_)*P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd zpred(3);
  float px = x_(0);
  float py = x_(1);
  float vx = x_(2);
  float vy = x_(3);
  float rh = sqrt(px*px+py*py);
  float theta = atan2(py, px);
  float rhdot = (px*vx+py*vy)/rh;
  // Find the predicted measurement
  zpred << rh, theta, rhdot;
  // Find the error in measurement
  VectorXd y = z-zpred;
  if (y(1) < -PI) {
    y(1) += 2.f*PI;
  }
  if (y(1) > PI) {
    y(1) -= 2.f*PI;
  }
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_*P_*Ht+R_;
  MatrixXd Si = S.inverse();
  MatrixXd K = P_*Ht*Si;
  // Update the state
  x_ = x_ + K*y;
  long xsize = x_.size();
  MatrixXd I = MatrixXd::Identity(xsize, xsize);
  P_ = (I-K*H_)*P_;
}
