#include "kalman_filter.h"
#include <math.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

double SNormalizeAngle(double phi);

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in; //object state
  P_ = P_in; //object covariance matrix
  F_ = F_in; //state transition matrix
  H_ = H_in; //measurement matrix
  R_ = R_in; //measurement covariance matrix
  Q_ = Q_in; //process covariance matrix
}

void KalmanFilter::Predict() {
  /**
  TODO:
    * predict the state
  */
  x_ = (F_*x_); //no external motion u
  MatrixXd F_t = F_.transpose();
  P_ = F_ * P_ * F_t + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd y = z - H_ * x_;
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd S_i = S.inverse();
  MatrixXd K = P_ * H_t * S_i;

  // new state. solution from Laser measurements
  x_ = x_ + K * y;
  long x_size = x_.size();
  //  Eigen::MatrixXd I = Eigen::MatrixXd::Constant(x_size,x_size,1.0);
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K* H_)* P_;
}

double SNormalizeAngle(double phi)
{
  const double Max = M_PI;
  const double Min = -M_PI;

    return phi < Min
		 ? Max + std::fmod(phi - Min, Max - Min)
		 : std::fmod(phi - Min, Max - Min) + Min;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  // The predicted measurement vector x is a vector containing values in the form [px, py, vx, vy].
  // The radar sensor will output values in polar coordinates.
  float rho = sqrt(x_(0)*x_(0) + x_(1)*x_(1));
  // Returns the principal value of the arc tangent of y/x, expressed in radians.
  float phi = atan2(x_(1),x_(0));
  float rho_dot;
  // To avoid division by zero
  if (fabs(rho) < 0.0001) {
    rho_dot = 0;
  } else {
    rho_dot = (x_(0)*x_(2) + x_(1)*x_(3))/rho;
  }
  
  // The predicted location x from Cartesian coordinates to polar coordinates
  VectorXd z_pred(3);
  z_pred << rho, phi, rho_dot;

  // calculate y for the radar sensor
  VectorXd y = z - z_pred;

  // Normalization to reduce RMSE resulted from wrong predictions across y = 0
  // This occur after subtraction of predicted z when y(1) can be greater than pi
  y[1] = SNormalizeAngle(y[1]);
  
  MatrixXd H_t = H_.transpose();
  MatrixXd S = H_ * P_ * H_t + R_;
  MatrixXd S_i = S.inverse();
  MatrixXd K = P_ * H_t * S_i;

  //  new state. solution from Laser measurements
  x_ = x_ + K * y;
  long x_size = x_.size();
  //  Eigen::MatrixXd I = Eigen::MatrixXd::Constant(x_size,x_size,1.0);
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K* H_)* P_;
}
