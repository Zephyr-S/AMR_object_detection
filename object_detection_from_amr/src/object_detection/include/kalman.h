#pragma once

#include <eigen3/Eigen/Dense>

class KalmanFilter
{
public:
  KalmanFilter(uint dim_in, uint dim_out, uint dim_state) : l(dim_in), m(dim_out), n(dim_state) {
    using namespace Eigen;

    A = MatrixXd::Identity(n,n);
    B = MatrixXd::Zero(n,l);
    C = MatrixXd::Zero(m,n);

    Q = MatrixXd::Identity(n,n);
    R = MatrixXd::Identity(m,m);
    P = MatrixXd::Identity(n,n);

    K = MatrixXd::Identity(n,m);
    I = MatrixXd::Identity(n,n);

    u = VectorXd::Zero(l);
    q_pred = VectorXd::Zero(n);
    q_est = VectorXd::Zero(n);
    y = VectorXd::Zero(m);
  }

  void predictState() {
    q_pred = A * q_est + B * u;
    P = A * P * A.transpose() + Q;
    q_est = q_pred;
  }

  void correctState() {

    Eigen::MatrixXd G;
    G = C * P * C.transpose() + R;
    K = P * C.transpose() * G.inverse();
    q_est = q_pred + K * (y - C * q_pred);
    P = (I - K * C) * P;
  }

  void updateState() {
    predictState();
    correctState();
  }

  // System matrices:
  Eigen::MatrixXd A; //State
  Eigen::MatrixXd B; //Input
  Eigen::MatrixXd C; //Output

  // Covariance matrices:
  Eigen::MatrixXd Q; // Process
  Eigen::MatrixXd R; // Measurement
  Eigen::MatrixXd P; // Estimate error

  // Kalman gain matrix:
  Eigen::MatrixXd K;

  // Identity matrix
  Eigen::MatrixXd I;

  // Signals:
  Eigen::VectorXd u;      // Input
  Eigen::VectorXd q_pred; // Predicted state
  Eigen::VectorXd q_est;  // Estimated state
  Eigen::VectorXd y;      // Measurement

private:
  // Dimensions:
  uint l;             // Input
  uint m;             // Output
  uint n;             // State
};