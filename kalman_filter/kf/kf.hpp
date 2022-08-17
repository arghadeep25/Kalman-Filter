#ifndef KALMAN_FILTER_HPP_
#define KALMAN_FILTER_HPP_

#include "utils/types.hpp"
#include <Eigen/Dense>
#include <iostream>

namespace kalman_filter {
/**
 * @brief Kalman filtering, also known as linear quadratic estimation (LQE),
 *        is an algorithm that uses a series of measurements observed over
 *        time, including statistical noise and other inaccuracies, and
 *        produces estimates of unknown variables that tend to be more accurate
 *        than those based on a single measurement alone, by estimating a joint
 *        probability distribution over the variables for each timeframe
 *
 *
 * @tparam DataType
 */
template <typename DataType> class KalmanFilter {
public:
  /**
   * @brief Default constructor for Kalman Filter
   *        n is the number of states
   *        m is the number of measurements
   *        c is the number of control inputs
   * @param dt - Time Step
   * @param A - State Transition Model              [nxn matrix]
   * @param B - Control Input                       [nxc matrix]
   * @param H - Observation Model                   [mxn matrix]
   * @param Q - Process Noise Covariance            [nxn matrix]
   * @param R - Observation Noise Covariance        [mxm matrix]
   * @param P - Predicted Estimate Error Covariance [nxn matrix]
   */
  KalmanFilter(const MatrixXt<DataType> &A, const MatrixXt<DataType> &B,
               const MatrixXt<DataType> &H, const MatrixXt<DataType> &Q,
               const MatrixXt<DataType> &R, const MatrixXt<DataType> &P)
      : A(A), B(B), H(H), Q(Q), R(R), P0(P), n(A.rows()), initialized(false) {}

public:
  /**
   * @brief Default destructor
   */
  ~KalmanFilter() = default;

public:
  /**
   * @brief Function to set the initial parameters trivially
   *        if the parameters are not initialized
   */
  inline void init() {
    x_hat = VectorXt<DataType>(n).setZero();
    P = P0;
    I = MatrixXt<DataType>(n, n).setIdentity();
    initialized = true;
  }

public:
  /**
   * @brief Function to initialize the state vector for the
   *        first iteration.
   * @param x0 Initial values of the state vector
   */
  inline void init(const VectorXt<DataType> &x0) {
    x_hat = VectorXt<DataType>(n).setZero();
    x_hat = x0;
    P = P0;
    I = MatrixXt<DataType>(n, n).setIdentity();
    initialized = true;
  }

public:
  /**
   * @brief Function to update the prediction
   *        \begin{equation}\hat{x}_{0,0}, {P}_{0,0}\end{equation}
   *        \begin{equation}{K}_{n} = {P}_{n,n-1}H^{T}(HP_{n,n-1}H^{T} + R_{n}
   * )^{-1}\end{equation}
   * @param z Control Input
   */
  inline void predict(const VectorXt<DataType> &u) {
    if (!initialized) {
      std::cout
          << "Filter is not initialized!! Initializing with Trivial State";
      init();
    }
    x_hat = A * x_hat + B * u;
    P = A * P * A.transpose() + Q;
  }

public:
  /**
   * @brief Function to update KF based on the measurement
   * @param z Measurement value
   */
  inline void update(const VectorXt<DataType> &z) {
    K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    x_hat += K * (z - H * x_hat);
    P = (I - K * H) * P;
  }

public:
  /**
   * @brief Function to update the state transition model
   * @param A State Transition Model
   */
  inline void updateDynamics(const MatrixXt<DataType> &A) { this->A = A; }

public:
  /**
   * @brief Function to update the control
   * @param H Observation Model
   */
  inline void updateObservation(const MatrixXt<DataType> &H) { this->H = H; }

public:
  /**
   * @brief Function to get the state predictions
   * @return State prediction
   */
  inline VectorXt<DataType> state() { return x_hat; }

public:
  /**
   * @brief Function to get the state covariance matrix
   * @return State Covariance Matrix
   */
  inline MatrixXt<DataType> stateCovar() { return P; }

private:
  MatrixXt<DataType> A;  // State Transition Model
  MatrixXt<DataType> B;  // Control Input
  MatrixXt<DataType> H;  // Observation Model
  MatrixXt<DataType> Q;  // Process Noise Covariance
  MatrixXt<DataType> R;  // Observation Noise Covariance
  MatrixXt<DataType> P;  // Predicted Estimate Error Covariance
  MatrixXt<DataType> K;  // Kalman Gain
  MatrixXt<DataType> P0; // Initial Estimate Error Covariance
  MatrixXt<DataType> I;  // Identity Matrix

  VectorXt<DataType> x_hat; // State Vector
  int n;
  bool initialized;
};
} // namespace kalman_filter
#endif