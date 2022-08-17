#ifndef KALMAN_FILTER_DATA_TYPES_HPP_
#define KALMAN_FILTER_DATA_TYPES_HPP_
#include <Eigen/Dense>

namespace kalman_filter {
template <typename DataType>
using VectorXt = Eigen::Matrix<DataType, Eigen::Dynamic, 1>;

template <typename DataType>
using MatrixXt = Eigen::Matrix<DataType, Eigen::Dynamic, Eigen::Dynamic>;

template <typename DataType, int size>
using SquareMatrix = Eigen::Matrix<DataType, size, size>;
} // namespace kalman_filter
#endif