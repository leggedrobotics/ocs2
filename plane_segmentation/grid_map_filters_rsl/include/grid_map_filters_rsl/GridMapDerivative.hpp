/**
 * @file        GridMapDerivative.hpp
 * @authors     Fabian Jenelten
 * @date        20.05, 2021
 * @affiliation ETH RSL
 * @brief       Computes first and second order derivatives for a grid map. Intended to be used for online applications (i.e, derivatives
 *              are not precomputed but computed at the time and location where they are needed).
 */

#pragma once

// grid_map_core
#include <grid_map_core/grid_map_core.hpp>

namespace grid_map {
namespace derivative {

class GridMapDerivative {
 private:
  static constexpr int kernelSize_ = 5;
  using Gradient = Eigen::Vector2d;
  using Curvature = Eigen::Matrix2d;
  using Kernel = Eigen::Matrix<float, kernelSize_, 1>;

 public:
  GridMapDerivative();
  ~GridMapDerivative() = default;

  /**
   * @brief Initialize function
   * @param res     resolution of the grid map
   * @return        true iff successful
   */
  bool initialize(float res);

  /**
   * @brief Compute local gradient using grid map. Gradient is set to zero if the index is outside of the grid map.
   * @param gridMap     The grid map
   * @param gradient    gradient vector in world frame
   * @param index       grid map index
   * @param H           grid map corresponding to layer (Eigen implementation)
   */
  void estimateGradient(const grid_map::GridMap& gridMap, Gradient& gradient, const grid_map::Index& index,
                        const grid_map::Matrix& H) const;

  /**
   * @brief Compute local height gradient and curvature using grid map. Gradient and curvature are set to zero if the index is outside of
   * the grid map.
   * @param gridMap         The grid map
   * @param gradient        gradient vector in world frame
   * @param curvature       curvature matrix in world frame
   * @param index           grid map index
   * @param H               grid map corresponding to layer (Eigen implementation)
   */
  void estimateGradientAndCurvature(const grid_map::GridMap& gridMap, Gradient& gradient, Curvature& curvature,
                                    const grid_map::Index& index, const grid_map::Matrix& H) const;

 private:
  /**
   * @brief Return center of kernel s.t. kernel does not reach boundaries of grid map. By default returns 0 (equals central difference).
   * @param gridMap         The grid map
   * @param centerIndex     index at which we want to apply the kernel
   * @param maxKernelId     max index of kernel in positive direction
   * @return                center of kernel
   */
  static Eigen::Vector2i getKernelCenter(const grid_map::GridMap& gridMap, const grid_map::Index& centerIndex, int maxKernelId);

  //! First order derivative kernel.
  Kernel kernelD1_;

  //! Second order derivative kernel.
  Kernel kernelD2_;
};
}  // namespace derivative
}  // namespace grid_map
