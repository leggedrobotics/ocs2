/**
 * @file        GridMapDerivative.cpp
 * @authors     Fabian Jenelten
 * @date        20.05, 2021
 * @affiliation ETH RSL
 * @brief       Computes first and second order derivatives for a grid map. Intended to be used for online applications (i.e, derivatives
 *              are not precomputed but computed at the time and location where they are needed).
 */

// grid map filters rsl.
#include <grid_map_filters_rsl/GridMapDerivative.hpp>

namespace grid_map {
namespace derivative {
constexpr int GridMapDerivative::kernelSize_;

GridMapDerivative::GridMapDerivative() : kernelD1_(Kernel::Zero()), kernelD2_(Kernel::Zero()) {}

bool GridMapDerivative::initialize(float res) {
  // Central finite difference: https://en.wikipedia.org/wiki/Finite_difference_coefficient
  kernelD1_ << -1.0 / 12.0, 2.0 / 3.0, 0.0, -2.0 / 3.0, 1.0 / 12.0;
  kernelD2_ << -1.0 / 12.0, 4.0 / 3.0, -5.0 / 2.0, 4.0 / 3.0, -1.0 / 12.0;
  kernelD1_ /= res;
  kernelD2_ /= (res * res);
  return res > 0.0;
}

void GridMapDerivative::estimateGradient(const grid_map::GridMap& gridMap, Gradient& gradient, const grid_map::Index& index,
                                         const grid_map::Matrix& H) const {
  // Indices.
  constexpr auto maxKernelId = (kernelSize_ - 1) / 2;
  const Eigen::Vector2i centerId = getKernelCenter(gridMap, index, maxKernelId);

  // Gradient.
  gradient.x() = kernelD1_.dot(H.block<kernelSize_, 1>(index.x() - maxKernelId + centerId.x(), index.y()));
  gradient.y() = kernelD1_.dot(H.block<1, kernelSize_>(index.x(), index.y() - maxKernelId + centerId.y()));
}

void GridMapDerivative::estimateGradientAndCurvature(const grid_map::GridMap& gridMap, Gradient& gradient, Curvature& curvature,
                                                     const grid_map::Index& index, const grid_map::Matrix& H) const {
  // Note: Block implementation is twice as fast as iterating over the grid cells.

  // Indices.
  constexpr auto maxKernelId = (kernelSize_ - 1) / 2;
  const Eigen::Vector2i centerId = getKernelCenter(gridMap, index, maxKernelId);
  const Eigen::Vector2i shiftedId(index.x() - maxKernelId + centerId.x(), index.y() - maxKernelId + centerId.y());

  // Gradient in x for different y (used for computing the cross hessian).
  Kernel gradientYArray;
  for (auto idY = 0; idY < kernelSize_; ++idY) {
    gradientYArray[idY] = kernelD1_.dot(H.block<kernelSize_, 1>(shiftedId.x(), shiftedId.y() + idY));
  }

  // Gradient.
  gradient(0U) = kernelD1_.dot(H.block<kernelSize_, 1>(shiftedId.x(), index.y()));
  gradient(1U) = kernelD1_.dot(H.block<1, kernelSize_>(index.x(), shiftedId.y()));

  // Curvature.
  curvature(0U, 0U) = kernelD2_.dot(H.block<kernelSize_, 1>(shiftedId.x(), index.y()));
  curvature(1U, 1U) = kernelD2_.dot(H.block<1, kernelSize_>(index.x(), shiftedId.y()));
  curvature(0U, 1U) = kernelD1_.dot(gradientYArray);
  curvature(1U, 0U) = curvature(0U, 1U);
}

Eigen::Vector2i GridMapDerivative::getKernelCenter(const grid_map::GridMap& gridMap, const grid_map::Index& centerIndex, int maxKernelId) {
  constexpr auto minId = 0;
  Eigen::Vector2i centerId;
  for (auto dim = 0; dim < 2; ++dim) {
    const auto maxId = gridMap.getSize()(dim) - 1;
    centerId(dim) = -std::min(centerIndex(dim) - maxKernelId, minId) - std::max(centerIndex(dim) + maxKernelId - maxId, 0);
  }
  return centerId;
}

}  // namespace derivative
}  // namespace grid_map
