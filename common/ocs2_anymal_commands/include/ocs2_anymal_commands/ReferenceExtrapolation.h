//
// Created by rgrandia on 02.03.20.
//

#pragma once

#include <grid_map_core/GridMap.hpp>

#include <ocs2_switched_model_interface/terrain/TerrainPlane.h>

namespace switched_model {

struct BaseReferenceHorizon {
  double dt;
  size_t N;
};

struct BaseReferenceState {
  double t0;
  Eigen::Vector3d positionInWorld;
  Eigen::Vector3d eulerXyz;
};

struct BaseReferenceCommand {
  double headingVelocity;
  double lateralVelocity;
  double yawRate;
  double baseHeight;
};

struct Base2dReferenceTrajectory {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<double> time;
  std::vector<double> yaw;
  std::vector<Eigen::Vector2d> positionInWorld;
};

struct BaseReferenceTrajectory {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  std::vector<double> time;
  std::vector<Eigen::Vector3d> eulerXyz;
  std::vector<Eigen::Vector3d> positionInWorld;
  std::vector<Eigen::Vector3d> linearVelocityInWorld;
  std::vector<Eigen::Vector3d> angularVelocityInWorld;
};

Eigen::Vector2d velocityCommand2dInWorld(double headingVelocity, double lateralVelocity, double yaw);

Base2dReferenceTrajectory generate2DExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                                const BaseReferenceCommand& command);

BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                          const BaseReferenceCommand& command,
                                                          const switched_model::TerrainPlane& projectedHeadingFrame);

BaseReferenceTrajectory generateExtrapolatedBaseReference(const BaseReferenceHorizon& horizon, const BaseReferenceState& initialState,
                                                          const BaseReferenceCommand& command, const grid_map::GridMap& gridMap,
                                                          double nominalStanceWidthInHeading, double nominalStanceWidthLateral);

}  // namespace switched_model
