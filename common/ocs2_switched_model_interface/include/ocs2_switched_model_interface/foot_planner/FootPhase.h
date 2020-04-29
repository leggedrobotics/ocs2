//
// Created by rgrandia on 24.04.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SplineCpg.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

/**
 * Linear constraint A_p * p + A_v * v + b = 0
 */
struct FootNormalConstraintMatrix {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<scalar_t, 1, 3> positionMatrix;
  Eigen::Matrix<scalar_t, 1, 3> velocityMatrix;
  scalar_t constant;
};

class FootPhase {
 public:
  virtual FootNormalConstraintMatrix getFootNormalConstraintInWorldFrame(scalar_t time, scalar_t positionGain) const = 0;
};

class StancePhase final : public FootPhase {
 public:
  StancePhase(const TerrainPlane& stanceTerrain);
  FootNormalConstraintMatrix getFootNormalConstraintInWorldFrame(scalar_t time, scalar_t positionGain) const override;

 private:
  const TerrainPlane* stanceTerrain_;
};

class SwingPhase final : public FootPhase {
 public:
  struct SwingEvent {
    scalar_t time;
    scalar_t velocity;
    const TerrainPlane* terrainPlane;
  };

  SwingPhase(SwingEvent liftOff, scalar_t swingHeight, SwingEvent touchDown);
  FootNormalConstraintMatrix getFootNormalConstraintInWorldFrame(scalar_t time, scalar_t positionGain) const override;

 private:
  void setFullSwing(scalar_t swingHeight);
  void setHalveSwing(scalar_t swingHeight);

  scalar_t getScaling(scalar_t time) const;

  SwingEvent liftOff_;
  SwingEvent touchDown_;
  std::unique_ptr<SplineCpg> liftOffMotion_;
  std::unique_ptr<SplineCpg> touchdownMotion_;
};

}  // namespace switched_model
