//
// Created by rgrandia on 24.04.20.
//

#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/SwingSpline3d.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"
#include "ocs2_switched_model_interface/terrain/TerrainModel.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

/**
 * Linear constraint A_p * p_world + A_v * v_world + b = 0
 */
struct FootNormalConstraintMatrix {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Matrix<scalar_t, 1, 3> positionMatrix = Eigen::Matrix<scalar_t, 1, 3>::Zero();
  Eigen::Matrix<scalar_t, 1, 3> velocityMatrix = Eigen::Matrix<scalar_t, 1, 3>::Zero();
  scalar_t constant = 0;
};

/**
 * Linear inequality constraint A_p * p_world + b >=  0
 */
struct FootTangentialConstraintMatrix {
  Eigen::Matrix<scalar_t, -1, 3> A;
  vector_t b;
};

/**
 * Proves information for the constraint sdf(x) >= minimumDistance
 */
struct SignedDistanceConstraint {
  const SignedDistanceField* signedDistanceField;
  scalar_t minimumDistance;
};

FootTangentialConstraintMatrix tangentialConstraintsFromConvexTerrain(const ConvexTerrain& stanceTerrain, scalar_t margin);

/**
 * Base class for a planned foot phase : Stance or Swing.
 */
class FootPhase {
 public:
  FootPhase() = default;
  virtual ~FootPhase() = default;
  FootPhase(const FootPhase&) = delete;
  FootPhase& operator=(const FootPhase&) = delete;

  /** Returns the contact flag for this phase. Stance phase: True, Swing phase: false */
  virtual bool contactFlag() const = 0;

  /** Returns the unit vector pointing in the normal direction */
  virtual vector3_t normalDirectionInWorldFrame(scalar_t time) const = 0;

  /** Nominal foothold location (upcoming for swinglegs) */
  virtual vector3_t nominalFootholdLocation() const = 0;

  virtual vector3_t getPositionInWorld(scalar_t time) const = 0;
  virtual vector3_t getVelocityInWorld(scalar_t time) const = 0;
  virtual vector3_t getAccelerationInWorld(scalar_t time) const = 0;

  /** Returns the velocity equality constraint formulated in the normal direction */
  virtual FootNormalConstraintMatrix getFootNormalConstraintInWorldFrame(scalar_t time) const = 0;

  /** Returns the position inequality constraints formulated in the tangential direction */
  virtual const FootTangentialConstraintMatrix* getFootTangentialConstraintInWorldFrame() const { return nullptr; };

  virtual SignedDistanceConstraint getSignedDistanceConstraint(scalar_t time) const { return {nullptr, 0.0}; };
};

/**
 * Encodes a planned stance phase on a terrain plane.
 * The normal constraint makes the foot converge to the terrain plane when positionGain > 0.0
 */
class StancePhase final : public FootPhase {
 public:
  explicit StancePhase(const ConvexTerrain& stanceTerrain, scalar_t positionGain = 0.0, scalar_t terrainMargin = 0.0);
  ~StancePhase() override = default;

  bool contactFlag() const override { return true; };
  vector3_t normalDirectionInWorldFrame(scalar_t time) const override;
  vector3_t nominalFootholdLocation() const override;
  vector3_t getPositionInWorld(scalar_t time) const override;
  vector3_t getVelocityInWorld(scalar_t time) const override;
  vector3_t getAccelerationInWorld(scalar_t time) const override;
  FootNormalConstraintMatrix getFootNormalConstraintInWorldFrame(scalar_t time) const override;
  const FootTangentialConstraintMatrix* getFootTangentialConstraintInWorldFrame() const override;

 private:
  const ConvexTerrain* stanceTerrain_;
  FootTangentialConstraintMatrix footTangentialConstraint_;
  scalar_t positionGain_;
};

/**
 * Encodes a swing trajectory between two terrain planes.
 * A cubic spline is designed in both liftoff and target plane. The constraint then smoothly interpolates between the two splines.
 */
class SwingPhase final : public FootPhase {
 public:
  struct SwingEvent {
    scalar_t time;
    scalar_t velocity;
    const TerrainPlane* terrainPlane;
  };

  SwingPhase(SwingEvent liftOff, scalar_t swingHeight, SwingEvent touchDown, const TerrainModel* terrainModel = nullptr,
             scalar_t positionGain = 0.0, scalar_t sdfMidswingMargin = 0.0);
  ~SwingPhase() override = default;

  bool contactFlag() const override { return false; };
  vector3_t normalDirectionInWorldFrame(scalar_t time) const override;
  vector3_t nominalFootholdLocation() const override;
  vector3_t getPositionInWorld(scalar_t time) const override;
  vector3_t getVelocityInWorld(scalar_t time) const override;
  vector3_t getAccelerationInWorld(scalar_t time) const override;
  FootNormalConstraintMatrix getFootNormalConstraintInWorldFrame(scalar_t time) const override;
  SignedDistanceConstraint getSignedDistanceConstraint(scalar_t time) const override;
  const TerrainPlane& getLiftOffFrame() const { return *liftOff_.terrainPlane; };
  const TerrainPlane& getTouchDownFrame() const { return *touchDown_.terrainPlane; };

 private:
  void setFullSwing(scalar_t swingHeight);
  void setHalveSwing(scalar_t swingHeight);

  scalar_t getScaling(scalar_t time) const;

  SwingEvent liftOff_;
  SwingEvent touchDown_;
  std::unique_ptr<SwingSpline3d> motion_;
  scalar_t positionGain_;
  const TerrainModel* terrainModel_;

  std::unique_ptr<QuinticSwing> terrainClearanceMotion_;
  const scalar_t sdfMidswingMargin_;
  const scalar_t startEndMargin_ = 0.02;
};

}  // namespace switched_model
