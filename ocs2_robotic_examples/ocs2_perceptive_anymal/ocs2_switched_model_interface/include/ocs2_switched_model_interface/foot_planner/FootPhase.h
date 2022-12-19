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
 * Linear inequality constraint A_p * p_world + b >=  0
 */
struct FootTangentialConstraintMatrix {
  Eigen::Matrix<scalar_t, -1, 3> A;
  vector_t b;
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

  /** Convex terrain that constrains the foot placement. (null for swinglegs) */
  virtual const ConvexTerrain* nominalFootholdConstraint() const { return nullptr; };

  /** Foot reference position in world frame */
  virtual vector3_t getPositionInWorld(scalar_t time) const = 0;

  /** Foot reference velocity in world frame */
  virtual vector3_t getVelocityInWorld(scalar_t time) const = 0;

  /** Foot reference acceleration in world frame */
  virtual vector3_t getAccelerationInWorld(scalar_t time) const = 0;

  /** Returns the position inequality constraints formulated in the tangential direction */
  virtual const FootTangentialConstraintMatrix* getFootTangentialConstraintInWorldFrame() const { return nullptr; };

  virtual scalar_t getMinimumFootClearance(scalar_t time) const { return 0.0; };
};

/**
 * Encodes a planned stance phase on a terrain plane.
 * The normal constraint makes the foot converge to the terrain plane when positionGain > 0.0
 */
class StancePhase final : public FootPhase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit StancePhase(ConvexTerrain stanceTerrain, scalar_t terrainMargin = 0.0);
  ~StancePhase() override = default;

  bool contactFlag() const override { return true; };
  vector3_t normalDirectionInWorldFrame(scalar_t time) const override;
  vector3_t nominalFootholdLocation() const override;
  const ConvexTerrain* nominalFootholdConstraint() const override;
  vector3_t getPositionInWorld(scalar_t time) const override;
  vector3_t getVelocityInWorld(scalar_t time) const override;
  vector3_t getAccelerationInWorld(scalar_t time) const override;
  const FootTangentialConstraintMatrix* getFootTangentialConstraintInWorldFrame() const override;

 private:
  ConvexTerrain stanceTerrain_;
  const vector3_t nominalFootholdLocation_;
  const vector3_t surfaceNormalInWorldFrame_;
  const FootTangentialConstraintMatrix footTangentialConstraint_;
};

/**
 * Encodes a swing trajectory between two terrain planes.
 * A spline based swing motion is designed in both liftoff and target plane.
 */
class SwingPhase final : public FootPhase {
 public:
  struct SwingEvent {
    scalar_t time;
    scalar_t velocity;
    const TerrainPlane* terrainPlane;
  };

  struct SwingProfile {
    struct Node {
      /// Time progress in the swing phase in [0, 1]
      scalar_t phase = 0.5;
      /// Swing height in normal direction
      scalar_t swingHeight = 0.1;
      /// Velocity in normal direction
      scalar_t normalVelocity = 0.0;
      /// Swing progress in tangential direction in [0, 1]
      scalar_t tangentialProgress = 0.5;
      /// Tangantial velocity as a factor of the average velocity. The tangential velocity will be: velocityFactor * swingdistance / dt
      scalar_t tangentialVelocityFactor = 1.0;
    };

    /// Height / velocity profile
    std::vector<Node> nodes;
    /// Desired SDF clearance at the middle of the swing phase.
    scalar_t sdfMidswingMargin = 0.0;
    /// Desired SDF clearance at liftoff and touchdown. Slight negative margin allows a bit of ground penetration
    scalar_t sdfStartEndMargin = -0.02;
    /// Limits the amount of additional swing height from terrain adaptation
    scalar_t maxSwingHeightAdaptation = 0.3;
  };

  /**
   * Construct a swing phase:
   *    Creates a 3D swing reference motion
   *    Creates a 1D clearance profile for SDF based obstacle avoidance.
   * @param liftOff : Information about the liftoff event.
   * @param touchDown : Information about the touchdown event.
   * @param SwingProfile : Settings to shape the swing profile
   * @param terrainModel : (optional) Pointer to the terrain model. Terrain model should be kept alive externall as long as the swingphase
   * object exists. Will extract SDF and obstacle information from the terrain.
   */
  SwingPhase(SwingEvent liftOff, SwingEvent touchDown, const SwingProfile& swingProfile, const TerrainModel* terrainModel = nullptr);
  ~SwingPhase() override = default;

  bool contactFlag() const override { return false; };
  vector3_t normalDirectionInWorldFrame(scalar_t time) const override;
  vector3_t nominalFootholdLocation() const override;
  vector3_t getPositionInWorld(scalar_t time) const override;
  vector3_t getVelocityInWorld(scalar_t time) const override;
  vector3_t getAccelerationInWorld(scalar_t time) const override;
  scalar_t getMinimumFootClearance(scalar_t time) const override;

 private:
  void setFullSwing(const SwingProfile& swingProfile, const TerrainModel* terrainModel);
  void setHalveSwing(const SwingProfile& swingProfile, const TerrainModel* terrainModel);

  scalar_t getScaling(scalar_t time) const;

  SwingEvent liftOff_;
  SwingEvent touchDown_;

  std::unique_ptr<SwingSpline3d> motion_;
  std::unique_ptr<QuinticSwing> terrainClearanceMotion_;
};

/**
 * Encodes a swing trajectory derived directly from the reference motion
 */
class ExternalSwingPhase final : public FootPhase {
 public:
  ExternalSwingPhase(std::vector<scalar_t> timeTrajectory, std::vector<vector3_t> positionTrajectory,
                     std::vector<vector3_t> velocityTrajectory);
  ~ExternalSwingPhase() override = default;

  bool contactFlag() const override { return false; };
  vector3_t normalDirectionInWorldFrame(scalar_t time) const override { return {0.0, 0.0, 1.0}; }
  vector3_t nominalFootholdLocation() const override { return positionTrajectory_.back(); }
  vector3_t getPositionInWorld(scalar_t time) const override;
  vector3_t getVelocityInWorld(scalar_t time) const override;
  vector3_t getAccelerationInWorld(scalar_t time) const override { return vector3_t::Zero(); }
  scalar_t getMinimumFootClearance(scalar_t time) const override { return 0.0; }

 private:
  std::vector<scalar_t> timeTrajectory_;
  std::vector<vector3_t> positionTrajectory_;
  std::vector<vector3_t> velocityTrajectory_;
};

}  // namespace switched_model
