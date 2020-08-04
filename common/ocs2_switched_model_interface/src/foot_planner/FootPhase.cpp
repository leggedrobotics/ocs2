//
// Created by rgrandia on 24.04.20.
//

#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"

namespace switched_model {

FootNormalConstraintMatrix computeFootNormalConstraint(scalar_t feedforwardVelocityInNormalDirection, scalar_t desiredTerrainDistance,
                                                       const TerrainPlane& terrainPlane, scalar_t positionGain) {
  // in surface normal direction : v_foot = v_ff - kp * (p_foot - p_des)
  // ==> (n')* v_foot + (kp* n')* p_foot - (v_ff + kp* p_des) = 0
  const vector3_t surfaceNormal = surfaceNormalInWorld(terrainPlane);

  FootNormalConstraintMatrix footNormalConstraint;
  footNormalConstraint.velocityMatrix = surfaceNormal.transpose();
  footNormalConstraint.positionMatrix = positionGain * surfaceNormal.transpose();
  footNormalConstraint.constant =
      -feedforwardVelocityInNormalDirection - positionGain * (surfaceNormal.dot(terrainPlane.positionInWorld) + desiredTerrainDistance);
  return footNormalConstraint;
}

StancePhase::StancePhase(const TerrainPlane& stanceTerrain) : stanceTerrain_(&stanceTerrain) {}

FootNormalConstraintMatrix StancePhase::getFootNormalConstraintInWorldFrame(scalar_t time, scalar_t positionGain) const {
  const scalar_t feedForwardVelocity = 0.0;
  const scalar_t desiredTerrainDistance = 0.0;
  return computeFootNormalConstraint(feedForwardVelocity, desiredTerrainDistance, *stanceTerrain_, positionGain);
}

SwingPhase::SwingPhase(SwingEvent liftOff, scalar_t swingHeight, SwingEvent touchDown) : liftOff_(liftOff), touchDown_(touchDown) {
  if (touchDown_.terrainPlane == nullptr) {
    setHalveSwing(swingHeight);
  } else {
    setFullSwing(swingHeight);
  }
}

void SwingPhase::setFullSwing(scalar_t swingHeight) {
  {  // LiftOff Motion
    const CubicSpline::Node liftOffInLiftOffFrame{liftOff_.time, 0.0, liftOff_.velocity};

    // touchdown seen from liftoff frame
    const scalar_t touchDownHeightInLiftOffFrame =
        terrainSignedDistanceFromPositionInWorld(touchDown_.terrainPlane->positionInWorld, *liftOff_.terrainPlane);
    const scalar_t touchDownVelocityInLiftOffFrame =
        surfaceNormalInWorld(*liftOff_.terrainPlane).dot(touchDown_.velocity * surfaceNormalInWorld(*touchDown_.terrainPlane));
    const CubicSpline::Node touchDownInLiftOffFrame{touchDown_.time, touchDownHeightInLiftOffFrame, touchDownVelocityInLiftOffFrame};

    // Midpoint, seen from liftoff
    const vector3_t touchDownClearancePointInWorld =
        swingHeight * surfaceNormalInWorld(*touchDown_.terrainPlane) + touchDown_.terrainPlane->positionInWorld;
    const scalar_t midHeightInLiftOffFrame =
        std::max(swingHeight, terrainSignedDistanceFromPositionInWorld(touchDownClearancePointInWorld, *liftOff_.terrainPlane));

    // Create spline in liftOffFrame
    liftOffMotion_.reset(new SplineCpg(liftOffInLiftOffFrame, midHeightInLiftOffFrame, touchDownInLiftOffFrame));
  }

  {  // Touchdown Motion
    CubicSpline::Node touchDownInTouchDownFrame{touchDown_.time, 0.0, touchDown_.velocity};

    // liftOff seen from touchdown frame
    const scalar_t liftOffHeightInTouchDownFrame =
        terrainSignedDistanceFromPositionInWorld(liftOff_.terrainPlane->positionInWorld, *touchDown_.terrainPlane);
    const scalar_t liftOffVelocityInTouchDownFrame =
        surfaceNormalInWorld(*touchDown_.terrainPlane).dot(liftOff_.velocity * surfaceNormalInWorld(*liftOff_.terrainPlane));
    const CubicSpline::Node liftOffInTouchDownFrame{liftOff_.time, liftOffHeightInTouchDownFrame, liftOffVelocityInTouchDownFrame};

    // Midpoint, seen from touchdown
    const vector3_t liftOffClearancePointInWorld =
        swingHeight * surfaceNormalInWorld(*liftOff_.terrainPlane) + liftOff_.terrainPlane->positionInWorld;
    const scalar_t midHeightInTouchDownFrame =
        std::max(swingHeight, terrainSignedDistanceFromPositionInWorld(liftOffClearancePointInWorld, *touchDown_.terrainPlane));

    // Create spline in touchDownFrame
    touchdownMotion_.reset(new SplineCpg(liftOffInTouchDownFrame, midHeightInTouchDownFrame, touchDownInTouchDownFrame));
  }
}

void SwingPhase::setHalveSwing(scalar_t swingHeight) {
  const CubicSpline::Node liftOffInLiftOffFrame{liftOff_.time, 0.0, liftOff_.velocity};
  const CubicSpline::Node touchDownInLiftOffFrame{touchDown_.time, swingHeight, 0.0};

  // The two motions are equal and defined in the liftoff plane
  liftOffMotion_.reset(new SplineCpg(liftOffInLiftOffFrame, swingHeight, touchDownInLiftOffFrame));
  touchdownMotion_.reset(new SplineCpg(*liftOffMotion_));
  touchDown_.terrainPlane = liftOff_.terrainPlane;
}

FootNormalConstraintMatrix SwingPhase::getFootNormalConstraintInWorldFrame(scalar_t time, scalar_t positionGain) const {
  const auto liftOffConstraint =
      computeFootNormalConstraint(liftOffMotion_->velocity(time), liftOffMotion_->position(time), *liftOff_.terrainPlane, positionGain);
  const auto touchDownConstraint = computeFootNormalConstraint(touchdownMotion_->velocity(time), touchdownMotion_->position(time),
                                                               *touchDown_.terrainPlane, positionGain);
  const scalar_t scaling = getScaling(time);

  FootNormalConstraintMatrix footNormalConstraint;
  footNormalConstraint.velocityMatrix = (1.0 - scaling) * liftOffConstraint.velocityMatrix + scaling * touchDownConstraint.velocityMatrix;
  footNormalConstraint.positionMatrix = (1.0 - scaling) * liftOffConstraint.positionMatrix + scaling * touchDownConstraint.positionMatrix;
  footNormalConstraint.constant = (1.0 - scaling) * liftOffConstraint.constant + scaling * touchDownConstraint.constant;
  return footNormalConstraint;
}

scalar_t SwingPhase::getScaling(scalar_t time) const {
  // Cubic scaling from 25% till 75% of swing duration
  static const CubicSpline scalingSpline({0.25, 0.0, 0.0}, {0.75, 1.0, 0.0});

  const scalar_t normalizedTime = (time - liftOff_.time) / (touchDown_.time - liftOff_.time);
  if (normalizedTime < 0.25) {
    return 0.0;
  } else if (normalizedTime < 0.75) {
    return scalingSpline.position(normalizedTime);
  } else {
    return 1.0;
  }
}

}  // namespace switched_model
