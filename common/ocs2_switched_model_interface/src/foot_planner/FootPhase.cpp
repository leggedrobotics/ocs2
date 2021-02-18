//
// Created by rgrandia on 24.04.20.
//

#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/foot_planner/CubicSpline.h"

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

StancePhase::StancePhase(const TerrainPlane& stanceTerrain, scalar_t positionGain)
    : stanceTerrain_(&stanceTerrain), positionGain_(positionGain) {}

vector3_t StancePhase::normalDirectionInWorldFrame(scalar_t time) const {
  return surfaceNormalInWorld(*stanceTerrain_);
}

FootNormalConstraintMatrix StancePhase::getFootNormalConstraintInWorldFrame(scalar_t time) const {
  const scalar_t feedForwardVelocity = 0.0;
  const scalar_t desiredTerrainDistance = 0.0;
  return computeFootNormalConstraint(feedForwardVelocity, desiredTerrainDistance, *stanceTerrain_, positionGain_);
}

SwingPhase::SwingPhase(SwingEvent liftOff, scalar_t swingHeight, SwingEvent touchDown, scalar_t positionGain)
    : liftOff_(liftOff), touchDown_(touchDown), positionGain_(positionGain) {
  if (touchDown_.terrainPlane == nullptr) {
    setHalveSwing(swingHeight);
  } else {
    setFullSwing(swingHeight);
  }
}

void SwingPhase::setFullSwing(scalar_t swingHeight) {
  {  // LiftOff Motion
    const SwingNode liftOffInLiftOffFrame{liftOff_.time, 0.0, liftOff_.velocity};

    // touchdown seen from liftoff frame
    const scalar_t touchDownHeightInLiftOffFrame =
        terrainSignedDistanceFromPositionInWorld(touchDown_.terrainPlane->positionInWorld, *liftOff_.terrainPlane);
    const scalar_t touchDownVelocityInLiftOffFrame =
        surfaceNormalInWorld(*liftOff_.terrainPlane).dot(touchDown_.velocity * surfaceNormalInWorld(*touchDown_.terrainPlane));
    const SwingNode touchDownInLiftOffFrame{touchDown_.time, touchDownHeightInLiftOffFrame, touchDownVelocityInLiftOffFrame};

    // Midpoint, seen from liftoff
    const vector3_t touchDownClearancePointInWorld =
        swingHeight * surfaceNormalInWorld(*touchDown_.terrainPlane) + touchDown_.terrainPlane->positionInWorld;
    const scalar_t midHeightInLiftOffFrame =
        std::max(swingHeight, terrainSignedDistanceFromPositionInWorld(touchDownClearancePointInWorld, *liftOff_.terrainPlane));

    // Create spline in liftOffFrame
    liftOffMotion_.reset(new QuinticSwing(liftOffInLiftOffFrame, midHeightInLiftOffFrame, touchDownInLiftOffFrame));
  }

  {  // Touchdown Motion
    SwingNode touchDownInTouchDownFrame{touchDown_.time, 0.0, touchDown_.velocity};

    // liftOff seen from touchdown frame
    const scalar_t liftOffHeightInTouchDownFrame =
        terrainSignedDistanceFromPositionInWorld(liftOff_.terrainPlane->positionInWorld, *touchDown_.terrainPlane);
    const scalar_t liftOffVelocityInTouchDownFrame =
        surfaceNormalInWorld(*touchDown_.terrainPlane).dot(liftOff_.velocity * surfaceNormalInWorld(*liftOff_.terrainPlane));
    const SwingNode liftOffInTouchDownFrame{liftOff_.time, liftOffHeightInTouchDownFrame, liftOffVelocityInTouchDownFrame};

    // Midpoint, seen from touchdown
    const vector3_t liftOffClearancePointInWorld =
        swingHeight * surfaceNormalInWorld(*liftOff_.terrainPlane) + liftOff_.terrainPlane->positionInWorld;
    const scalar_t midHeightInTouchDownFrame =
        std::max(swingHeight, terrainSignedDistanceFromPositionInWorld(liftOffClearancePointInWorld, *touchDown_.terrainPlane));

    // Create spline in touchDownFrame
    touchdownMotion_.reset(new QuinticSwing(liftOffInTouchDownFrame, midHeightInTouchDownFrame, touchDownInTouchDownFrame));
  }
}

void SwingPhase::setHalveSwing(scalar_t swingHeight) {
  const SwingNode liftOffInLiftOffFrame{liftOff_.time, 0.0, liftOff_.velocity};
  const SwingNode touchDownInLiftOffFrame{touchDown_.time, swingHeight, 0.0};

  // The two motions are equal and defined in the liftoff plane
  liftOffMotion_.reset(new QuinticSwing(liftOffInLiftOffFrame, swingHeight, touchDownInLiftOffFrame));
  touchdownMotion_.reset(new QuinticSwing(*liftOffMotion_));
  touchDown_.terrainPlane = liftOff_.terrainPlane;
}

vector3_t SwingPhase::normalDirectionInWorldFrame(scalar_t time) const {
  // Returns "average" surface normal.
  const scalar_t scaling = getScaling(time);
  return ((1.0 - scaling) * surfaceNormalInWorld(*liftOff_.terrainPlane) + scaling * surfaceNormalInWorld(*touchDown_.terrainPlane))
      .normalized();
}

FootNormalConstraintMatrix SwingPhase::getFootNormalConstraintInWorldFrame(scalar_t time) const {
  const auto liftOffConstraint =
      computeFootNormalConstraint(liftOffMotion_->velocity(time), liftOffMotion_->position(time), *liftOff_.terrainPlane, positionGain_);
  const auto touchDownConstraint = computeFootNormalConstraint(touchdownMotion_->velocity(time), touchdownMotion_->position(time),
                                                               *touchDown_.terrainPlane, positionGain_);
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
