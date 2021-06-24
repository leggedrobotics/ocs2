//
// Created by rgrandia on 24.04.20.
//

#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/foot_planner/CubicSpline.h"

namespace switched_model {

FootTangentialConstraintMatrix tangentialConstraintsFromConvexTerrain(const ConvexTerrain& stanceTerrain, scalar_t margin) {
  FootTangentialConstraintMatrix constraints{};

  int numberOfConstraints = stanceTerrain.boundary.size();
  constraints.A.resize(numberOfConstraints, 3);
  constraints.b.resize(numberOfConstraints, 1);

  if (numberOfConstraints > 0) {
    for (int i = 0; i < stanceTerrain.boundary.size(); ++i) {
      // Determine next point in counter clockwise direction
      int j = ((i + 1) < stanceTerrain.boundary.size()) ? i + 1 : 0;  // next point with wrap around
      vector2_t start2end = stanceTerrain.boundary[j] - stanceTerrain.boundary[i];

      // Get 2D constraint A_terrain * p_terrain + b - margin >= 0
      vector2_t A_terrain{-start2end.y(), start2end.x()};  // rotate 90deg around z to get a vector pointing to interior of the polygon
      A_terrain.normalize();                               // make it a unit vector.
      scalar_t b_terrain = -A_terrain.dot(stanceTerrain.boundary[i]) - margin;  // Take offset of one of the points

      // 3D position in terrain frame = T_R_W * (p_w - p_0)
      // Take x-y only for the constraints: p_terrain = diag(1, 1, 0) * T_R_W * (p_w - p_0)
      // Gives:
      //    A_world = A_terrain * diag(1, 1, 0) * T_R_W
      //    b_world = b_terrain  - A_terrain * diag(1, 1, 0) * T_R_W * p_0 = b_terrain - A_world * p_0
      constraints.A.row(i) = A_terrain.transpose() * stanceTerrain.plane.orientationWorldToTerrain.topRows<2>();
      constraints.b(i) = b_terrain - constraints.A.row(i) * stanceTerrain.plane.positionInWorld;
    }
  }

  return constraints;
}

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

StancePhase::StancePhase(const ConvexTerrain& stanceTerrain, scalar_t positionGain, scalar_t terrainMargin)
    : stanceTerrain_(&stanceTerrain),
      footTangentialConstraint_(tangentialConstraintsFromConvexTerrain(stanceTerrain, terrainMargin)),
      positionGain_(positionGain) {}

vector3_t StancePhase::normalDirectionInWorldFrame(scalar_t time) const {
  return surfaceNormalInWorld(stanceTerrain_->plane);
}

FootNormalConstraintMatrix StancePhase::getFootNormalConstraintInWorldFrame(scalar_t time) const {
  const scalar_t feedForwardVelocity = 0.0;
  const scalar_t desiredTerrainDistance = 0.0;
  return computeFootNormalConstraint(feedForwardVelocity, desiredTerrainDistance, stanceTerrain_->plane, positionGain_);
}

const FootTangentialConstraintMatrix* StancePhase::getFootTangentialConstraintInWorldFrame() const {
  if (footTangentialConstraint_.A.rows() > 0) {
    return &footTangentialConstraint_;
  } else {
    return nullptr;
  }
}

SwingPhase::SwingPhase(SwingEvent liftOff, scalar_t swingHeight, SwingEvent touchDown, const SignedDistanceField* signedDistanceField,
                       scalar_t positionGain, scalar_t sdfMidswingMargin)
    : liftOff_(liftOff),
      touchDown_(touchDown),
      signedDistanceField_(signedDistanceField),
      positionGain_(positionGain),
      sdfMidswingMargin_(sdfMidswingMargin) {
  if (touchDown_.terrainPlane == nullptr) {
    setHalveSwing(swingHeight);
  } else {
    setFullSwing(swingHeight);
  }
}

void SwingPhase::setFullSwing(scalar_t swingHeight) {
  const scalar_t swingDuration = (touchDown_.time - liftOff_.time);
  const auto& liftOffPositionInWorld = liftOff_.terrainPlane->positionInWorld;
  const vector3_t liftOffVelocityInWorld{0.0, 0.0, liftOff_.velocity};
  const auto& touchDownPositionInWorld = touchDown_.terrainPlane->positionInWorld;
  const vector3_t touchDownVelocityInWorld{0.0, 0.0, touchDown_.velocity};

  vector3_t apexPositionInWorld{0.5 * (liftOffPositionInWorld.x() + touchDownPositionInWorld.x()),
                                0.5 * (liftOffPositionInWorld.y() + touchDownPositionInWorld.y()),
                                swingHeight + std::max(liftOffPositionInWorld.z(), touchDownPositionInWorld.z())};
  // Correct apex position if signed distance is available
  if (signedDistanceField_ != nullptr) {
    const auto apexSdf = signedDistanceField_->valueAndDerivative(apexPositionInWorld);
    if (apexSdf.first < swingHeight) {
      apexPositionInWorld += (swingHeight - apexSdf.first) * apexSdf.second.normalized();
    }
  }

  const vector3_t apexVelocityInWorld{(touchDownPositionInWorld.x() - liftOffPositionInWorld.x()) / swingDuration,
                                      (touchDownPositionInWorld.y() - liftOffPositionInWorld.y()) / swingDuration, 0.0};

  const scalar_t distanceLiftoffToApex = (apexPositionInWorld - liftOffPositionInWorld).norm();
  const scalar_t distancetouchDownToApex = (apexPositionInWorld - touchDownPositionInWorld).norm();
  const scalar_t apexTime = liftOff_.time + distanceLiftoffToApex / (distanceLiftoffToApex + distancetouchDownToApex) * swingDuration;

  // LiftOff Motion
  const vector3_t liftoffNormal = surfaceNormalInWorld(*liftOff_.terrainPlane);
  const SwingNode liftOffInLiftOffFrame{liftOff_.time, 0.0, liftoffNormal.dot(liftOffVelocityInWorld)};
  const SwingNode apexInLiftOffFrame{apexTime, terrainSignedDistanceFromPositionInWorld(apexPositionInWorld, *liftOff_.terrainPlane),
                                     liftoffNormal.dot(apexVelocityInWorld)};
  const SwingNode touchDownInLiftOffFrame{touchDown_.time,
                                          terrainSignedDistanceFromPositionInWorld(touchDownPositionInWorld, *liftOff_.terrainPlane),
                                          liftoffNormal.dot(touchDownVelocityInWorld)};

  // Create spline in liftOffFrame
  liftOffMotion_.reset(new QuinticSwing(liftOffInLiftOffFrame, apexInLiftOffFrame, touchDownInLiftOffFrame));

  // Touchdown Motion
  const vector3_t touchDownNormal = surfaceNormalInWorld(*touchDown_.terrainPlane);
  const SwingNode liftOffInTouchDownFrame{liftOff_.time,
                                          terrainSignedDistanceFromPositionInWorld(liftOffPositionInWorld, *touchDown_.terrainPlane),
                                          touchDownNormal.dot(liftOffVelocityInWorld)};
  const SwingNode apexInTouchDownFrame{apexTime, terrainSignedDistanceFromPositionInWorld(apexPositionInWorld, *touchDown_.terrainPlane),
                                       touchDownNormal.dot(apexVelocityInWorld)};
  SwingNode touchDownInTouchDownFrame{touchDown_.time, 0.0, touchDownNormal.dot(touchDownVelocityInWorld)};

  // Create spline in touchDownFrame
  touchdownMotion_.reset(new QuinticSwing(liftOffInTouchDownFrame, apexInTouchDownFrame, touchDownInTouchDownFrame));

  // Terrain clearance
  if (signedDistanceField_ != nullptr) {
    const scalar_t sdfStartClearance_ = std::min(signedDistanceField_->value(liftOffPositionInWorld), 0.0);
    const scalar_t sdfEndClearance_ = std::min(signedDistanceField_->value(touchDownPositionInWorld), 0.0);
    const scalar_t midSwingTime = 0.5 * (liftOff_.time + touchDown_.time);
    SwingNode startNode{liftOff_.time, sdfStartClearance_ - startEndMargin_, liftOffInLiftOffFrame.velocity};
    SwingNode apexNode{midSwingTime, sdfMidswingMargin_, 0.0};
    SwingNode endNode{touchDown_.time, sdfEndClearance_ - startEndMargin_, touchDownInTouchDownFrame.velocity};
    terrainClearanceMotion_.reset(new QuinticSwing(startNode, apexNode, endNode));
  } else {
    terrainClearanceMotion_.reset();
  }
}

void SwingPhase::setHalveSwing(scalar_t swingHeight) {
  const SwingNode liftOffInLiftOffFrame{liftOff_.time, 0.0, liftOff_.velocity};
  const SwingNode touchDownInLiftOffFrame{touchDown_.time, swingHeight, 0.0};

  // The two motions are equal and defined in the liftoff plane
  liftOffMotion_.reset(new QuinticSwing(liftOffInLiftOffFrame, swingHeight, touchDownInLiftOffFrame));
  touchdownMotion_.reset(new QuinticSwing(*liftOffMotion_));
  touchDown_.terrainPlane = liftOff_.terrainPlane;

  {  // Terrain clearance
    if (signedDistanceField_ != nullptr) {
      const scalar_t sdfStartClearance_ = std::min(signedDistanceField_->value(liftOff_.terrainPlane->positionInWorld), 0.0);
      SwingNode startNode{liftOff_.time, sdfStartClearance_ - startEndMargin_, liftOffInLiftOffFrame.velocity};
      SwingNode endNode{touchDown_.time, sdfMidswingMargin_, 0.0};
      terrainClearanceMotion_.reset(new QuinticSwing(startNode, sdfMidswingMargin_, endNode));
    } else {
      terrainClearanceMotion_.reset();
    }
  }
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

SignedDistanceConstraint SwingPhase::getSignedDistanceConstraint(scalar_t time) const {
  if (signedDistanceField_ != nullptr) {
    return {signedDistanceField_, terrainClearanceMotion_->position(time)};
  } else {
    return {nullptr, 0.0};
  }
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
