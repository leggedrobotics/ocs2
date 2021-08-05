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

FootNormalConstraintMatrix computeFootNormalConstraint(const vector3_t& feedforwardVelocityInWorld,
                                                       const vector3_t& feedforwardPositionInWorld, const TerrainPlane& terrainPlane,
                                                       scalar_t positionGain) {
  // in surface normal direction : v_foot = v_ff - kp * (p_foot - p_des)
  // ==> (n')* v_foot + (kp* n')* p_foot - (n') * (v_ff + kp* p_des) = 0
  const vector3_t surfaceNormal = surfaceNormalInWorld(terrainPlane);

  FootNormalConstraintMatrix footNormalConstraint;
  footNormalConstraint.velocityMatrix = surfaceNormal.transpose();
  footNormalConstraint.positionMatrix = positionGain * surfaceNormal.transpose();
  footNormalConstraint.constant = -surfaceNormal.dot(feedforwardVelocityInWorld + positionGain * feedforwardPositionInWorld);
  return footNormalConstraint;
}

StancePhase::StancePhase(const ConvexTerrain& stanceTerrain, scalar_t positionGain, scalar_t terrainMargin)
    : stanceTerrain_(&stanceTerrain),
      footTangentialConstraint_(tangentialConstraintsFromConvexTerrain(stanceTerrain, terrainMargin)),
      positionGain_(positionGain) {}

vector3_t StancePhase::normalDirectionInWorldFrame(scalar_t time) const {
  return surfaceNormalInWorld(stanceTerrain_->plane);
}

vector3_t StancePhase::nominalFootholdLocation() const {
  return stanceTerrain_->plane.positionInWorld;
}

vector3_t StancePhase::getPositionInWorld(scalar_t time) const {
  return nominalFootholdLocation();
}

vector3_t StancePhase::getVelocityInWorld(scalar_t time) const {
  return vector3_t::Zero();
}

vector3_t StancePhase::getAccelerationInWorld(scalar_t time) const {
  return vector3_t::Zero();
}

FootNormalConstraintMatrix StancePhase::getFootNormalConstraintInWorldFrame(scalar_t time) const {
  return computeFootNormalConstraint(vector3_t::Zero(), stanceTerrain_->plane.positionInWorld, stanceTerrain_->plane, positionGain_);
}

const FootTangentialConstraintMatrix* StancePhase::getFootTangentialConstraintInWorldFrame() const {
  if (footTangentialConstraint_.A.rows() > 0) {
    return &footTangentialConstraint_;
  } else {
    return nullptr;
  }
}

SwingPhase::SwingPhase(SwingEvent liftOff, scalar_t swingHeight, SwingEvent touchDown, const TerrainModel* terrainModel,
                       scalar_t positionGain, scalar_t sdfMidswingMargin)
    : liftOff_(liftOff),
      touchDown_(touchDown),
      terrainModel_(terrainModel),
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

  // liftoff conditions
  const auto& liftOffPositionInWorld = liftOff_.terrainPlane->positionInWorld;
  const vector3_t liftOffVelocityInWorld = {0.0, 0.0, liftOff_.velocity};
  const SwingNode3d start{liftOff_.time, liftOffPositionInWorld, liftOffVelocityInWorld};

  // toucdown conditions
  const auto& touchDownPositionInWorld = touchDown_.terrainPlane->positionInWorld;
  const vector3_t touchDownVelocityInWorld = touchDown_.velocity * surfaceNormalInWorld(*touchDown_.terrainPlane);
  const SwingNode3d end{touchDown_.time, touchDownPositionInWorld, touchDownVelocityInWorld};

  // Apex
  scalar_t apexHeight = swingHeight + std::max(liftOffPositionInWorld.z(), touchDownPositionInWorld.z());
  if (terrainModel_ != nullptr) {
    const auto highestObstacle = terrainModel_->getHighestObstacleAlongLine(liftOffPositionInWorld, touchDownPositionInWorld);
    apexHeight = std::max(apexHeight, highestObstacle.z() + swingHeight);
    // limit adaptation to 3 times swing height
    apexHeight = std::min(apexHeight, 3.0 * swingHeight + std::max(liftOffPositionInWorld.z(), touchDownPositionInWorld.z()));
  }

  vector3_t apexPositionInWorld{0.5 * (liftOffPositionInWorld.x() + touchDownPositionInWorld.x()),
                                0.5 * (liftOffPositionInWorld.y() + touchDownPositionInWorld.y()), apexHeight};

  const scalar_t distanceLiftoffToApex = (apexPositionInWorld - liftOffPositionInWorld).norm();
  const scalar_t distancetouchDownToApex = (apexPositionInWorld - touchDownPositionInWorld).norm();
  const scalar_t apexTime = liftOff_.time + distanceLiftoffToApex / (distanceLiftoffToApex + distancetouchDownToApex) * swingDuration;

  const scalar_t velocityFactor = 2.0;  // TODO: check what velocity creates the right XY profile
  const vector3_t apexVelocityInWorld{velocityFactor * (touchDownPositionInWorld.x() - liftOffPositionInWorld.x()) / swingDuration,
                                      velocityFactor * (touchDownPositionInWorld.y() - liftOffPositionInWorld.y()) / swingDuration, 0.0};

  const SwingNode3d apex{apexTime, apexPositionInWorld, apexVelocityInWorld};

  motion_.reset(new SwingSpline3d(start, apex, end));

  // Terrain clearance
  if (terrainModel_ != nullptr && terrainModel_->getSignedDistanceField() != nullptr) {
    const auto* signedDistanceField = terrainModel_->getSignedDistanceField();
    const scalar_t sdfStartClearance = std::min(signedDistanceField->value(liftOffPositionInWorld), 0.0);
    const scalar_t sdfEndClearance = std::min(signedDistanceField->value(touchDownPositionInWorld), 0.0);
    const scalar_t midSwingTime = 0.5 * (liftOff_.time + touchDown_.time);
    SwingNode startNode{liftOff_.time, sdfStartClearance - startEndMargin_, liftOff_.velocity};
    SwingNode apexNode{midSwingTime, sdfMidswingMargin_, 0.0};
    SwingNode endNode{touchDown_.time, sdfEndClearance - startEndMargin_, touchDown_.velocity};
    terrainClearanceMotion_.reset(new QuinticSwing(startNode, apexNode, endNode));
  } else {
    terrainClearanceMotion_.reset();
  }
}

void SwingPhase::setHalveSwing(scalar_t swingHeight) {
  // liftoff conditions
  const auto& liftOffPositionInWorld = liftOff_.terrainPlane->positionInWorld;
  const vector3_t liftOffVelocityInWorld = liftOff_.velocity * surfaceNormalInWorld(*liftOff_.terrainPlane);
  const SwingNode3d start{liftOff_.time, liftOffPositionInWorld, liftOffVelocityInWorld};

  // touchdown conditions
  touchDown_.terrainPlane = liftOff_.terrainPlane;
  const vector3_t touchDownPositionInWorld = liftOffPositionInWorld + vector3_t(0.0, 0.0, swingHeight);
  const vector3_t touchDownVelocityInWorld = vector3_t::Zero();
  const SwingNode3d end{touchDown_.time, touchDownPositionInWorld, touchDownVelocityInWorld};

  const SwingNode3d apex{0.5 * (liftOff_.time + touchDown_.time), touchDownPositionInWorld, touchDownVelocityInWorld};

  // The two motions are equal and defined in the liftoff plane
  motion_.reset(new SwingSpline3d(start, apex, end));

  {  // Terrain clearance
    if (terrainModel_ != nullptr && terrainModel_->getSignedDistanceField() != nullptr) {
      const auto* signedDistanceField = terrainModel_->getSignedDistanceField();
      const scalar_t sdfStartClearance = std::min(signedDistanceField->value(liftOff_.terrainPlane->positionInWorld), 0.0);
      SwingNode startNode{liftOff_.time, sdfStartClearance - startEndMargin_, liftOff_.velocity};
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

vector3_t SwingPhase::nominalFootholdLocation() const {
  return touchDown_.terrainPlane->positionInWorld;
}

vector3_t SwingPhase::getPositionInWorld(scalar_t time) const {
  return motion_->position(time);
}

vector3_t SwingPhase::getVelocityInWorld(scalar_t time) const {
  return motion_->velocity(time);
}

vector3_t SwingPhase::getAccelerationInWorld(scalar_t time) const {
  return motion_->acceleration(time);
}

FootNormalConstraintMatrix SwingPhase::getFootNormalConstraintInWorldFrame(scalar_t time) const {
  const auto liftOffConstraint =
      computeFootNormalConstraint(motion_->velocity(time), motion_->position(time), *liftOff_.terrainPlane, positionGain_);
  const auto touchDownConstraint =
      computeFootNormalConstraint(motion_->velocity(time), motion_->position(time), *touchDown_.terrainPlane, positionGain_);
  const scalar_t scaling = getScaling(time);

  FootNormalConstraintMatrix footNormalConstraint;
  footNormalConstraint.velocityMatrix = (1.0 - scaling) * liftOffConstraint.velocityMatrix + scaling * touchDownConstraint.velocityMatrix;
  footNormalConstraint.positionMatrix = (1.0 - scaling) * liftOffConstraint.positionMatrix + scaling * touchDownConstraint.positionMatrix;
  footNormalConstraint.constant = (1.0 - scaling) * liftOffConstraint.constant + scaling * touchDownConstraint.constant;
  return footNormalConstraint;
}

SignedDistanceConstraint SwingPhase::getSignedDistanceConstraint(scalar_t time) const {
  if (terrainModel_ != nullptr && terrainModel_->getSignedDistanceField() != nullptr) {
    return {terrainModel_->getSignedDistanceField(), terrainClearanceMotion_->position(time)};
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
