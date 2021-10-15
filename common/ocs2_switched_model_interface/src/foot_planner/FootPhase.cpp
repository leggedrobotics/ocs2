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

namespace {
/**
 * Constructs a velocity constraint in surface normal direction :
 *  ==> v_foot = v_ff - kp * (p_foot - p_des)
 *  ==> (n')* v_foot + (kp* n')* p_foot - (n') * (v_ff + kp* p_des) = 0
 *
 *  Gives us
 *  ==> A_p * p_world + A_v * v_world + b = 0
 *  A_p = kp * n'
 *  A_v = n'
 *  b = (n') * (v_ff + kp* p_des)
 */
FootNormalConstraintMatrix computeFootNormalConstraint(const vector3_t& surfaceNormalInWorld, const vector3_t& feedforwardVelocityInWorld,
                                                       const vector3_t& feedforwardPositionInWorld, scalar_t positionGain) {
  FootNormalConstraintMatrix footNormalConstraint;
  footNormalConstraint.velocityMatrix = surfaceNormalInWorld.transpose();
  footNormalConstraint.positionMatrix = positionGain * surfaceNormalInWorld.transpose();
  footNormalConstraint.constant = -surfaceNormalInWorld.dot(feedforwardVelocityInWorld + positionGain * feedforwardPositionInWorld);
  return footNormalConstraint;
}
}  // namespace

StancePhase::StancePhase(const ConvexTerrain& stanceTerrain, scalar_t positionGain, scalar_t terrainMargin)
    : nominalFootholdLocation_(stanceTerrain.plane.positionInWorld),
      surfaceNormalInWorldFrame_(surfaceNormalInWorld(stanceTerrain.plane)),
      footNormalConstraint_(
          computeFootNormalConstraint(surfaceNormalInWorldFrame_, vector3_t::Zero(), stanceTerrain.plane.positionInWorld, positionGain)),
      footTangentialConstraint_(tangentialConstraintsFromConvexTerrain(stanceTerrain, terrainMargin)) {}

vector3_t StancePhase::normalDirectionInWorldFrame(scalar_t time) const {
  return surfaceNormalInWorldFrame_;
}

vector3_t StancePhase::nominalFootholdLocation() const {
  return nominalFootholdLocation_;
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
  return footNormalConstraint_;
}

const FootTangentialConstraintMatrix* StancePhase::getFootTangentialConstraintInWorldFrame() const {
  if (footTangentialConstraint_.A.rows() > 0) {
    return &footTangentialConstraint_;
  } else {
    return nullptr;
  }
}

SwingPhase::SwingPhase(SwingEvent liftOff, SwingEvent touchDown, const SwingProfile& swingProfile, const TerrainModel* terrainModel,
                       scalar_t positionGain)
    : liftOff_(liftOff), touchDown_(touchDown), positionGain_(positionGain) {
  if (touchDown_.terrainPlane == nullptr) {
    setHalveSwing(swingProfile, terrainModel);
  } else {
    setFullSwing(swingProfile, terrainModel);
  }
}

void SwingPhase::setFullSwing(const SwingProfile& swingProfile, const TerrainModel* terrainModel) {
  const scalar_t swingHeight = swingProfile.swingHeight;
  const scalar_t apexVelocityFactor = swingProfile.apexVelocityFactor;
  const scalar_t sdfMidswingMargin = swingProfile.sdfMidswingMargin;
  const scalar_t sdfStartEndMargin = swingProfile.sdfStartEndMargin;
  const scalar_t swingDuration = (touchDown_.time - liftOff_.time);

  // liftoff conditions: lifting off in vertical direction
  const auto& liftOffPositionInWorld = liftOff_.terrainPlane->positionInWorld;
  const vector3_t liftOffVelocityInWorld = {0.0, 0.0, liftOff_.velocity};
  const SwingNode3d start{liftOff_.time, liftOffPositionInWorld, liftOffVelocityInWorld};

  // touchdown conditions: touching down in surface normal direction
  const auto& touchDownPositionInWorld = touchDown_.terrainPlane->positionInWorld;
  const vector3_t touchDownVelocityInWorld = touchDown_.velocity * surfaceNormalInWorld(*touchDown_.terrainPlane);
  const SwingNode3d end{touchDown_.time, touchDownPositionInWorld, touchDownVelocityInWorld};

  // Apex
  SwingNode3d apex;
  apex.time = 0.5 * (start.time + end.time);
  const vector3_t swingVector = touchDownPositionInWorld - liftOffPositionInWorld;
  if (swingVector.head<2>().norm() > 0.01) {
    const vector3_t middleStraightlinePoint = 0.5 * (liftOffPositionInWorld + touchDownPositionInWorld);

    // Get a unit vector perpendicular to the swing vector and in the plane formed by the World-Z axis and the swing vector.
    const vector3_t swingTrajectoryNormal = swingVector.cross(vector3_t(0.0, 0.0, 1.0).cross(swingVector)).normalized();

    // Blind swing height
    apex.position = middleStraightlinePoint + swingHeight * swingTrajectoryNormal;

    // Terrain adaptation if information is available
    if (terrainModel != nullptr) {
      const auto heightProfile = terrainModel->getHeightProfileAlongLine(liftOffPositionInWorld, touchDownPositionInWorld);

      // Find the maximum obstacle. An obstacle is a height point sticking out above the line between liftoff and touchdown.
      scalar_t maxObstacleHeight = 0.0;
      for (const auto& point : heightProfile) {
        const scalar_t pointProgress = point[0];
        const scalar_t pointHeight = point[1];
        const scalar_t heightRelativeToSwingLine = pointHeight - (liftOffPositionInWorld.z() + pointProgress * swingVector.z());
        const scalar_t relativeHeightProjectedOnNormal = heightRelativeToSwingLine * swingTrajectoryNormal.z();
        if (relativeHeightProjectedOnNormal > maxObstacleHeight) {
          maxObstacleHeight = relativeHeightProjectedOnNormal;
        }
      }
      // Clip max obstacle to two times swing height to protect against outliers. (Total swing height will be max 3 * swing height)
      maxObstacleHeight = std::min(maxObstacleHeight, 2.0 * swingHeight);

      apex.position += maxObstacleHeight * swingTrajectoryNormal;
    }

    // Velocity at apex points purely in swing direction
    apex.velocity = apexVelocityFactor / swingDuration * swingVector;
  } else {  // (cases where target is above or intersecting with current position)
    // No point in checking the terrain. Start and end point will fall in the same cell.
    apex.position = touchDownPositionInWorld;
    apex.position.z() = std::max(liftOffPositionInWorld.z(), touchDownPositionInWorld.z()) + swingHeight;
    apex.velocity = vector3_t::Zero();
  }

  motion_.reset(new SwingSpline3d(start, apex, end));

  // Terrain clearance
  if (terrainModel != nullptr && terrainModel->getSignedDistanceField() != nullptr) {
    const auto& sdf = *terrainModel->getSignedDistanceField();
    const scalar_t sdfStartClearance = std::min(sdf.value(liftOffPositionInWorld), 0.0) + sdfStartEndMargin;
    const scalar_t sdfEndClearance = std::min(sdf.value(touchDownPositionInWorld), 0.0) + sdfStartEndMargin;
    const scalar_t midSwingTime = 0.5 * (liftOff_.time + touchDown_.time);
    SwingNode startNode{liftOff_.time, sdfStartClearance, liftOff_.velocity};
    SwingNode apexNode{midSwingTime, sdfMidswingMargin, 0.0};
    SwingNode endNode{touchDown_.time, sdfEndClearance, touchDown_.velocity};
    terrainClearanceMotion_.reset(new QuinticSwing(startNode, apexNode, endNode));
  } else {
    terrainClearanceMotion_.reset();
  }
}

void SwingPhase::setHalveSwing(const SwingProfile& swingProfile, const TerrainModel* terrainModel) {
  const scalar_t swingHeight = swingProfile.swingHeight;
  const scalar_t sdfMidswingMargin = swingProfile.sdfMidswingMargin;
  const scalar_t sdfStartEndMargin = swingProfile.sdfStartEndMargin;

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
    if (terrainModel != nullptr && terrainModel->getSignedDistanceField() != nullptr) {
      const auto& sdf = *terrainModel->getSignedDistanceField();
      const scalar_t sdfStartClearance = std::min(sdf.value(liftOffPositionInWorld), 0.0) + sdfStartEndMargin;
      SwingNode startNode{liftOff_.time, sdfStartClearance, liftOff_.velocity};
      SwingNode endNode{touchDown_.time, sdfMidswingMargin, 0.0};
      terrainClearanceMotion_.reset(new QuinticSwing(startNode, sdfMidswingMargin, endNode));
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
  const auto surfaceNormal = this->normalDirectionInWorldFrame(time);
  return computeFootNormalConstraint(surfaceNormal, motion_->velocity(time), motion_->position(time), positionGain_);
}

scalar_t SwingPhase::getMinimumFootClearance(scalar_t time) const {
  if (terrainClearanceMotion_ != nullptr) {
    return terrainClearanceMotion_->position(time);
  } else {
    return 0.0;
  }
}

scalar_t SwingPhase::getScaling(scalar_t time) const {
  // Cubic scaling from 25% till 75% of swing duration
  const scalar_t startInterpolation = 0.25;
  const scalar_t endInterpolation = 0.75;
  static const CubicSpline scalingSpline({startInterpolation, 0.0, 0.0}, {endInterpolation, 1.0, 0.0});

  const scalar_t normalizedTime = (time - liftOff_.time) / (touchDown_.time - liftOff_.time);
  if (normalizedTime < startInterpolation) {
    return 0.0;
  } else if (normalizedTime < endInterpolation) {
    return scalingSpline.position(normalizedTime);
  } else {
    return 1.0;
  }
}

}  // namespace switched_model
