//
// Created by rgrandia on 24.04.20.
//

#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"

#include <ocs2_core/misc/LinearInterpolation.h>

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
      int j = getNextVertex(i, stanceTerrain.boundary.size());
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

StancePhase::StancePhase(ConvexTerrain stanceTerrain, scalar_t terrainMargin)
    : stanceTerrain_(std::move(stanceTerrain)),
      nominalFootholdLocation_(stanceTerrain_.plane.positionInWorld),
      surfaceNormalInWorldFrame_(surfaceNormalInWorld(stanceTerrain_.plane)),
      footTangentialConstraint_(tangentialConstraintsFromConvexTerrain(stanceTerrain_, terrainMargin)) {}

vector3_t StancePhase::normalDirectionInWorldFrame(scalar_t time) const {
  return surfaceNormalInWorldFrame_;
}

vector3_t StancePhase::nominalFootholdLocation() const {
  return nominalFootholdLocation_;
}

const ConvexTerrain* StancePhase::nominalFootholdConstraint() const {
  return &stanceTerrain_;
};

vector3_t StancePhase::getPositionInWorld(scalar_t time) const {
  return nominalFootholdLocation();
}

vector3_t StancePhase::getVelocityInWorld(scalar_t time) const {
  return vector3_t::Zero();
}

vector3_t StancePhase::getAccelerationInWorld(scalar_t time) const {
  return vector3_t::Zero();
}

const FootTangentialConstraintMatrix* StancePhase::getFootTangentialConstraintInWorldFrame() const {
  if (footTangentialConstraint_.A.rows() > 0) {
    return &footTangentialConstraint_;
  } else {
    return nullptr;
  }
}

SwingPhase::SwingPhase(SwingEvent liftOff, SwingEvent touchDown, const SwingProfile& swingProfile, const TerrainModel* terrainModel)
    : liftOff_(liftOff), touchDown_(touchDown) {
  if (touchDown_.terrainPlane == nullptr) {
    setHalveSwing(swingProfile, terrainModel);
  } else {
    setFullSwing(swingProfile, terrainModel);
  }
}

void SwingPhase::setFullSwing(const SwingProfile& swingProfile, const TerrainModel* terrainModel) {
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
  std::vector<SwingNode3d> swingPoints = {start};
  const vector3_t swingVector = touchDownPositionInWorld - liftOffPositionInWorld;
  if (swingVector.head<2>().norm() > 0.01) {
    // Get a unit vector perpendicular to the swing vector and in the plane formed by the World-Z axis and the swing vector.
    const vector3_t swingTrajectoryNormal = swingVector.cross(vector3_t(0.0, 0.0, 1.0).cross(swingVector)).normalized();

    // Terrain adaptation if information is available
    scalar_t maxObstacleHeight = 0.0;
    if (terrainModel != nullptr) {
      const auto heightProfile = terrainModel->getHeightProfileAlongLine(liftOffPositionInWorld, touchDownPositionInWorld);

      // Find the maximum obstacle. An obstacle is a height point sticking out above the line between liftoff and touchdown.
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
      maxObstacleHeight = std::min(maxObstacleHeight, swingProfile.maxSwingHeightAdaptation);
    }

    for (const auto& node : swingProfile.nodes) {
      SwingNode3d midPoint;
      midPoint.time = liftOff_.time + node.phase * swingDuration;
      // position = liftoff + progress towards touchdown + height offset in normal direction
      midPoint.position =
          liftOffPositionInWorld + node.tangentialProgress * swingVector + (maxObstacleHeight + node.swingHeight) * swingTrajectoryNormal;
      // velocity = factor * average velocity between liftoff and touchdown + velocity in normal direction
      midPoint.velocity = node.tangentialVelocityFactor / swingDuration * swingVector + node.normalVelocity * swingTrajectoryNormal;
      swingPoints.push_back(midPoint);
    }
  } else {  // (cases where target is above or intersecting with current position)
    // No point in checking the terrain. Start and end point will fall in the same cell.
    for (const auto& node : swingProfile.nodes) {
      SwingNode3d midPoint;
      midPoint.time = liftOff_.time + node.phase * swingDuration;
      midPoint.position = touchDownPositionInWorld;
      midPoint.position.z() = std::max(liftOffPositionInWorld.z(), touchDownPositionInWorld.z()) + node.swingHeight;
      midPoint.velocity = vector3_t::Zero();
      midPoint.velocity.z() = node.normalVelocity;
      swingPoints.push_back(midPoint);
    }
  }

  swingPoints.push_back(end);

  motion_.reset(new SwingSpline3d(swingPoints));

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
  const scalar_t sdfMidswingMargin = swingProfile.sdfMidswingMargin;
  const scalar_t sdfStartEndMargin = swingProfile.sdfStartEndMargin;
  const scalar_t swingDuration = (touchDown_.time - liftOff_.time);

  // liftoff conditions
  const auto& liftOffPositionInWorld = liftOff_.terrainPlane->positionInWorld;
  const vector3_t liftOffVelocityInWorld = liftOff_.velocity * surfaceNormalInWorld(*liftOff_.terrainPlane);
  const SwingNode3d start{liftOff_.time, liftOffPositionInWorld, liftOffVelocityInWorld};

  // touchdown conditions
  touchDown_.terrainPlane = liftOff_.terrainPlane;
  const vector3_t touchDownPositionInWorld = liftOffPositionInWorld + vector3_t(0.0, 0.0, swingProfile.nodes.back().swingHeight);
  const vector3_t touchDownVelocityInWorld = vector3_t::Zero();
  const SwingNode3d end{touchDown_.time, touchDownPositionInWorld, touchDownVelocityInWorld};

  std::vector<SwingNode3d> swingPoints = {start};
  for (const auto& node : swingProfile.nodes) {
    SwingNode3d midPoint;
    midPoint.time = liftOff_.time + node.phase * swingDuration;
    midPoint.position = touchDownPositionInWorld;
    midPoint.position.z() = std::max(liftOffPositionInWorld.z(), touchDownPositionInWorld.z()) + node.swingHeight;
    midPoint.velocity = vector3_t::Zero();
    midPoint.velocity.z() = node.normalVelocity;
    swingPoints.push_back(midPoint);
  }
  swingPoints.push_back(end);

  // The two motions are equal and defined in the liftoff plane
  motion_.reset(new SwingSpline3d(swingPoints));

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

ExternalSwingPhase::ExternalSwingPhase(std::vector<scalar_t> timeTrajectory, std::vector<vector3_t> positionTrajectory,
                                       std::vector<vector3_t> velocityTrajectory)
    : timeTrajectory_(std::move(timeTrajectory)),
      positionTrajectory_(std::move(positionTrajectory)),
      velocityTrajectory_(std::move(velocityTrajectory)) {}

vector3_t ExternalSwingPhase::getPositionInWorld(scalar_t time) const {
  return ocs2::LinearInterpolation::interpolate(time, timeTrajectory_, positionTrajectory_);
}

vector3_t ExternalSwingPhase::getVelocityInWorld(scalar_t time) const {
  return ocs2::LinearInterpolation::interpolate(time, timeTrajectory_, velocityTrajectory_);
}

}  // namespace switched_model
