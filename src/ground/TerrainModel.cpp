//
// Created by rgrandia on 27.07.19.
//

#include "ocs2_switched_model_interface/ground/TerrainModel.h"
#include "ocs2_core/misc/Lookup.h"

namespace switched_model {

TerrainModel::TerrainModel(std::shared_ptr<const GaitSequence> gaitSequence) : gaitSequence_(std::move(gaitSequence)) {}

TerrainModel::~TerrainModel() = default;

void TerrainModel::update(double initTime, double finalTime, const Eigen::Matrix<double, -1, 1>& currentState,
                          const ocs2::CostDesiredTrajectories<double>& costDesiredTrajectory,
                          const ocs2::HybridLogicRules* hybridLogicRules) {
  std::cout << "TerrainModel Updated from MPC" << std::endl;

  initTime_ = initTime;
  finalTime_ = finalTime;

  // extract raw trajectory for the base
  BaseTrajectory rawBaseTrajectory_;
  ocs2::EigenLinearInterpolation<BaseCoordinates> rawBaseInterpolation_;
  rawBaseTrajectory_.time = costDesiredTrajectory.desiredTimeTrajectory();
  rawBaseTrajectory_.data.clear();
  for (auto state_des : costDesiredTrajectory.desiredStateTrajectory()) {
    rawBaseTrajectory_.data.emplace_back(state_des.segment(0, 6));
  }
  rawBaseInterpolation_.setData(&rawBaseTrajectory_.time, &rawBaseTrajectory_.data);

  baseTrajectory_.time.clear();
  baseTrajectory_.data.clear();
  baseTrajectory_.time.push_back(initTime);
  baseTrajectory_.data.emplace_back(currentState.segment(0, 6));
  double dt = 0.1;
  double a = 2.0 * dt;  // settling time ~ 2.2 / a_continuous
  while (baseTrajectory_.time.back() < finalTime) {
    baseTrajectory_.time.push_back(baseTrajectory_.time.back() + dt);
    BaseCoordinates desiredBase;
    rawBaseInterpolation_.interpolate(baseTrajectory_.time.back(), desiredBase);
    baseTrajectory_.data.emplace_back((1 - a) * baseTrajectory_.data.back() + a * desiredBase);
  }

  if (baseTrajectory_.time.size() != baseTrajectory_.data.size()) {
    throw std::runtime_error("[TerrainModel::update] wrong basetrajectory sizes");
  }

  baseInterpolation_.setData(&baseTrajectory_.time, &baseTrajectory_.data);

  //  //TODO(ruben): fill
  //  footPositions_, footVelocities_

  assignPolytopes();
  display();
}

void TerrainModel::setPolytopes(ConvexPlanarPolytope3dArray terrainPolytopes) { terrainPolytopes_ = std::move(terrainPolytopes); }

const ConvexPlanarPolytope3dArray& TerrainModel::getPolytopes() const { return terrainPolytopes_; }

void TerrainModel::loadTestTerrain() {
  std::cout << "TerrainModel loaded" << std::endl;
  ConvexPlanarPolytope3dArray polytopes;

  // Start with four steps
  auto firstRing = createPolytope(4, 0.25, M_PI / 4.0, Eigen::Vector3d::Zero());
  for (auto& point : firstRing) {
    polytopes.push_back(createPolytope(6, 0.15, 0.0, point));
  }

  auto secondRing = createPolytope(12, 0.55, 0.0, Eigen::Vector3d::Zero());
  for (auto& point : secondRing) {
    polytopes.push_back(createPolytope(5, 0.15, 0.0, point));
  }

  auto thirdRing = createPolytope(22, 0.85, 0.0, Eigen::Vector3d::Zero());
  for (auto& point : thirdRing) {
    polytopes.push_back(createPolytope(4, 0.15, M_PI / 4.0, point));
  }

  auto fourthRing = createPolytope(26, 1.2, 0.0, Eigen::Vector3d::Zero());
  for (auto& point : fourthRing) {
    polytopes.push_back(createPolytope(4, 0.15, M_PI / 4.0, point));
  }

  setPolytopes(std::move(polytopes));
}

Eigen::Matrix<double, -1, 4> TerrainModel::getTerrainConstraints(double time, int legID) const {
  auto index = ocs2::lookup::findIndexInTimeArray(polytopeIDSequencePerLeg_[legID].time, time);
  auto selectedPolytopeID = polytopeIDSequencePerLeg_[legID].data[index];
  if (selectedPolytopeID >= 0) {
    return toHalfSpaces(terrainPolytopes_[selectedPolytopeID]);
  } else {  // no constraints
    return Eigen::Matrix<double, -1, 4>();
  }
}

void TerrainModel::display() const {
  std::cout << "[TerrainModel assignment] time [" << initTime_ << ", " << finalTime_ << "]" << std::endl;
  for (int leg = 0; leg < 4; leg++) {
    std::cout << "Leg " << leg;
    std::cout << "\n\t eventTimes : ";
    for (auto t : polytopeIDSequencePerLeg_[leg].time) {
      std::cout << t << ", ";
    }
    std::cout << "\n\t polytopes : ";
    for (auto id : polytopeIDSequencePerLeg_[leg].data) {
      std::cout << id << ", ";
    }
    std::cout << std::endl;
  }

  std::cout << "[terrainBaseDesired]" << std::endl;
  for (int k = 0; k < baseTrajectory_.time.size(); k++) {
    std::cout << "t: " << baseTrajectory_.time[k] << "\t x: " << baseTrajectory_.data[k].transpose() << std::endl;
  }
}

void TerrainModel::assignPolytopes() {
  std::cout << "assign polytopes" << std::endl;
  auto gaitSequenceTranspose = gaitSequence_->transpose();

  for (int leg = 0; leg < 4; leg++) {
    auto& eventTimes = gaitSequenceTranspose.time[leg];
    auto& contactStates = gaitSequenceTranspose.contactFlags[leg];
    auto& assignedPolytopes = polytopeIDSequencePerLeg_[leg];
    PolytopeIDsequence newPolytopeSequence;
    //    auto &footPosition = footPositions_[leg];
    //    auto &footVelocity = footVelocities_[leg];

    auto firstMotionPhase = ocs2::lookup::findIndexInTimeArray(eventTimes, initTime_);  // [0, size(eventTimes)]
    auto lastMotionPhase = ocs2::lookup::findIndexInTimeArray(eventTimes, finalTime_);  // [0, size(eventTimes)]

    // First Phase (starting at initTime)
    auto motionPhase = firstMotionPhase;
    if (contactStates[motionPhase]) {
      // Start in stance: Assign the polytope it is already standing on
      auto endOfStancePhaseTime = (motionPhase < eventTimes.size()) ? eventTimes[motionPhase] : finalTime_;
      newPolytopeSequence.data.push_back(findPolytopeForCurrentStanceLeg(initTime_, endOfStancePhaseTime, leg));
    } else {
      newPolytopeSequence.data.push_back(-1);
    }

    // Second Phase
    motionPhase++;
    if (motionPhase <= lastMotionPhase) {
      auto transitionTime = eventTimes[motionPhase - 1];  // event time that started this  phase
      newPolytopeSequence.time.push_back(transitionTime);
      if (contactStates[motionPhase]) {
        // Started in swing, find a suitable polytope for the first stance phase
        auto endOfStancePhaseTime = (motionPhase < eventTimes.size())
                                        ? eventTimes[motionPhase]
                                        : finalTime_;  // Assume contact phase ends at final time if no other event time available
        newPolytopeSequence.data.push_back(findPolytopeForCurrentSwingLeg(transitionTime, endOfStancePhaseTime, leg));
      } else {
        newPolytopeSequence.data.push_back(-1);
      }
    }

    // All subsequent phases
    motionPhase++;
    while (motionPhase <= lastMotionPhase) {
      auto transitionTime = eventTimes[motionPhase - 1];  // event time that started this  phase
      newPolytopeSequence.time.push_back(transitionTime);
      if (contactStates[motionPhase]) {
        // Stance phases after the first or second motion phase use nominal leg positions to select polytopes
        auto endOfStancePhaseTime = (motionPhase < eventTimes.size())
                                        ? eventTimes[motionPhase]
                                        : finalTime_;  // Assume contact phase ends at final time if no other event time available
        newPolytopeSequence.data.push_back(findPolytopeForNominalStancePhase(transitionTime, endOfStancePhaseTime, leg));
      } else {
        newPolytopeSequence.data.push_back(-1);
      }
      motionPhase++;
    }

    // Overwrite old selection
    assignedPolytopes = std::move(newPolytopeSequence);
  }
}

int TerrainModel::findPolytopeForStanceLeg(const Eigen::Vector3d footPosition) {
  Eigen::Vector4d footPositionHomogeneous;
  footPositionHomogeneous << footPosition, 1;

  // Linearly traverse polytopes and find closest one.
  int selectedPolytope = 0;
  double distance = 1e12;
  for (int i = 0; i < terrainPolytopes_.size(); i++) {
    auto constraints = toHalfSpaces(terrainPolytopes_[i]);
    double maxViolation = (constraints * footPositionHomogeneous).array().maxCoeff();
    if (maxViolation < distance) {
      selectedPolytope = i;
      distance = maxViolation;
    }
  }

  return selectedPolytope;
}

int TerrainModel::findPolytopeForCurrentSwingLeg(double startTimeOfStancePhase, double endTimeOfStancePhase, int leg) {
  auto& oldPolytopeSequence = polytopeIDSequencePerLeg_[leg];
  auto motionPhase = ocs2::lookup::findIndexInTimeArray(oldPolytopeSequence.time, startTimeOfStancePhase);

  if (motionPhase + 1 < oldPolytopeSequence.data.size()) {
    // Previously selected polytope
    return oldPolytopeSequence.data[motionPhase + 1];
  } else {
    return findPolytopeForNominalStancePhase(startTimeOfStancePhase, endTimeOfStancePhase, leg);
  }
}

int TerrainModel::findPolytopeForCurrentStanceLeg(double startTimeOfStancePhase, double endTimeOfStancePhase, int leg) {
  auto& oldPolytopeSequence = polytopeIDSequencePerLeg_[leg];
  auto motionPhase = ocs2::lookup::findIndexInTimeArray(oldPolytopeSequence.time, startTimeOfStancePhase);

  if (motionPhase < oldPolytopeSequence.data.size()) {
    // Previously selected polytope
    return oldPolytopeSequence.data[motionPhase];
  } else {  // no need to find new one for current stance leg
    return -1;
  }
}

int TerrainModel::findPolytopeForNominalStancePhase(double startTimeOfStancePhase, double endTimeOfStancePhase, int leg) {
  double middleTime = 0.5 * (startTimeOfStancePhase + endTimeOfStancePhase);
  Eigen::Vector3d footPosition = getNominalStanceFootPositionInWorld(middleTime, leg);
  return findPolytopeForStanceLeg(footPosition);
}

Eigen::Vector3d TerrainModel::getNominalStanceFootPositionInWorld(double time, int leg) {
  BaseCoordinates baseCoordinates;
  baseInterpolation_.interpolate(time, baseCoordinates);
  Eigen::Vector3d baseOrientation = baseCoordinates.segment(0, 3);
  Eigen::Vector3d basePosition = baseCoordinates.segment(3, 3);
  Eigen::Matrix3d rotationBaseToOrigin = RotationMatrixBasetoOrigin(baseOrientation);

  Eigen::Vector3d footOffset;
  double dx = 0.35;
  double dy = 0.18;
  double dz = -0.42;
  switch (leg) {
    case 0:
      footOffset << dx, dy, dz;
      break;
    case 1:
      footOffset << dx, -dy, dz;
      break;
    case 2:
      footOffset << -dx, dy, dz;
      break;
    case 3:
      footOffset << -dx, -dy, dz;
      break;
    default:
      throw std::runtime_error("[findClosestPolytope] Invalid leg number");
  }

  Eigen::Vector3d footPosition = basePosition + rotationBaseToOrigin * footOffset;
  return footPosition;
}

// void TerrainModel::assignPolytopes() {
//  //TODO(Ruben) : What if the polytopes changed?
//
//  std::cout << "assign polytopes" << std::endl;
//  auto gaitSequenceTranspose = gaitSequence_->transpose();
//
//  for (int leg = 0; leg < 4; leg++) {
//    auto &eventTimes = gaitSequenceTranspose.time[leg];
//    auto &assignedPolytopes = polytopeIDSequencePerLeg_[leg];
//    if (eventTimes.size() > 0) {
//      auto &contactStates = gaitSequenceTranspose.contactFlags[leg];
//      auto firstMotionPhase = ocs2::lookup::findIndexInTimeArray(eventTimes, initTime_);
//
//      //
//      if (assignedPolytopes.data.size() > 0) {
//        // get Polytope currently assigned to beginning of the time horizon
//        auto startIndex = ocs2::lookup::findIndexInTimeArray(assignedPolytopes.time, initTime_);
//        auto startingPolytope = assignedPolytopes.data[startIndex];
//
//        assignedPolytopes.time.clear();
//        assignedPolytopes.data.clear();
//
//        // When the foot is in contact at start of the horizon, keep the old polytope
//        if (firstMotionPhase < eventTimes.size()) {
//          assignedPolytopes.time.push_back(eventTimes[firstMotionPhase]);
//        }
//        if (contactStates[firstMotionPhase]) {
//          assignedPolytopes.data.push_back(startingPolytope);
//        } else {
//          assignedPolytopes.data.push_back(-1);
//        }
//      } else { // no polytopes were assigned yet. Set -1 even for feet already in contact
//        assignedPolytopes.time.clear();
//        assignedPolytopes.data.clear();
//
//        assignedPolytopes.time.push_back(eventTimes[firstMotionPhase]);
//        assignedPolytopes.data.push_back(-1);
//      }
//
//      // Reassign polytopes for next motion phases
//      for (int motionPhase = (firstMotionPhase + 1); motionPhase < contactStates.size(); motionPhase++) {
//        if (motionPhase < contactStates.size() - 1) { // for the last motion phase there is no time to push
//          assignedPolytopes.time.push_back(eventTimes[motionPhase]);
//        }
//        if (contactStates[motionPhase]) {
//          double middleTime;
//          if (motionPhase < contactStates.size() - 1) {
//            middleTime = 0.5 * (eventTimes[motionPhase] + eventTimes[motionPhase - 1]);
//          } else {
//            middleTime = 0.5 * (finalTime_ + eventTimes[motionPhase - 1]);
//          }
//          BaseCoordinates BaseAtMiddleOfStancePhase;
//          baseInterpolation_.interpolate(middleTime, BaseAtMiddleOfStancePhase);
//          assignedPolytopes.data.push_back(findClosestPolytope(BaseAtMiddleOfStancePhase, leg));
//        } else {
//          assignedPolytopes.data.push_back(-1);
//        }
//      }
//    } else { // no event times
//      assignedPolytopes.time.clear();
//      assignedPolytopes.data.clear();
//      assignedPolytopes.data.push_back(-1);
//    }
//  }
//}

int TerrainModel::findClosestPolytope(const BaseCoordinates& baseCoordinates, int leg) const {
  Eigen::Vector3d baseOrientation = baseCoordinates.segment(0, 3);
  Eigen::Vector3d basePosition = baseCoordinates.segment(3, 3);
  Eigen::Matrix3d rotationBaseToOrigin = RotationMatrixBasetoOrigin(baseOrientation);

  Eigen::Vector3d footOffset;
  double dx = 0.3;
  double dy = 0.2;
  double dz = -0.42;
  switch (leg) {
    case 0:
      footOffset << dx, dy, dz;
      break;
    case 1:
      footOffset << dx, -dy, dz;
      break;
    case 2:
      footOffset << -dx, dy, dz;
      break;
    case 3:
      footOffset << -dx, -dy, dz;
      break;
    default:
      throw std::runtime_error("[findClosestPolytope] Invalid leg number");
  }

  Eigen::Vector3d footPosition = basePosition + rotationBaseToOrigin * footOffset;
  Eigen::Vector4d footPositionHomogeneous;
  footPositionHomogeneous << footPosition, 1;

  // Linearly traverse polytopes and find closest one.
  int selectedPolytope = 0;
  double distance = 1e12;
  for (int i = 0; i < terrainPolytopes_.size(); i++) {
    auto constraints = toHalfSpaces(terrainPolytopes_[i]);
    double maxViolation = (constraints * footPositionHomogeneous).array().maxCoeff();
    if (maxViolation < distance) {
      selectedPolytope = i;
      distance = maxViolation;
    }
  }

  return selectedPolytope;
}

};  // namespace switched_model
