//
// Created by rgrandia on 27.07.19.
//

#include "ocs2_switched_model_interface/ground/TerrainModel.h"
#include "ocs2_core/misc/Lookup.h"


namespace switched_model {

TerrainModel::TerrainModel(std::shared_ptr<const GaitSequence> gaitSequence) : gaitSequence_(std::move(gaitSequence)) {

}

TerrainModel::~TerrainModel() = default;

void TerrainModel::update(double initTime,
                          double finalTime,
                          const Eigen::Matrix<double, -1, 1>& currentState,
                          const ocs2::CostDesiredTrajectories<double> &costDesiredTrajectory,
                          const ocs2::HybridLogicRules* hybridLogicRules) {
  std::cout << "TerrainModel Updated from MPC" << std::endl;

  initTime_ = initTime;
  finalTime_ = finalTime;

  // extract raw trajectory for the base
  BaseTrajectory rawBaseTrajectory_;
  ocs2::EigenLinearInterpolation<BaseCoordinates> rawBaseInterpolation_;
  rawBaseTrajectory_.time = costDesiredTrajectory.desiredTimeTrajectory();
  rawBaseTrajectory_.data.clear();
  for (auto state_des : costDesiredTrajectory.desiredStateTrajectory()){
    rawBaseTrajectory_.data.emplace_back(state_des.segment(0, 6));
  }
  rawBaseInterpolation_.setData(&rawBaseTrajectory_.time, &rawBaseTrajectory_.data);

  baseTrajectory_.time.clear();
  baseTrajectory_.data.clear();
  baseTrajectory_.time.push_back(initTime);
  baseTrajectory_.data.emplace_back(currentState.segment(0,6));
  double dt = 0.1;
  double a = 2.0 * dt; // settling time ~ 2.2 / a_continuous
  while (baseTrajectory_.time.back() < finalTime) {
    baseTrajectory_.time.push_back( baseTrajectory_.time.back() + dt );
    BaseCoordinates desiredBase;
    rawBaseInterpolation_.interpolate(baseTrajectory_.time.back(), desiredBase);
    baseTrajectory_.data.emplace_back( (1-a) * baseTrajectory_.data.back() + a * desiredBase );
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

void TerrainModel::setPolytopes(ConvexPlanarPolytope3dArray terrainPolytopes) {
  terrainPolytopes_ = std::move(terrainPolytopes);
}

const ConvexPlanarPolytope3dArray& TerrainModel::getPolytopes() const {
  return terrainPolytopes_;
}

void TerrainModel::loadTestTerrain() {
    std::cout << "TerrainModel loaded" << std::endl;
//      // LF
//    ConvexPlanarPolytope3d polytope0;
//    {
//    Eigen::Vector3d offset0{0.2, 0.2, 0.0};
//    polytope0.reserve(4);
//    Eigen::Vector3d point00{ -0.0, -0.141, 0.0}; point00 += offset0; polytope0.push_back(point00);
//    Eigen::Vector3d point01{  0.141, 0.0, 0.0};  point01 += offset0; polytope0.push_back(point01);
//    Eigen::Vector3d point02{  0.0, 0.141, 0.0};  point02 += offset0; polytope0.push_back(point02);
//    Eigen::Vector3d point03{ -0.141, 0.0, 0.0};  point03 += offset0; polytope0.push_back(point03);
//    }
//
//    // RH
//    ConvexPlanarPolytope3d polytope1;
//    {
//    Eigen::Vector3d offset1{0.35, -0.25, 0.0};
//    polytope1.reserve(3);
//    Eigen::Vector3d point10 {-0.1, -0.1, 0.0}; point10 += offset1; polytope1.push_back(point10);
//    Eigen::Vector3d point11 { 0.1, -0.1, 0.0};  point11 += offset1; polytope1.push_back(point11);
//    Eigen::Vector3d point12 {  0.0, 0.1, 0.0};  point12 += offset1; polytope1.push_back(point12);
//    }
//
//    // LH
//    ConvexPlanarPolytope3d polytope2;
//    {
//    Eigen::Vector3d offset2 {-0.3, 0.2, 0.0};
//    polytope2.reserve(4);
//    Eigen::Vector3d point20{ -0.1, -0.1, 0.0};  point20 += offset2; polytope2.push_back(point20);
//    Eigen::Vector3d point21{ 0.1, -0.1, 0.0};  point21 += offset2; polytope2.push_back(point21);
//    Eigen::Vector3d point22{ 0.1, 0.1, 0.0}; point22 += offset2; polytope2.push_back(point22);
//    Eigen::Vector3d point23{ -0.1, 0.1, 0.0};  point23 += offset2; polytope2.push_back(point23);
//    }
//
//    // RH
//    ConvexPlanarPolytope3d polytope3;
//    {
//      Eigen::Vector3d offset3 {-0.2, -0.15, 0.0};
//      polytope2.reserve(5);
//      Eigen::Vector3d point30{-0.1, -0.1, 0.0}; point30 += offset3; polytope3.push_back(point30);
//      Eigen::Vector3d point31{0.1, -0.1, 0.0}; point31 += offset3; polytope3.push_back(point31);
//      Eigen::Vector3d point32{0.167, 0.067, 0.0}; point32 += offset3; polytope3.push_back(point32);
//      Eigen::Vector3d point33{0.0, 0.167, 0.0}; point33 += offset3; polytope3.push_back(point33);
//      Eigen::Vector3d point34{-0.167, 0.067, 0.0}; point34 += offset3; polytope3.push_back(point34);
//    }

    ConvexPlanarPolytope3dArray polytopes;
    polytopes.reserve(4);
    double squareSize = 0.15;
    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{0.25, 0.25, 0.0}));
    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{0.25, -0.25, 0.0}));
    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{-0.25, 0.25, 0.0}));
    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{-0.25, -0.25, 0.0}));

    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{0.75, 0.0, 0.0}));
    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{-0.75, 0.0, 0.0}));
    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{0., 0.75, 0.0}));
    polytopes.push_back(createSquare(squareSize, Eigen::Vector3d{0.0, -0.75, 0.0}));

    setPolytopes(std::move(polytopes));
}

Eigen::Matrix<double, -1, 4> TerrainModel::getTerrainConstraints(double time, int legID) const {
  auto index = ocs2::lookup::findIndexInTimeArray(polytopeIDSequencePerLeg_[legID].time, time);
  auto selectedPolytopeID = polytopeIDSequencePerLeg_[legID].data[index];
  if (selectedPolytopeID >= 0) {
    return toHalfSpaces(terrainPolytopes_[selectedPolytopeID]);
  } else { // no constraints
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
  for (int k = 0; k<baseTrajectory_.time.size(); k++){
    std::cout << "t: " << baseTrajectory_.time[k] << "\t x: " << baseTrajectory_.data[k].transpose() << std::endl;
  }

}

void TerrainModel::assignPolytopes() {
  std::cout << "assign polytopes" << std::endl;
  auto gaitSequenceTranspose = gaitSequence_->transpose();

  for (int leg = 0; leg < 4; leg++) {
    auto &eventTimes = gaitSequenceTranspose.time[leg];
    auto &contactStates = gaitSequenceTranspose.contactFlags[leg];
    auto &assignedPolytopes = polytopeIDSequencePerLeg_[leg];
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
      auto transitionTime = eventTimes[motionPhase-1]; // event time that started this  phase
      newPolytopeSequence.time.push_back(transitionTime);
      if (contactStates[motionPhase]) {
        // Started in swing, find a suitable polytope for the first stance phase
        auto endOfStancePhaseTime = (motionPhase < eventTimes.size()) ? eventTimes[motionPhase] : finalTime_; // Assume contact phase ends at final time if no other event time available
        newPolytopeSequence.data.push_back(findPolytopeForCurrentSwingLeg(transitionTime, endOfStancePhaseTime, leg));
      } else {
        newPolytopeSequence.data.push_back(-1);
      }
    }

    // All subsequent phases
    motionPhase++;
    while (motionPhase <= lastMotionPhase) {
      auto transitionTime = eventTimes[motionPhase-1]; // event time that started this  phase
      newPolytopeSequence.time.push_back(transitionTime);
      if (contactStates[motionPhase]) {
        // Stance phases after the first or second motion phase use nominal leg positions to select polytopes
        auto endOfStancePhaseTime = (motionPhase < eventTimes.size()) ? eventTimes[motionPhase] : finalTime_; // Assume contact phase ends at final time if no other event time available
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
  for (int i = 0; i<terrainPolytopes_.size(); i++){
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

  if (motionPhase+1 < oldPolytopeSequence.data.size()){
    // Previously selected polytope
      return oldPolytopeSequence.data[motionPhase+1];
  } else {
      return findPolytopeForNominalStancePhase(startTimeOfStancePhase, endTimeOfStancePhase, leg);
  }
}

int TerrainModel::findPolytopeForCurrentStanceLeg(double startTimeOfStancePhase, double endTimeOfStancePhase, int leg) {
  auto& oldPolytopeSequence = polytopeIDSequencePerLeg_[leg];
  auto motionPhase = ocs2::lookup::findIndexInTimeArray(oldPolytopeSequence.time, startTimeOfStancePhase);

  if (motionPhase < oldPolytopeSequence.data.size()){
    // Previously selected polytope
    return oldPolytopeSequence.data[motionPhase];
  } else { // no need to find new one for current stance leg
    return -1;
  }
}

int TerrainModel::findPolytopeForNominalStancePhase(double startTimeOfStancePhase,
                                                    double endTimeOfStancePhase,
                                                    int leg) {
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
  double dx = 0.3;
  double dy = 0.2;
  double dz = -0.42;
  switch (leg) {
    case 0 :
      footOffset << dx, dy, dz;
      break;
    case 1 :
      footOffset << dx, -dy, dz;
      break;
    case 2 :
      footOffset << -dx, dy, dz;
      break;
    case 3 :
      footOffset << -dx, -dy, dz;
      break;
    default:
      throw std::runtime_error("[findClosestPolytope] Invalid leg number");
  }

  Eigen::Vector3d footPosition = basePosition + rotationBaseToOrigin * footOffset;
  return footPosition;
}

//void TerrainModel::assignPolytopes() {
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

int TerrainModel::findClosestPolytope(const BaseCoordinates &baseCoordinates, int leg) const {
  Eigen::Vector3d baseOrientation = baseCoordinates.segment(0, 3);
  Eigen::Vector3d basePosition = baseCoordinates.segment(3, 3);
  Eigen::Matrix3d rotationBaseToOrigin = RotationMatrixBasetoOrigin(baseOrientation);

  Eigen::Vector3d footOffset;
  double dx = 0.3;
  double dy = 0.2;
  double dz = -0.42;
  switch (leg) {
    case 0 :
      footOffset << dx, dy, dz;
      break;
    case 1 :
      footOffset << dx, -dy, dz;
      break;
    case 2 :
      footOffset << -dx, dy, dz;
      break;
    case 3 :
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
  for (int i = 0; i<terrainPolytopes_.size(); i++){
    auto constraints = toHalfSpaces(terrainPolytopes_[i]);
    double maxViolation = (constraints * footPositionHomogeneous).array().maxCoeff();
    if (maxViolation < distance) {
      selectedPolytope = i;
      distance = maxViolation;
    }
  }

  return selectedPolytope;
}

}; // namespace switched_model
