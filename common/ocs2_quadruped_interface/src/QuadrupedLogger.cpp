//
// Created by rgrandia on 05.11.20.
//

#include "ocs2_quadruped_interface/QuadrupedLogger.h"

#include <fstream>

#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <ocs2_switched_model_interface/core/TorqueApproximation.h>

namespace switched_model {

QuadrupedLogger::QuadrupedLogger(std::string logFileName, const kinematic_model_t& kinematicModel, const com_model_t& comModel,
                                 std::vector<std::string> additionalColumns)
    : logFileName_(std::move(logFileName)),
      kinematicModel_(kinematicModel.clone()),
      comModel_(comModel.clone()),
      additionalColumns_(std::move(additionalColumns)) {}

QuadrupedLogger::~QuadrupedLogger() {
  Eigen::IOFormat CommaFmt(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", ", ", "", "", "", "");

  if (std::ofstream logfile{logFileName_}) {
    logfile << getLogHeader() << std::endl;
    for (int i = 1; i < buffer_.size(); ++i) {
      logfile << buffer_[i].transpose().format(CommaFmt) << ", \n";
    }
    std::cerr << "[QuadrupedLogger] Log written to '" << logFileName_ << "'\n";
  } else {
    std::cerr << "[QuadrupedLogger] Unable to open '" << logFileName_ << "'\n";
  }
}

std::string QuadrupedLogger::getLogHeader() const {
  std::string delim = ", ";
  std::stringstream header;
  // clang-format off
  header <<
         "time" << delim <<
         "contactflag_LF" << delim <<
         "contactflag_RF" << delim <<
         "contactflag_LH" << delim <<
         "contactflag_RH" << delim <<
         "base_positionInWorld_x" << delim <<
         "base_positionInWorld_y" << delim <<
         "base_positionInWorld_z" << delim <<
         "base_quaternion_w" << delim <<
         "base_quaternion_x" << delim <<
         "base_quaternion_y" << delim <<
         "base_quaternion_z" << delim <<
         "base_linearvelocityInBase_x" << delim <<
         "base_linearvelocityInBase_y" << delim <<
         "base_linearvelocityInBase_z" << delim <<
         "base_angularvelocityInBase_x" << delim <<
         "base_angularvelocityInBase_y" << delim <<
         "base_angularvelocityInBase_z" << delim;
  // clang-format on
  for (const auto& name : namesPerLeg("jointAngle", {"HAA", "HFE", "KFE"})) {
    header << name << delim;
  }
  for (const auto& name : namesPerLeg("jointVelocity", {"HAA", "HFE", "KFE"})) {
    header << name << delim;
  }
  for (const auto& name : namesPerLeg("contactForcesInWorld", {"x", "y", "z"})) {
    header << name << delim;
  }
  for (const auto& name : additionalColumns_) {
    header << name << delim;
  }
  return header.str();
}

int QuadrupedLogger::getNumColumns() const {
  int numColumns = 0;
  numColumns += 1;                          // time
  numColumns += NUM_CONTACT_POINTS;         // contactFlags
  numColumns += BASE_COORDINATE_SIZE + 1;   // base pose (with quaternion)
  numColumns += BASE_COORDINATE_SIZE;       // base twist
  numColumns += JOINT_COORDINATE_SIZE;      // joint angles
  numColumns += JOINT_COORDINATE_SIZE;      // joint velocities
  numColumns += 3 * NUM_CONTACT_POINTS;     // contact forces
  numColumns += additionalColumns_.size();  // extra columns
  return numColumns;
}

void QuadrupedLogger::addLine(const ocs2::SystemObservation& observation, const vector_t& additionalColumns) {
  if (!buffer_.empty() && observation.time == buffer_.back()[0]) {  // time is same as last one
    return;
  }
  comkino_state_t state(observation.state);
  comkino_input_t input(observation.input);

  // Extract elements from state
  const base_coordinate_t basePose = getBasePose(state);
  const auto basePosition = getPositionInOrigin(basePose);
  const base_coordinate_t baseLocalVelocities = getBaseLocalVelocities(state);
  const auto baseAngularVelocities = getAngularVelocity(baseLocalVelocities);
  const auto baseLinearVelocities = getLinearVelocity(baseLocalVelocities);
  const joint_coordinate_t qJoints = getJointPositions(state);
  const joint_coordinate_t dqJoints = getJointVelocities(input);
  const Eigen::Matrix3d o_R_b = rotationMatrixBaseToOrigin(getOrientation(basePose));
  const auto quat = ocs2::matrixToQuaternion(o_R_b);

  // Contact state
  const contact_flag_t contactFlags = modeNumber2StanceLeg(observation.mode);

  // Forces
  joint_coordinate_t forceInputs = input.head<3 * NUM_CONTACT_POINTS>();
  const auto contactForcesInBase = toArray(forceInputs);
  feet_array_t<vector3_t> contactForcesInWorld;
  for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
    contactForcesInWorld[i] = o_R_b * contactForcesInBase[i];
  }

  // Fill log
  vector_t logEntry(getNumColumns());
  // clang-format off
  logEntry <<
      observation.time,
      static_cast<double>(contactFlags[0]),
      static_cast<double>(contactFlags[1]),
      static_cast<double>(contactFlags[2]),
      static_cast<double>(contactFlags[3]),
      basePosition,
      quat.w(),
      quat.x(),
      quat.y(),
      quat.z(),
      baseLinearVelocities,
      baseAngularVelocities,
      qJoints,
      dqJoints,
      contactForcesInWorld[0],
      contactForcesInWorld[1],
      contactForcesInWorld[2],
      contactForcesInWorld[3],
      additionalColumns;
  // clang-format on

  buffer_.push_back(std::move(logEntry));
}

std::vector<std::string> QuadrupedLogger::namesPerLeg(const std::string& prefix, const std::vector<std::string>& postfixes) {
  std::vector<std::string> names;
  for (const auto& legName : std::vector<std::string>{"LF", "RF", "LH", "RH"}) {
    for (const auto& postfix : postfixes) {
      names.emplace_back(prefix + "_" + legName + "_" + postfix);
    }
  }
  return names;
}

}  // namespace switched_model