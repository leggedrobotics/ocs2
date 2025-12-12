//
// Created by rgrandia on 05.10.21.
//

#include "ocs2_anymal_commands/LoadMotions.h"

#include <ocs2_anymal_commands/TerrainAdaptation.h>
#include <ocs2_robotic_tools/common/RotationTransforms.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/Rotations.h>

namespace switched_model {

CsvData readCsv(const std::string& fileName) {
  CsvData result;

  // Create an input filestream
  std::ifstream myFile(fileName);
  std::string line;

  // Make sure the file is open
  if (!myFile.is_open() || myFile.bad()) {
    throw std::runtime_error("Could not open file: " + fileName);
  }

  {  // Read the column names
    if (std::getline(myFile, line)) {
      std::istringstream ss(line);

      std::string colname;
      // Extract each column name
      while (std::getline(ss, colname, ',')) {
        // Remove additional endline marker generated in python or windows.
        colname.erase(std::remove(colname.begin(), colname.end(), '\r'),
                      colname.end());

        result.header.push_back(colname);
        ss >> std::ws;  // remove white space
      }
    }
  }
  const size_t numCols = result.header.size();

  while (std::getline(myFile, line)) {
    std::istringstream ss(line);
    result.data.emplace_back(vector_t::Zero(numCols));

    int colIdx = 0;
    scalar_t val;
    while (ss >> val) {
      // Add current value to the right column
      result.data.back()[colIdx] = val;
      colIdx++;

      // If the next token is a comma, ignore it and move on
      if (ss.peek() == ',') {
        ss.ignore();
        ss >> std::ws;  // remove white space
      }
    }
  }

  myFile.close();

  return result;
}

std::pair<ocs2::TargetTrajectories, Gait> readMotion(const CsvData& csvData,
                                                     scalar_t dt) {
  verifyHeader(csvData.header);

  const auto numDataPoints = csvData.data.size();
  const scalar_t t0 = csvData.data.front()[0];
  const scalar_t duration = csvData.data.back()[0] - t0;

  const auto getMode = [](const vector_t dataLine) -> size_t {
    const contact_flag_t contactFlags{dataLine[1] > 0.5, dataLine[2] > 0.5,
                                      dataLine[3] > 0.5, dataLine[4] > 0.5};
    return stanceLeg2ModeNumber(contactFlags);
  };

  ocs2::TargetTrajectories targetTrajectories;
  targetTrajectories.timeTrajectory.reserve(numDataPoints);
  targetTrajectories.stateTrajectory.reserve(numDataPoints);
  targetTrajectories.inputTrajectory.reserve(numDataPoints);

  Gait gait;
  gait.duration = duration;
  gait.modeSequence.push_back(getMode(csvData.data.front()));
  for (const auto& dataLine : csvData.data) {
    const scalar_t t = dataLine[0];

    // Extend gait if the mode changes
    const size_t mode = getMode(dataLine);
    if (mode != gait.modeSequence.back()) {
      gait.eventPhases.push_back((t - t0) / duration);
      gait.modeSequence.push_back(mode);
    } else {
      // Drop a point if dt is smaller than desired
      if (!targetTrajectories.empty() &&
          t < targetTrajectories.timeTrajectory.back() + dt) {
        continue;
      }
    }

    // Time trajectory
    targetTrajectories.timeTrajectory.push_back(t);

    // Read the test of the line
    size_t colId = 5;  // after time and 4 contact flags
    const vector3_t basePositionInWorld = dataLine.segment(colId, 3);
    colId += 3;
    const Eigen::Quaterniond quaternion =
        Eigen::Quaterniond(dataLine[colId], dataLine[colId + 1],
                           dataLine[colId + 2], dataLine[colId + 3])
            .normalized();
    colId += 4;
    const vector3_t baseLinearvelocityInBase = dataLine.segment(colId, 3);
    colId += 3;
    const vector3_t baseAngularvelocityInBase = dataLine.segment(colId, 3);
    colId += 3;
    const joint_coordinate_t jointPositions =
        dataLine.segment(colId, JOINT_COORDINATE_SIZE);
    colId += JOINT_COORDINATE_SIZE;
    const joint_coordinate_t jointVelocities =
        dataLine.segment(colId, JOINT_COORDINATE_SIZE);
    colId += JOINT_COORDINATE_SIZE;
    const vector_t contactForces =
        dataLine.segment(colId, 3 * NUM_CONTACT_POINTS);

    // Convert orientation to ocs2 convention
    auto newOrientation = eulerAnglesFromQuaternionBaseToOrigin(quaternion);
    ocs2::makeEulerAnglesUnique(newOrientation);
    const auto prevYaw = targetTrajectories.stateTrajectory.empty()
                             ? 0.0
                             : targetTrajectories.stateTrajectory.back()[2];
    const auto newYaw =
        ocs2::moduloAngleWithReference(newOrientation[2], prevYaw);
    const vector3_t eulerXYZ(newOrientation[0], newOrientation[1], newYaw);
    const matrix3_t o_R_b = rotationMatrixBaseToOrigin(eulerXYZ);

    assert(o_R_b.isApprox(quaternion.toRotationMatrix()));

    vector_t contactForcesInBase(3 * NUM_CONTACT_POINTS);
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      contactForcesInBase.segment<3>(3 * i) =
          o_R_b.transpose() * contactForces.segment<3>(3 * i);
    }

    // State trajectory
    targetTrajectories.stateTrajectory.push_back(vector_t(STATE_DIM));
    targetTrajectories.stateTrajectory.back() << eulerXYZ, basePositionInWorld,
        baseAngularvelocityInBase, baseLinearvelocityInBase, jointPositions;

    // Input trajectory
    targetTrajectories.inputTrajectory.push_back(vector_t(INPUT_DIM));
    targetTrajectories.inputTrajectory.back() << contactForcesInBase,
        jointVelocities;
  }
  return {targetTrajectories, gait};
}

void verifyHeader(const std::vector<std::string>& provided) {
  const std::vector<std::string> expectedHeader = {
      "time",
      "contactflag_LF",
      "contactflag_RF",
      "contactflag_LH",
      "contactflag_RH",
      "base_positionInWorld_x",
      "base_positionInWorld_y",
      "base_positionInWorld_z",
      "base_quaternion_w",
      "base_quaternion_x",
      "base_quaternion_y",
      "base_quaternion_z",
      "base_linearvelocityInBase_x",
      "base_linearvelocityInBase_y",
      "base_linearvelocityInBase_z",
      "base_angularvelocityInBase_x",
      "base_angularvelocityInBase_y",
      "base_angularvelocityInBase_z",
      "jointAngle_LF_HAA",
      "jointAngle_LF_HFE",
      "jointAngle_LF_KFE",
      "jointAngle_RF_HAA",
      "jointAngle_RF_HFE",
      "jointAngle_RF_KFE",
      "jointAngle_LH_HAA",
      "jointAngle_LH_HFE",
      "jointAngle_LH_KFE",
      "jointAngle_RH_HAA",
      "jointAngle_RH_HFE",
      "jointAngle_RH_KFE",
      "jointVelocity_LF_HAA",
      "jointVelocity_LF_HFE",
      "jointVelocity_LF_KFE",
      "jointVelocity_RF_HAA",
      "jointVelocity_RF_HFE",
      "jointVelocity_RF_KFE",
      "jointVelocity_LH_HAA",
      "jointVelocity_LH_HFE",
      "jointVelocity_LH_KFE",
      "jointVelocity_RH_HAA",
      "jointVelocity_RH_HFE",
      "jointVelocity_RH_KFE",
      "contactForcesInWorld_LF_x",
      "contactForcesInWorld_LF_y",
      "contactForcesInWorld_LF_z",
      "contactForcesInWorld_RF_x",
      "contactForcesInWorld_RF_y",
      "contactForcesInWorld_RF_z",
      "contactForcesInWorld_LH_x",
      "contactForcesInWorld_LH_y",
      "contactForcesInWorld_LH_z",
      "contactForcesInWorld_RH_x",
      "contactForcesInWorld_RH_y",
      "contactForcesInWorld_RH_z"};
  verifyHeaderImpl(expectedHeader, provided);
}

std::pair<ocs2::TargetTrajectories, Gait> readCartesianMotion(
    const CsvData& csvData, scalar_t dt) {
  verifyCartesianHeader(csvData.header);

  const auto numDataPoints = csvData.data.size();
  const scalar_t t0 = csvData.data.front()[0];
  const scalar_t duration = csvData.data.back()[0] - t0;

  const auto getMode = [](const vector_t dataLine) -> size_t {
    const contact_flag_t contactFlags{dataLine[1] > 0.5, dataLine[2] > 0.5,
                                      dataLine[3] > 0.5, dataLine[4] > 0.5};
    return stanceLeg2ModeNumber(contactFlags);
  };

  ocs2::TargetTrajectories targetTrajectories;
  targetTrajectories.timeTrajectory.reserve(numDataPoints);
  targetTrajectories.stateTrajectory.reserve(numDataPoints);
  targetTrajectories.inputTrajectory.reserve(numDataPoints);

  Gait gait;
  gait.duration = duration;
  gait.modeSequence.push_back(getMode(csvData.data.front()));
  for (const auto& dataLine : csvData.data) {
    const scalar_t t = dataLine[0];

    // Extend gait if the mode changes
    const size_t mode = getMode(dataLine);
    if (mode != gait.modeSequence.back()) {
      gait.eventPhases.push_back((t - t0) / duration);
      gait.modeSequence.push_back(mode);
    } else {
      // Drop a point if dt is smaller than desired
      if (!targetTrajectories.empty() &&
          t < targetTrajectories.timeTrajectory.back() + dt) {
        continue;
      }
    }

    // Time trajectory
    targetTrajectories.timeTrajectory.push_back(t);

    // Read the test of the line
    size_t colId = 5;  // after time and 4 contact flags
    const vector3_t basePositionInWorld = dataLine.segment(colId, 3);
    colId += 3;
    const Eigen::Quaterniond quaternion =
        Eigen::Quaterniond(dataLine[colId], dataLine[colId + 1],
                           dataLine[colId + 2], dataLine[colId + 3])
            .normalized();
    colId += 4;
    const vector3_t baseLinearvelocityInBase = dataLine.segment(colId, 3);
    colId += 3;
    const vector3_t baseAngularvelocityInBase = dataLine.segment(colId, 3);
    colId += 3;
    const auto feetPositions = dataLine.segment(colId, 3 * NUM_CONTACT_POINTS);
    colId += 3 * NUM_CONTACT_POINTS;
    const auto feetVelocities = dataLine.segment(colId, 3 * NUM_CONTACT_POINTS);
    colId += 3 * NUM_CONTACT_POINTS;
    const vector_t contactForces =
        dataLine.segment(colId, 3 * NUM_CONTACT_POINTS);

    // Convert orientation to ocs2 convention
    auto newOrientation = eulerAnglesFromQuaternionBaseToOrigin(quaternion);
    ocs2::makeEulerAnglesUnique(newOrientation);
    const auto prevYaw = targetTrajectories.stateTrajectory.empty()
                             ? 0.0
                             : targetTrajectories.stateTrajectory.back()[2];
    const auto newYaw =
        ocs2::moduloAngleWithReference(newOrientation[2], prevYaw);
    const vector3_t eulerXYZ(newOrientation[0], newOrientation[1], newYaw);
    const matrix3_t o_R_b = rotationMatrixBaseToOrigin(eulerXYZ);

    assert(o_R_b.isApprox(quaternion.toRotationMatrix()));

    vector_t contactForcesInBase(3 * NUM_CONTACT_POINTS);
    for (size_t i = 0; i < NUM_CONTACT_POINTS; i++) {
      contactForcesInBase.segment<3>(3 * i) =
          o_R_b.transpose() * contactForces.segment<3>(3 * i);
    }

    // State trajectory
    targetTrajectories.stateTrajectory.push_back(
        vector_t(2 * BASE_COORDINATE_SIZE + 6 * NUM_CONTACT_POINTS));
    targetTrajectories.stateTrajectory.back() << eulerXYZ, basePositionInWorld,
        baseAngularvelocityInBase, baseLinearvelocityInBase, feetPositions,
        feetVelocities;

    // Input trajectory
    targetTrajectories.inputTrajectory.push_back(
        vector_t(3 * NUM_CONTACT_POINTS));
    targetTrajectories.inputTrajectory.back() << contactForcesInBase;
  }
  return {targetTrajectories, gait};
}

void verifyCartesianHeader(const std::vector<std::string>& provided) {
  const std::vector<std::string> expectedHeader = {
      "time",
      "contactflag_LF",
      "contactflag_RF",
      "contactflag_LH",
      "contactflag_RH",
      "base_positionInWorld_x",
      "base_positionInWorld_y",
      "base_positionInWorld_z",
      "base_quaternion_w",
      "base_quaternion_x",
      "base_quaternion_y",
      "base_quaternion_z",
      "base_linearvelocityInBase_x",
      "base_linearvelocityInBase_y",
      "base_linearvelocityInBase_z",
      "base_angularvelocityInBase_x",
      "base_angularvelocityInBase_y",
      "base_angularvelocityInBase_z",
      "footPositionInWorld_LF_x",
      "footPositionInWorld_LF_y",
      "footPositionInWorld_LF_z",
      "footPositionInWorld_RF_x",
      "footPositionInWorld_RF_y",
      "footPositionInWorld_RF_z",
      "footPositionInWorld_LH_x",
      "footPositionInWorld_LH_y",
      "footPositionInWorld_LH_z",
      "footPositionInWorld_RH_x",
      "footPositionInWorld_RH_y",
      "footPositionInWorld_RH_z",
      "footVelocityInWorld_LF_x",
      "footVelocityInWorld_LF_y",
      "footVelocityInWorld_LF_z",
      "footVelocityInWorld_RF_x",
      "footVelocityInWorld_RF_y",
      "footVelocityInWorld_RF_z",
      "footVelocityInWorld_LH_x",
      "footVelocityInWorld_LH_y",
      "footVelocityInWorld_LH_z",
      "footVelocityInWorld_RH_x",
      "footVelocityInWorld_RH_y",
      "footVelocityInWorld_RH_z",
      "contactForcesInWorld_LF_x",
      "contactForcesInWorld_LF_y",
      "contactForcesInWorld_LF_z",
      "contactForcesInWorld_RF_x",
      "contactForcesInWorld_RF_y",
      "contactForcesInWorld_RF_z",
      "contactForcesInWorld_LH_x",
      "contactForcesInWorld_LH_y",
      "contactForcesInWorld_LH_z",
      "contactForcesInWorld_RH_x",
      "contactForcesInWorld_RH_y",
      "contactForcesInWorld_RH_z"};
  verifyHeaderImpl(expectedHeader, provided);
}

void verifyHeaderImpl(const std::vector<std::string>& expected,
                      const std::vector<std::string>& provided) {
  // Check header
  if (provided.size() != expected.size()) {
    throw std::runtime_error("Incorrect amount of columns. Expected: " +
                             std::to_string(expected.size()) + ", but got " +
                             std::to_string(provided.size()));
  }
  for (size_t i = 0; i < expected.size(); ++i) {
    if (provided[i] != expected[i]) {
      throw std::runtime_error(
          "Incorrect header of column " + std::to_string(i) +
          ", expected: " + expected[i] + ", but got " + provided[i]);
    }
  }
}

}  // namespace switched_model