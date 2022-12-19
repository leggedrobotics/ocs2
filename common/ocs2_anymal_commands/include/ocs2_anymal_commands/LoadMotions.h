//
// Created by rgrandia on 05.10.21.
//

#pragma once

#include <ocs2_core/reference/TargetTrajectories.h>
#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/Gait.h>

namespace switched_model {

/**
 * Struct to store the result of reading a csv file with 1 header and lines of floating point data
 */
struct CsvData {
  std::vector<std::string> header;
  std::vector<vector_t> data;
};

/**
 * Read a csv file with 1 header and lines of floating point data
 */
CsvData readCsv(const std::string& fileName);

/**
 * Convert csv data into a motion reference and gait
 *
 * Expects the following header:
 * time, contactflag_LF, contactflag_RF, contactflag_LH, contactflag_RH, base_positionInWorld_x, base_positionInWorld_y,
 * base_positionInWorld_z, base_quaternion_w, base_quaternion_x, base_quaternion_y, base_quaternion_z,  base_linearvelocityInBase_x,
 * base_linearvelocityInBase_y, base_linearvelocityInBase_z, base_angularvelocityInBase_x, base_angularvelocityInBase_y,
 * base_angularvelocityInBase_z, jointAngle_LF_HAA, jointAngle_LF_HFE, jointAngle_LF_KFE, jointAngle_RF_HAA, jointAngle_RF_HFE,
 * jointAngle_RF_KFE, jointAngle_LH_HAA, jointAngle_LH_HFE, jointAngle_LH_KFE, jointAngle_RH_HAA, jointAngle_RH_HFE, jointAngle_RH_KFE,
 * jointVelocity_LF_HAA, jointVelocity_LF_HFE, jointVelocity_LF_KFE, jointVelocity_RF_HAA, jointVelocity_RF_HFE, jointVelocity_RF_KFE,
 * jointVelocity_LH_HAA, jointVelocity_LH_HFE, jointVelocity_LH_KFE, jointVelocity_RH_HAA, jointVelocity_RH_HFE, jointVelocity_RH_KFE,
 * contactForcesInWorld_LF_x, contactForcesInWorld_LF_y, contactForcesInWorld_LF_z, contactForcesInWorld_RF_x, contactForcesInWorld_RF_y,
 * contactForcesInWorld_RF_z, contactForcesInWorld_LH_x, contactForcesInWorld_LH_y, contactForcesInWorld_LH_z, contactForcesInWorld_RH_x,
 * contactForcesInWorld_RH_y, contactForcesInWorld_RH_z
 *
 * @param fileName : absolute path of the file
 * @param dt : approximate sampling interval. Reference points closer than this dt will be dropped. Set to negative to load all points.
 * @return reference motion and gait
 */
std::pair<ocs2::TargetTrajectories, Gait> readMotion(const CsvData& csvData, scalar_t dt = -1.0);

void verifyHeader(const std::vector<std::string>& provided);

/**
 * Convert csv data into a cartesian motion reference and gait
 *
 * Expects the following header:
 *
 * @param fileName : absolute path of the file
 * @param dt : approximate sampling interval. Reference points closer than this dt will be dropped. Set to negative to load all points.
 * @return reference motion and gait
 */
std::pair<ocs2::TargetTrajectories, Gait> readCartesianMotion(const CsvData& csvData, scalar_t dt = -1.0);

void verifyCartesianHeader(const std::vector<std::string>& provided);

void verifyHeaderImpl(const std::vector<std::string>& expected, const std::vector<std::string>& provided);

}  // namespace switched_model