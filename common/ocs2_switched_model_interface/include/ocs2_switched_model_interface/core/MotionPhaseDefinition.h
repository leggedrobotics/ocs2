/*
 * MotionPhaseDefinition.h
 *
 *  Created on: Nov 16, 2017
 *      Author: farbod
 */

#ifndef MOTIONPHASEDEFINITION_H_
#define MOTIONPHASEDEFINITION_H_

#include <iostream>
#include <map>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include "ocs2_switched_model_interface/core/SwitchedModel.h"

#include <ocs2_core/logic/rules/ModeSequenceTemplate.h>

namespace switched_model {

enum ModeNumber {  // {LF, RF, LH, RH}
  FLY = 0,
  RH = 1,
  LH = 2,
  LH_RH = 3,
  RF = 4,
  RF_RH = 5,
  RF_LH = 6,
  RF_LH_RH = 7,
  LF = 8,
  LF_RH = 9,
  LF_LH = 10,
  LF_LH_RH = 11,
  LF_RF = 12,
  LF_RF_RH = 13,
  LF_RF_LH = 14,
  STANCE = 15,
};

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline contact_flag_t modeNumber2StanceLeg(const size_t& modeNumber) {
  contact_flag_t stanceLegs;  // {LF, RF, LH, RH}

  switch (modeNumber) {
    case 0:
      stanceLegs = contact_flag_t{0, 0, 0, 0};
      break;  // 0:  0-leg-stance
    case 1:
      stanceLegs = contact_flag_t{0, 0, 0, 1};
      break;  // 1:  RH
    case 2:
      stanceLegs = contact_flag_t{0, 0, 1, 0};
      break;  // 2:  LH
    case 3:
      stanceLegs = contact_flag_t{0, 0, 1, 1};
      break;  // 3:  RH, LH
    case 4:
      stanceLegs = contact_flag_t{0, 1, 0, 0};
      break;  // 4:  RF
    case 5:
      stanceLegs = contact_flag_t{0, 1, 0, 1};
      break;  // 5:  RF, RH
    case 6:
      stanceLegs = contact_flag_t{0, 1, 1, 0};
      break;  // 6:  RF, LH
    case 7:
      stanceLegs = contact_flag_t{0, 1, 1, 1};
      break;  // 7:  RF, LH, RH
    case 8:
      stanceLegs = contact_flag_t{1, 0, 0, 0};
      break;  // 8:  LF,
    case 9:
      stanceLegs = contact_flag_t{1, 0, 0, 1};
      break;  // 9:  LF, RH
    case 10:
      stanceLegs = contact_flag_t{1, 0, 1, 0};
      break;  // 10: LF, LH
    case 11:
      stanceLegs = contact_flag_t{1, 0, 1, 1};
      break;  // 11: LF, LH, RH
    case 12:
      stanceLegs = contact_flag_t{1, 1, 0, 0};
      break;  // 12: LF, RF
    case 13:
      stanceLegs = contact_flag_t{1, 1, 0, 1};
      break;  // 13: LF, RF, RH
    case 14:
      stanceLegs = contact_flag_t{1, 1, 1, 0};
      break;  // 14: LF, RF, LH
    case 15:
      stanceLegs = contact_flag_t{1, 1, 1, 1};
      break;  // 15: 4-leg-stance
  }

  return stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t stanceLeg2ModeNumber(const contact_flag_t& stanceLegs) {
  return stanceLegs[3] + 2 * stanceLegs[2] + 4 * stanceLegs[1] + 8 * stanceLegs[0];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline std::string modeNumber2String(const size_t& modeNumber) {
  // build the map from mode number to name
  std::map<size_t, std::string> modeToName;
  modeToName[FLY] = "FLY";
  modeToName[RH] = "RH";
  modeToName[LH] = "LH";
  modeToName[LH_RH] = "LH_RH";
  modeToName[RF] = "RF";
  modeToName[RF_RH] = "RF_RH";
  modeToName[RF_LH] = "RF_LH";
  modeToName[RF_LH_RH] = "RF_LH_RH";
  modeToName[LF] = "LF";
  modeToName[LF_RH] = "LF_RH";
  modeToName[LF_LH] = "LF_LH";
  modeToName[LF_LH_RH] = "LF_LH_RH";
  modeToName[LF_RF] = "LF_RF";
  modeToName[LF_RF_RH] = "LF_RF_RH";
  modeToName[LF_RF_LH] = "LF_RF_LH";
  modeToName[STANCE] = "STANCE";

  return modeToName[modeNumber];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
inline size_t string2ModeNumber(const std::string& modeString) {
  // build the map from name to mode number
  std::map<std::string, size_t> nameToMode;
  nameToMode["FLY"] = FLY;
  nameToMode["RH"] = RH;
  nameToMode["LH"] = LH;
  nameToMode["LH_RH"] = LH_RH;
  nameToMode["RF"] = RF;
  nameToMode["RF_RH"] = RF_RH;
  nameToMode["RF_LH"] = RF_LH;
  nameToMode["RF_LH_RH"] = RF_LH_RH;
  nameToMode["LF"] = LF;
  nameToMode["LF_RH"] = LF_RH;
  nameToMode["LF_LH"] = LF_LH;
  nameToMode["LF_LH_RH"] = LF_LH_RH;
  nameToMode["LF_RF"] = LF_RF;
  nameToMode["LF_RF_RH"] = LF_RF_RH;
  nameToMode["LF_RF_LH"] = LF_RF_LH;
  nameToMode["STANCE"] = STANCE;

  return nameToMode[modeString];
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename T>
void loadStdVector(const std::string& filename, const std::string& topicName, std::vector<T>& loadVector, bool verbose = true) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  // read the modes from taskFile
  size_t vectorSize = 0;
  loadVector.clear();
  while (true) {
    try {
      loadVector.push_back(pt.get<T>(topicName + ".[" + std::to_string(vectorSize) + "]"));
      vectorSize++;
    } catch (const std::exception& e) {
      break;
    }
  }  // end of while loop

  // display
  if (verbose == true) {
    if (vectorSize == 0) {
      std::cerr << topicName << ": { }";
    } else {
      std::cerr << topicName << ": {";
      for (size_t i = 0; i < vectorSize; i++) std::cerr << loadVector[i] << ", ";
      std::cerr << "\b\b}" << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
inline void loadModes(const std::string& filename, const std::string& topicName, std::vector<SCALAR_T>& switchingModes,
                      bool verbose = true) {
  // read the modes from taskFile
  std::vector<std::string> switchingModesString;
  loadStdVector(filename, topicName, switchingModesString, false);

  const size_t numSubsystems = switchingModesString.size();

  // convert the mode name to mode enum
  switchingModes.resize(numSubsystems);
  for (size_t i = 0; i < numSubsystems; i++) switchingModes[i] = string2ModeNumber(switchingModesString[i]);

  // display
  if (verbose == true) {
    if (numSubsystems == 0) {
      std::cerr << topicName << ": { }";
    } else {
      std::cerr << topicName << ": {";
      for (size_t i = 0; i < numSubsystems; i++) std::cerr << modeNumber2String(switchingModes[i]) << ", ";
      std::cerr << "\b\b}" << std::endl;
    }
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
template <typename SCALAR_T>
inline void loadModeSequenceTemplate(const std::string& filename, const std::string& topicName,
                                     ocs2::ModeSequenceTemplate<SCALAR_T>& modeSequenceTemplate, bool verbose = true) {
  // read the modes from file
  try {
    loadModes(filename, topicName + ".templateSubsystemsSequence", modeSequenceTemplate.templateSubsystemsSequence_, verbose);
    loadStdVector(filename, topicName + ".templateSwitchingTimes", modeSequenceTemplate.templateSwitchingTimes_, verbose);
  } catch (const std::exception& e) {
    std::cerr << "WARNING: Failed to load " + topicName + "!" << std::endl;
  }
}

}  // end of namespace switched_model

#endif /* MOTIONPHASEDEFINITION_H_ */
