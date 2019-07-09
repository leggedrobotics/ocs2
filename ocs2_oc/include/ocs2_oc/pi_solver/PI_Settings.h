#pragma once

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>
#include <iostream>
#include <string>

#include <ocs2_oc/rollout/Rollout_Settings.h>

namespace ocs2 {

/**
 * @brief The PI_Settings class stores settings for the path integral algorithm
 */
class PI_Settings {
 public:
  /**
   * @brief PI_Settings constructor with default values
   */
  PI_Settings(double rollout_dt = 1e-3, double gamma = 0.1, size_t numSamples = 100, bool debugPrint = false)
      : gamma_(gamma), numSamples_(numSamples), debugPrint_(debugPrint) {}

  /**
   * @brief loadSettings reads the settings from a config file
   * @param filename Absolute path to the settings file
   * @param fieldName How the field is called that contains the path integral settings
   * @param verbose Flag to determine whether to print out the loaded settings or not
   */
  void loadSettings(const std::string& filename, const std::string& fieldName = "pathIntegral", bool verbose = true);

  double gamma_;       //! temperature/level of noise
  size_t numSamples_;  //! how many trajectories to sample
  bool debugPrint_;    //! verbose printing output for debugging

  Rollout_Settings rolloutSettings_; //! settings for rollouts used in PI solver
};

void PI_Settings::loadSettings(const std::string& filename, const std::string& fieldName, bool verbose) {

  rolloutSettings_.loadSettings(filename, fieldName+".rollout_settings", verbose);

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### Path Integral Settings:\n"
              << " #### =============================================================================" << std::endl;
  }

  try {
    gamma_ = pt.get<double>(fieldName + ".gamma");
    if (verbose) {
      std::cerr << " #### Option loader : option 'gamma' ....................... " << gamma_ << std::endl;
    }
  } catch (const boost::property_tree::ptree_bad_path&) {
    if (verbose) {
      std::cerr << " #### Option loader : option 'gamma' ....................... " << gamma_ << "\t(default)" << std::endl;
    }
  }

  try {
    numSamples_ = pt.get<size_t>(fieldName + ".numSamples");
    if (verbose) {
      std::cerr << " #### Option loader : option 'numSamples' ....................... " << numSamples_ << std::endl;
    }
  } catch (const boost::property_tree::ptree_bad_path&) {
    if (verbose) {
      std::cerr << " #### Option loader : option 'numSamples' ....................... " << numSamples_ << "\t(default)" << std::endl;
    }
  }

  try {
    debugPrint_ = pt.get<bool>(fieldName + ".debugPrint");
    if (verbose) {
      std::cerr << " #### Option loader : option 'debugPrint' ....................... " << debugPrint_ << std::endl;
    }
  } catch (const boost::property_tree::ptree_bad_path&) {
    if (verbose) {
      std::cerr << " #### Option loader : option 'debugPrint' ....................... " << debugPrint_ << "\t(default)" << std::endl;
    }
  }
}

}  // namespace ocs2
