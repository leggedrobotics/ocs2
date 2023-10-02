//
// Created by rgrandia on 05.11.20.
//

#pragma once

#include <ocs2_mpc/SystemObservation.h>
#include <ocs2_ros_interfaces/mrt/DummyObserver.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

/**
 * Simple logger for Quadruped observations.
 * Logs a sequence of quadruped observations to a file. Has the option to log additional user-defined data.
 *
 * Can be either used stand-alone or as a dummy observer
 */
class QuadrupedLogger : public ocs2::DummyObserver {
 public:
  using com_model_t = ComModelBase<double>;
  using kinematic_model_t = KinematicsModelBase<double>;

  QuadrupedLogger(std::string logFileName, const kinematic_model_t& kinematicModel, const com_model_t& comModel,
                  std::vector<std::string> additionalColumns = {});

  /**
   * The destructor writes the buffered log data to file.
   */
  ~QuadrupedLogger() override;

  /**
   * Dummy observer interface. Forwards to the addline method
   */
  void update(const ocs2::SystemObservation& observation, const ocs2::PrimalSolution& primalSolution,
              const ocs2::CommandData& command) override {
    addLine(observation);
  }

  /**
   * Adds a line to the log. If the observation time is already present in the log, the new line is dropped to avoid duplicates.
   * The user data should match the amount of additional columns declared in the constructor.
   */
  void addLine(const ocs2::SystemObservation& observation, const vector_t& additionalColumns = vector_t());

  /**
   * Generates names for {LF, RF, LH, RH} for the give pre and post fixes.
   * @return {<prefix>_LF_<postfix[0]>, ... , <prefix>_LF_<postfix[N-1]>, <prefix>_RF_<postfix[0]>, ... }
   */
  static std::vector<std::string> namesPerLeg(const std::string& prefix, const std::vector<std::string>& postfixes);

 private:
  std::string getLogHeader() const;
  int getNumColumns() const;

  std::string logFileName_;
  std::vector<std::string> additionalColumns_;

  std::vector<vector_t> buffer_;

  std::unique_ptr<kinematic_model_t> kinematicModel_;
  std::unique_ptr<com_model_t> comModel_;
};

}  // namespace switched_model