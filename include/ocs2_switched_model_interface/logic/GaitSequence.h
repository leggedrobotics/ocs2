//
// Created by rgrandia on 27.07.19.
//

#ifndef OCS2_CTRL_GAITSEQUENCE_H
#define OCS2_CTRL_GAITSEQUENCE_H

#include <ocs2_mpc/MpcSynchronizedModule.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

namespace switched_model {

/**
 * Contains per leg the event times
 */
struct GaitSequenceTranspose {
  std::array<std::vector<double>, 4> time;
  std::array<std::vector<bool>, 4> contactFlags;
};

/**
 * Contains the contact flags per event
 */
struct GaitSequence {
  std::vector<double> time;
  std::vector<std::array<bool, 4>> contactFlags;

  GaitSequenceTranspose transpose() const {
    GaitSequenceTranspose gaitSequenceTranspose;

    for (int leg = 0; leg<4; leg++){
      // Start with contact state of current leg
      gaitSequenceTranspose.contactFlags[leg].push_back(contactFlags.front()[leg]);

      // Add event time for a leg only if it switches state
      for (int motionPhase = 0; motionPhase<(contactFlags.size()-1); motionPhase++){
        if (gaitSequenceTranspose.contactFlags[leg].back() != contactFlags[motionPhase+1][leg]) {
          gaitSequenceTranspose.time[leg].push_back(time[motionPhase]);
          gaitSequenceTranspose.contactFlags[leg].push_back(contactFlags[motionPhase+1][leg]);
        }
      }
    }
    return gaitSequenceTranspose;
  };

  void display() {
    std::cout << "[GaitSequence]" << std::endl;
    std::cout << "\teventTimes : ";
    for (auto t : time) {
      std::cout << t << ", ";
    }
    for (int leg = 0; leg < 4; leg++) {
      std::cout << "\nLeg " << leg;
      std::cout << "\n\t contacts : ";
      for (auto contact : contactFlags) {
        std::cout << contact[leg] << ", ";
      }
      std::cout << std::endl;
    }
  }
};

class MpcGaitSequence : public ocs2::MpcSynchronizedModule<double>, public GaitSequence {
 public:
  ~MpcGaitSequence() override = default;
  void update(double initTime, double finalTime, const Eigen::Matrix<double, -1, 1>& currentState, const ocs2::CostDesiredTrajectories<double>& costDesiredTrajectory, const ocs2::HybridLogicRules* hybridLogicRules) override {
    time = hybridLogicRules->eventTimes();

    contactFlags.clear();
    for (auto subsystem : hybridLogicRules->subsystemsSequence()) {
      contactFlags.push_back(modeNumber2StanceLeg(subsystem));
    }

    display();
  }

};




} // namespace switched_model


#endif //OCS2_CTRL_GAITSEQUENCE_H
