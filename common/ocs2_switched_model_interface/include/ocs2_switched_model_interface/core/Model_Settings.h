#pragma once

#include <iostream>
#include <string>
#include <vector>

#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

class Model_Settings {
 public:
  Model_Settings()
      : constrainedIntegration_(true),
        gravitationalAcceleration_(9.81),
        contactForceWeight_(0.1),
        zDirectionPositionWeight_(5.0),
        zDirectionVelocityWeight_(0.5),
        swingLegLiftOff_(0.3),
        liftOffVelocity_(0.0),
        touchDownVelocity_(0.0),
        phaseTransitionStanceTime_(0.4),
        mpcGoalCommandDelay_(0.5),
        targetDisplacementVelocity_(0.5),
        targetRotationVelocity_(0.3),
        useFeetTrajectoryFiltering_(true),
        feetFilterFrequency_(50.0),
        torqueMixingFactor_(0.0),
        gaitOptimization_(false),
        enforceFrictionConeConstraint_(false),
        frictionCoefficient_(1.0),
        enforceTorqueConstraint_(false),
        torqueLimit_(40.0),
        recompileLibraries_(true) {}

  bool constrainedIntegration_;
  double gravitationalAcceleration_;
  double contactForceWeight_;
  double zDirectionPositionWeight_;
  double zDirectionVelocityWeight_;
  double swingLegLiftOff_;
  double liftOffVelocity_;
  double touchDownVelocity_;
  double phaseTransitionStanceTime_;
  double mpcGoalCommandDelay_;
  double targetDisplacementVelocity_;
  double targetRotationVelocity_;
  bool useFeetTrajectoryFiltering_;
  double feetFilterFrequency_;
  double torqueMixingFactor_;
  bool gaitOptimization_;
  bool enforceFrictionConeConstraint_;
  double frictionCoefficient_;
  bool enforceTorqueConstraint_;
  double torqueLimit_;
  bool recompileLibraries_;

  double eps_ = 0.01;
  double eta_ = 10.0;

  void loadSettings(const std::string& filename, bool verbose = true);
};

}  // end of namespace switched_model

