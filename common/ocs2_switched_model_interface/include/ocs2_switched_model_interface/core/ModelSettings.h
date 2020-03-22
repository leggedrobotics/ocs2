#pragma once

#include <string>

namespace switched_model {

struct ModelSettings {
  bool constrainedIntegration_ = true;
  double gravitationalAcceleration_ = 9.81;
  double contactForceWeight_ = 0.1;
  double zDirectionPositionWeight_ = 5.0;
  double zDirectionVelocityWeight_ = 0.5;
  double swingLegLiftOff_ = 0.3;
  double liftOffVelocity_ = 0.0;
  double touchDownVelocity_ = 0.0;
  double swingTimeScale_ = 1.0;
  double phaseTransitionStanceTime_ = 0.4;
  double mpcGoalCommandDelay_ = 0.5;
  double targetDisplacementVelocity_ = 0.5;
  double targetRotationVelocity_ = 0.3;
  bool useFeetTrajectoryFiltering_ = true;
  double feetFilterFrequency_ = 50.0;
  bool gaitOptimization_ = false;
  bool enforceFrictionConeConstraint_ = false;
  double frictionCoefficient_ = 1.0;
  bool enforceTorqueConstraint_ = false;
  double torqueLimit_ = 40.0;
  bool recompileLibraries_ = true;
  double eps_ = 0.01;
  double eta_ = 10.0;
};

ModelSettings loadModelSettings(const std::string& filename, bool verbose = true);

}  // end of namespace switched_model
