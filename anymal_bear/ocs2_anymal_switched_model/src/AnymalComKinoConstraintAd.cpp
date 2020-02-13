#include "ocs2_anymal_switched_model/constraint/AnymalComKinoConstraintAd.h"

// Constraints
#include <ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h>
#include <ocs2_switched_model_interface/constraint/FrictionConeConstraint.h>
#include <ocs2_switched_model_interface/constraint/ZeroForceConstraint.h>

namespace switched_model {

  AnymalComKinoConstraintAd::AnymalComKinoConstraintAd(const anymalKinematics_t& anymalKinematicModel, const ad_kinematic_model_t& adKinematicModel, const ad_com_model_t& adComModel,
      std::shared_ptr<const logic_rules_t> logicRulesPtr, const ModelSettings& options
      )  : Base(adKinematicModel, adComModel, logicRulesPtr, options), anymalKinematicModel_(anymalKinematicModel)
  {
    initializeConstraintTerms();
  }

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
AnymalComKinoConstraintAd* AnymalComKinoConstraintAd::clone() const {
  return new AnymalComKinoConstraintAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void AnymalComKinoConstraintAd::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
  //TODO(oharley): remove fix the derived method
  stateInputConstraintsComputed_ = false;
  inequalityConstraintsComputed_ = false;

  Base::setCurrentStateAndControl(t, x, u);

  const auto o_R_b = rotationMatrixBaseToOrigin<scalar_t>(getOrientation(getComPose(x)));
  for (auto footIdx = 0; footIdx < NUM_CONTACT_POINTS; ++footIdx) {
    auto footName = switched_model::FeetNames[footIdx];

    // Active friction cone constraint for stanceLegs
    Base::inequalityConstraintCollection_.modifyConstraint(footName + "_FrictionCone")->setActivity(footIdx);
    // Zero forces active for swing legs
    Base::equalityStateInputConstraintCollection_.modifyConstraint(footName + "_ZeroForce")->setActivity(!stanceLegs_[footIdx]);

    // Active foot placement for stance legs
    auto _o_EEVel =
      Base::equalityStateInputConstraintCollection_.template modifyConstraint<EndEffectorVelocityConstraint>(footName + "_o_EEVel");

    EndEffectorVelocityConstraintSettings _o_eeVelSettings;

    if (stanceLegs_[footIdx]) {  // in stance: y,z velocitys are zero
      _o_eeVelSettings.b.resize(2);
      _o_eeVelSettings.A.resize(2, 3);

      _o_eeVelSettings.b << 0, 0;

      const auto b_R_e = anymalKinematicModel_.footOrientationRelativeToBase(footIdx, getJointPositions(x));
      _o_eeVelSettings.A.template block<1,2>(0,0) = (o_R_b * b_R_e).col(2).template head<2>().eval();
      // _o_eeVelSettings.A.topRows<1>() = o_R_e.col(2).eval();
      _o_eeVelSettings.A(0,2) = 0;
      _o_eeVelSettings.A.bottomRows<1>() << 0, 0, 1; // Up(Z) constraint
    } else {
      // in swing: z-velocity is provided
      // TODO(oharley) This needs more advanced switching behaviour to differentiate between 'dumb'
      // gaits and generated trajectories
      _o_eeVelSettings.b.resize(1);
      _o_eeVelSettings.A.resize(1, 3);

      _o_eeVelSettings.b << -zDirectionRefsPtr_[footIdx]->calculateVelocity(Base::t_);
      _o_eeVelSettings.A << 0, 0, 1;

    }
      _o_EEVel->configure(_o_eeVelSettings);
      _o_EEVel->setActivity(true);
  }
}


}  // end of namespace switched_model
