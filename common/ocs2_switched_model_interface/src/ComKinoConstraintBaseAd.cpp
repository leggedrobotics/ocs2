#include "ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h"

// Constraints
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraint.h"
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityConstraintInBase.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"
#include <ocs2_core/automatic_differentiation/implementation/CppAdInterface.h>

//forward declare hack
namespace switched_model {
  template <typename SCALAR_T>
    class KinematicsModelBase;
}
namespace anymal{ namespace tpl {
  // template<typename SCALAR_T>
  // class AnymalKinematics;

  template<typename SCALAR_T>
    class AnymalKinematics : public switched_model::KinematicsModelBase<SCALAR_T>{
      public:
        template<typename scalar_t>
      switched_model::matrix3_s_t<scalar_t> fr_base_X_fr_FOOT( size_t footIndex) const;
    };
} }

namespace switched_model {
  // using KinematicsChild_t = std::unique_ptr<anymal::tpl::AnymalKinematics<ComKinoConstraintBaseAd::ad_scalar_t>>;
  using KinematicsChild_t = anymal::tpl::AnymalKinematics<ocs2::CppAdInterface<double>::ad_scalar_t>;

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
ComKinoConstraintBaseAd* ComKinoConstraintBaseAd::clone() const {
  return new ComKinoConstraintBaseAd(*this);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::initializeConstraintTerms() {
  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footName = feetNames[i];

    // Friction cone constraint
    auto frictionCone = std::unique_ptr<ConstraintTerm_t>(new FrictionConeConstraint(options_.frictionCoefficient_, 25.0, i));

    // EE force
    auto zeroForceConstraint = std::unique_ptr<ConstraintTerm_t>(new ZeroForceConstraint(i));

    // Velocity Constraint
    auto _o_endEffectorVelocityConstraint = std::unique_ptr<ConstraintTerm_t>(new EndEffectorVelocityConstraint(
        i, EndEffectorVelocityConstraintSettings(), *adComModelPtr_, *adKinematicModelPtr_, options_.recompileLibraries_));

    // Inequalities
    inequalityConstraintCollection_.add(std::move(frictionCone), footName + "_FrictionCone");

    // State input equalities
    equalityStateInputConstraintCollection_.add(std::move(zeroForceConstraint), footName + "_ZeroForce");
    equalityStateInputConstraintCollection_.add(std::move(_o_endEffectorVelocityConstraint), footName + "_o_EEVel");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) {
  stateInputConstraintsComputed_ = false;
  inequalityConstraintsComputed_ = false;

  Base::setCurrentStateAndControl(t, x, u);
  numEventTimes_ = logicRulesPtr_->getNumEventTimes();
  auto activeSubsystem = logicRulesPtr_->getEventTimeCount(t);
  logicRulesPtr_->getMotionPhaseLogics(activeSubsystem, stanceLegs_, zDirectionRefsPtr_);

  // const base_coordinate_s_t comPose = getComPose(x);
  const auto o_R_b = rotationMatrixBaseToOrigin(getOrientation(getComPose(x)));

  for (int i = 0; i < NUM_CONTACT_POINTS; i++) {
    auto footName = feetNames[i];

    // Active friction cone constraint for stanceLegs
    inequalityConstraintCollection_.modifyConstraint(footName + "_FrictionCone")->setActivity(stanceLegs_[i]);

    // Zero forces active for swing legs
    equalityStateInputConstraintCollection_.modifyConstraint(footName + "_ZeroForce")->setActivity(!stanceLegs_[i]);

    // Active foot placement for stance legs
    auto _o_EEVelConstraint =
        equalityStateInputConstraintCollection_.template modifyConstraint<EndEffectorVelocityConstraint>(footName + "_o_EEVel");

    EndEffectorVelocityConstraintSettings _o_eeVelConSettings;

    _o_EEVelConstraint->setActivity(true);

    //TODO(oharley): The static_cast a bit of a hack → this should all be subclassed or templated
    // const auto b_R_e = rotationMatrixBaseToOrigin(static_cast<KinematicsChild_t*>(adKinematicModelPtr_.get())->fr_base_X_fr_FOOT<scalar_t>(i));
    if (stanceLegs_[i]) {
      // in stance: y,z velocitys are zero
      _o_eeVelConSettings.b.resize(2);
      _o_eeVelConSettings.b << 0, 0;
      _o_eeVelConSettings.A.resize(2, 3);
      // Lateral Constraint
      const auto b_R_e = static_cast<KinematicsChild_t*>(adKinematicModelPtr_.get())->fr_base_X_fr_FOOT<scalar_t>(i);
      auto o_eeVelLateral = (o_R_b * b_R_e).col(2).template head<2>().eval(); // X,Y components
      _o_eeVelConSettings.A.template block<1,2>(0,0) = o_eeVelLateral;
      _o_eeVelConSettings.A(0,2) = 0;
      // Up(Z) constraint
      _o_eeVelConSettings.A.bottomRows<1>() << 0, 0, 1;

    } else {  // in swing: z-velocity is provided
      _o_eeVelConSettings.b.resize(1);
      _o_eeVelConSettings.A.resize(1, 3);
      _o_eeVelConSettings.b << -zDirectionRefsPtr_[i]->calculateVelocity(Base::t_);
      _o_eeVelConSettings.A << 0, 0, 1;
    }
    _o_EEVelConstraint->configure(_o_eeVelConSettings);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getConstraint1(constraint1_vector_t& e) {
  size_t numConstraints = numStateInputConstraint(Base::t_);
  e.head(numConstraints) = equalityStateInputConstraintCollection_.getConstraints().getValueAsVector(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ComKinoConstraintBaseAd::numStateInputConstraint(const scalar_t& time) {
  return equalityStateInputConstraintCollection_.getConstraints().getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getConstraint2(constraint2_vector_t& h) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ComKinoConstraintBaseAd::numStateOnlyConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getInequalityConstraint(scalar_array_t& h) {
  h = inequalityConstraintCollection_.getConstraints().getValue(Base::t_, Base::x_, Base::u_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ComKinoConstraintBaseAd::numInequalityConstraint(const scalar_t& time) {
  return inequalityConstraintCollection_.getConstraints().getNumConstraints(time);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getFinalConstraint2(constraint2_vector_t& h_f) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
size_t ComKinoConstraintBaseAd::numStateOnlyFinalConstraint(const scalar_t& time) {
  return 0;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getConstraint1DerivativesState(constraint1_state_matrix_t& C) {
  if (!stateInputConstraintsComputed_) {
    linearStateInputConstraintApproximation_ =
        equalityStateInputConstraintCollection_.getConstraints().getLinearApproximationAsMatrices(Base::t_, Base::x_, Base::u_);
    stateInputConstraintsComputed_ = true;
  }
  size_t numConstraints = numStateInputConstraint(Base::t_);
  C.topRows(numConstraints) = linearStateInputConstraintApproximation_.derivativeState.topRows(numConstraints);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getConstraint1DerivativesControl(constraint1_input_matrix_t& D) {
  if (!stateInputConstraintsComputed_) {
    linearStateInputConstraintApproximation_ =
        equalityStateInputConstraintCollection_.getConstraints().getLinearApproximationAsMatrices(Base::t_, Base::x_, Base::u_);
    stateInputConstraintsComputed_ = true;
  }
  size_t numConstraints = numStateInputConstraint(Base::t_);
  D.topRows(numConstraints) = linearStateInputConstraintApproximation_.derivativeInput.topRows(numConstraints);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getConstraint1DerivativesEventTimes(constraint1_vector_array_t& g1DevArray) {
  // set all to zero
  g1DevArray.resize(numEventTimes_);
  for (constraint1_vector_t& g1Dev : g1DevArray) {
    g1Dev.setZero();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ =
        inequalityConstraintCollection_.getConstraints().getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  dhdx = std::move(quadraticInequalityConstraintApproximation_.derivativeState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getInequalityConstraintDerivativesInput(switched_model::ComKinoConstraintBaseAd::input_vector_array_t& dhdu) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ =
        inequalityConstraintCollection_.getConstraints().getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  dhdu = std::move(quadraticInequalityConstraintApproximation_.derivativeInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getInequalityConstraintSecondDerivativesState(
    switched_model::ComKinoConstraintBaseAd::state_matrix_array_t& ddhdxdx) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ =
        inequalityConstraintCollection_.getConstraints().getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  ddhdxdx = std::move(quadraticInequalityConstraintApproximation_.secondDerivativesState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getInequalityConstraintSecondDerivativesInput(
    switched_model::ComKinoConstraintBaseAd::input_matrix_array_t& ddhdudu) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ =
        inequalityConstraintCollection_.getConstraints().getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  ddhdudu = std::move(quadraticInequalityConstraintApproximation_.secondDerivativesInput);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getInequalityConstraintDerivativesInputState(
    switched_model::ComKinoConstraintBaseAd::input_state_matrix_array_t& ddhdudx) {
  if (!inequalityConstraintsComputed_) {
    quadraticInequalityConstraintApproximation_ =
        inequalityConstraintCollection_.getConstraints().getQuadraticApproximation(Base::t_, Base::x_, Base::u_);
    inequalityConstraintsComputed_ = true;
  }
  ddhdudx = std::move(quadraticInequalityConstraintApproximation_.derivativesInputState);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F) {}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::setStanceLegs(const contact_flag_t& stanceLegs) {
  stanceLegs_ = stanceLegs;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void ComKinoConstraintBaseAd::getStanceLegs(contact_flag_t& stanceLegs) {
  stanceLegs = stanceLegs_;
}

}  // end of namespace switched_model
