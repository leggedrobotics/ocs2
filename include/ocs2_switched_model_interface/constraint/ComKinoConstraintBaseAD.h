/*
 * ComKinoConstraintBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: Farbod
 */

#ifndef COMKINOCONSTRAINTBASE_AD_H_
#define COMKINOCONSTRAINTBASE_AD_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <array>
#include <iostream>
#include <memory>
#include <string>

#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/ConstraintCollection.h>

#include "ocs2_switched_model_interface/constraint/EndEffectorPositionConstraint.h"
#include "ocs2_switched_model_interface/constraint/EndEffectorVelocityContraint.h"
#include "ocs2_switched_model_interface/constraint/FrictionConeConstraint.h"
#include "ocs2_switched_model_interface/constraint/ZeroForceConstraint.h"
#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/Model_Settings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/ground/TerrainModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"
#include "ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class ComKinoConstraintBaseAD : public ocs2::ConstraintBase<12 + JOINT_COORD_SIZE, 12 + JOINT_COORD_SIZE> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  enum { STATE_DIM_ = STATE_DIM, INPUT_DIM_ = INPUT_DIM, NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS };

  typedef SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double> logic_rules_t;
  typedef typename logic_rules_t::foot_cpg_t foot_cpg_t;

  using ad_base_t = CppAD::cg::CG<double>;
  using ad_scalar_t = CppAD::AD<ad_base_t>;
  using com_model_t = ComModelBase<JOINT_COORD_SIZE>;
  using ad_com_model_t = ComModelBase<JOINT_COORD_SIZE, ad_scalar_t>;
  using kinematic_model_t = KinematicsModelBase<JOINT_COORD_SIZE>;
  using ad_kinematic_model_t = KinematicsModelBase<JOINT_COORD_SIZE, ad_scalar_t>;

  using Base = ocs2::ConstraintBase<STATE_DIM, INPUT_DIM>;
  using typename Base::constraint1_input_matrix_t;
  using typename Base::constraint1_state_matrix_t;
  using typename Base::constraint1_vector_array_t;
  using typename Base::constraint1_vector_t;
  using typename Base::constraint2_state_matrix_t;
  using typename Base::constraint2_vector_array_t;
  using typename Base::constraint2_vector_t;
  using typename Base::input_matrix_array_t;
  using typename Base::input_matrix_t;
  using typename Base::input_state_matrix_array_t;
  using typename Base::input_state_matrix_t;
  using typename Base::input_vector_array_t;
  using typename Base::input_vector_t;
  using typename Base::scalar_array_t;
  using typename Base::scalar_t;
  using typename Base::state_input_matrix_t;
  using typename Base::state_matrix_array_t;
  using typename Base::state_matrix_t;
  using typename Base::state_vector_array_t;
  using typename Base::state_vector_t;

  using contact_flag_t = typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t;
  using base_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t;
  using joint_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t;

  using ConstraintCollection_t = ocs2::ConstraintCollection<STATE_DIM, INPUT_DIM>;
  using LinearConstraintApproximationAsMatrices_t = ocs2::LinearConstraintApproximationAsMatrices<STATE_DIM, INPUT_DIM>;
  using QuadraticConstraintApproximation_t = ocs2::QuadraticConstraintApproximation<STATE_DIM, INPUT_DIM>;
  using ConstraintTerm_t = ocs2::ConstraintTerm<STATE_DIM, INPUT_DIM>;
  using FrictionConeConstraint_t = FrictionConeConstraint<STATE_DIM, INPUT_DIM>;
  using EndEffectorVelocityConstraint_t = EndEffectorVelocityConstraint<STATE_DIM, INPUT_DIM>;
  using EndEffectorPositionConstraint_t = EndEffectorPositionConstraint<STATE_DIM, INPUT_DIM>;
  using ZeroForceConstraint_t = ZeroForceConstraint<STATE_DIM, INPUT_DIM>;

  // Enumeration and naming
  enum class FeetEnum { LF, RF, LH, RH };
  const std::array<std::string, 4> feetNames{"LF", "RF", "LH", "RH"};

  ComKinoConstraintBaseAD(const kinematic_model_t& kinematicModel, const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel,
                        const ad_com_model_t& adComModel, std::shared_ptr<const logic_rules_t> logicRulesPtr, std::shared_ptr<const TerrainModel> terrainModel = nullptr,
                        const Model_Settings& options = Model_Settings())
      : Base(), adKinematicModelPtr_(adKinematicModel.clone()), adComModelPtr_(adComModel.clone()), logicRulesPtr_(std::move(logicRulesPtr)), terrainModel_(std::move(terrainModel)), options_(options) {
    if (!logicRulesPtr_) {
      throw std::runtime_error("[ComKinoConstraintBaseAD] logicRules cannot be a nullptr");
    }
    InitializeConstraintTerms();
  }

  ComKinoConstraintBaseAD(const ComKinoConstraintBaseAD& rhs)
      : Base(rhs),
        adKinematicModelPtr_(rhs.adKinematicModelPtr_->clone()),
        adComModelPtr_(rhs.adComModelPtr_->clone()),
        logicRulesPtr_(rhs.logicRulesPtr_),
        options_(rhs.options_),
        inequalityConstraintCollection_(rhs.inequalityConstraintCollection_),
        equalityStateInputConstraintCollection_(rhs.equalityStateInputConstraintCollection_),
        eePosConSettings_(rhs.eePosConSettings_),
        eeVelConSettings_(rhs.eeVelConSettings_),
        terrainModel_(rhs.terrainModel_){}

  /**
   * Initialize Constraint Terms
   */
  void InitializeConstraintTerms() {

    for (int i = 0; i < NUM_CONTACT_POINTS_; i++) {
      auto footName = feetNames[i];

      // Friction cone constraint
      auto frictionCone = std::unique_ptr<ConstraintTerm_t>(new FrictionConeConstraint_t(options_.frictionCoefficient_, 25.0, i));

      // EE position
      auto endEffectorConstraint = std::unique_ptr<ConstraintTerm_t>(
          new EndEffectorPositionConstraint_t(i, eePosConSettings_[i], *adComModelPtr_.get(), *adKinematicModelPtr_.get(), options_.generateModels_));

      // EE force
      auto zeroForceConstraint = std::unique_ptr<ConstraintTerm_t>(new ZeroForceConstraint_t(i));

      // Velocity Constraint
      auto endEffectorVelocityConstraint = std::unique_ptr<ConstraintTerm_t>(
          new EndEffectorVelocityConstraint_t(i, eeVelConSettings_[i], *adComModelPtr_.get(), *adKinematicModelPtr_.get(), options_.generateModels_));

      // Inequalities
      inequalityConstraintCollection_.add(std::move(frictionCone), footName + "_FrictionCone");
      inequalityConstraintCollection_.add(std::move(endEffectorConstraint), footName + "_EEPos");

      // State input equalities
      equalityStateInputConstraintCollection_.add(std::move(zeroForceConstraint), footName + "_ZeroForce");
      equalityStateInputConstraintCollection_.add(std::move(endEffectorVelocityConstraint), footName + "_EEVel");
    }
  }

  ~ComKinoConstraintBaseAD() override = default;

  ComKinoConstraintBaseAD<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>* clone() const override;

  void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

  size_t numStateInputConstraint(const scalar_t& time) override;
  void getConstraint1(constraint1_vector_t& e) override;

  size_t numStateOnlyConstraint(const scalar_t& time) override;
  void getConstraint2(constraint2_vector_t& h) override;
  void getConstraint2DerivativesState(constraint2_state_matrix_t& F) override;

  size_t numStateOnlyFinalConstraint(const scalar_t& time) override;
  void getFinalConstraint2(constraint2_vector_t& h_f) override;
  void getFinalConstraint2DerivativesState(constraint2_state_matrix_t& F_final) override;

  void getConstraint1DerivativesState(constraint1_state_matrix_t& C) override;
  void getConstraint1DerivativesControl(constraint1_input_matrix_t& D) override;
  void getConstraint1DerivativesEventTimes(constraint1_vector_array_t& g1DevArray) override;

  size_t numInequalityConstraint(const scalar_t& time) override;
  void getInequalityConstraint(scalar_array_t& h) override;
  void getInequalityConstraintDerivativesState(state_vector_array_t& dhdx) override;
  void getInequalityConstraintDerivativesInput(input_vector_array_t& dhdu) override;
  void getInequalityConstraintSecondDerivativesState(state_matrix_array_t& ddhdxdx) override;
  void getInequalityConstraintSecondDerivativesInput(input_matrix_array_t& ddhdudu) override;
  void getInequalityConstraintDerivativesInputState(input_state_matrix_array_t& ddhdudx) override;

  /**
   * set the stance legs
   */
  void setStanceLegs(const contact_flag_t& stanceLegs);

  /**
   * get the model's stance leg
   */
  void getStanceLegs(contact_flag_t& stanceLegs);

 private:
  ConstraintCollection_t inequalityConstraintCollection_;
  ConstraintCollection_t equalityStateInputConstraintCollection_;

  std::array<EndEffectorVelocityConstraintSettings, 4> eeVelConSettings_;
  std::array<EndEffectorPositionConstraintSettings, 4> eePosConSettings_;
  std::shared_ptr<const TerrainModel> terrainModel_;
  ConvexPlanarPolytope3dArray polytopes;

  LinearConstraintApproximationAsMatrices_t linearStateInputConstraintApproximation_;
  QuadraticConstraintApproximation_t quadraticInequalityConstraintApproximation_;

  typename ad_kinematic_model_t::Ptr adKinematicModelPtr_;
  typename ad_com_model_t::Ptr adComModelPtr_;
  Model_Settings options_;

  std::shared_ptr<const logic_rules_t> logicRulesPtr_;
  contact_flag_t stanceLegs_;
  size_t numEventTimes_;

  std::array<const foot_cpg_t*, NUM_CONTACT_POINTS_> zDirectionRefsPtr_;

};

}  // end of namespace switched_model

#include "implementation/ComKinoConstraintBaseAD.h"

#endif /* COMKINOCONSTRAINTBASE_AD_H_ */
