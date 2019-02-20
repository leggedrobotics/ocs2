//
// Created by rgrandia on 12.02.19.
//

#include <ocs2_switched_model_interface/foot_planner/cpg/SplineCPG.h>
#include <ocs2_switched_model_interface/foot_planner/FeetZDirectionPlanner.h>
#include <ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h>
#include <ocs2_core/logic/machine/LogicRulesMachine.h>
#include <ocs2_anymal_switched_model/constraint/AnymalComKinoConstraint.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>

int main( int argc, char* argv[] )
{
    typedef anymal::AnymalComKinoConstraint::scalar_t scalar_t;

    typedef anymal::AnymalComKinoConstraint::scalar_array_t scalar_array_t;
    typedef anymal::AnymalComKinoConstraint::input_vector_array_t input_vector_array_t;
    typedef anymal::AnymalComKinoConstraint::input_matrix_array_t input_matrix_array_t;
    typedef anymal::AnymalComKinoConstraint::state_vector_t state_vector_t;
    typedef anymal::AnymalComKinoConstraint::input_vector_t input_vector_t;
    typedef anymal::AnymalComKinoConstraint::input_matrix_t input_matrix_t;
    typedef anymal::AnymalComKinoConstraint::contact_flag_t contact_flag_t;
    typedef switched_model::FeetZDirectionPlanner<anymal::AnymalComKinoConstraint::scalar_t, switched_model::SplineCPG<scalar_t>>	feet_z_planner_t;

    typedef typename feet_z_planner_t::Ptr			            feet_z_planner_ptr_t;
    typedef anymal::AnymalComKinoConstraint::logic_rules_t 	    logic_rules_t;
    typedef typename logic_rules_t::Ptr                         logic_rules_ptr_t;
    typedef logic_rules_t::size_array_t                         size_array_t;
    typedef ocs2::LogicRulesMachine<logic_rules_t>              logic_rules_machine_t;
    typedef typename logic_rules_machine_t::Ptr	                logic_rules_machine_ptr_t;


    // logic rule
    feet_z_planner_ptr_t feetZPlannerPtr( new feet_z_planner_t(0.3,
                                                               1.0 /*swingTimeScale*/,
                                                               0.0,
                                                               0.0)	);
    logic_rules_ptr_t logicRulesPtr_ = logic_rules_ptr_t( new logic_rules_t(feetZPlannerPtr) );
    size_array_t   initSwitchingModes_ = {15, 9, 15};
    scalar_array_t initEventTimes_ = {0.5, 0.75};
    std::vector<switched_model::EndEffectorConstraintBase::ConstPtr> gapIndicatorPtrs_;
    logicRulesPtr_->setMotionConstraints(initSwitchingModes_, initEventTimes_, gapIndicatorPtrs_);
    logic_rules_machine_ptr_t logicRulesMachinePtr_ = logic_rules_machine_ptr_t( new logic_rules_machine_t(*logicRulesPtr_) );

    anymal::AnymalComKinoConstraint comKinoConstraint;
    ocs2::RelaxedBarrierPenalty<24, 24> relaxedBarrierPenalty(0.1, 5.0);

    contact_flag_t stanceLegs = {true, false, true, false};
    state_vector_t x;
    x.setZero();
    input_vector_t u;
    u.setZero();
    u(3*0 + 0) = 20.0;
    u(3*0 + 1) = 10.0;
    u(3*0 + 2) = 70.0;

    u(3*2 + 0) = 20.0;
    u(3*2 + 1) = -10.0;
    u(3*2 + 2) = -70.0;


    scalar_t t = 0.6;

    comKinoConstraint.initializeModel(*logicRulesMachinePtr_, 0, "SLQ");
    comKinoConstraint.setCurrentStateAndControl(t, x, u);
    comKinoConstraint.setStanceLegs(stanceLegs);

    scalar_array_t h;
    input_vector_array_t dhdu;
    input_matrix_array_t ddhdudu;
    size_t nIneq = comKinoConstraint.numInequalityConstraint(t);
    comKinoConstraint.getInequalityConstraint(h);
    comKinoConstraint.getInequalityConstraintDerivativesInput(dhdu);
    comKinoConstraint.getInequalityConstraintSecondDerivativesInput(ddhdudu);

    for (size_t i = 0; i < nIneq; i++){ // Loop over inequality constraints
        std::cout << "h[" << i << "]: " << h[i] << std::endl;
        std::cout << "dhdu[" << i << "]: " << dhdu[i].transpose() << std::endl;
        std::cout << "ddhdudu[" << i << "]: \n" << ddhdudu[i].transpose() << std::endl;
    }

    scalar_t p;
    input_vector_t dpdu;
    input_matrix_t ddpdudu;
    p = relaxedBarrierPenalty.getPenaltyCost(nIneq, h);
    relaxedBarrierPenalty.getPenaltyCostDerivativeInput(nIneq, h, dhdu, dpdu);
    relaxedBarrierPenalty.getPenaltyCostSecondDerivativeInput(nIneq, h, dhdu, ddhdudu, ddpdudu);
    std::cout << "p: " << p << std::endl;
    std::cout << "dpdu: " << dpdu.transpose() << std::endl;
    std::cout << "ddpdudu: \n" << ddpdudu << std::endl;

    return 0;
}