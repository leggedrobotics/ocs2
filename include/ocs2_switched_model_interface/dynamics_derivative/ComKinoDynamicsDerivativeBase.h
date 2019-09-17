/*
 * ComKinoDynamicsDerivativeBase.h
 *
 *  Created on: Nov 12, 2017
 *      Author: farbod
 */

#ifndef COMKINODYNAMICSDERIVATIVEBASE_H_
#define COMKINODYNAMICSDERIVATIVEBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_core/dynamics/DerivativesBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/Model_Settings.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/dynamics_derivative/ComDynamicsDerivativeBase.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"
#include "ocs2_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE,
		size_t STATE_DIM=12+JOINT_COORD_SIZE,
		size_t INPUT_DIM=12+JOINT_COORD_SIZE>
class ComKinoDynamicsDerivativeBase : public
ocs2::DerivativesBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM_ = STATE_DIM,
		INPUT_DIM_ = INPUT_DIM,
		NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS
	};

	using Base = ocs2::DerivativesBase<STATE_DIM, INPUT_DIM>;
	using typename Base::scalar_t;
	using typename Base::state_matrix_t;
	using typename Base::state_vector_t;
	using typename Base::input_vector_t;
	using typename Base::state_input_matrix_t;
	using typename Base::constraint1_state_matrix_t;
	using typename Base::constraint1_input_matrix_t;
	using typename Base::constraint2_state_matrix_t;

	using com_model_t = ComModelBase<JOINT_COORD_SIZE>;
	using kinematic_model_t = KinematicsModelBase<JOINT_COORD_SIZE>;
	using contact_flag_t = typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t;
	using base_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t;
	using joint_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t;
	using base_jacobian_matrix_t = Eigen::Matrix<double, 6, JOINT_COORD_SIZE>;


	ComKinoDynamicsDerivativeBase(const kinematic_model_t& kinematicModel, const com_model_t& comModel,
			const Model_Settings& options = Model_Settings())

	: Base(),
	  kinematicModelPtr_(kinematicModel.clone()),
	  comModelPtr_(comModel.clone()),
	  o_gravityVector_(0.0, 0.0, -options.gravitationalAcceleration_),
	  options_(options),
	  comDynamicsDerivative_(kinematicModel, comModel, options.gravitationalAcceleration_, options.constrainedIntegration_)
	{}

	/**
	 * copy constructor of ComKinoDynamicsDerivativeBase
	 */
	ComKinoDynamicsDerivativeBase(const ComKinoDynamicsDerivativeBase& rhs)

	: Base(rhs)
	, kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
	, o_gravityVector_(rhs.o_gravityVector_)
	, options_(rhs.options_)
	, comDynamicsDerivative_(rhs.comDynamicsDerivative_)
	{}

	~ComKinoDynamicsDerivativeBase() override {}

	/**
	 * clone ComKinoDynamicsDerivativeBase class.
	 */
	ComKinoDynamicsDerivativeBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>* clone() const override;

	/**
	 * Set the current state and contact force input
	 *
	 * @param t: current time
	 * @param x: current switched state vector (centroidal dynamics plus joints' angles)
	 * @param u: current switched input vector (contact forces plus joints' velocities)
	 */
	void setCurrentStateAndControl(const scalar_t& t, const state_vector_t& x, const input_vector_t& u) override;

	/**
	 * calculate and retrieve the A matrix (i.e. the state derivative of the dynamics w.r.t. state vector).
	 *
	 * @param A: a nx-by-nx matrix
	 */
	void getFlowMapDerivativeState(state_matrix_t& A) final override;

	/**
	 * calculate and retrieve the B matrix (i.e. the state derivative of the dynamics w.r.t. input vector).
	 *
	 * @param B: a ny-by-nu matrix
	 */
	void getFlowMapDerivativeInput(state_input_matrix_t& B) final override;


private:

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d   o_gravityVector_;
	Model_Settings options_;

	ComDynamicsDerivativeBase<JOINT_COORD_SIZE> comDynamicsDerivative_;
};

} // end of namespace switched_model

#include "implementation/ComKinoDynamicsDerivativeBase.h"

#endif /* COMKINODYNAMICSDERIVATIVEBASE_H_ */
