/*
 * ComKinoOperatingPointsBase.h
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#ifndef COMKINOOPERATINGPOINTSBASE_H_
#define COMKINOOPERATINGPOINTSBASE_H_

#include <ocs2_core/initialization/SystemOperatingPoint.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/Model_Settings.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelLogicRulesBase.h"
#include "ocs2_switched_model_interface/misc/WeightCompensationForces.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM=12+JOINT_COORD_SIZE, size_t INPUT_DIM=12+JOINT_COORD_SIZE>
class ComKinoOperatingPointsBase : public
ocs2::SystemOperatingPoint<STATE_DIM, INPUT_DIM>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS
	};

	using Base = ocs2::SystemOperatingPoint<STATE_DIM, INPUT_DIM>;
	using typename Base::scalar_t;
	using typename Base::scalar_array_t;
	using typename Base::state_vector_t;
	using typename Base::state_vector_array_t;
	using typename Base::input_vector_t;
	using typename Base::input_vector_array_t;

	using com_model_t = ComModelBase<JOINT_COORD_SIZE>;
	using kinematic_model_t = KinematicsModelBase<JOINT_COORD_SIZE>;
	using logic_rules_t = SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, double>;
	using contact_flag_t = typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t;
	using base_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t;
	using joint_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t;
	using generalized_coordinate_t = typename SwitchedModel<JOINT_COORD_SIZE>::generalized_coordinate_t;

	using vector3d_t = Eigen::Matrix<scalar_t,3,1>;
	using matrix3d_t =  Eigen::Matrix<scalar_t,3,3>;

	/**
	 * Default constructor
	 */
	ComKinoOperatingPointsBase(
			const kinematic_model_t& kinematicModel,
			const com_model_t& comModel,
			std::shared_ptr<const logic_rules_t> logicRulesPtr,
			const Model_Settings& options = Model_Settings(),
			const generalized_coordinate_t& defaultConfiguration = generalized_coordinate_t::Zero)
	: Base()
	, kinematicModelPtr_(kinematicModel.clone())
	, comModelPtr_(comModel.clone())
	, logicRulesPtr_(std::move(logicRulesPtr))
	, o_gravityVector_(0.0, 0.0, -options.gravitationalAcceleration_)
	, options_(options)
	, weightCompensationForces_(kinematicModel, comModel)
	{
	  if (!logicRulesPtr_) {
	    throw std::runtime_error("[ComKinoOperatingPointsBase] logicRules cannot be a nullptr");
	  }

	  computeOperatingPoints(defaultConfiguration, possibleStanceLegs_, allStateOperatingPoints_, allInputOperatingPoints_);
	}

	/**
	 * Copy constructor
	 */
	ComKinoOperatingPointsBase(const ComKinoOperatingPointsBase& rhs)
	: Base(rhs)
	, kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
	, logicRulesPtr_(rhs.logicRulesPtr_)
	, o_gravityVector_(rhs.o_gravityVector_)
	, options_(rhs.options_)
	, weightCompensationForces_(rhs.weightCompensationForces_)
	, possibleStanceLegs_(rhs.possibleStanceLegs_)
	, allStateOperatingPoints_(rhs.allStateOperatingPoints_)
	, allInputOperatingPoints_(rhs.allInputOperatingPoints_)
	{}

	/**
	 * Default destructor.
	 */
	virtual ~ComKinoOperatingPointsBase() = default;

	/**
	 * Returns pointer to the class.
	 *
	 * @return A raw pointer to the class.
	 */
	virtual ComKinoOperatingPointsBase<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>* clone() const override;

	/**
	 * Gets the Operating points for the system in time interval [startTime, finalTime] where there is
	 * no intermediate switches except possibly the end time.
	 *
	 * @param [in] initialState: Initial state.
	 * @param [in] startTime: Initial time.
	 * @param [in] finalTime: Final time.
	 * @param [out] timeTrajectory: Output time stamp trajectory.
	 * @param [out] stateTrajectory: Output state trajectory.
	 * @param [out] inputTrajectory: Output control input trajectory.
	 * @param [in] concatOutput: Whether to concatenate the output to the input trajectories or
	 * override (default).
	 */
	virtual void getSystemOperatingTrajectories(
			const state_vector_t& initialState,
			const scalar_t& startTime,
			const scalar_t& finalTime,
			scalar_array_t& timeTrajectory,
			state_vector_array_t& stateTrajectory,
			input_vector_array_t& inputTrajectory,
			bool concatOutput = false) override;

	/**
	 *
	 * @param defaultConfiguration
	 * @param possibleStanceLegs
	 * @param stateOperatingPoints
	 * @param inputOperatingPoints
	 */
	void computeOperatingPoints(
			const generalized_coordinate_t& defaultConfiguration,
			std::vector<contact_flag_t>& possibleStanceLegs,
			state_vector_array_t& stateOperatingPoints,
			input_vector_array_t& inputOperatingPoints);

	/**
	 *
	 * @param switchedState
	 * @param stanceLegs
	 * @param inputOperatingPoints
	 */
	void computeInputOperatingPoints(
			const state_vector_t& switchedState,
			const contact_flag_t& stanceLegs,
			input_vector_t& inputOperatingPoints);

private:

	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	vector3d_t o_gravityVector_;
	Model_Settings options_;

	WeightCompensationForces<JOINT_COORD_SIZE, scalar_t> weightCompensationForces_;

	std::vector<contact_flag_t> possibleStanceLegs_;
	state_vector_array_t allStateOperatingPoints_;
	input_vector_array_t allInputOperatingPoints_;

	std::shared_ptr<const logic_rules_t> logicRulesPtr_;

	contact_flag_t stanceLegs_;
};

} // end of namespace switched_model

#include "implementation/ComKinoOperatingPointsBase.h"

#endif /* COMKINOOPERATINGPOINTSBASE_H_ */
