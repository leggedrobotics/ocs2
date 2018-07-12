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
ocs2::SystemOperatingPoint<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE, SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, double>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM_ = STATE_DIM,
		INPUT_DIM_ = INPUT_DIM,
		NUM_CONTACT_POINTS_ = SwitchedModel<JOINT_COORD_SIZE>::NUM_CONTACT_POINTS
	};

	typedef SwitchedModelPlannerLogicRules<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM, double> logic_rules_t;
	typedef ocs2::LogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_t> logic_rules_machine_t;

	typedef ocs2::SystemOperatingPoint<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::contact_flag_t 			contact_flag_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t 		base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t 		joint_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::generalized_coordinate_t 	generalized_coordinate_t;

	typedef typename Base::scalar_t 			scalar_t;
	typedef typename Base::scalar_array_t 		scalar_array_t;
	typedef typename Base::state_vector_t 		state_vector_t;
	typedef typename Base::state_vector_array_t state_vector_array_t;
	typedef typename Base::input_vector_t 		input_vector_t;
	typedef typename Base::input_vector_array_t input_vector_array_t;

	typedef Eigen::Matrix<scalar_t,3,1> vector3d_t;
	typedef Eigen::Matrix<scalar_t,3,3> matrix3d_t;

	/**
	 * Default constructor
	 */
	ComKinoOperatingPointsBase(
			const kinematic_model_t& kinematicModel,
			const com_model_t& comModel,
			const Model_Settings& options = Model_Settings(),
			const generalized_coordinate_t& defaultConfiguration = generalized_coordinate_t::Zero)
	: Base()
	, kinematicModelPtr_(kinematicModel.clone())
	, comModelPtr_(comModel.clone())
	, o_gravityVector_(0.0, 0.0, -options.gravitationalAcceleration_)
	, options_(options)
	, weightCompensationForces_(kinematicModel, comModel)
	{
		computeOperatingPoints(defaultConfiguration, possibleStanceLegs_, allStateOperatingPoints_, allInputOperatingPoints_);
	}

	/**
	 * Copy constructor
	 */
	ComKinoOperatingPointsBase(const ComKinoOperatingPointsBase& rhs)
	: Base(rhs)
	, kinematicModelPtr_(rhs.kinematicModelPtr_->clone())
	, comModelPtr_(rhs.comModelPtr_->clone())
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
	 * Initializes the operating trajectories class.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(
			logic_rules_machine_t& logicRulesMachine,
			const size_t& partitionIndex,
			const char* algorithmName=NULL) override;

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

	logic_rules_t* logicRulesPtr_;

	std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

	contact_flag_t stanceLegs_;

	std::string algorithmName_;
};

} // end of namespace switched_model

#include "implementation/ComKinoOperatingPointsBase.h"

#endif /* COMKINOOPERATINGPOINTSBASE_H_ */
