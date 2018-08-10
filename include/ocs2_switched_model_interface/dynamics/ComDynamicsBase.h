/*
 * ComDynamicsBase.h
 *
 *  Created on: Nov 7, 2017
 *      Author: farbod
 */

#ifndef COMDYNAMICSBASE_H_
#define COMDYNAMICSBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <Eigen/Dense>

#include <ocs2_core/dynamics/ControlledSystemBase.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE>
class ComDynamicsBase : public ocs2::ControlledSystemBase<12, 12, ocs2::NullLogicRules<12,12>>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	enum
	{
		STATE_DIM = 12,
		INPUT_DIM = 12
	};

	typedef ocs2::NullLogicRules<12,12> logic_rules_t;
	typedef ocs2::LogicRulesMachine<STATE_DIM, INPUT_DIM, logic_rules_t> logic_rules_machine_t;

	typedef ocs2::ControlledSystemBase<STATE_DIM, INPUT_DIM, logic_rules_t> Base;

	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;


	ComDynamicsBase(const kinematic_model_t& kinematicModel, const com_model_t& comModel,
			const double& gravitationalAcceleration=9.81, const bool& constrainedIntegration=true)

	: Base(),
	  kinematicModelPtr_(kinematicModel.clone()),
	  comModelPtr_(comModel.clone()),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  constrainedIntegration_(constrainedIntegration)
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value.");
	}

	virtual ~ComDynamicsBase() = default;

	/**
	 * copy constructor
	 */
	ComDynamicsBase(const ComDynamicsBase& rhs)

	: Base(rhs),
	  kinematicModelPtr_(rhs.kinematicModelPtr_->clone()),
	  comModelPtr_(rhs.comModelPtr_->clone()),
	  o_gravityVector_(rhs.o_gravityVector_),
	  constrainedIntegration_(rhs.constrainedIntegration_)
	{}

	/**
	 * clone this class.
	 */
	virtual ComDynamicsBase<JOINT_COORD_SIZE>* clone() const  override;

	/**
	 * Initializes the system dynamics. This method should always be called at the very first call of the model.
	 *
	 * @param [in] logicRulesMachine: A class which contains and parse the logic rules e.g
	 * method findActiveSubsystemHandle returns a Lambda expression which can be used to
	 * find the ID of the current active subsystem.
	 * @param [in] partitionIndex: index of the time partition.
	 * @param [in] algorithmName: The algorithm that class this class (default not defined).
	 */
	virtual void initializeModel(logic_rules_machine_t& logicRulesMachine,
			const size_t& partitionIndex, const char* algorithmName=NULL) override;

	/**
	 * Calculates the CoM state time evolution based on the current CoM state (x), contact forces input
	 * at all feet (u), the stance leg configuration (stanceLegs_), the current joints' angel (qJoints_),
	 * and the current joints' velocities (dqJoints_).
	 *
	 * The CoM state (x) consists of:
	 * 		+ Base orientation w.r.t Origin frame (3-states)
	 * 		+ CoM position w.r.t Origin frame (3-states)
	 * 		+ CoM local angular and linear velocities in CoM frame (inertia frame coincide at CoM and is
	 * 		  parallel to Base frame) (6-states)
	 *
	 * The control input (u) consists of:
	 * 		+ feet's contact forces in the CoM frame with spherical coordinate (12-inputs = 3-XYZ-forces * 4-feet)
	 *
	 * The CoM state time derivatives (dxdt) consists of:
	 * 		+ Base angular velocity w.r.t Origin frame (3-states)
	 * 		+ CoM linear velocity w.r.t Origin frame (3-states)
	 * 		+ CoM angular and linear accelerations w.r.t CoM frame (6-states)
	 *
	 */
	virtual void computeFlowMap(const scalar_t& t,
			const state_vector_t& x,
			const input_vector_t& u,
			state_vector_t& dxdt) final;

	/**
	 * set joints' angel and angular velocity, and stance leg configuration. This data is
	 * required in computeFlowMap() method.
	 */
	void setData(const std::array<bool,4>& stanceLegs,
			const joint_coordinate_t& qJoints,
			const joint_coordinate_t& dqJoints);

	/**
	 * @brief Computes the matrix which transforms derivatives of angular velocities in the body frame to euler angles derivatives
	 * WARNING: matrix is singular when rotation around y axis is +/- 90 degrees
	 * @param[in] eulerAngles: euler angles in xyz convention
	 * @return M: matrix that does the transformation
	 */
	static Eigen::Matrix3d AngularVelocitiesToEulerAngleDerivativesMatrix (const Eigen::Vector3d& eulerAngles);

	/**
	 * user interface for retrieving stance legs. Note this value is updated after each call of computeFlowMap() method.
	 */
	void getStanceLegs(std::array<bool,4>& stanceLegs) const;

	/**
	 * user interface for retrieving feet's positions. Note this value is updated after each call of computeFlowMap() method.
	 */
	void getFeetPositions(std::array<Eigen::Vector3d,4>& b_base2StanceFeet) const;

	/**
	 * user interface for retrieving base pose in origin frame. Note this value is updated after each call of computeFlowMap() method.
	 */
	void getBasePose(base_coordinate_t& o_basePose) const;

	/**
	 * user interface for CoM dynamics elements: The inertial matrix. Note this value is updated after each call of computeFlowMap() method.
	 */
	Eigen::Matrix<double, 6, 6>  getM() const;

	/**
	 * user interface for CoM dynamics elements: The inverse of the inertial matrix. Note this value is updated after each call of computeFlowMap() method.
	 */
	Eigen::Matrix<double, 6, 6>  getMInverse() const;

	/**
	 * user interface for CoM dynamics elements: The coriolis and centrifugal forces vector. Note this value is updated after each call of computeFlowMap() method.
	 */
	Eigen::Matrix<double, 6, 1>  getC() const;

	/**
	 * user interface for CoM dynamics elements: The gravity force vector. Note this value is updated after each call of computeFlowMap() method.
	 */
	Eigen::Matrix<double, 6, 1>  getG() const;


private:
	typename kinematic_model_t::Ptr kinematicModelPtr_;
	typename com_model_t::Ptr comModelPtr_;
	Eigen::Vector3d o_gravityVector_;

	bool constrainedIntegration_;

	std::array<bool,4> stanceLegs_;
	base_coordinate_t  q_base_;
	joint_coordinate_t qJoints_;
	joint_coordinate_t dqJoints_;

	Eigen::Vector3d com_base2CoM_;
	Eigen::Matrix<double,3,JOINT_COORD_SIZE> b_comJacobainOmega_;
	std::array<Eigen::Vector3d,4> com_base2StanceFeet_;

	// Inertia matrix and its derivative
	Eigen::Matrix<double, 6, 6> M_;
	Eigen::Matrix<double, 6, 6> MInverse_;
	Eigen::Matrix<double, 6, 6> dMdt_;
	// Coriolis and centrifugal forces
	Eigen::Matrix<double, 6, 1> C_;
	// gravity effect on CoM in CoM coordinate
	Eigen::Matrix<double, 6, 1> MInverseG_;


	std::string algorithmName_;
};

} //end of namespace switched_model

#include "implementation/ComDynamicsBase.h"

#endif /* COMDYNAMICSBASE_H_ */
