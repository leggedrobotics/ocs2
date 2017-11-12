/*
 * ComKinoDynamicsBase.h
 *
 *  Created on: Nov 10, 2017
 *      Author: farbod
 */

#ifndef COMKINODYNAMICSBASE_H_
#define COMKINODYNAMICSBASE_H_

#include <array>
#include <memory>
#include <iostream>
#include <string>
#include <Eigen/Dense>

#include <ocs2_core/dynamics/ControlledSystemBase.h>

#include "SwitchedModel.h"
#include "KinematicsModelBase.h"
#include "ComModelBase.h"
#include "ComDynamicsBase.h"
#include "misc/FeetZDirectionPlanner.h"
#include "misc/EndEffectorConstraintBase.h"

template <size_t JOINT_COORD_SIZE>
class ComKinoDynamicsBase : public ocs2::ControlledSystemBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef ocs2::ControlledSystemBase<12+JOINT_COORD_SIZE, 12+JOINT_COORD_SIZE> Base;
	typedef ComModelBase<JOINT_COORD_SIZE> com_model_t;
	typedef KinematicsModelBase<JOINT_COORD_SIZE> kinematic_model_t;

	typedef typename SwitchedModel<JOINT_COORD_SIZE>::base_coordinate_t  base_coordinate_t;
	typedef typename SwitchedModel<JOINT_COORD_SIZE>::joint_coordinate_t joint_coordinate_t;

	ComKinoDynamicsBase(const kinematic_model_t& kinematicModel, const com_model_t& comModel,
			const std::array<bool,4>& stanceLegs, const double& gravitationalAcceleration=9.81, const Options& options = Options(),
			const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner=NULL,
			const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints = std::vector<EndEffectorConstraintBase::Ptr>())

	: kinematicModel_(kinematicModel),
	  comModel_(comModel),
	  o_gravityVector_(0.0, 0.0, -gravitationalAcceleration),
	  comDynamics_(kinematicModel, comModel, gravitationalAcceleration, options.constrainedIntegration_),
	  stanceLegs_(stanceLegs),
	  options_(options),
	  feetZDirectionPlanner_(feetZDirectionPlanner),
	  endEffectorStateConstraints_(endEffectorStateConstraints)
	{
		if (gravitationalAcceleration<0)  throw std::runtime_error("Gravitational acceleration should be a positive value (e.g. +9.81).");
	}

	/**
	 * copy construntor
	 */
	ComKinoDynamicsBase(const ComKinoDynamicsBase& rhs)

	: kinematicModel_(rhs.kinematicModel_),
	  comModel_(rhs.comModel_),
	  o_gravityVector_(rhs.o_gravityVector_),
	  comDynamics_(rhs.comDynamics_),
	  stanceLegs_(rhs.stanceLegs_),
	  options_(rhs.options_),
	  feetZDirectionPlanner_(rhs.feetZDirectionPlanner_->clone()),
	  endEffectorStateConstraints_(rhs.endEffectorStateConstraints_.size())
	{
		for (size_t i=0; i<rhs.endEffectorStateConstraints_.size(); i++)
			endEffectorStateConstraints_[i] = rhs.endEffectorStateConstraints_[i]->clone();
	}

	virtual ~ComKinoDynamicsBase() {}

	/**
	 * clone ComKinoDynamicsBase class.
	 */
	virtual std::shared_ptr<Base> clone() const  override;

	/**
	 * Initializes the model: This method should always be called at the very first call of the model.
	 */
	virtual void initializeModel(const std::vector<size_t>& systemStockIndexes, const std::vector<scalar_t>& switchingTimes,
			const state_vector_t& initState, const size_t& activeSubsystemIndex=0, const char* algorithmName=NULL) override;

	/**
	 * Calculates the extended state time evolution based on the current extended state and control input.
	 *
	 * The extended state (x) consists of:
	 * 		+ CoM position and orientation (6-states)
	 * 		+ CoM linear and angular velocities (6-states)
	 * 		+ feet's XY position in Origin Frame (8-states)
	 * 		+ hyQ's joints (12-joints)
	 *
	 * The control input (u) consists of:
	 * 		+ feet's contact forces (12-inputs = 3-XYZ-forces * 4-feet)
	 * 		+ feet XY linear momentum in the Frame which coincides with Base and is parallel to Origin (8-inputs = 2-XY * 4-feet)
	 *
	 * The extended state time derivatives (dxdt) consists of:
	 * 		+ CoM linear and angular velocities (6-states)
	 * 		+ CoM linear and angular accelerations (6-states)
	 * 		+ feet's XY velocities (8-states)
	 * 		+ hyQ's joints' angel time derivatives (12-state)
	 *
	 */
	virtual void computeDerivative(const scalar_t& t,
			const state_vector_t& x,
			const control_vector_t& u,
			state_vector_t& dxdt) override;

	/**
	 * Equality constraint type-1 consists of states and inputs.
	 */
	virtual void computeConstriant1(const scalar_t& t,
			const state_vector_t& x,
			const control_vector_t& u,
			size_t& numConstraint1,
			constraint1_vector_t& g1) override;

	/**
	 * Equality and inequality constraint type-2 consists of states
	 */
	virtual void computeConstriant2(const scalar_t& t,
			const state_vector_t& x,
			size_t& numConstraint2,
			constraint2_vector_t& g2) override;

	/**
	 * Equality and inequality final constraint type-2 consists of states
	 */
	virtual void computeFinalConstriant2(const scalar_t& t,
			const state_vector_t& x,
			size_t& numFinalConstraint2,
			constraint2_vector_t& g2Final) override;

	/**
	 * set the stance legs
	 */
	void setStanceLegs (const std::array<bool,4>& stanceLegs);

	/**
	 * get the stance legs
	 */
	void getStanceLegs (std::array<bool,4>& stanceLegs) const;

	/**
	 * set the swing legs z direction CPGs planner
	 */
	void setSwingLegsCpgsPlanner (const FeetZDirectionPlannerBase::Ptr& feetZDirectionPlanner);

	/**
	 * set the swing legs z direction CPGs
	 */
	void setSwingLegsCpgs (const std::array<CpgBase::Ptr,4>& feetZDirectionCPGs);

	/**
	 * set the endEffectorStateConstraints vector pointer
	 */
	void setEndEffectorStateConstraints (const std::vector<EndEffectorConstraintBase::Ptr>& endEffectorStateConstraints);

	/**
	 * get the endEffectorStateConstraints vector pointer
	 */
	std::vector<EndEffectorConstraintBase::Ptr>& getEndEffectorStateConstraints() const;


private:

	kinematic_model_t kinematicModel_;
	com_model_t 	  comModel_;
	Eigen::Vector3d   o_gravityVector_;

	ComDynamicsBase<JOINT_COORD_SIZE> comDynamics_;

	std::array<bool,4> stanceLegs_;
	std::array<bool,4> nextPhaseStanceLegs_;

	Options options_;

	FeetZDirectionPlannerBase::Ptr feetZDirectionPlanner_;
	CpgBase::PtrArray feetZDirectionCPGs_;

	std::vector<EndEffectorConstraintBase::Ptr> endEffectorStateConstraints_;

	Eigen::Vector3d com_base2CoM_;
	Eigen::Matrix<double,6,12> b_comJacobain_;
	std::array<Eigen::Vector3d,4> com_base2StanceFeet_;

	std::string algorithmName_;
};

#include "implementation/ComKinoDynamicsBase.h"

#endif /* COMKINODYNAMICSBASE_H_ */
