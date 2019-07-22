/*
 * AnymalComKinoOperatingPoints.h
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#ifndef ANYMAL_COMKINOOPERATINGPOINTS_H_
#define ANYMAL_COMKINOOPERATINGPOINTS_H_

#include <ocs2_anymal_switched_model/dynamics/AnymalCom.h>
#include <ocs2_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

namespace anymal {

class AnymalComKinoOperatingPoints : public switched_model::ComKinoOperatingPointsBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Base = switched_model::ComKinoOperatingPointsBase<12>;
  	using logic_rules_t = switched_model::SwitchedModelPlannerLogicRules<12, double>;

	AnymalComKinoOperatingPoints(std::shared_ptr<const logic_rules_t> logicRulesPtr,
								const switched_model::Model_Settings& options = switched_model::Model_Settings(),
								const generalized_coordinate_t& defaultConfiguration = generalized_coordinate_t::Zero());

	AnymalComKinoOperatingPoints(const AnymalComKinoOperatingPoints& rhs);

	~AnymalComKinoOperatingPoints() {}

	AnymalComKinoOperatingPoints* clone() const override;

private:

};

} //end of namespace anymal

#endif /* ANYMAL_COMKINOOPERATINGPOINTS_H_ */
