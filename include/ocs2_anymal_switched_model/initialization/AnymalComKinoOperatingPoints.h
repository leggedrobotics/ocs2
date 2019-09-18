/*
 * AnymalComKinoOperatingPoints.h
 *
 *  Created on: Feb 9, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_switched_model_interface/initialization/ComKinoOperatingPointsBase.h>

namespace anymal {

class AnymalComKinoOperatingPoints final : public switched_model::ComKinoOperatingPointsBase<12>
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
};

} //end of namespace anymal
