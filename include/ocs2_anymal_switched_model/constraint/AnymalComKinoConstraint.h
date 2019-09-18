#pragma once

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBase.h>

namespace anymal {

class AnymalComKinoConstraint final : public switched_model::ComKinoConstraintBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Base = switched_model::ComKinoConstraintBase<12>;
 	using logic_rules_t = switched_model::SwitchedModelPlannerLogicRules<12, double>;

	AnymalComKinoConstraint(std::shared_ptr<const logic_rules_t> logicRulesPtr, const switched_model::Model_Settings& options = switched_model::Model_Settings());

	AnymalComKinoConstraint(const AnymalComKinoConstraint& rhs);

	virtual ~AnymalComKinoConstraint() = default;

	AnymalComKinoConstraint* clone() const override;
};

} //end of namespace anymal

