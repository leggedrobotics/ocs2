#pragma once

#include <ocs2_switched_model_interface/constraint/ComKinoConstraintBaseAd.h>

namespace anymal {

class AnymalComKinoConstraintAd final : public switched_model::ComKinoConstraintBaseAd<12> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = switched_model::ComKinoConstraintBaseAd<12>;
  using logic_rules_t = switched_model::SwitchedModelPlannerLogicRules<12, double>;

  AnymalComKinoConstraintAd(std::shared_ptr<const logic_rules_t> logicRulesPtr,
                            const switched_model::Model_Settings& options = switched_model::Model_Settings());

  AnymalComKinoConstraintAd(const AnymalComKinoConstraintAd& rhs);

  virtual ~AnymalComKinoConstraintAd() = default;

  AnymalComKinoConstraintAd* clone() const override;
};

}  // end of namespace anymal
