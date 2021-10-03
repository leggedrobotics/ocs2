//
// Created by ruben on 18.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

class LoopshapingStateInputConstraint : public StateInputConstraintCollection {
 public:
  ~LoopshapingStateInputConstraint() override = default;

  vector_t getValue(scalar_t time, const vector_t& state, const vector_t& input, const PreComputation& preComp) const override;

 protected:
  LoopshapingStateInputConstraint(const StateInputConstraintCollection& systemConstraint,
                                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : StateInputConstraintCollection(systemConstraint), loopshapingDefinition_(std::move(loopshapingDefinition)){};

  LoopshapingStateInputConstraint(const LoopshapingStateInputConstraint& other) = default;

  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};

}  // namespace ocs2
