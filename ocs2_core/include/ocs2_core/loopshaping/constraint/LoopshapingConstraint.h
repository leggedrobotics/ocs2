//
// Created by ruben on 18.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/constraint/StateConstraintCollection.h>
#include <ocs2_core/constraint/StateInputConstraintCollection.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {
namespace LoopshapingConstraint {

std::unique_ptr<StateConstraintCollection> create(const StateConstraintCollection& systemConstraint,
                                                  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

std::unique_ptr<StateInputConstraintCollection> create(const StateInputConstraintCollection& systemConstraint,
                                                       std::shared_ptr<LoopshapingDefinition> loopshapingDefinition);

}  // namespace LoopshapingConstraint
}  // namespace ocs2
