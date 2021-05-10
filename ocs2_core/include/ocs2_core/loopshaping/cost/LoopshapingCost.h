//
// Created by ruben on 24.09.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {

/**
 * Loopshaping state-only cost decorator class
 */
class LoopshapingCost final : public CostFunctionBase {
 public:
};

}  // namespace ocs2
