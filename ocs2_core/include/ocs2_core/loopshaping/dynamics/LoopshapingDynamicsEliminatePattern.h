

#ifndef OCS2_LOOPSHAPINGDYNAMICSELIMINATEPATTERN_H
#define OCS2_LOOPSHAPINGDYNAMICSELIMINATEPATTERN_H

#include "LoopshapingDynamicsInputPattern.h"

namespace ocs2 {
template <size_t FULL_STATE_DIM, size_t FULL_INPUT_DIM, size_t SYSTEM_STATE_DIM, size_t SYSTEM_INPUT_DIM, size_t FILTER_STATE_DIM,
          size_t FILTER_INPUT_DIM>
using LoopshapingDynamicsEliminatePattern =
    LoopshapingDynamicsInputPattern<FULL_STATE_DIM, FULL_INPUT_DIM, SYSTEM_STATE_DIM, SYSTEM_INPUT_DIM, FILTER_STATE_DIM, FILTER_INPUT_DIM>;
}  // namespace ocs2

#endif  // OCS2_LOOPSHAPINGDYNAMICSELIMINATEPATTERN_H
