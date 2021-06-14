// Approximate model
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

// oc_data
#include <ocs2_oc/oc_data/PrimalSolution.h>

// oc_solver
#include <ocs2_oc/oc_solver/PerformanceIndex.h>
#include <ocs2_oc/oc_solver/SolverBase.h>

// synchronized_module
#include <ocs2_oc/synchronized_module/LoopshapingModeScheduleManager.h>
#include <ocs2_oc/synchronized_module/ModeScheduleManager.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

// rollout
#include <ocs2_oc/rollout/InitializerRollout.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/RolloutSettings.h>
#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// dummy target for clang toolchain
int main() {
  return 0;
}
