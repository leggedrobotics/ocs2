// Approximate model
#include <ocs2_oc/approximate_model/LinearQuadraticApproximator.h>

// oc_data
#include <ocs2_oc/oc_data/DualSolution.h>
#include <ocs2_oc/oc_data/LoopshapingPrimalSolution.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>

// oc_problem
#include <ocs2_oc/oc_problem/LoopshapingOptimalControlProblem.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/oc_problem/OptimalControlProblemHelperFunction.h>

// oc_solver
#include <ocs2_oc/oc_solver/PerformanceIndex.h>
#include <ocs2_oc/oc_solver/SolverBase.h>

// synchronized_module
#include <ocs2_oc/synchronized_module/LoopshapingReferenceManager.h>
#include <ocs2_oc/synchronized_module/LoopshapingSynchronizedModule.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerDecorator.h>
#include <ocs2_oc/synchronized_module/ReferenceManagerInterface.h>
#include <ocs2_oc/synchronized_module/SolverObserver.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

// rollout
#include <ocs2_oc/rollout/InitializerRollout.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/rollout/RolloutSettings.h>
#include <ocs2_oc/rollout/StateTriggeredRollout.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>

// trajectory_adjustment
#include <ocs2_oc/trajectory_adjustment/TrajectorySpreading.h>

// dummy target for clang toolchain
int main() {
  return 0;
}
