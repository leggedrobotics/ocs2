#include <ocs2_core/Dimensions.h>
#include <ocs2_core/OCS2NumericTraits.h>

// Automatic Differentation
#include <ocs2_core/automatic_differentiation/AutomaticDifferentiationBase.h>
#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>

// Constraint
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/ConstraintBaseAD.h>
#include <ocs2_core/constraint/LinearConstraint.h>
#include <ocs2_core/constraint/PenaltyBase.h>
#include <ocs2_core/constraint/RelaxedBarrierPenalty.h>

// Control
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/control/ControllerType.h>
#include <ocs2_core/control/FeedforwardController.h>
#include <ocs2_core/control/LinearController.h>

// Cost
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/cost/CostFunctionBaseAD.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>

// Dynamics
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBaseAD.h>
#include <ocs2_core/dynamics/SystemDynamicsLinearizer.h>
#include <ocs2_core/dynamics/TransferFunctionBase.h>

// Initialization
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>

// Integration
#include <ocs2_core/integration/EventHandlerBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/IntegratorBase.h>
#include <ocs2_core/integration/Observer.h>
#include <ocs2_core/integration/OdeBase.h>
#include <ocs2_core/integration/OdeFunc.h>
#include <ocs2_core/integration/StateTriggeredEventHandler.h>
#include <ocs2_core/integration/steppers.h>
#include <ocs2_core/integration/SystemEventHandler.h>

// Logic
#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>
#include <ocs2_core/logic/machine/LogicRulesMachine.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>

// Loopshaping
#include <ocs2_core/loopshaping/Loopshaping.h>

// Misc
#include <ocs2_core/misc/FindActiveIntervalIndex.h>
#include <ocs2_core/misc/LinearFunction.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/loadEigenMatrix.h>
#include <ocs2_core/misc/LTI_Equations.h>
#include <ocs2_core/misc/SetThreadPriority.h>
#include <ocs2_core/misc/TrajectorySpreadingController.h>

// dummy target for clang toolchain
int main() { return 0; }
