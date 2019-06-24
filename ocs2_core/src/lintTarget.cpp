#include <ocs2_core/Dimensions.h>
#include <ocs2_core/OCS2NumericTraits.h>
#include <ocs2_core/automatic_differentiation/AutomaticDifferentiationBase.h>
#include <ocs2_core/automatic_differentiation/CppAdCodeGenInterface.h>
#include <ocs2_core/constraint/ConstraintBase.h>
#include <ocs2_core/constraint/ConstraintBaseAD.h>
#include <ocs2_core/constraint/LinearConstraint.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/cost/CostFunctionBase.h>
#include <ocs2_core/cost/CostFunctionBaseAD.h>
#include <ocs2_core/cost/QuadraticCostFunction.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>
#include <ocs2_core/dynamics/SystemDynamicsLinearizer.h>
#include <ocs2_core/dynamics/TransferFunctionBase.h>
#include <ocs2_core/initialization/SystemOperatingPoint.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/integration/EventHandlerBase.h>
#include <ocs2_core/integration/Integrator.h>
#include <ocs2_core/integration/Observer.h>
#include <ocs2_core/integration/steppers.h>
#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>
#include <ocs2_core/logic/machine/LogicRulesMachine.h>
#include <ocs2_core/logic/rules/HybridLogicRules.h>
#include <ocs2_core/logic/rules/NullLogicRules.h>
#include <ocs2_core/misc/FindActiveIntervalIndex.h>
#include <ocs2_core/misc/LTI_Equations.h>
#include <ocs2_core/misc/LinearFunction.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/misc/TrajectorySpreadingController.h>
#include <ocs2_core/misc/loadEigenMatrix.h>

// dummy target for clang toolchain
int main() { return 0; }
