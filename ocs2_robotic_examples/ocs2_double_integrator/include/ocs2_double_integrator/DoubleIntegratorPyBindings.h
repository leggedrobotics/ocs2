#pragma once

#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_python_interface/PythonInterface.h>

#include "ocs2_double_integrator/DoubleIntegratorInterface.h"
#include "ocs2_double_integrator/definitions.h"

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorPyBindings final : public PythonInterface {
 public:
  explicit DoubleIntegratorPyBindings(const std::string& taskFileFolder) {
    // Robot interface
    DoubleIntegratorInterface doubleIntegratorInterface(taskFileFolder);

    // MPC
    std::unique_ptr<MPC_DDP> mpcPtr(new MPC_DDP(
        doubleIntegratorInterface.mpcSettings(), doubleIntegratorInterface.ddpSettings(), doubleIntegratorInterface.getRollout(),
        doubleIntegratorInterface.getOptimalControlProblem(), doubleIntegratorInterface.getInitializer()));
    mpcPtr->getSolverPtr()->setReferenceManager(doubleIntegratorInterface.getReferenceManagerPtr());

    // Python interface
    PythonInterface::init(doubleIntegratorInterface, std::move(mpcPtr));
  }
};

}  // namespace double_integrator
}  // namespace ocs2
