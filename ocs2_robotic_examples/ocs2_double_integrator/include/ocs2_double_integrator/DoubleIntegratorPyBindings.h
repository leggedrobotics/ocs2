#pragma once

#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_python_interface/PythonInterface.h>

#include "ocs2_double_integrator/DoubleIntegratorInterface.h"
#include "ocs2_double_integrator/definitions.h"

namespace ocs2 {
namespace double_integrator {

class DoubleIntegratorPyBindings final : public PythonInterface {
 public:
  /**
   * Constructor
   *
   * @note Creates directory for generated library into if it does not exist.
   * @throw Invalid argument error if input task file does not exist.
   *
   * @param [in] taskFile: The absolute path to the configuration file for the MPC.
   * @param [in] libraryFolder: The absolute path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The absolute path to the URDF of the robot. This is not used for double integrator.
   */
  DoubleIntegratorPyBindings(const std::string& taskFile, const std::string& libraryFolder, const std::string urdfFile = "") {
    // System dimensions
    stateDim_ = static_cast<int>(STATE_DIM);
    inputDim_ = static_cast<int>(INPUT_DIM);

    // Robot interface
    DoubleIntegratorInterface doubleIntegratorInterface(taskFile, libraryFolder);

    // MPC
    auto mpcPtr = std::make_unique<GaussNewtonDDP_MPC>(
        doubleIntegratorInterface.mpcSettings(), doubleIntegratorInterface.ddpSettings(), doubleIntegratorInterface.getRollout(),
        doubleIntegratorInterface.getOptimalControlProblem(), doubleIntegratorInterface.getInitializer());
    mpcPtr->getSolverPtr()->setReferenceManager(doubleIntegratorInterface.getReferenceManagerPtr());

    // Python interface
    PythonInterface::init(doubleIntegratorInterface, std::move(mpcPtr));
  }
};

}  // namespace double_integrator
}  // namespace ocs2
