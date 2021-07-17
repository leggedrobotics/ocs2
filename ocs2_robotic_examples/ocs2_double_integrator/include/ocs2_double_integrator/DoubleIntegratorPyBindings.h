#pragma once

#include <ocs2_mpc/MPC_DDP.h>
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
   * @param [in] taskFile: The path to the configuration file for the MPC.
   * @param [in] libraryFolder: The path to the directory to generate CppAD library into.
   * @param [in] urdfFile: The path to the URDF of the robot. This is not used for double integrator.
   */
  explicit DoubleIntegratorPyBindings(const std::string& taskFile, const std::string& libraryFolder, const std::string urdfFile = "") {
    // Robot interface
    DoubleIntegratorInterface doubleIntegratorInterface(taskFile, libraryFolder);

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
