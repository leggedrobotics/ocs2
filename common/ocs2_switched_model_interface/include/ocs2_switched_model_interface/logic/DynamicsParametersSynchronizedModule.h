#pragma once

#include <ocs2_core/thread_support/Synchronized.h>
#include <ocs2_oc/synchronized_module/SolverSynchronizedModule.h>

#include "ocs2_switched_model_interface/dynamics/ComKinoDynamicsParameters.h"

namespace switched_model {

class DynamicsParametersSynchronizedModule : public ocs2::SolverSynchronizedModule {
 public:
  DynamicsParametersSynchronizedModule();

  void preSolverRun(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                    const ocs2::ReferenceManagerInterface& referenceManager) override;

  void postSolverRun(const ocs2::PrimalSolution& primalSolution) override{};

  // Write-able access to dynamics parameters
  ocs2::Synchronized<ComKinoSystemDynamicsParameters<scalar_t>>& getDynamicsParameters() { return newDynamicsParameters_; }
  const ocs2::Synchronized<ComKinoSystemDynamicsParameters<scalar_t>>& getDynamicsParameters() const { return newDynamicsParameters_; }

  // Read-only access to active dynamics parameters (Not thread safe while MPC is running!)
  const ComKinoSystemDynamicsParameters<scalar_t>& getActiveDynamicsParameters() const { return activeDynamicsParameters_; }

 private:
  //! Parameters active in the current MPC optimization
  ComKinoSystemDynamicsParameters<scalar_t> activeDynamicsParameters_;
  //! Updated externally, becomes active in next MPC iteration
  ocs2::Synchronized<ComKinoSystemDynamicsParameters<scalar_t>> newDynamicsParameters_;
};

}  // namespace switched_model
