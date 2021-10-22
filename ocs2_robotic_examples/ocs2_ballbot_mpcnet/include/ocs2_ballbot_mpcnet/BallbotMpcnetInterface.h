#pragma once

#include <ocs2_ballbot/BallbotInterface.h>
#include <ocs2_mpcnet/MpcnetInterfaceBase.h>

namespace ocs2 {
namespace ballbot {

/**
 *  Ballbot MPC-Net interface between C++ and Python.
 */
class BallbotMpcnetInterface : public MpcnetInterfaceBase {
 public:
  /**
   * Constructor.
   * @param [in] nDataGenerationThreads : Number of data generation threads.
   * @param [in] nPolicyEvaluationThreads : Number of policy evaluation threads.
   * @param [in] raisim : Whether to use RaiSim for the rollouts.
   */
  BallbotMpcnetInterface(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads, bool raisim);

  /**
   * Default destructor.
   */
  virtual ~BallbotMpcnetInterface() = default;

 private:
  /**
   * Helper to get the MPC.
   * @param [in] ballbotInterface : The ballbot interface.
   * @return Pointer to the MPC.
   */
  std::unique_ptr<MPC_BASE> getMpc(BallbotInterface& ballbotInterface);
};

}  // namespace ballbot
}  // namespace ocs2
