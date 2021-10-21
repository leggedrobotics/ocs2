#pragma once

#include <ocs2_legged_robot/LeggedRobotInterface.h>
#include <ocs2_legged_robot_raisim/LeggedRobotRaisimConversions.h>
#include <ocs2_mpcnet/MpcnetInterfaceBase.h>

namespace ocs2 {
namespace legged_robot {

/**
 *  Legged robot MPC-Net interface between C++ and Python.
 */
class LeggedRobotMpcnetInterface : public MpcnetInterfaceBase {
 public:
  /**
   * Constructor.
   * @param [in] nDataGenerationThreads : Number of data generation threads.
   * @param [in] nPolicyEvaluationThreads : Number of policy evaluation threads.
   * @param [in] raisim : Whether to use RaiSim for the rollouts.
   */
  LeggedRobotMpcnetInterface(size_t nDataGenerationThreads, size_t nPolicyEvaluationThreads, bool raisim);

  /**
   * Default destructor.
   */
  virtual ~LeggedRobotMpcnetInterface() = default;

 private:
  /**
   * Helper to get the MPC.
   * @param [in] leggedRobotInterface : The legged robot interface.
   * @return Pointer to the MPC.
   */
  std::unique_ptr<MPC_BASE> getMpc(LeggedRobotInterface& leggedRobotInterface);

  // Legged robot interface pointers (keep alive for Pinocchio interface)
  std::vector<std::unique_ptr<LeggedRobotInterface>> leggedRobotInterfacePtrs_;
  // Legged robot RaiSim conversions pointers (keep alive for RaiSim rollout)
  std::vector<std::unique_ptr<LeggedRobotRaisimConversions>> leggedRobotRaisimConversionsPtrs_;
};

}  // namespace legged_robot
}  // namespace ocs2
