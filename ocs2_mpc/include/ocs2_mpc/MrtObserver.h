//
// Created by rgrandia on 08.09.20.
//

#pragma once

#include <ocs2_oc/oc_data/PrimalSolution.h>

#include "ocs2_mpc/CommandData.h"

namespace ocs2 {

/**
 * This class allows modification of the MPC solution at two separate points of the MRT process.
 * Moreover, it can be used to add custom operations in sync with the MRT update strategy.
 *
 * The MRT uses a buffer structure to allow access to a in-use policy while a new policy is being prepared in a separate thread.
 * After a new policy is available in the separate thread, it is loaded into a policy buffer.
 *      - At this point the "modifyBufferPolicy" of this class is called.
 *
 * When a user requests an update, the in-use policy is swapped for the buffered policy.
 *      - At this point the "modifyPolicy" of this class is called.
 *
 * Both filling of the buffer and the update swapping are protected by the same mutex.
 */
class MrtObserver {
 public:
  MrtObserver() = default;
  virtual ~MrtObserver() = default;

  // Delete copy and move operations
  MrtObserver(const MrtObserver&) = delete;
  MrtObserver& operator=(const MrtObserver&) = delete;
  MrtObserver(MrtObserver&&) = delete;
  MrtObserver& operator=(MrtObserver&&) = delete;

  /**
   * This method is called as part of MRT_BASE::updatePolicy().
   * It allows the user to modify the policy that will become in-use after the updatePolicy function returns.
   *
   * A call to this function is protected by the same mutex as modifyBufferPolicy.
   */
  virtual void modifyPolicy(const CommandData& command, PrimalSolution& primalSolution) {}

  /**
   * This method is called by the MRT after a new policy is loaded into the policyBuffer.
   * It allows the user to modify the buffered policy before it can be swapped during the modify policy.
   *
   * A call to this function is protected by the same mutex as modifyPolicy.
   */
  virtual void modifyBufferPolicy(const CommandData& commandBuffer, PrimalSolution& primalSolutionBuffer) {}
};

}  // namespace ocs2