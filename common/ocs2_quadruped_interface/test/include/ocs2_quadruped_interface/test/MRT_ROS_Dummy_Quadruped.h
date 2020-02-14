/*
 * MRT_ROS_Dummy_Quadruped.h
 *
 *  Created on: May 28, 2018
 *      Author: farbod
 */

#pragma once

#include <ocs2_comm_interfaces/test/MRT_ROS_Dummy_Loop.h>

#include "ocs2_quadruped_interface/MRT_ROS_Quadruped.h"
#include "ocs2_quadruped_interface/QuadrupedXppVisualizer.h"

namespace switched_model {

template <size_t JOINT_COORD_SIZE, size_t STATE_DIM = 12 + JOINT_COORD_SIZE, size_t INPUT_DIM = 12 + JOINT_COORD_SIZE>
class MRT_ROS_Dummy_Quadruped : public ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = ocs2::MRT_ROS_Dummy_Loop<STATE_DIM, INPUT_DIM>;
  using typename BASE::command_data_t;
  using typename BASE::primal_solution_t;
  using typename BASE::scalar_t;
  using typename BASE::system_observation_t;

  using mrt_t = MRT_ROS_Quadruped<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;
  using visualizer_t = QuadrupedXppVisualizer<JOINT_COORD_SIZE, STATE_DIM, INPUT_DIM>;

  /**
   * Constructor.
   *
   * @param [in] visualization object
   * @param [in] mrt: The underlying MRT class to be used.
   * @param [in] mrtDesiredFrequency: The MRT loop frequency in Hz
   * @param [in] mpcDesiredFrequency: The MPC loop frequency in Hz
   */
  MRT_ROS_Dummy_Quadruped(std::unique_ptr<visualizer_t> quadrupedXppVisualizer, mrt_t& mrt, scalar_t mrtDesiredFrequency,
                          scalar_t mpcDesiredFrequency = -1);

  /**
   * Default destructor.
   */
  ~MRT_ROS_Dummy_Quadruped() override = default;

 protected:
  void launchVisualizerNode(int argc, char* argv[]) override;

  void publishVisualizer(const system_observation_t& observation, const primal_solution_t& primalSolution,
                         const command_data_t& command) override;

 private:
  std::unique_ptr<visualizer_t> quadrupedXppVisualizer_;
};

}  // end of namespace switched_model

#include "implementation/MRT_ROS_Dummy_Quadruped.h"
