//
// Created by johannes on 01.04.19.
//

#ifndef OCS2_WACO_INTERFACE_MPC_INTERFACE_H
#define OCS2_WACO_INTERFACE_MPC_INTERFACE_H

#include <array>
#include <memory>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <csignal>
#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include <Eigen/Dense>

#include <ocs2_mpc/MPC_BASE.h>
#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_core/logic/machine/HybridLogicRulesMachine.h>

//For RosWarnStream, can be removed if std::cout is used instead
#include <ros/ros.h>


namespace ocs2 {

/**
 * This class implements MPC communication interface using ROS.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 * @tparam LOGIC_RULES_T: Logic Rules type (default NullLogicRules).
 */
template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T=NullLogicRules>
class MPC_Interface
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef std::shared_ptr<MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>> Ptr;

  typedef MPC_BASE<STATE_DIM, INPUT_DIM, LOGIC_RULES_T> mpc_t;

  typedef typename mpc_t::scalar_t                   scalar_t;
  typedef typename mpc_t::scalar_array_t             scalar_array_t;
  typedef typename mpc_t::size_array_t               size_array_t;
  typedef typename mpc_t::state_vector_t             state_vector_t;
  typedef typename mpc_t::state_vector_array_t       state_vector_array_t;
  typedef typename mpc_t::state_vector_array2_t      state_vector_array2_t;
  typedef typename mpc_t::input_vector_t             input_vector_t;
  typedef typename mpc_t::input_vector_array_t       input_vector_array_t;
  typedef typename mpc_t::input_vector_array2_t      input_vector_array2_t;
  typedef typename mpc_t::controller_t               controller_t;
  typedef typename mpc_t::controller_array_t         controller_array_t;
  typedef typename mpc_t::input_state_matrix_t       input_state_matrix_t;
  typedef typename mpc_t::input_state_matrix_array_t input_state_matrix_array_t;

  typedef typename mpc_t::cost_desired_trajectories_t  cost_desired_trajectories_t;
  typedef typename mpc_t::mode_sequence_template_t     mode_sequence_template_t;

  typedef SystemObservation<STATE_DIM, INPUT_DIM> system_observation_t;

  typedef LinearInterpolation<state_vector_t, Eigen::aligned_allocator<state_vector_t> > state_linear_interpolation_t;
  typedef LinearInterpolation<input_vector_t, Eigen::aligned_allocator<input_vector_t> > input_linear_interpolation_t;
  typedef LinearInterpolation<input_state_matrix_t, Eigen::aligned_allocator<input_state_matrix_t>> gain_linear_interpolation_t;
  typedef HybridLogicRulesMachine<LOGIC_RULES_T> logic_machine_t;


  /**
   * Default constructor
   */
  MPC_Interface() = default;

  /**
   * Constructor.
   *
   * @param [in] mpc: The MPC object to be interfaced.
   * @param [in] robotName: The robot's name.
   */
  MPC_Interface(
      mpc_t& mpc,
      const LOGIC_RULES_T& logicRules,
      const bool& useFeedforwardPolicy = true);

  /**
   * Destructor.
   */
  virtual ~MPC_Interface() = default;

  /**
   * Resets the class to its instantiate state.
   */
  virtual void reset();

  void setCurrentObservation(const system_observation_t& currentObservation);

  void setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories);

  void setModeSequence(const mode_sequence_template_t& modeSequenceTemplate);

  void advanceMpc();

  /**
 * Whether the initial MPC policy has been already received.
 */
  bool policyReceived() const { return !initialCall_;};;

  /**
   * Gets a reference to CostDesiredTrajectories for which the current policy is optimized for.
   *
   * @return a constant reference to CostDesiredTrajectories of the policy.
   */
  const cost_desired_trajectories_t& mpcCostDesiredTrajectories() const;

  /**
   * Evaluates the latest feedforward policy at the given time. The SLQ-MPC feedforward
   * policy includes two components. The optimized state and input trajectories. Moreover
   * it finds the active subsystem at the given time.
   *
   * @param [in] time: The inquiry time.
   * @param [out] mpcState: The feedforward policy's optimized state.
   * @param [out] mpcInput: The feedforward policy's optimized input.
   * @param [out] subsystem: The active subsystem.
   */
  void evaluateFeedforwardPolicy(
      const scalar_t& time,
      state_vector_t& mpcState,
      input_vector_t& mpcInput,
      size_t& subsystem);

  /**
   * Evaluates the latest feedback policy at the given time. The SLQ-MPC feedback
   * policy is defined as an affine time-state dependent function. Moreover it finds
   * the active subsystem at the given time.
   *
   * @param [in] time: The inquiry time.
   * @param [out] mpcUff: The feedback policy's optimized uff.
   * @param [out] mpcGain: The feedback policy's optimized gain matrix.
   * @param [out] subsystem: The active subsystem.
   */
  void evaluateFeedbackPolicy(
      const scalar_t& time,
      input_vector_t& mpcUff,
      input_state_matrix_t& mpcGain,
      size_t& subsystem);

protected:

  /*
   * Variables
   */
  mpc_t* mpcPtr_;
  MPC_Settings mpcSettings_;

  size_t numIterations_;
  scalar_t maxDelay_ = 0;
  scalar_t meanDelay_ = 0;
  scalar_t currentDelay_ = 0;

  std::chrono::time_point<std::chrono::steady_clock> startTimePoint_;
  std::chrono::time_point<std::chrono::steady_clock> finalTimePoint_;

  bool initialCall_ = false;

  std::atomic<bool> observationUpdated_;
  std::atomic<bool> desiredTrajectoriesUpdated_;
  std::atomic<bool> modeSequenceUpdated_;

  //MPC inputs
  cost_desired_trajectories_t costDesiredTrajectories_;
  mode_sequence_template_t modeSequenceTemplate_;
  system_observation_t currentObservation_;

  //MPC outputs:
  size_array_t   mpcSubsystemsSequenceBuffer_;
  scalar_array_t mpcEventTimesBuffer_;
  scalar_array_t mpcTimeTrajectoryBuffer_;
  state_vector_array_t mpcStateTrajectoryBuffer_;
  input_vector_array_t mpcInputTrajectoryBuffer_;
  controller_t   mpcControllerBuffer_;
  cost_desired_trajectories_t  mpcSolverCostDesiredTrajectoriesBuffer_;
  std::atomic<bool> mpcOutputBufferUpdated_;
  bool logicUpdated_;
  std::mutex mpcBufferMutex;

  //variables for the tracking controller:
  bool useFeedforwardPolicy_;
  scalar_array_t eventTimes_;
  size_array_t   subsystemsSequence_;
  scalar_array_t partitioningTimes_;

  controller_t mpcController_;
  scalar_array_t mpcTimeTrajectory_;
  state_vector_array_t mpcStateTrajectory_;
  input_vector_array_t mpcInputTrajectory_;
  state_linear_interpolation_t mpcLinInterpolateState_;
  input_linear_interpolation_t mpcLinInterpolateInput_;
  input_linear_interpolation_t mpcLinInterpolateUff_;
  gain_linear_interpolation_t  mpcLinInterpolateK_;
  cost_desired_trajectories_t  mpcCostDesiredTrajectories_;

  logic_machine_t logicMachine_;
  std::function<size_t(scalar_t)> findActiveSubsystemFnc_;

protected:
  /**
 * Checks the data buffer for an update of the MPC policy. If a new policy
 * is available on the buffer this method will load it to the in-use policy.
 *
 * @return True if the policy is updated.
 */
  bool updatePolicy();

  /**
 * Constructs a partitioningTimes vector with 2 elements: current observation time
 * and the maximum value of the numeric type scalar_t. This prevents
 * the frequent update of the logicRules.
 *
 * @param [out] partitioningTimes: Partitioning time.
 */
  void partitioningTimesUpdate(
      scalar_array_t& partitioningTimes) const;


};

} // namespace ocs2


#include "implementation/MPC_Interface.h"
#endif //OCS2_WACO_INTERFACE_MPC_INTERFACE_H
