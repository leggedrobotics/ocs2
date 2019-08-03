#pragma once

#include <Eigen/Dense>

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>

#include <ocs2_comm_interfaces/SystemObservation.h>
#include <ocs2_core/Dimensions.h>
#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/cost/CostDesiredTrajectories.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_oc/rollout/RolloutBase.h>

namespace ocs2 {

/**
 * This class implements core MRT (Model Reference Tracking) functionality.
 * The responsibility of filling the buffer variables is left to the deriving classes.
 *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
template <size_t STATE_DIM, size_t INPUT_DIM>
class MRT_BASE {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dim_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using scalar_array_t = typename dim_t::scalar_array_t;
  using scalar_t = typename dim_t::scalar_t;
  using size_array_t = typename dim_t::size_array_t;
  using state_vector_t = typename dim_t::state_vector_t;
  using state_vector_array_t = typename dim_t::state_vector_array_t;
  using input_vector_t = typename dim_t::input_vector_t;
  using input_vector_array_t = typename dim_t::input_vector_array_t;

  using controller_t = ControllerBase<STATE_DIM, INPUT_DIM>;
  using state_linear_interpolation_t = EigenLinearInterpolation<typename dim_t::state_vector_t>;

  /**
   * @brief MRT_BASE constructor
   * @param[in] logicRules: Optional pointer to the logic rules.
   */
  explicit MRT_BASE(std::shared_ptr<HybridLogicRules> logicRules = nullptr);

  /**
   * @brief Default destructor
   */
  ~MRT_BASE() = default;

  /**
   * Resets the class to its instantiated state.
   */
  void reset();

  /**
   * Request the MPC node to reset. This method is a blocking method.
   *
   * @param [in] initCostDesiredTrajectories: The initial desired cost trajectories.
   */
  virtual void resetMpcNode(const CostDesiredTrajectories<scalar_t>& initCostDesiredTrajectories) = 0;

  /**
   * Whether the initial MPC policy has been already received.
   */
  bool initialPolicyReceived() const { return policyReceivedEver_; }

  /**
   * @brief setCurrentObservation notifies MPC of a new state
   * @param observation: the current measurement to send to the MPC
   */
  virtual void setCurrentObservation(const SystemObservation<STATE_DIM, INPUT_DIM>& observation) = 0;

  /**
   * Gets a reference to CostDesiredTrajectories for which the current policy is optimized for.
   * @warning access to the returned reference is not threadsafe. Read access and calls to updatePolicy() must be synced by the user.
   *
   * @return a constant reference to CostDesiredTrajectories of the policy.
   */
  const CostDesiredTrajectories<scalar_t>& mpcCostDesiredTrajectories() const;

  /**
   * Initializes rollout class to roll out a feedback policy
   *
   * @param [in] controlledSystemBase: System to roll out.
   * @param [in] rolloutSettings
   */
  void initRollout(const ControlledSystemBase<STATE_DIM, INPUT_DIM>& controlledSystemBase, const Rollout_Settings& rolloutSettings);

  /**
   * @brief Evaluates the controller
   *
   * @param [in] currentTime: the query time.
   * @param [in] currentState: the query state.
   * @param [out] mpcState: the current nominal state of MPC.
   * @param [out] mpcInput: the optimized control input.
   * @param [out] subsystem: the active subsystem.
   */
  void evaluatePolicy(scalar_t currentTime, const state_vector_t& currentState, state_vector_t& mpcState, input_vector_t& mpcInput,
                      size_t& subsystem);

  /**
   * @brief Rolls out the control policy from the current time and state to get the next state and input using the MPC policy.
   *
   * @param [in] currentTime: start time of the rollout.
   * @param [in] currentState: state to start rollout from.
   * @param [in] timeStep: duration of the forward rollout.
   * @param [out] mpcState: the new forwarded state of MPC.
   * @param [out] mpcInput: the new control input of MPC.
   * @param [out] subsystem: the active subsystem.
   */
  void rolloutPolicy(scalar_t currentTime, const state_vector_t& currentState, const scalar_t& timeStep, state_vector_t& mpcState,
                     input_vector_t& mpcInput, size_t& subsystem);

  /**
   * Checks the data buffer for an update of the MPC policy. If a new policy
   * is available on the buffer this method will load it to the in-use policy.
   * This method also calls the modifyPolicy() method.
   *
   * @return True if the policy is updated.
   */
  bool updatePolicy();

  void setLogicRules(std::shared_ptr<HybridLogicRules> logicRules);

 protected:
  /**
   * The updatePolicy() method will call this method which allows the user to
   * customize the in-use policy. Note that this method is already
   * protected with a mutex which blocks the policy callback. Moreover, this method
   * may be called in the main thread of the program. Thus, for efficiency and
   * practical considerations you should avoid computationally expensive operations.
   * For such operations you may want to use the modifyBufferPolicy()
   * methods which runs on a separate thread which directly modifies the received
   * policy messages on the data buffer.
   *
   * @param logicUpdated: Whether eventTimes or subsystemsSequence are updated form the last call.
   * @param mpcController: The optimized control policy of MPC.
   * @param mpcTimeTrajectory: The optimized time trajectory of the policy message on the buffer.
   * @param mpcStateTrajectory: The optimized state trajectory of the policy message on the buffer.
   * @param eventTimes: The event times of the policy.
   * @param subsystemsSequence: The subsystems sequence of the policy.
   */
  virtual void modifyPolicy(bool& logicUpdated, controller_t& mpcController, scalar_array_t& mpcTimeTrajectory,
                            state_vector_array_t& mpcStateTrajectory, scalar_array_t& eventTimes, size_array_t& subsystemsSequence) {}

  /**
   * This method can be used to modify the policy on the buffer without inputting the
   * main thread. Note that the variables that are on the buffer have the suffix Buffer. It is
   * important if any new variables are added to the policy also obey this rule. These buffer
   * variables can be later, in the modifyPolicy() method, swept to the in-use policy memory.
   *
   * @param [in] mpcInitObservationBuffer: The observation of the policy message on the buffer.
   * @param mpcControllerBuffer: The optimized controller of the policy message on the buffer.
   * @param mpcTimeTrajectoryBuffer: The optimized time trajectory of the policy message on the buffer.
   * @param mpcStateTrajectoryBuffer: The optimized state trajectory of the policy message on the buffer.
   * @param eventTimesBuffer: The event times of the policy message on the buffer.
   * @param subsystemsSequenceBuffer: The subsystems sequence of the policy message on the buffer.
   */
  virtual void modifyBufferPolicy(const SystemObservation<STATE_DIM, INPUT_DIM>& mpcInitObservationBuffer,
                                  controller_t& mpcControllerBuffer, scalar_array_t& mpcTimeTrajectoryBuffer,
                                  state_vector_array_t& mpcStateTrajectoryBuffer, scalar_array_t& eventTimesBuffer,
                                  size_array_t& subsystemsSequenceBuffer) {}

  /**
   * Constructs a partitioningTimes vector with 2 elements: minimum of the already
   * received times and the maximum value of the numeric type scalar_t. This prevents
   * the frequent update of the logicRules.
   *
   * @param [in] time: The current time.
   * @param [out] partitioningTimes: Partitioning time.
   */
  void partitioningTimesUpdate(scalar_t time, scalar_array_t& partitioningTimes) const;

 protected:
  // flags on state of the class
  std::atomic_bool policyReceivedEver_;
  bool newPolicyInBuffer_;  //! Whether a new policy is waiting to be swapped in

  // variables related to the MPC output
  bool logicUpdated_;               //! Whether the logic rules have changed (from MPC)
  std::atomic_bool policyUpdated_;  //! Whether the policy was updated by MPC (i.e., MPC succeeded)
  bool policyUpdatedBuffer_;        //! Whether the policy in buffer was upated by MPC (i.e., MPC succeeded)
  std::unique_ptr<controller_t> mpcController_;
  std::unique_ptr<controller_t> mpcControllerBuffer_;
  scalar_array_t mpcTimeTrajectory_;
  scalar_array_t mpcTimeTrajectoryBuffer_;
  state_vector_array_t mpcStateTrajectory_;
  state_vector_array_t mpcStateTrajectoryBuffer_;
  state_linear_interpolation_t mpcLinInterpolateState_;
  CostDesiredTrajectories<scalar_t> mpcCostDesiredTrajectories_;
  CostDesiredTrajectories<scalar_t> mpcCostDesiredTrajectoriesBuffer_;
  scalar_array_t eventTimes_;
  scalar_array_t eventTimesBuffer_;
  size_array_t subsystemsSequence_;
  size_array_t subsystemsSequenceBuffer_;
  scalar_array_t partitioningTimes_;
  scalar_array_t partitioningTimesBuffer_;
  SystemObservation<STATE_DIM, INPUT_DIM> mpcInitObservation_;
  SystemObservation<STATE_DIM, INPUT_DIM> mpcInitObservationBuffer_;

  // thread safety
  mutable std::mutex policyBufferMutex_;  // for policy variables WITH suffix (*Buffer_)

  // variables needed for policy evaluation
  std::function<size_t(scalar_t)> findActiveSubsystemFnc_;
  std::unique_ptr<RolloutBase<STATE_DIM, INPUT_DIM>> rolloutPtr_;
  HybridLogicRulesMachine::Ptr logicMachinePtr_;

  // Varia
  SystemObservation<STATE_DIM, INPUT_DIM> initPlanObservation_;  //! The initial observation of the first plan ever received
};

}  // namespace ocs2

#include "implementation/MRT_BASE.h"
