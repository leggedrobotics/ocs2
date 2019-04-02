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

//#define PUBLISH_DUMMY
#define PUBLISH_THREAD

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
      const std::string& robotName = "robot");

  /**
   * Destructor.
   */
  virtual ~MPC_Interface() = default;

  /**
   * Sets the class as its constructor.
   *
   * @param [in] mpcPtr: The MPC object to be interfaced.
   * @param [in] robotName: The robot's name.
   */
  void set(mpc_t& mpc,
           const std::string& robotName = "robot");

  /**
   * Resets the class to its instantiate state.
   */
  virtual void reset();

  /**
   * Provides the initial mode sequence for time-triggered hybrid systems.
   *
   * @param [in] initObservation: The observation after the very fist call of the class or after call to reset().
   */
  virtual void initModeSequence(
      const system_observation_t& initObservation) {}

  bool resetMpcCallback();

  void setCurrentObservation(const system_observation_t& currentObservation);

  void setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories);

  void setModeSequence(const mode_sequence_template_t& modeSequenceTemplate);

  void advance();

protected:

  /*
   * Variables
   */
  mpc_t* mpcPtr_;
  MPC_Settings mpcSettings_;

  std::string robotName_;


  size_t numIterations_;
  scalar_t maxDelay_;
  scalar_t meanDelay_;
  scalar_t currentDelay_;

  std::chrono::time_point<std::chrono::steady_clock> startTimePoint_;
  std::chrono::time_point<std::chrono::steady_clock> finalTimePoint_;

  bool initialCall_;

  std::atomic<bool> desiredTrajectoriesUpdated_;
  std::atomic<bool> modeSequenceUpdated_;

  //MPC inputs
  cost_desired_trajectories_t costDesiredTrajectories_;
  cost_desired_trajectories_t defaultCostDesiredTrajectories_;
  mode_sequence_template_t modeSequenceTemplate_;
  system_observation_t currentObservation_;

  //MPC outputs:
  std::vector<scalar_array_t>* timeTrajectoriesPtr_;
  state_vector_array2_t* stateTrajectoriesPtr_;
  input_vector_array2_t* inputTrajectoriesPtr_;
  controller_array_t* controllersPtr_;
  scalar_array_t* eventTimesPtr_;
  size_array_t* subsystemsSequencePtr_;

  //Solver saved inputs
  cost_desired_trajectories_t* solverCostDesiredTrajectoriesPtr_;
};

} // namespace ocs2


#include "implementation/MPC_Interface.h"
#endif //OCS2_WACO_INTERFACE_MPC_INTERFACE_H
