#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/MPC_Interface.h>
#include <ocs2_robotic_tools/common/RobotInterfaceBase.h>
#include <ocs2_core/dynamics/ControlledSystemBase.h>
#include <ocs2_core/dynamics/DerivativesBase.h>
#include <ocs2_core/cost/CostFunctionBase.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM, class LOGIC_RULES_T = NullLogicRules>
class PythonInterface {
 public:
  using dim_t = ocs2::Dimensions<STATE_DIM, INPUT_DIM>;
  using mpc_t = ocs2::MPC_Interface<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>;
  using state_vector_t = typename dim_t::state_vector_t;
  using input_vector_t = typename dim_t::input_vector_t;
  using scalar_array_t = typename dim_t::scalar_array_t;
  using state_vector_array_t = typename dim_t::state_vector_array_t;
  using input_vector_array_t = typename dim_t::input_vector_array_t;
  using state_matrix_t = typename dim_t::state_matrix_t;
  using state_input_matrix_t = typename dim_t::state_input_matrix_t;
  using input_state_matrix_array_t = typename dim_t::input_state_matrix_array_t;
  using state_matrix_array_t = typename dim_t::state_matrix_array_t;
  using cost_desired_trajectories_t = typename mpc_t::cost_desired_trajectories_t;
  using cost_t = CostFunctionBase<STATE_DIM, INPUT_DIM, LOGIC_RULES_T>;
  using dynamic_vector_array_t = typename dim_t::dynamic_vector_array_t;

  PythonInterface(bool async = false);
  virtual ~PythonInterface();

  /**
   * @brief initializes the class. This must happen before any other method is called
   * @note init is not called in the constructor because it internally calls pure virtual initRobotInterface
   * @param taskFileFolder path of settings file
   */
  void init(const std::string& taskFileFolder);

  void setObservation(double t, Eigen::Ref<const state_vector_t> x);
  void setTargetTrajectories(const cost_desired_trajectories_t& targetTrajectories);

  void advanceMpc();
  void runMpcAsync();

  void getMpcSolution(scalar_array_t& t, state_vector_array_t& x, input_vector_array_t& u, state_matrix_array_t& sigmaX);

  state_vector_t computeFlowMap(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);
  void setFlowMapDerivativeStateAndControl(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);
  state_matrix_t computeFlowMapDerivativeState();
  state_input_matrix_t computeFlowMapDerivativeInput();

  double getRunningCost(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);
  state_vector_t getRunningCostDerivativeState(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);
  input_vector_t getRunningCostDerivativeInput(double t, Eigen::Ref<const state_vector_t> x, Eigen::Ref<const input_vector_t> u);

  state_vector_t getValueFunctionStateDerivative(double t, Eigen::Ref<const state_vector_t> x);

protected:

  /**
   * @brief initRobotInterface Must be implemented by a derived class to instantiate
   * the robot-specific robotInterface_
   * @param taskFileFolder folder for config files
   */
  virtual void initRobotInterface(const std::string& taskFileFolder) = 0;

  // Member variables
 protected:
  std::unique_ptr<RobotInterfaceBase<STATE_DIM, INPUT_DIM>> robotInterface_;
  std::unique_ptr<mpc_t> mpcInterface_;

  std::unique_ptr<ControlledSystemBase<STATE_DIM, INPUT_DIM>> dynamics_;
  std::unique_ptr<DerivativesBase<STATE_DIM, INPUT_DIM>> dynamicsDerivatives_;

  cost_t* cost_;

  // multithreading helper variables
  bool run_mpc_async_;
  std::thread run_mpc_worker_;
  std::mutex run_mpc_mutex_;
  std::condition_variable run_mpc_cv_;
  bool run_mpc_done_;
  bool run_mpc_requested_;
  bool shutdown_requested_;
};

}  // namespace ocs2

#include "implementation/Python_Interface.h"
