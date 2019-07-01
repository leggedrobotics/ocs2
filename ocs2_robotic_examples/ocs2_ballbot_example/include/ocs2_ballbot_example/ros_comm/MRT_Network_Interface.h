#pragma once

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>
#include "ocs2_ballbot_example/definitions.h"

#include <ocs2_core/control/NetworkController.h>



namespace ocs2 {
namespace ballbot {

class MRT_Network_Interface final : public MRT_ROS_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = MRT_ROS_Interface<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;
  using dim_t = Dimensions<ballbot::STATE_DIM_, ballbot::INPUT_DIM_>;
  using size_array_t = typename dim_t::size_array_t;
  using scalar_t = typename dim_t::scalar_t;
  using scalar_array_t = typename dim_t::scalar_array_t;
  using state_vector_t = typename dim_t::state_vector_t;
  using state_vector_array_t = typename dim_t::state_vector_array_t;
  using input_vector_t = typename dim_t::input_vector_t;
  using input_vector_array_t = typename dim_t::input_vector_array_t;
  using cost_desired_trajectories_t = typename Base::cost_desired_trajectories_t;

  /**
   * Default constructor
   */
  MRT_Network_Interface() = default;

  MRT_Network_Interface(const std::string& robotName = "Ballbot") : Base(NullLogicRules(), robotName) {}

  virtual ~MRT_Network_Interface() = default;

  void rolloutPolicy(const scalar_t& currentTime, const state_vector_t& currentState, const scalar_t& timeStep, state_vector_t& mpcState,
                     input_vector_t& mpcInput, size_t& subsystem) override;

  void resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) override;

 protected:
  NetworkController<ballbot::STATE_DIM_, ballbot::INPUT_DIM_> ctrl_;
};

}  // namespace ballbot
}  // namespace ocs2
