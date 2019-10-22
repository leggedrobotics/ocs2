#pragma once

#include <ocs2_anymal_interface/OCS2AnymalInterface.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <ocs2_quadruped_interface/QuadrupedXppVisualizer.h>

namespace anymal {

class AnymalPyBindings final : public ocs2::PythonInterface<OCS2AnymalInterface::BASE::state_dim_, OCS2AnymalInterface::BASE::input_dim_> {
 public:
  using Base = ocs2::PythonInterface<OCS2AnymalInterface::BASE::state_dim_, OCS2AnymalInterface::BASE::input_dim_>;
  using visualizer_t = switched_model::QuadrupedXppVisualizer<switched_model::JOINT_COORDINATE_SIZE,
                                                              OCS2AnymalInterface::BASE::state_dim_, OCS2AnymalInterface::BASE::input_dim_>;

  AnymalPyBindings(const std::string& taskFileFolder, bool async = false);

  void initRobotInterface(const std::string& taskFileFolder) override;

  void visualizeTrajectory(const scalar_array_t& t, const state_vector_array_t& x, const input_vector_array_t& u, double speed) override;

 protected:
  std::unique_ptr<visualizer_t> visualizer_;
  std::string taskFileFolder_;
};

}  // namespace anymal
