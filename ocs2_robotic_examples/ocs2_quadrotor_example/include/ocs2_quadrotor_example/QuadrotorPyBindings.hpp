#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>
#include <ocs2_quadrotor_example/definitions.h>

namespace ocs2 {
namespace quadrotor {

class QuadrotorPyBindings final : public PythonInterface<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_> {
 public:
  using Base = PythonInterface<quadrotor::STATE_DIM_, quadrotor::INPUT_DIM_>;

  QuadrotorPyBindings(const std::string& taskFileFolder, bool async = false);

  void initRobotInterface(const std::string& taskFileFolder) override;
};

}  // namespace quadrotor
}  // namespace ocs2
