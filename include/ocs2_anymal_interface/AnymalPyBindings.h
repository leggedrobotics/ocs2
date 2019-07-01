#pragma once

#include <ocs2_anymal_interface/OCS2AnymalInterface.h>
#include <ocs2_comm_interfaces/ocs2_interfaces/Python_Interface.h>

namespace anymal {

class AnymalPyBindings final : public ocs2::PythonInterface<OCS2AnymalInterface::BASE::state_dim_, OCS2AnymalInterface::BASE::input_dim_,
                                                            OCS2AnymalInterface::BASE::logic_rules_t> {
 public:
  using Base = ocs2::PythonInterface<OCS2AnymalInterface::BASE::state_dim_, OCS2AnymalInterface::BASE::input_dim_,
                                     OCS2AnymalInterface::BASE::logic_rules_t>;

  AnymalPyBindings(const std::string& taskFileFolder, bool async = false);

  void initRobotInterface(const std::string& taskFileFolder) override;
};

}  // namespace anymal
