#include <ocs2_ballbot_example/ros_comm/MRT_Network_Interface.h>

namespace ocs2 {
namespace ballbot {

void MRT_Network_Interface::rolloutPolicy(const scalar_t& currentTime, const state_vector_t& currentState, const scalar_t& timeStep,
                                          state_vector_t& mpcState, input_vector_t& mpcInput, size_t& subsystem) {
  if (!rolloutPtr_) throw std::runtime_error("MRT_ROS_interface: rolloutPtr is not initialized, call initRollout first.");

  const size_t activePartitionIndex = 0;  // there is only one partition.
  scalar_t finalTime = currentTime + timeStep;
  scalar_array_t timeTrajectory;
  size_array_t eventsPastTheEndIndeces;
  state_vector_array_t stateTrajectory;
  input_vector_array_t inputTrajectory;

  rolloutPtr_->run(activePartitionIndex, currentTime, currentState, finalTime, &ctrl_, *logicMachinePtr_, timeTrajectory,
                   eventsPastTheEndIndeces, stateTrajectory, inputTrajectory);

  mpcState = stateTrajectory.back();
  mpcInput = inputTrajectory.back();

  subsystem = 0;
}

void MRT_Network_Interface::resetMpcNode(const cost_desired_trajectories_t& initCostDesiredTrajectories) {
  // TODO reset policy time

  // load the network into the controller here
  ctrl_.loadNetwork("/tmp/mpcPolicy_.pt");
}

}  // namespace ballbot
}  // namespace ocs2

using namespace ocs2;
using namespace ballbot;

int main(int argc, char **argv){

  MRT_Network_Interface interface("ballbot");
  std::cout << interface.mpcIsTerminated() << std::endl;
  return 0;
}
