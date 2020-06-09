#include <ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_ros_interfaces/RosMsgConversions.h>

namespace switched_model {
namespace ros_msg_conversions {

template <class ContainerAllocator>
void createGaitMsg(const switched_model::Gait& gait, switched_model_msgs::gait_<ContainerAllocator>& gaitMsg) {
  gaitMsg.duration = gait.duration;

  gaitMsg.eventPhases.resize(gait.eventPhases.size());
  for (const auto& phase : gait.eventPhases) {
    gaitMsg.eventPhases.push_back(phase);
  }

  gaitMsg.modeSequence.resize(gait.modeSequence.size());
  for (const auto& mode : gait.modeSequence) {
    gaitMsg.modeSequence.push_back(mode);
  }
}

template <class ContainerAllocator>
void readGaitMsg(const switched_model_msgs::gait_<ContainerAllocator>& gaitMsg, switched_model::Gait& gait) {
  gait.duration = gaitMsg.duration;

  auto Nphases = gaitMsg.eventPhases.size();
  if (Nphases == 0) {
    throw std::runtime_error("An empty target trajectories message is received.");
  }
  gait.eventPhases.assign(gaitMsg.eventPhases.begin(), gaitMsg.eventPhases.end());
  gait.modeSequence.assign(gaitMsg.modeSequence.begin(), gaitMsg.modeSequence.end());
}

template <class ContainerAllocator>
void createScheduledGaitMsg(const switched_model::Gait& gait, scalar_t startTime,
                            switched_model_msgs::scheduled_gait_<ContainerAllocator>& scheduledGaitMsg) {
  createGaitMsg(gait, scheduledGaitMsg.gait);
  scheduledGaitMsg.startTime = startTime;
}

template <class ContainerAllocator>
void readScheduledGaitMsg(const switched_model_msgs::scheduled_gait_<ContainerAllocator>& scheduledGaitMsg, scalar_t& startTime,
                          switched_model::Gait& gait) {
  readGaitMsg(scheduledGaitMsg.gait, gait);
  startTime = scheduledGaitMsg.startTime;
}

template <class ContainerAllocator>
void createGaitSequenceMsg(const switched_model::GaitSchedule::GaitSequence& gaitSequence, const std::vector<scalar_t> startTimes,
                           switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg) {
  if (gaitSequence.size() != startTimes.size()) {
    std::cerr << "creating gaitschedule with bad length of startTimes.";
  }

  auto N = gaitSequence.size();
  gaitSequenceMsg.gaits.resize(N);

  for (auto i = 0; i < N; ++i) {
    createScheduledGaitMsg(gaitSequence[i], startTimes[i], gaitSequenceMsg.gaits[i]);
  }
}

template <class ContainerAllocator>
void readGaitSequenceMsg(const switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
                         switched_model::GaitSchedule::GaitSequence& gaitSequence, std::vector<scalar_t> startTimes) {
  auto N = gaitSequenceMsg.gaits.size();
  gaitSequence.resize(N);
  startTimes.resize(N);

  for (auto i = 0; i < N; ++i) {
    readScheduledGaitMsg(gaitSequenceMsg.gaits[i], startTimes[i], gaitSequence[i]);
  }
}

void createTrajectoryRequestMsg(std::string command,
                                const ocs2::SystemObservation<switched_model::STATE_DIM, switched_model::INPUT_DIM>& observation,
                                const scalar_t offsetTime, switched_model_msgs::trajectory_request::Request& requestMsg) {
  ocs2::ros_msg_conversions::createObservationMsg(observation, requestMsg.observation);
  requestMsg.trajectoryCommand = std::move(command);
  requestMsg.offsetTime = offsetTime;
}

void readTrajectoryRequestMsg(const switched_model_msgs::trajectory_request::Request& requestMsg, std::string& command,
                              ocs2::SystemObservation<switched_model::STATE_DIM, switched_model::INPUT_DIM>& observation,
                              scalar_t& offsetTime) {
  ocs2::ros_msg_conversions::readObservationMsg(requestMsg.observation, observation);
  offsetTime = requestMsg.offsetTime;
  command = requestMsg.trajectoryCommand;
}

void createTrajectoryResponseMsg(const ocs2::CostDesiredTrajectories& costTrajectories,
                                 const switched_model::GaitSchedule::GaitSequence& gaitSequence, std::vector<scalar_t> startTimes,
                                 switched_model_msgs::trajectory_request::Response& responseMsg) {
  ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(costTrajectories, responseMsg.trajectory);
  createGaitSequenceMsg(gaitSequence, startTimes, responseMsg.gaits);
}

void readTrajectoryResponseMsg(const switched_model_msgs::trajectory_request::Response& responseMsg,
                               CostDesiredTrajectories& costTrajectories, switched_model::GaitSchedule::GaitSequence& gaitSequence,
                               std::vector<scalar_t> startTimes) {
  ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(responseMsg.trajectory, costTrajectories);
  readGaitSequenceMsg(responseMsg.gaits, gaitSequence, startTimes);
}

}  // namespace ros_msg_conversions
}  // namespace switched_model
