#include <ocs2_comm_interfaces/ocs2_ros_interfaces/common/RosMsgConversions.h>
#include <ocs2_switched_model_ros_interfaces/RosMsgConversions.h>

namespace switched_model {
namespace ros_msg_conversions {

template <class ContainerAllocator>
void createGaitMsg(const switched_model::Gait& gait, switched_model_msgs::gait_<ContainerAllocator>& gaitMsg) {
  gaitMsg.duration = gait.duration;

  gaitMsg.eventPhases.value.resize(gait.eventPhases().size());
  for (const auto& phase : gait.eventPhases) {
    gaitMsg.eventPhases.value.assign = phase;
  }

  gaitMsg.modeSequence.value.resize(gait.modeSequence().size());
  for (const auto& mode : gait.modeSequence) {
    gaitMsg.modeSequence.value.assign = mode;
  }
}

template <class ContainerAllocator>
void readGaitMsg(const switched_model_msgs::gait_<ContainerAllocator>& gaitMsg, switched_model::Gait& gait) {
  gait.duration = gaitMsg.duration;

  auto Nphases = gaitMsg.eventPhases.size();
  if (N == 0) {
    throw std::runtime_error("An empty target trajectories message is received.");
  }
  gait.eventPhases.assign(gaitMsg.eventPhases.data.begin(), gaitMsg.eventPhases.data.end());
  gait.modeSequence.assign(gaitMsg.modeSequence.data.begin(), gaitMsg.modeSequence.data.end());
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
  startTime = scheduledGaitMsg.startTime.value;
}

template <class ContainerAllocator>
void createGaitSequenceMsg(const switched_model::GaitSchedule::GaitSequence& gaitSequence, const std::vector<scalar_t> startTimes,
                           switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg) {
  if (gaitSequence.size() != startTimes.size()) {
    std::cerr << "creating gaitschedule with bad length of startTimes.";
  }

  auto N = gaitSequence.size();
  gaitSequenceMsg.scheduled_gaits.resize(N);

  for (auto i = 0; i < N; ++i) {
    createScheduledGaitMsg(gaitSequence[i], startTimes[i], gaitSequenceMsg.scheduled_gaits[i]);
  }
}

template <class ContainerAllocator>
void readGaitSequenceMsg(const switched_model_msgs::gait_sequence_<ContainerAllocator>& gaitSequenceMsg,
                         switched_model::GaitSchedule::GaitSequence& gaitSequence, std::vector<scalar_t> startTimes) {
  auto N = gaitSequenceMsg.size();
  gaitSequence.resize(N);
  startTimes.resize(N);

  for (auto i = 0; i < N; ++i) {
    readScheduledGaitMsg(gaitSequence[i], gaitSequence[i], startTimes[i]);
  }
}

template <class ContainerAllocator>
void createTrajectoryRequestMsg(const ocs2::SystemObservation<switched_model::STATE_DIM, switched_model::INPUT_DIM>& observation,
                                const scalar_t offsetTime,
                                switched_model_srvs::trajectory_request::Request<ContainerAllocator>& requestMsg) {
  ocs2::ros_msg_conversions::createObservationMsg(observation, requestMsg.observation);
  requestMsg.trajectoryCommand.data = command;
  requestMsg.offsetTime.value = offsetTime;
}

template <class ContainerAllocator>
void readTrajectoryRequestMsg(const switched_model_srvs::trajectory_request::Request<ContainerAllocator>& requestMsg,
                              ocs2::SystemObservation<switched_model::STATE_DIM, switched_model::INPUT_DIM>& observation,
                              std::string& command, scalar_t& offsetTime) {
  ocs2::ros_msg_conversions::readObservationMsg(requestMsg.observation, observation);
  offsetTime = requestMsg.offsetTime.value;
  requestMsg.trajectoryCommand.data;
}

template <class ContainerAllocator>
void createTrajectoryResponseMsg(const switched_model::GaitSchedule::GaitSequence& gaitSequence,
                                 const ocs2::CostDesiredTrajectories& costTrajectories, std::vector<scalar_t> startTimes,
                                 switched_model_srvs::trajectory_request::Response<ContainerAllocator>& responseMsg) {
  ocs2::ros_msg_conversions::createTargetTrajectoriesMsg(costTrajectories, responseMsg.trajectory);
  createGaitSequenceMsg(gaitSequence, startTimes, responseMsg.gaits);
}

template <class ContainerAllocator>
void readTrajectoryResponseMsg(const switched_model_srvs::trajectory_request::Response<ContainerAllocator>& responseMsg,
                               CostDesiredTrajectories& costTrajectories, switched_model::GaitSchedule::GaitSequence& gaitSequence,
                               std::vector<scalar_t> startTimes) {
  ocs2::ros_msg_conversions::readTargetTrajectoriesMsg(responseMsg.trajectory, costTrajectories);
  readGaitSequenceMsg(responseMsg.gaits, gaitSequence, startTimes);
}

}  // namespace ros_msg_conversions
}  // namespace switched_model
