#include "ocs2_double_slit_example/DoubleSlitInterface.h"
#include "ocs2_double_slit_example/definitions.h"
#include "ocs2_double_slit_example/ros_comm/MRT_ROS_Double_Slit.h"
#include "ocs2_double_slit_example/ros_comm/MRT_ROS_Dummy_Double_Slit.h"

int main(int argc, char* argv[]) {
  using DoubleSlitInterface = ocs2::double_slit::DoubleSlitInterface;
  using MrtRosDummyDoubleSlit = ocs2::double_slit::MrtRosDummyDoubleSlit;
  using mrt_t = ocs2::double_slit::MrtRosDoubleSlit;

  // task file
  if (argc <= 1) {
    throw std::runtime_error("No task file specified. Aborting.");
  }
  std::string taskFileFolderName = std::string(argv[1]);  // NOLINT(cppcoreguidelines-pro-bounds-pointer-arithmetic)

  // double_slitInterface
  DoubleSlitInterface double_slitInterface(taskFileFolderName);
  double rollout_dt;
  ocs2::loadScalar(double_slitInterface.taskFile_, "pathIntegral.rollout_settings.minTimeStep", rollout_dt);

  using mrt_ptr_t = mrt_t::Ptr;
  using scalar_t = mrt_t::scalar_t;
  using system_observation_t = mrt_t::system_observation_t;

  mrt_ptr_t mrtPtr(new mrt_t("double_slit"));

  // Dummy double_slit
  MrtRosDummyDoubleSlit dummyDoubleSlit(mrtPtr, double_slitInterface.mpcSettings().mrtDesiredFrequency_,
                                        double_slitInterface.mpcSettings().mpcDesiredFrequency_,
                                        double_slitInterface.getDynamicsPtr().get(),
                                        ocs2::Rollout_Settings(1e-9, 1e-6, 5000, rollout_dt, ocs2::IntegratorType::EULER, false, true));

  dummyDoubleSlit.launchNodes(argc, argv);

  // Run dummy
  MrtRosDummyDoubleSlit::system_observation_t initObservation;
  double_slitInterface.getInitialState(initObservation.state());

  MrtRosDummyDoubleSlit::cost_desired_trajectories_t initCostDesiredTraj;
  initCostDesiredTraj.desiredTimeTrajectory().push_back(0.0);
  initCostDesiredTraj.desiredInputTrajectory().push_back(DoubleSlitInterface::input_vector_t::Zero());
  initCostDesiredTraj.desiredStateTrajectory().push_back(DoubleSlitInterface::state_vector_t::Zero());

  dummyDoubleSlit.run(initObservation, initCostDesiredTraj);

  // Successful exit
  return 0;
}
