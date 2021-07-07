#include "ocs2_anymal_mpc/AnymalPyBindings.h"

#include <ocs2_core/soft_constraint/penalties/RelaxedBarrierPenalty.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_DDP.h>
#include <ocs2_mpc/MPC_Settings.h>

#include "ocs2_anymal_mpc/AnymalInterface.h"

#include <boost/program_options.hpp>

namespace anymal {

AnymalPyBindings::AnymalPyBindings(std::string varargs) {
  // parsing varargs
  {
    namespace po = boost::program_options;
    auto args = po::split_unix(varargs);
    po::options_description desc("Options");
    desc.add_options()("help", "produce help message and quit")("name,n", po::value<std::string>(&robotName_), "robot name (e.g., croc)")(
        "config,c", po::value<std::string>(&configName_), "configuration name (e.g., c_series)");

    po::command_line_parser parser(args);
    parser.options(desc);
    po::parsed_options parsed_options = parser.run();

    po::variables_map vm;
    po::store(parsed_options, vm);
    po::notify(vm);

    if (vm.count("name") + vm.count("config") == 0 || vm.count("help") > 0) {
      std::cerr << desc << std::endl;
      std::stringstream ss;
      ss << "Varargs wrong when instantiating AnymalPyBindings:\n" << desc << std::endl;
      throw std::runtime_error(ss.str());
    }
  }

  auto anymalInterface = getAnymalInterface(stringToAnymalModel(robotName_), getConfigFolder(configName_));
  const auto mpcSettings = ocs2::mpc::loadSettings(getTaskFilePath(configName_));
  const auto ddpSettings = ocs2::ddp::loadSettings(getTaskFilePath(configName_));

  std::unique_ptr<ocs2::MPC_DDP> mpcPtr(new ocs2::MPC_DDP(
      &anymalInterface->getRollout(), &anymalInterface->getDynamics(), anymalInterface->getConstraintPtr(), &anymalInterface->getCost(),
      &anymalInterface->getInitializer(), ddpSettings, mpcSettings, anymalInterface->getTerminalCostPtr()));
  mpcPtr->getSolverPtr()->setReferenceManager(anymalInterface->getReferenceManagerPtr());

  init(*anymalInterface, std::move(mpcPtr));

  penalty_.reset(new ocs2::RelaxedBarrierPenalty(
      ocs2::RelaxedBarrierPenalty::Config(ddpSettings.inequalityConstraintMu_, ddpSettings.inequalityConstraintDelta_)));
}

void AnymalPyBindings::visualizeTrajectory(const ocs2::scalar_array_t& t, const ocs2::vector_array_t& x, const ocs2::vector_array_t& u,
                                           ocs2::scalar_t speed) {
  if (!visualizer_) {
    auto anymalInterface = getAnymalInterface(stringToAnymalModel(robotName_), getConfigFolder(configName_));
    int fake_argc = 1;
    char arg0[] = "no_name";
    char* fake_argv[] = {arg0};
    ros::init(fake_argc, fake_argv, "anymal_visualization_node");
    ros::NodeHandle n;
    visualizer_.reset(new switched_model::QuadrupedVisualizer(anymalInterface->getKinematicModel(), anymalInterface->getComModel(), n));
  }

  assert(t.size() == x.size());
  std::vector<ocs2::SystemObservation> observations(t.size());

  const bool inputProvided = !u.empty();

  for (int i = 0; i < t.size(); i++) {
    observations[i].time = t[i];
    observations[i].state = x[i];
    if (inputProvided) {
      observations[i].input = u[i];
    }
  }

  visualizer_->publishTrajectory(observations, speed);
}

}  // namespace anymal
