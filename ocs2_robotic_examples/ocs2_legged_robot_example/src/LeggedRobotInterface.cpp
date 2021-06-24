#include <ocs2_legged_robot_example/LeggedRobotInterface.h>

#include <ocs2_pinocchio_interface/urdf.h>

namespace ocs2 {
namespace legged_robot {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
LeggedRobotInterface::LeggedRobotInterface(const std::string& taskFileFolderName, const ::urdf::ModelInterfaceSharedPtr& urdfTree) {
  pinocchioInterfacePtr_.reset(new PinocchioInterface(buildPinocchioInterface(urdfTree)));

  // Load the task file
  std::string taskFolder = ros::package::getPath("ocs2_legged_robot_example") + "/config/" + taskFileFolderName;
  taskFile_ = taskFolder + "/task.info";
  std::cerr << "Loading task file: " << taskFile_ << std::endl;

  // load setting from loading file
  loadSettings(taskFile_);

  // MPC
  setupOptimizer(taskFile_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::loadSettings(const std::string& taskFile) {
  ddpSettings_ = ocs2::ddp::loadSettings(taskFile);
  mpcSettings_ = ocs2::mpc::loadSettings(taskFile);
  rolloutSettings_ = ocs2::rollout::loadSettings(taskFile, "rollout");
  modelSettings_ = loadModelSettings(taskFile);
  std::cerr << std::endl;

  // Gait Schedule
  std::cerr << std::endl;
  const auto initModeSchedule = loadModeSchedule(taskFile, "initialModeSchedule", false);
  const auto defaultModeSequenceTemplate = loadModeSequenceTemplate(taskFile, "defaultModeSequenceTemplate", false);
  const auto defaultGait = [&] {
    Gait gait{};
    gait.duration = defaultModeSequenceTemplate.switchingTimes.back();
    // Events: from time -> phase
    std::for_each(defaultModeSequenceTemplate.switchingTimes.begin() + 1, defaultModeSequenceTemplate.switchingTimes.end() - 1,
                  [&](double eventTime) { gait.eventPhases.push_back(eventTime / gait.duration); });
    // Modes:
    gait.modeSequence = defaultModeSequenceTemplate.modeSequence;
    return gait;
  }();

  auto gaitSchedule =
      std::make_shared<GaitSchedule>(initModeSchedule, defaultModeSequenceTemplate, modelSettings().phaseTransitionStanceTime_);

  // Swing trajectory planner
  const auto swingTrajectorySettings = loadSwingTrajectorySettings(taskFile);
  std::unique_ptr<SwingTrajectoryPlanner> swingTrajectoryPlanner(new SwingTrajectoryPlanner(swingTrajectorySettings));

  // Mode schedule manager
  modeScheduleManagerPtr_ = std::make_shared<SwitchedModelModeScheduleManager>(std::move(gaitSchedule), std::move(swingTrajectoryPlanner));

  // Display
  std::cerr << "\nInitial Modes Schedule: \n" << initModeSchedule << std::endl;
  std::cerr << "\nDefault Modes Sequence Template: \n" << defaultModeSequenceTemplate << std::endl;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void LeggedRobotInterface::setupOptimizer(const std::string& taskFile) {
  /*
   * Initialization
   */
  initializerPtr_.reset(new LeggedRobotInitializer(*modeScheduleManagerPtr_));
  initialState_.setZero();
  loadData::loadEigenMatrix(taskFile_, "initialState", initialState_);
  std::cerr << "[Cost] " << initialState_.transpose() << "\n";

  const vector_t& qNominal = initialState_.tail(GENERALIZED_VEL_NUM_);
  CentroidalModelInfoTpl<scalar_t> info(*pinocchioInterfacePtr_, CentroidalModelType::FullCentroidalDynamics, qNominal,
                                        LEGGED_ROBOT_3_DOF_CONTACT_NAMES_, LEGGED_ROBOT_6_DOF_CONTACT_NAMES_);
  pinocchioMappingPtr_.reset(new CentroidalModelPinocchioMapping<scalar_t>(info));

  CentroidalModelInfoTpl<ad_scalar_t> infoAD(*pinocchioInterfacePtr_, CentroidalModelType::FullCentroidalDynamics, qNominal,
                                             LEGGED_ROBOT_3_DOF_CONTACT_NAMES_, LEGGED_ROBOT_6_DOF_CONTACT_NAMES_);
  pinocchioMappingAdPtr_.reset(new CentroidalModelPinocchioMapping<ad_scalar_t>(infoAD));

  /*
   * Cost function
   */
  costPtr_.reset(new LeggedRobotCost(*modeScheduleManagerPtr_, *pinocchioInterfacePtr_, *pinocchioMappingPtr_, taskFile_));

  /*
   * Constraints
   */
  bool useAnalyticalGradientsConstraints = false;
  loadData::loadCppDataType(taskFile_, "legged_robot_interface.useAnalyticalGradientsConstraints", useAnalyticalGradientsConstraints);
  if (useAnalyticalGradientsConstraints) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimizer] The analytical constraint class is not yet implemented.");
  } else {
    constraintsPtr_.reset(new LeggedRobotConstraintAD(*modeScheduleManagerPtr_, modeScheduleManagerPtr_->getSwingTrajectoryPlanner(),
                                                      *pinocchioInterfacePtr_, *pinocchioMappingAdPtr_, modelSettings_));
  }

  /*
   * Dynamics
   */
  bool useAnalyticalGradientsDynamics = false;
  loadData::loadCppDataType(taskFile_, "legged_robot_interface.useAnalyticalGradientsDynamics", useAnalyticalGradientsDynamics);
  if (useAnalyticalGradientsDynamics) {
    throw std::runtime_error("[LeggedRobotInterface::setupOptimizer] The analytical dynamics class is not yet implemented.");
  } else {
    dynamicsPtr_.reset(new LeggedRobotDynamicsAD(*pinocchioInterfacePtr_, *pinocchioMappingAdPtr_, "dynamics", "/tmp/ocs2",
                                                 modelSettings_.recompileLibraries_, true));
  }

  /*
   * Rollout
   */
  rolloutPtr_.reset(new TimeTriggeredRollout(*dynamicsPtr_, rolloutSettings_));

  /*
   * Solver
   */
  mpcPtr_.reset(new ocs2::MPC_DDP(rolloutPtr_.get(), dynamicsPtr_.get(), constraintsPtr_.get(), costPtr_.get(), initializerPtr_.get(),
                                  ddpSettings_, mpcSettings_));
  mpcPtr_->getSolverPtr()->setModeScheduleManager(modeScheduleManagerPtr_);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface LeggedRobotInterface::buildPinocchioInterface(const std::string& urdfPath) {
  // Add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

  return ocs2::getPinocchioInterfaceFromUrdfFile(urdfPath, jointComposite);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
PinocchioInterface LeggedRobotInterface::buildPinocchioInterface(const ::urdf::ModelInterfaceSharedPtr& urdfTree) {
  // Add 6 DoF for the floating base
  pinocchio::JointModelComposite jointComposite(2);
  jointComposite.addJoint(pinocchio::JointModelTranslation());
  jointComposite.addJoint(pinocchio::JointModelSphericalZYX());

  // Remove extraneous joints from urdf
  ::urdf::ModelInterfaceSharedPtr newModel = std::make_shared<::urdf::ModelInterface>(*urdfTree);
  for (std::pair<const std::string, std::shared_ptr<::urdf::Joint>>& jointPair : newModel->joints_) {
    if (std::find(JOINT_NAMES_.begin(), JOINT_NAMES_.end(), jointPair.first) == JOINT_NAMES_.end()) {
      jointPair.second->type = urdf::Joint::FIXED;
    }
  }

  return getPinocchioInterfaceFromUrdfModel(newModel, jointComposite);
}

}  // namespace legged_robot
}  // namespace ocs2
