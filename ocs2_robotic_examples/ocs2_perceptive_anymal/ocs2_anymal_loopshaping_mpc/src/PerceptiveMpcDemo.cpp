//
// Created by rgrandia on 31.03.22.
//

#include <ros/init.h>
#include <ros/node_handle.h>

#include <grid_map_ros/GridMapRosConverter.hpp>

// Plane segmentation
#include <convex_plane_decomposition/LoadGridmapFromImage.h>
#include <convex_plane_decomposition/PlaneDecompositionPipeline.h>
#include <convex_plane_decomposition_ros/ParameterLoading.h>
#include <convex_plane_decomposition_ros/RosVisualizations.h>

// ocs2_dev
#include <ocs2_mpc/MPC_MRT_Interface.h>
#include <ocs2_oc/oc_data/LoopshapingPrimalSolution.h>
#include <ocs2_ros_interfaces/visualization/VisualizationHelpers.h>

// ocs2_anymal
#include <ocs2_anymal_commands/ReferenceExtrapolation.h>
#include <ocs2_anymal_loopshaping_mpc/AnymalLoopshapingInterface.h>
#include <ocs2_anymal_models/AnymalModels.h>
#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>
#include <ocs2_quadruped_loopshaping_interface/QuadrupedLoopshapingMpc.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>
#include <ocs2_switched_model_interface/core/Rotations.h>
#include <segmented_planes_terrain_model/SegmentedPlanesTerrainModel.h>

using namespace switched_model;

int main(int argc, char* argv[]) {
  const std::string path(__FILE__);
  const std::string ocs2_anymal = path.substr(0, path.find_last_of("/")) + "/../../";
  const std::string taskFolder = ocs2_anymal + "ocs2_anymal_loopshaping_mpc/config/";
  const std::string terrainFolder = ocs2_anymal + "ocs2_anymal_loopshaping_mpc/data/";

  const bool use_ros = false;

  std::string urdfString = getUrdfString(anymal::AnymalModel::Camel);
  std::string configName = "c_series";
  const std::string robotName = "anymal";

  ros::init(argc, argv, "anymal_perceptive_mpc_demo");
  ros::NodeHandle nodeHandle;

  ros::param::get("ocs2_anymal_description", urdfString);
  nodeHandle.getParam("/config_name", configName);

  convex_plane_decomposition::PlaneDecompositionPipeline::Config perceptionConfig;
  perceptionConfig.preprocessingParameters =
      convex_plane_decomposition::loadPreprocessingParameters(nodeHandle, "/ocs2_anymal_loopshaping_mpc_perceptive_demo/preprocessing/");
  perceptionConfig.contourExtractionParameters = convex_plane_decomposition::loadContourExtractionParameters(
      nodeHandle, "/ocs2_anymal_loopshaping_mpc_perceptive_demo/contour_extraction/");
  perceptionConfig.ransacPlaneExtractorParameters = convex_plane_decomposition::loadRansacPlaneExtractorParameters(
      nodeHandle, "/ocs2_anymal_loopshaping_mpc_perceptive_demo/ransac_plane_refinement/");
  perceptionConfig.slidingWindowPlaneExtractorParameters = convex_plane_decomposition::loadSlidingWindowPlaneExtractorParameters(
      nodeHandle, "/ocs2_anymal_loopshaping_mpc_perceptive_demo/sliding_window_plane_extractor/");
  perceptionConfig.postprocessingParameters =
      convex_plane_decomposition::loadPostprocessingParameters(nodeHandle, "/ocs2_anymal_loopshaping_mpc_perceptive_demo/postprocessing/");

  auto anymalInterface = anymal::getAnymalLoopshapingInterface(urdfString, taskFolder + configName);
  const ocs2::vector_t initSystemState = anymalInterface->getInitialState().head(switched_model::STATE_DIM);

  // ====== Scenario settings ========
  ocs2::scalar_t forwardVelocity{0.0};
  nodeHandle.getParam("/forward_velocity", forwardVelocity);
  ocs2::scalar_t gaitDuration{0.8};
  ocs2::scalar_t forwardDistance{3.0};

  ocs2::scalar_t initTime = 0.0;
  ocs2::scalar_t stanceTime = 1.0;
  int numGaitCycles = std::ceil((forwardDistance / forwardVelocity) / gaitDuration);
  ocs2::scalar_t walkTime = numGaitCycles * gaitDuration;
  ocs2::scalar_t finalTime = walkTime + 2 * stanceTime;
  ocs2::scalar_t f_mpc = 100;  // hz

  // Load a map
  const std::string elevationLayer{"elevation"};
  const std::string frameId{"world"};
  std::string terrainFile{""};
  nodeHandle.getParam("/terrain_name", terrainFile);
  double heightScale{1.0};
  nodeHandle.getParam("/terrain_scale", heightScale);
  auto gridMap = convex_plane_decomposition::loadGridmapFromImage(terrainFolder + "/" + terrainFile, elevationLayer, frameId,
                                                                  perceptionConfig.preprocessingParameters.resolution, heightScale);
  gridMap.get(elevationLayer).array() -= gridMap.atPosition(elevationLayer, {0., 0.});

  // Gait
  using MN = switched_model::ModeNumber;
  switched_model::Gait stance;
  stance.duration = stanceTime;
  stance.eventPhases = {};
  stance.modeSequence = {MN::STANCE};

  switched_model::Gait gait;
  gait.duration = gaitDuration;
  gait.eventPhases = {0.5};
  gait.modeSequence = {MN::LF_RH, MN::RF_LH};

  GaitSchedule::GaitSequence gaitSequence{stance};
  for (int i = 0; i < numGaitCycles; ++i) {
    gaitSequence.push_back(gait);
  }
  gaitSequence.push_back(stance);

  // Reference trajectory
  bool adaptReferenceToTerrain = true;
  nodeHandle.getParam("/adaptReferenceToTerrain", adaptReferenceToTerrain);

  const double dtRef = 0.1;
  const BaseReferenceHorizon commandHorizon{dtRef, size_t(walkTime / dtRef) + 1};
  BaseReferenceCommand command;
  command.baseHeight = getPositionInOrigin(getBasePose(initSystemState)).z();
  command.yawRate = 0.0;
  command.headingVelocity = forwardVelocity;
  command.lateralVelocity = 0.0;

  // ====== Run the perception pipeline ========
  convex_plane_decomposition::PlaneDecompositionPipeline planeDecompositionPipeline(perceptionConfig);
  planeDecompositionPipeline.update(grid_map::GridMap(gridMap), elevationLayer);
  auto& planarTerrain = planeDecompositionPipeline.getPlanarTerrain();
  auto terrainModel = std::unique_ptr<SegmentedPlanesTerrainModel>(new SegmentedPlanesTerrainModel(planarTerrain));

  // Read min-max from elevation map
  const float heightMargin = 0.5;  // Create SDF till this amount above and below the map.
  const auto& elevationData = gridMap.get(elevationLayer);
  const float minValue = elevationData.minCoeffOfFinites() - heightMargin;
  const float maxValue = elevationData.maxCoeffOfFinites() + heightMargin;
  terrainModel->createSignedDistanceBetween({-1e30, -1e30, minValue}, {1e30, 1e30, maxValue});  // will project XY range to map limits

  // ====== Generate reference trajectory ========
  const auto& baseToHipInBase = anymalInterface->getKinematicModel().baseToLegRootInBaseFrame(0);
  const double nominalStanceWidthInHeading = 2.0 * (std::abs(baseToHipInBase.x()) + 0.15);
  const double nominalStanceWidthLateral = 2.0 * (std::abs(baseToHipInBase.y()) + 0.10);

  BaseReferenceState initialBaseState{stanceTime, getPositionInOrigin(getBasePose(initSystemState)),
                                      getOrientation(getBasePose(initSystemState))};

  BaseReferenceTrajectory terrainAdaptedBaseReference;
  if (adaptReferenceToTerrain) {
    terrainAdaptedBaseReference = generateExtrapolatedBaseReference(commandHorizon, initialBaseState, command, planarTerrain.gridMap,
                                                                    nominalStanceWidthInHeading, nominalStanceWidthLateral);
  } else {
    terrainAdaptedBaseReference = generateExtrapolatedBaseReference(commandHorizon, initialBaseState, command, TerrainPlane());
  }

  ocs2::TargetTrajectories targetTrajectories;
  targetTrajectories.timeTrajectory.push_back(initTime);
  targetTrajectories.timeTrajectory.push_back(stanceTime);
  targetTrajectories.stateTrajectory.push_back(initSystemState);
  targetTrajectories.stateTrajectory.push_back(initSystemState);
  targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
  targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
  for (int k = 0; k < terrainAdaptedBaseReference.time.size(); ++k) {
    targetTrajectories.timeTrajectory.push_back(terrainAdaptedBaseReference.time[k]);

    const auto R_WtoB = rotationMatrixOriginToBase(terrainAdaptedBaseReference.eulerXyz[k]);

    Eigen::VectorXd costReference(switched_model::STATE_DIM);
    costReference << terrainAdaptedBaseReference.eulerXyz[k], terrainAdaptedBaseReference.positionInWorld[k],
        R_WtoB * terrainAdaptedBaseReference.angularVelocityInWorld[k], R_WtoB * terrainAdaptedBaseReference.linearVelocityInWorld[k],
        getJointPositions(initSystemState);
    targetTrajectories.stateTrajectory.push_back(std::move(costReference));
    targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
  }
  targetTrajectories.timeTrajectory.push_back(stanceTime + walkTime);
  targetTrajectories.timeTrajectory.push_back(finalTime);
  targetTrajectories.stateTrajectory.push_back(targetTrajectories.stateTrajectory.back());
  targetTrajectories.stateTrajectory.push_back(targetTrajectories.stateTrajectory.back());
  targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));
  targetTrajectories.inputTrajectory.push_back(vector_t::Zero(INPUT_DIM));

  // ====== Set the scenario to the correct interfaces ========
  auto referenceManager = anymalInterface->getQuadrupedInterface().getSwitchedModelModeScheduleManagerPtr();

  // Register the terrain model
  referenceManager->getTerrainModel().reset(std::move(terrainModel));

  // Register the gait
  referenceManager->getGaitSchedule()->setGaitSequenceAtTime(gaitSequence, initTime);

  // Register the target trajectory
  referenceManager->setTargetTrajectories(targetTrajectories);

  // ====== Create MPC solver ========
  const auto mpcSettings = ocs2::mpc::loadSettings(taskFolder + configName + "/task.info");

  std::unique_ptr<ocs2::MPC_BASE> mpcPtr;
  const auto sqpSettings = ocs2::sqp::loadSettings(taskFolder + configName + "/multiple_shooting.info");
  switch (anymalInterface->modelSettings().algorithm_) {
    case switched_model::Algorithm::DDP: {
      const auto ddpSettings = ocs2::ddp::loadSettings(taskFolder + configName + "/task.info");
      mpcPtr = getDdpMpc(*anymalInterface, mpcSettings, ddpSettings);
      break;
    }
    case switched_model::Algorithm::SQP: {
      mpcPtr = getSqpMpc(*anymalInterface, mpcSettings, sqpSettings);
      break;
    }
  }
  ocs2::MPC_MRT_Interface mpcInterface(*mpcPtr);

  std::unique_ptr<ocs2::RolloutBase> rollout(anymalInterface->getRollout().clone());

  // ====== Execute the scenario ========
  ocs2::SystemObservation observation;
  observation.time = initTime;
  observation.state = anymalInterface->getInitialState();
  observation.input.setZero(switched_model_loopshaping::INPUT_DIM);

  // Wait for the first policy
  mpcInterface.setCurrentObservation(observation);
  while (!mpcInterface.initialPolicyReceived()) {
    mpcInterface.advanceMpc();
  }

  // run MPC till final time
  ocs2::PrimalSolution closedLoopSolution;
  std::vector<ocs2::PerformanceIndex> performances;
  while (observation.time < finalTime) {
    std::cout << "t: " << observation.time << "\n";
    try {
      // run MPC at current observation
      mpcInterface.setCurrentObservation(observation);
      mpcInterface.advanceMpc();
      mpcInterface.updatePolicy();

      performances.push_back(mpcInterface.getPerformanceIndices());

      // Evaluate the optimized solution - change to optimal controller
      ocs2::vector_t tmp;
      mpcInterface.evaluatePolicy(observation.time, observation.state, tmp, observation.input, observation.mode);
      observation.input = ocs2::LinearInterpolation::interpolate(observation.time, mpcInterface.getPolicy().timeTrajectory_,
                                                                 mpcInterface.getPolicy().inputTrajectory_);

      closedLoopSolution.timeTrajectory_.push_back(observation.time);
      closedLoopSolution.stateTrajectory_.push_back(observation.state);
      closedLoopSolution.inputTrajectory_.push_back(observation.input);
      if (closedLoopSolution.modeSchedule_.modeSequence.empty()) {
        closedLoopSolution.modeSchedule_.modeSequence.push_back(observation.mode);
      } else if (closedLoopSolution.modeSchedule_.modeSequence.back() != observation.mode) {
        closedLoopSolution.modeSchedule_.modeSequence.push_back(observation.mode);
        closedLoopSolution.modeSchedule_.eventTimes.push_back(observation.time - 0.5 / f_mpc);
      }

      // perform a rollout
      scalar_array_t timeTrajectory;
      size_array_t postEventIndicesStock;
      vector_array_t stateTrajectory, inputTrajectory;
      const scalar_t finalTime = observation.time + 1.0 / f_mpc;
      auto modeschedule = mpcInterface.getPolicy().modeSchedule_;
      rollout->run(observation.time, observation.state, finalTime, mpcInterface.getPolicy().controllerPtr_.get(), modeschedule,
                   timeTrajectory, postEventIndicesStock, stateTrajectory, inputTrajectory);

      observation.time = finalTime;
      observation.state = stateTrajectory.back();
      observation.input.setZero(switched_model_loopshaping::INPUT_DIM);  // reset
    } catch (std::exception& e) {
      std::cout << "MPC failed\n";
      std::cout << e.what() << "\n";
      break;
    }
  }
  const auto closedLoopSystemSolution =
      ocs2::loopshapingToSystemPrimalSolution(closedLoopSolution, *anymalInterface->getLoopshapingDefinition());

  // ====== Print result ==========
  const auto totalCost = std::accumulate(performances.cbegin(), performances.cend(), 0.0,
                                         [](double v, const ocs2::PerformanceIndex& p) { return v + p.cost; });
  const auto totalDynamics =
      std::accumulate(performances.cbegin(), performances.cend(), 0.0,
                      [](double v, const ocs2::PerformanceIndex& p) { return v + std::sqrt(p.dynamicsViolationSSE); });
  const auto maxDynamics = std::sqrt(std::max_element(performances.cbegin(), performances.cend(),
                                                      [](const ocs2::PerformanceIndex& lhs, const ocs2::PerformanceIndex& rhs) {
                                                        return lhs.dynamicsViolationSSE < rhs.dynamicsViolationSSE;
                                                      })
                                         ->dynamicsViolationSSE);
  const auto totalEquality =
      std::accumulate(performances.cbegin(), performances.cend(), 0.0,
                      [](double v, const ocs2::PerformanceIndex& p) { return v + std::sqrt(p.equalityConstraintsSSE); });
  const auto maxEquality = std::sqrt(std::max_element(performances.cbegin(), performances.cend(),
                                                      [](const ocs2::PerformanceIndex& lhs, const ocs2::PerformanceIndex& rhs) {
                                                        return lhs.equalityConstraintsSSE < rhs.equalityConstraintsSSE;
                                                      })
                                         ->equalityConstraintsSSE);

  double achievedWalkTime = observation.time - stanceTime;
  std::cout << "Speed: " << forwardVelocity << "\n";
  std::cout << "Scale: " << heightScale << "\n";
  std::cout << "Completed: " << std::min(1.0, (achievedWalkTime / walkTime)) * 100.0 << "\n";
  std::cout << "average Cost: " << totalCost / performances.size() << "\n";
  std::cout << "average Dynamics constr: " << totalDynamics / performances.size() << "\n";
  std::cout << "max Dynamics constr: " << maxDynamics << "\n";
  std::cout << "average Equality constr: " << totalEquality / performances.size() << "\n";
  std::cout << "max Equality constr: " << maxEquality << "\n";

  // ====== Visualize ==========
  QuadrupedVisualizer visualizer(anymalInterface->getKinematicModel(), anymalInterface->getJointNames(), anymalInterface->getBaseName(),
                                 nodeHandle);
  ros::Publisher elevationmapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("elevation_map", 1);
  ros::Publisher filteredmapPublisher = nodeHandle.advertise<grid_map_msgs::GridMap>("filtered_map", 1);
  ros::Publisher boundaryPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("boundaries", 1);
  ros::Publisher insetPublisher = nodeHandle.advertise<visualization_msgs::MarkerArray>("insets", 1);
  ros::Publisher distanceFieldPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("signed_distance_field", 1, true);

  // Create pointcloud for visualization (terrain model ownership is now with the swing planner
  const auto* sdfPtr =
      dynamic_cast<const SegmentedPlanesSignedDistanceField*>(referenceManager->getSwingTrajectoryPlanner().getSignedDistanceField());
  sensor_msgs::PointCloud2 pointCloud2Msg;
  if (sdfPtr != nullptr) {
    const auto& sdf = sdfPtr->asGridmapSdf();
    grid_map::GridMapRosConverter::toPointCloud(sdf, pointCloud2Msg, 1, [](float val) { return val <= 0.0F; });
  }

  // Grid map
  grid_map_msgs::GridMap filteredMapMessage;
  grid_map::GridMapRosConverter::toMessage(planarTerrain.gridMap, filteredMapMessage);
  grid_map_msgs::GridMap elevationMapMessage;
  grid_map::GridMapRosConverter::toMessage(gridMap, elevationMapMessage);

  // Segmentation
  const double lineWidth = 0.005;  // [m] RViz marker size
  auto boundaries = convertBoundariesToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                  planarTerrain.gridMap.getTimestamp(), lineWidth);
  auto boundaryInsets = convertInsetsToRosMarkers(planarTerrain.planarRegions, planarTerrain.gridMap.getFrameId(),
                                                  planarTerrain.gridMap.getTimestamp(), lineWidth);

  while (ros::ok()) {
    visualizer.publishOptimizedStateTrajectory(ros::Time::now(), closedLoopSystemSolution.timeTrajectory_,
                                               closedLoopSystemSolution.stateTrajectory_, closedLoopSystemSolution.modeSchedule_);
    visualizer.publishDesiredTrajectory(ros::Time::now(), targetTrajectories);

    filteredmapPublisher.publish(filteredMapMessage);
    elevationmapPublisher.publish(elevationMapMessage);
    boundaryPublisher.publish(boundaries);
    insetPublisher.publish(boundaryInsets);

    if (sdfPtr != nullptr) {
      distanceFieldPublisher.publish(pointCloud2Msg);
    }

    // Visualize the individual execution
    double speed = 1.0;
    for (size_t k = 0; k < closedLoopSystemSolution.timeTrajectory_.size() - 1; k++) {
      double frameDuration = speed * (closedLoopSystemSolution.timeTrajectory_[k + 1] - closedLoopSystemSolution.timeTrajectory_[k]);
      double publishDuration = ocs2::timedExecutionInSeconds([&]() {
        ocs2::SystemObservation observation;
        observation.time = closedLoopSystemSolution.timeTrajectory_[k];
        observation.state = closedLoopSystemSolution.stateTrajectory_[k];
        observation.input = closedLoopSystemSolution.inputTrajectory_[k];
        observation.mode = closedLoopSystemSolution.modeSchedule_.modeAtTime(observation.time);
        visualizer.publishObservation(ros::Time::now(), observation);
      });
      if (frameDuration > publishDuration) {
        ros::WallDuration(frameDuration - publishDuration).sleep();
      }
    }

    ros::WallDuration(1.0).sleep();
  }
}
