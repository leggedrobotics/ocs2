//
// Created by rgrandia on 13.03.20.
//

#pragma once

#include <ocs2_core/Types.h>
#include <ocs2_core/reference/TargetTrajectories.h>

#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/logic/SingleLegLogic.h"
#include "ocs2_switched_model_interface/terrain/ConvexTerrain.h"
#include "ocs2_switched_model_interface/terrain/TerrainModel.h"
#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

namespace switched_model {

struct SwingTrajectoryPlannerSettings {
  scalar_t liftOffVelocity = 0.0;
  scalar_t touchDownVelocity = 0.0;
  scalar_t swingHeight = 0.1;

  scalar_t errorGain = 0.0;          // proportional gain for returning to the planned swing trajectory. 10-90%-rise_time ~= 2.2 / errorGain
                                     // alternatively can be measured as (velocity feedback) / (tracking error) ([m/s] / [m])
  scalar_t swingTimeScale = 0.15;    // swing phases shorter than this time will be scaled down in height and velocity
  scalar_t sdfMidswingMargin = 0.0;  // desired sdf based clearance in the middle of the swing phase [m]
  scalar_t terrainMargin = 0.0;      // shrinkage of the convex terrain constrains in [m]

  scalar_t previousFootholdFactor = 0.0;        // factor in [0, 1] with which to take previous foothold into account.
  scalar_t previousFootholdDeadzone = 0.0;      // previous foothold is taken if the new reference is within this threshold. [m]
  scalar_t previousFootholdTimeDeadzone = 0.0;  // previous foothold is taken if the contact phase is starting withing this time. [s]
  scalar_t invertedPendulumHeight = 0.5;        // height used for the inverted pendulum foothold adjustment

  scalar_t nominalLegExtension = 0.55;     // Leg extension beyond this length [m] will be penalized in terrain selection
  scalar_t legOverExtensionPenalty = 5.0;  // Weight of the leg overextension penalty

  scalar_t referenceExtensionAfterHorizon = 1.0;  // base and foot references generated for this amount of seconds after the horizon ends.

  bool swingTrajectoryFromReference = false;  // Flag to take the swing trajectory from the reference trajectory
};

SwingTrajectoryPlannerSettings loadSwingTrajectorySettings(const std::string& filename, bool verbose = true);

using inverse_kinematics_function_t = std::function<vector3_t(int, const vector3_t&)>;

class SwingTrajectoryPlanner {
 public:
  SwingTrajectoryPlanner(SwingTrajectoryPlannerSettings settings, const KinematicsModelBase<scalar_t>& kinematicsModel);

  void updateTerrain(std::unique_ptr<TerrainModel> terrainModel);

  void updateSwingMotions(scalar_t initTime, scalar_t finalTime, const comkino_state_t& currentState,
                          const ocs2::TargetTrajectories& targetTrajectories,
                          const feet_array_t<std::vector<ContactTiming>>& contactTimingsPerLeg);

  void adaptJointReferencesWithInverseKinematics(const inverse_kinematics_function_t& inverseKinematicsFunction, scalar_t finalTime);
  const ocs2::TargetTrajectories& getTargetTrajectories() const { return targetTrajectories_; }

  const FootPhase& getFootPhase(size_t leg, scalar_t time) const;

  joint_coordinate_t getJointPositionsReference(scalar_t time) const;
  joint_coordinate_t getJointVelocitiesReference(scalar_t time) const;

  std::vector<ConvexTerrain> getNominalFootholds(size_t leg) const { return nominalFootholdsPerLeg_[leg]; }
  std::vector<vector3_t> getHeuristicFootholds(size_t leg) const { return heuristicFootholdsPerLeg_[leg]; }

  const SignedDistanceField* getSignedDistanceField() const;

  const SwingTrajectoryPlannerSettings& settings() const { return settings_; }

 private:
  void updateLastContact(int leg, scalar_t expectedLiftOff, const vector3_t& currentFootPosition, const TerrainModel& terrainModel);

  std::pair<std::vector<scalar_t>, std::vector<std::unique_ptr<FootPhase>>> generateSwingTrajectories(
      int leg, const std::vector<ContactTiming>& contactTimings, scalar_t finalTime) const;

  std::vector<vector3_t> selectHeuristicFootholds(int leg, const std::vector<ContactTiming>& contactTimings,
                                                  const ocs2::TargetTrajectories& targetTrajectories, scalar_t initTime,
                                                  const comkino_state_t& currentState, scalar_t finalTime) const;

  std::vector<ConvexTerrain> selectNominalFootholdTerrain(int leg, const std::vector<ContactTiming>& contactTimings,
                                                          const std::vector<vector3_t>& heuristicFootholds,
                                                          const ocs2::TargetTrajectories& targetTrajectories, scalar_t initTime,
                                                          const comkino_state_t& currentState, scalar_t finalTime,
                                                          const TerrainModel& terrainModel) const;

  void applySwingMotionScaling(SwingPhase::SwingEvent& liftOff, SwingPhase::SwingEvent& touchDown,
                               SwingPhase::SwingProfile& swingProfile) const;

  std::vector<SwingPhase::SwingProfile::Node> extractSwingProfileFromReference(int leg, const SwingPhase::SwingEvent& liftoff,
                                                                               const SwingPhase::SwingEvent& touchdown) const;

  void setDefaultSwingProfile();

  scalar_t getContactEndTime(const ContactTiming& contactPhase, scalar_t finalTime) const;

  const FootPhase* getFootPhaseIfInContact(size_t leg, scalar_t time) const;

  vector3_t filterFoothold(const vector3_t& newFoothold, const vector3_t& previousFoothold) const;

  SwingTrajectoryPlannerSettings settings_;
  SwingPhase::SwingProfile defaultSwingProfile_;
  std::unique_ptr<KinematicsModelBase<scalar_t>> kinematicsModel_;

  feet_array_t<std::pair<scalar_t, TerrainPlane>> lastContacts_;
  feet_array_t<std::vector<std::unique_ptr<FootPhase>>> feetNormalTrajectories_;
  feet_array_t<std::vector<scalar_t>> feetNormalTrajectoriesEvents_;

  feet_array_t<std::vector<ConvexTerrain>> nominalFootholdsPerLeg_;
  feet_array_t<std::vector<vector3_t>> heuristicFootholdsPerLeg_;
  std::unique_ptr<TerrainModel> terrainModel_;

  ocs2::TargetTrajectories targetTrajectories_;
};

}  // namespace switched_model
