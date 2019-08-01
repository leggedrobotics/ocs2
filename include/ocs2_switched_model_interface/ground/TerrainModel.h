//
// Created by rgrandia on 27.07.19.
//

#ifndef OCS2_CTRL_TERRAINMODEL_H
#define OCS2_CTRL_TERRAINMODEL_H


#include <Eigen/StdVector>

#include <ocs2_mpc/MpcSynchronizedModule.h>

#include "ocs2_core/Trajectory.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/GaitSequence.h"
#include "ConvexPlanarPolytope3d.h"

namespace switched_model {

class TerrainModel final : public ocs2::MpcSynchronizedModule<double> {

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BaseCoordinates = Eigen::Matrix<double, 6, 1>;
  using BaseTrajectory = ocs2::Trajectory<BaseCoordinates>;
  using PolytopeIDsequence = ocs2::Trajectory<int>;

  explicit TerrainModel(std::shared_ptr<const GaitSequence> gaitSequence = nullptr);
  ~TerrainModel() override;

  void update(double initTime, double finalTime, const Eigen::Matrix<double, -1, 1>& currentState, const ocs2::CostDesiredTrajectories<double>& costDesiredTrajectory, const ocs2::HybridLogicRules* hybridLogicRules) override;

  void setPolytopes(ConvexPlanarPolytope3dArray terrainPolytopes);
  const ConvexPlanarPolytope3dArray& getPolytopes() const;

  void loadTestTerrain();

  Eigen::Matrix<double, -1, 4> getTerrainConstraints(double time, int legID) const;

  void display() const;

 private:
  void assignPolytopes();
  int findClosestPolytope(const BaseCoordinates& baseCoordinates, int leg) const;

  /**
   * Finds closest polytope to a certain point
   * @param footPosition
   * @return polytopeID
   */
  int findPolytopeForStanceLeg(const Eigen::Vector3d footPosition);

  /**
   * Finds a suitable next polytope for a leg that is in swing at the start of the optimization horizon
   * Makes a compromise the nominal stance location based on base motion and reachability within remaining time till stance
   *
   * @param startTimeOfStancePhase
   * @param endTimeOfStancePhase
   * @param footPosition : position at start of the optimization horizon
   * @param footVelocity
   * @param leg
   * @return polytopeID
   */
  int findPolytopeForSwingLeg(double startTimeOfStancePhase, double endTimeOfStancePhase, const Eigen::Vector3d footPosition, const Eigen::Vector3d footVelocity, int leg);

  /**
   * Return the polytope closes to the nominal stance position.
   * The nominal stance position is derived from the desired base trajectory.
   * @param startTimeOfStancePhase
   * @param endTimeOfStancePhase
   * @param leg
   * @return
   */
  int findPolytopeForNominalStancePhase(double startTimeOfStancePhase, double endTimeOfStancePhase, int leg);

  /**
   * @param time
   * @param leg
   * @return The interpolated basetrajectory plus a hardcoded deviation per leg
   */
  Eigen::Vector3d getNominalStanceFootPositionInWorld(double time, int leg);

  BaseTrajectory baseTrajectory_;
  ocs2::EigenLinearInterpolation<BaseCoordinates> baseInterpolation_;
  std::vector<Eigen::Vector3d> footPositions_, footVelocities_;

  std::shared_ptr<const GaitSequence> gaitSequence_;

  ConvexPlanarPolytope3dArray terrainPolytopes_;
  std::array<PolytopeIDsequence, 4> polytopeIDSequencePerLeg_;

  double initTime_;
  double finalTime_;
};

} // namespace switched_model


#endif //OCS2_CTRL_TERRAINMODEL_H
