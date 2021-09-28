//
// Created by rgrandia on 21.09.21.
//

#pragma once

#include <ocs2_core/PreComputation.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/KinematicsModelBase.h"
#include "ocs2_switched_model_interface/core/ModelSettings.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/foot_planner/FootPhase.h"
#include "ocs2_switched_model_interface/foot_planner/SwingTrajectoryPlanner.h"
#include "ocs2_switched_model_interface/terrain/SignedDistanceField.h"

namespace switched_model {

class SwitchedModelPreComputationMockup;

class SwitchedModelPreComputation : public ocs2::PreComputation {
 public:
  using com_model_t = ComModelBase<scalar_t>;
  using ad_com_model_t = ComModelBase<ad_scalar_t>;
  using kinematic_model_t = KinematicsModelBase<ocs2::scalar_t>;
  using ad_kinematic_model_t = KinematicsModelBase<ad_scalar_t>;

  SwitchedModelPreComputation(const SwingTrajectoryPlanner& swingTrajectoryPlanner, const kinematic_model_t& kinematicModel,
                              const ad_kinematic_model_t& adKinematicModel, const com_model_t& comModel, const ad_com_model_t& adComModel,
                              ModelSettings settings);

  ~SwitchedModelPreComputation() override = default;

  SwitchedModelPreComputation* clone() const override { return new SwitchedModelPreComputation(*this); }

  void request(ocs2::RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override;

  void requestPreJump(ocs2::RequestSet request, scalar_t t, const vector_t& x) override;

  void requestFinal(ocs2::RequestSet request, scalar_t t, const vector_t& x) override;

  // Precomputation access : always available
  const FootPhase& getFootPhase(size_t leg) const { return *feetPhases_[leg]; }
  const SignedDistanceField* getSignedDistanceField() const { return swingTrajectoryPlannerPtr_->getSignedDistanceField(); }
  const contact_flag_t& getContactFlags() const { return contactFlags_; };
  const vector3_t& getSurfaceNormalInOriginFrame(size_t leg) const { return surfaceNormalsInOriginFrame_[leg]; };

  // Precomputation access : any(cost, constraint, softConstraint)
  const vector3_t& footPositionInOriginFrame(size_t leg) const { return feetPositionInOriginFrame_[leg]; }
  const vector3_t& footVelocityInOriginFrame(size_t leg) const { return feetVelocitiesInOriginFrame_[leg]; }
  const std::vector<kinematic_model_t::CollisionSphere>& collisionSpheresInOriginFrame() const { return collisionSpheresInOriginFrame_; }

  // Precomputation access : any(cost, constraint, softConstraint) + (derivatives)
  const matrix_t& footPositionInOriginFrameStateDerivative(size_t leg) const { return feetPositionInOriginFrameStateDerivative_[leg]; }
  const VectorFunctionLinearApproximation& footVelocityInOriginFrameDerivative(size_t leg) const {
    return feetVelocitiesInOriginFrameDerivative_[leg];
  }
  const std::vector<matrix_t>& collisionSpheresInOriginFrameStateDerivative() const { return collisionSpheresDerivative_; }

 protected:
  SwitchedModelPreComputation() = default;
  SwitchedModelPreComputation(const SwitchedModelPreComputation& other);

 private:
  void updateFeetPhases(scalar_t t);

  static void intermediateLinearOutputs(const ad_com_model_t& adComModel, const ad_kinematic_model_t& adKinematicsModel,
                                        const ad_vector_t& state, const ad_vector_t& input, ad_vector_t& outputs);
  void updateIntermediateLinearOutputs(scalar_t t, const vector_t& x, const vector_t& u);
  void updateIntermediateLinearOutputDerivatives(scalar_t t, const vector_t& x, const vector_t& u);

  const SwingTrajectoryPlanner* swingTrajectoryPlannerPtr_;
  std::unique_ptr<ocs2::CppAdInterface> intermediateLinearOutputAdInterface_;
  std::vector<scalar_t> collisionRadii_;  // Does not include those for the feet

  // Storage for the precomputation
  feet_array_t<const FootPhase*> feetPhases_;
  contact_flag_t contactFlags_;
  feet_array_t<vector3_t> feetPositionInOriginFrame_;
  feet_array_t<vector3_t> surfaceNormalsInOriginFrame_;
  feet_array_t<matrix_t> feetPositionInOriginFrameStateDerivative_;
  feet_array_t<vector3_t> feetVelocitiesInOriginFrame_;
  feet_array_t<VectorFunctionLinearApproximation> feetVelocitiesInOriginFrameDerivative_;
  std::vector<kinematic_model_t::CollisionSphere> collisionSpheresInOriginFrame_;
  std::vector<matrix_t> collisionSpheresDerivative_;

  /// Friend access for unit testing purposes
  friend SwitchedModelPreComputationMockup;
};

/**
 * Helper class to set the internal state of the precomputation class in unit tests.
 */
class SwitchedModelPreComputationMockup : public SwitchedModelPreComputation {
 public:
  const FootPhase*& feetPhases(size_t leg) { return feetPhases_[leg]; };
  contact_flag_t& contactFlags() { return contactFlags_; }
  vector3_t& feetPositionInOriginFrame(size_t leg) { return feetPositionInOriginFrame_[leg]; }
  vector3_t& surfaceNormalInOriginFrame(size_t leg) { return surfaceNormalsInOriginFrame_[leg]; }
  matrix_t& feetPositionInOriginFrameStateDerivative(size_t leg) { return feetPositionInOriginFrameStateDerivative_[leg]; }
  vector3_t& feetVelocitiesInOriginFrame(size_t leg) { return feetVelocitiesInOriginFrame_[leg]; }
  VectorFunctionLinearApproximation& feetVelocitiesInOriginFrameDerivative(size_t leg) {
    return feetVelocitiesInOriginFrameDerivative_[leg];
  }
  std::vector<kinematic_model_t::CollisionSphere>& collisionSpheresInOriginFrame() { return collisionSpheresInOriginFrame_; }
  std::vector<matrix_t>& collisionSpheresDerivative() { return collisionSpheresDerivative_; }
};

}  // namespace switched_model