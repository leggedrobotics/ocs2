/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_core/Types.h>
#include <ocs2_core/augmented_lagrangian/AugmentedLagrangian.h>
#include <ocs2_core/constraint/LinearStateConstraint.h>
#include <ocs2_core/constraint/LinearStateInputConstraint.h>
#include <ocs2_core/cost/QuadraticStateInputCost.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_core/penalties/Penalties.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

namespace ocs2 {

class DoubleIntegratorReachingTask {
 public:
  static constexpr size_t STATE_DIM = 2;
  static constexpr size_t INPUT_DIM = 1;
  static constexpr scalar_t timeStep = 1e-2;
  static constexpr scalar_t minRelCost = 1e-12;  // to avoid early termination
  static constexpr scalar_t constraintTolerance = 1e-3;

  enum class PenaltyType {
    QuadraticPenalty,
    SmoothAbsolutePenalty,
  };

  DoubleIntegratorReachingTask() = default;
  virtual ~DoubleIntegratorReachingTask() = default;

 protected:
  const scalar_t tGoal = 1.0;
  const vector_t xInit = vector_t::Zero(STATE_DIM);
  const vector_t xGoal = (vector_t(STATE_DIM) << 2.0, 0.0).finished();

  std::unique_ptr<DefaultInitializer> getInitializer() const { return std::make_unique<DefaultInitializer>(INPUT_DIM); }

  std::shared_ptr<ReferenceManager> getReferenceManagerPtr() const {
    const ModeSchedule modeSchedule({tGoal}, {0, 1});
    const TargetTrajectories targetTrajectories({tGoal}, {xGoal}, {vector_t::Zero(INPUT_DIM)});
    return std::make_shared<ReferenceManager>(targetTrajectories, modeSchedule);
  }

  std::unique_ptr<StateInputCost> getCostPtr() const {
    matrix_t Q = matrix_t::Zero(STATE_DIM, STATE_DIM);
    matrix_t R = 0.1 * matrix_t::Identity(INPUT_DIM, INPUT_DIM);
    return std::make_unique<QuadraticStateInputCost>(std::move(Q), std::move(R));
  }

  std::unique_ptr<SystemDynamicsBase> getDynamicsPtr() const {
    matrix_t A = (matrix_t(STATE_DIM, STATE_DIM) << 0.0, 1.0, 0.0, 0.0).finished();
    matrix_t B = (matrix_t(STATE_DIM, INPUT_DIM) << 0.0, 1.0).finished();
    return std::make_unique<LinearSystemDynamics>(std::move(A), std::move(B));
  }

  std::unique_ptr<ocs2::StateAugmentedLagrangian> getGoalReachingAugmentedLagrangian(const vector_t& xGoal, PenaltyType penaltyType) {
    constexpr scalar_t scale = 10.0;
    constexpr scalar_t stepSize = 1.0;

    auto goalReachingConstraintPtr = std::make_unique<LinearStateConstraint>(-xGoal, matrix_t::Identity(xGoal.size(), xGoal.size()));

    switch (penaltyType) {
      case PenaltyType::QuadraticPenalty: {
        using penalty_type = augmented::QuadraticPenalty;
        penalty_type::Config boundsConfig{scale, stepSize};
        return create(std::move(goalReachingConstraintPtr), penalty_type::create(boundsConfig));
      }
      case PenaltyType::SmoothAbsolutePenalty: {
        using penalty_type = augmented::SmoothAbsolutePenalty;
        penalty_type::Config boundsConfig{scale, 0.1, stepSize};
        return create(std::move(goalReachingConstraintPtr), penalty_type::create(boundsConfig));
      }
      default:
        throw std::runtime_error("[TestCartpole::getPenalty] undefined penaltyType");
    }
  }

  /** This class enforces zero force at the second mode (mode = 1)*/
  class ZeroInputConstraint final : public StateInputConstraint {
   public:
    ZeroInputConstraint(const ReferenceManager& referenceManager)
        : StateInputConstraint(ConstraintOrder::Linear), referenceManagerPtr_(&referenceManager) {}

    ~ZeroInputConstraint() override = default;
    ZeroInputConstraint* clone() const override { return new ZeroInputConstraint(*this); }

    size_t getNumConstraints(scalar_t time) const override { return 1; }

    /** Only active after the fist mode */
    bool isActive(scalar_t time) const override { return referenceManagerPtr_->getModeSchedule().modeAtTime(time) > 0 ? true : false; }

    vector_t getValue(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) const override { return u; }

    VectorFunctionLinearApproximation getLinearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                             const PreComputation&) const override {
      VectorFunctionLinearApproximation approx;
      approx.f = u;
      approx.dfdx.setZero(getNumConstraints(t), x.size());
      approx.dfdu.setIdentity(getNumConstraints(t), u.size());
      return approx;
    }

   private:
    ZeroInputConstraint(const ZeroInputConstraint&) = default;
    const ReferenceManager* referenceManagerPtr_;
  };

  /*
   * printout trajectory. Use the following commands for plotting in MATLAB:
   * subplot(2, 1, 1); plot(timeTrajectory, stateTrajectory);
   * xlabel("time [sec]"); legend("pos", "vel");
   * subplot(2, 1, 2); plot(timeTrajectory, inputTrajectory);
   * xlabel("time [sec]"); legend("force");
   */
  void printSolution(const PrimalSolution& primalSolution, bool display) const {
    if (display) {
      std::cerr << "\n";
      // time
      std::cerr << "timeTrajectory = [";
      for (const auto& t : primalSolution.timeTrajectory_) {
        std::cerr << t << "; ";
      }
      std::cerr << "];\n";
      // state
      std::cerr << "stateTrajectory = [";
      for (const auto& x : primalSolution.stateTrajectory_) {
        std::cerr << x.transpose() << "; ";
      }
      std::cerr << "];\n";
      // input
      std::cerr << "inputTrajectory = [";
      for (const auto& u : primalSolution.inputTrajectory_) {
        std::cerr << u.transpose() << "; ";
      }
      std::cerr << "];\n";
    }
  }
};

constexpr size_t DoubleIntegratorReachingTask::STATE_DIM;
constexpr size_t DoubleIntegratorReachingTask::INPUT_DIM;
constexpr scalar_t DoubleIntegratorReachingTask::timeStep;
constexpr scalar_t DoubleIntegratorReachingTask::minRelCost;
constexpr scalar_t DoubleIntegratorReachingTask::constraintTolerance;

}  // namespace ocs2
