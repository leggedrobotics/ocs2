#pragma once

#include <memory>

#include <ocs2_core/control/ControllerBase.h>

#include "ocs2_mpcnet/control/MpcnetControllerBase.h"

namespace ocs2 {

/**
 * A behavioral controller that computes the input based on a mixture of an optimal policy (e.g. implicitly found via MPC)
 * and a learned policy (e.g. explicitly represented by a neural network).
 * The behavioral policy is pi_behavioral = alpha * pi_optimal + (1 - alpha) * pi_learned with alpha in [0, 1].
 */
class MpcnetBehavioralController : public ControllerBase {
 public:
  using Base = ControllerBase;
  using Optimal = ControllerBase;
  using Learned = MpcnetControllerBase;

  /**
   * Default constructor, leaves object uninitialized.
   */
  MpcnetBehavioralController() = default;

  /**
   * Constructor, initializes all required members of the controller.
   * @param [in] alpha : The mixture parameter.
   * @param [in] optimalControllerPtr : Pointer to the optimal controller.
   * @param [in] learnedControllerPtr : Pointer to the learned controller.
   */
  MpcnetBehavioralController(scalar_t alpha, Optimal* optimalControllerPtr, Learned* learnedControllerPtr)
      : alpha_(alpha),
        optimalControllerPtr_(std::unique_ptr<Optimal>(optimalControllerPtr)),
        learnedControllerPtr_(std::unique_ptr<Learned>(learnedControllerPtr)) {}

  /**
   * Copy constructor.
   */
  MpcnetBehavioralController(const MpcnetBehavioralController& other)
      : MpcnetBehavioralController(other.alpha_, other.optimalControllerPtr_->clone(), other.learnedControllerPtr_->clone()) {}

  /**
   * Default destructor.
   */
  ~MpcnetBehavioralController() override = default;

  /**
   * Set the mixture parameter.
   * @param [in] alpha : The mixture parameter.
   */
  void setAlpha(scalar_t alpha) { alpha_ = alpha; }

  /**
   * Set the optimal controller.
   * @param [in] optimalControllerPtr : Pointer to the optimal controller.
   */
  void setOptimalController(Optimal* optimalControllerPtr) { optimalControllerPtr_.reset(optimalControllerPtr); }

  /**
   * Set the learned controller.
   * @param [in] learnedControllerPtr : Pointer to the learned controller.
   */
  void setLearnedController(Learned* learnedControllerPtr) { learnedControllerPtr_.reset(learnedControllerPtr); }

  vector_t computeInput(scalar_t t, const vector_t& x) override;

  void concatenate(const Base* otherController, int index, int length) override;

  int size() const override;

  ControllerType getType() const override { return ControllerType::BEHAVIORAL; }

  void clear() override;

  bool empty() const override;

  MpcnetBehavioralController* clone() const override { return new MpcnetBehavioralController(*this); }

 private:
  scalar_t alpha_;
  std::unique_ptr<Optimal> optimalControllerPtr_;
  std::unique_ptr<Learned> learnedControllerPtr_;
};

}  // namespace ocs2
