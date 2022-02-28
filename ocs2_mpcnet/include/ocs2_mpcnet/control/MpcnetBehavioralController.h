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
class MpcnetBehavioralController final : public ControllerBase {
 public:
  /**
   * Default constructor, leaves object uninitialized.
   */
  MpcnetBehavioralController() = default;

  /**
   * Constructor, initializes all required members of the controller.
   * @param [in] alpha : The mixture parameter.
   * @param [in] optimalController : The optimal controller (this class takes ownership of a clone).
   * @param [in] learnedController : The learned controller (this class takes ownership of a clone).
   */
  MpcnetBehavioralController(scalar_t alpha, const ControllerBase& optimalController, const MpcnetControllerBase& learnedController)
      : alpha_(alpha),
        optimalControllerPtr_(std::unique_ptr<ControllerBase>(optimalController.clone())),
        learnedControllerPtr_(std::unique_ptr<MpcnetControllerBase>(learnedController.clone())) {}

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
   * @param [in] optimalController : The optimal controller (this class takes ownership of a clone).
   */
  void setOptimalController(const ControllerBase& optimalController) { optimalControllerPtr_.reset(optimalController.clone()); }

  /**
   * Set the learned controller.
   * @param [in] learnedController : The learned controller (this class takes ownership of a clone).
   */
  void setLearnedController(const MpcnetControllerBase& learnedController) { learnedControllerPtr_.reset(learnedController.clone()); }

  vector_t computeInput(scalar_t t, const vector_t& x) override;

  void concatenate(const ControllerBase* otherController, int index, int length) override;

  int size() const override;

  ControllerType getType() const override { return ControllerType::BEHAVIORAL; }

  void clear() override;

  bool empty() const override;

  MpcnetBehavioralController* clone() const override { return new MpcnetBehavioralController(*this); }

 private:
  /**
   * Copy constructor.
   */
  MpcnetBehavioralController(const MpcnetBehavioralController& other)
      : MpcnetBehavioralController(other.alpha_, *other.optimalControllerPtr_, *other.learnedControllerPtr_) {}

  scalar_t alpha_;
  std::unique_ptr<ControllerBase> optimalControllerPtr_;
  std::unique_ptr<MpcnetControllerBase> learnedControllerPtr_;
};

}  // namespace ocs2
