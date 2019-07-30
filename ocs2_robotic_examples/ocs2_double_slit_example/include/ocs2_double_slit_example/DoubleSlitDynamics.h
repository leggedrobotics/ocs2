#pragma once

#include <ocs2_core/dynamics/ControlledSystemBase.h>

#include "ocs2_double_slit_example/definitions.h"

namespace ocs2 {
namespace double_slit {

class DoubleSlitDynamics final : public ControlledSystemBase<DoubleSlit::STATE_DIM_, DoubleSlit::INPUT_DIM_> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Ptr = std::shared_ptr<DoubleSlitDynamics>;
  using ConstPtr = std::shared_ptr<const DoubleSlitDynamics>;

  using BASE = ControlledSystemBase<double_slit::STATE_DIM_, double_slit::INPUT_DIM_>;
  using scalar_t = typename BASE::scalar_t;
  using state_vector_t = typename BASE::state_vector_t;
  using input_vector_t = typename BASE::input_vector_t;

  /**
   * Constructor
   */
  DoubleSlitDynamics() = default;

  /**
   * Destructor
   */
  ~DoubleSlitDynamics() override = default;

  DoubleSlitDynamics* clone() const override { return new DoubleSlitDynamics(*this); }

  void computeFlowMap(const scalar_t&, const state_vector_t&, const input_vector_t& input, state_vector_t& stateDerivative) override {
    stateDerivative = input;
  }
};

}  // namespace double_slit
}  // namespace ocs2
