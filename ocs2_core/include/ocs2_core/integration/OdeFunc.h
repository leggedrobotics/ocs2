

#ifndef OCS2_ODE_FUNC_H
#define OCS2_ODE_FUNC_H

#include <functional>
#include "OdeBase.h"

namespace ocs2 {

template <int STATE_DIM>
class OdeFunc final : public OdeBase<STATE_DIM> {
 public:
  using BASE = OdeBase<STATE_DIM>;
  using typename BASE::scalar_t;
  using typename BASE::state_vector_t;

  explicit OdeFunc(std::function<void(const scalar_t& t, const state_vector_t& x, state_vector_t& dxdt)> flowMap)
      : flowMap_(std::move(flowMap)){};

  void computeFlowMap(const scalar_t& t, const state_vector_t& x, state_vector_t& dxdt) override { flowMap_(t, x, dxdt); };

  void setFlowMap(std::function<void(const scalar_t& t, const state_vector_t& x, state_vector_t& dxdt)>&& flowMap) { flowMap_ = flowMap; };

 private:
  std::function<void(const scalar_t& t, const state_vector_t& x, state_vector_t& dxdt)> flowMap_;
};
}  // namespace ocs2

#endif  // OCS2_ODE_FUNC_H
