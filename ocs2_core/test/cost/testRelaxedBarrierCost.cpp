#include <gtest/gtest.h>

#include <ocs2_core/cost/RelaxedBarrierCost.h>

using namespace ocs2;

class MyCost : public RelaxedBarrierCost {
 public:
  MyCost(Config config, size_t stateDim, size_t inputDim, size_t intermediateCostDim, size_t finalCostDim)
      : RelaxedBarrierCost(config, stateDim, inputDim, intermediateCostDim, finalCostDim) {}

  MyCost(const MyCost& rhs) : RelaxedBarrierCost(rhs) {}

  virtual CostFunctionBase* clone() const { return new MyCost(*this); };

 protected:
  virtual ad_vector_t intermediateCostFunction(ad_scalar_t time, const ad_vector_t& state, const ad_vector_t& input,
                                               const ad_vector_t& parameters) const override {
    return ad_vector_t::Zero(1);
  }
};

TEST(testRelaxedBarrierCost, canConstructCost) {
  MyCost rbcost({0.1, 5.0}, 3, 2, 1, 3);
  rbcost.initialize("foo", "/tmp/ocs2", true, true);
}
