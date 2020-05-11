#include <gtest/gtest.h>

#include <ocs2_core/cost/RelaxedBarrierCost.h>

using namespace ocs2;

class MyCost : public RelaxedBarrierCost {
 public:
  MyCost(Config config, size_t state_dim, size_t input_dim, size_t intermediate_cost_dim, size_t terminal_cost_dim)
      : RelaxedBarrierCost(config, state_dim, input_dim, intermediate_cost_dim, terminal_cost_dim) {}

  MyCost(const MyCost& rhs) : RelaxedBarrierCost(rhs) {}

  virtual CostFunctionBase* clone() const { return new MyCost(*this); };

 protected:
  virtual void intermediateCostFunction(ad_scalar_t time, const ad_dynamic_vector_t& state, const ad_dynamic_vector_t& input,
                                        const ad_dynamic_vector_t& parameters, ad_dynamic_vector_t& costValues) const override {}
};

TEST(testRelaxedBarrierCost, canConstructCost) {
  MyCost rbcost({0.1, 5.0}, 3, 2, 1, 3);
  rbcost.initialize("foo", "/tmp/ocs2", true, true);
}
