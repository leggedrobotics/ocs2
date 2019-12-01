#include <cmath>

namespace ocs2 {

class pendulum_logic final : public HybridLogicRules {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using BASE = HybridLogicRules;

  pendulum_logic() = default;
  ~pendulum_logic() override = default;

  pendulum_logic(scalar_array_t switchingTimes, size_array_t sybsystemsSequence) {}

  void rewind(const scalar_t& lowerBoundTime, const scalar_t& upperBoundTime) override {}

  void update() override {}

 protected:
  void insertModeSequenceTemplate(const logic_template_type& modeSequenceTemplate, const scalar_t& startTime,
                                  const scalar_t& finalTime) override{};
};

class pendulum_dyn final : public ControlledSystemBase<2, 1> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  pendulum_dyn() = default;
  ~pendulum_dyn() override = default;

  void computeFlowMap(const double& t, const Eigen::Vector2d& x, const Eigen::Matrix<double, 1, 1>& u, Eigen::Vector2d& dxdt) override {
    double g = 9.81;
    double L = 1;
    dxdt << x[1], -(g / L) * std::sin(x[0]);
  }

  void computeJumpMap(const scalar_t& time, const state_vector_t& state, state_vector_t& mappedState) override {
    mappedState[0] = state[0];
    mappedState[1] = -0.9 * state[1];
  }

  void computeGuardSurfaces(const scalar_t& time, const state_vector_t& state, dynamic_vector_t& guardSurfacesValue) override {
    guardSurfacesValue = Eigen::Matrix<double, 2, 1>();
    guardSurfacesValue[0] = state[0];
    guardSurfacesValue[1] = 1;
  }

  pendulum_dyn* clone() const override { return new pendulum_dyn(*this); }
};
}  // namespace ocs2
