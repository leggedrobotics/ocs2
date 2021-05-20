#include <gtest/gtest.h>

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/dynamics/LinearSystemDynamics.h>
#include <ocs2_core/dynamics/SystemDynamicsBase.h>

using namespace ocs2;

class DummyPreComputation final : public PreComputation {
 public:
  DummyPreComputation() = default;
  PreComputation* clone() const override { return new DummyPreComputation(*this); }

  void request(Request flags, scalar_t t, const vector_t& x, const vector_t& u) override { requestedFlags = flags; }

  void requestPreJump(Request flags, scalar_t t, const vector_t& x) override { requestedFlags = flags; }

  static void reset() { requestedFlags = static_cast<PreComputation::Request>(0); }

  static Request requestedFlags;
};

PreComputation::Request DummyPreComputation::requestedFlags = static_cast<PreComputation::Request>(0);

class DummySystem final : public SystemDynamicsBase {
 public:
  DummySystem() : SystemDynamicsBase(DummyPreComputation()) {}
  ~DummySystem() override = default;
  DummySystem* clone() const override { return new DummySystem(*this); }

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation*) override {
    vector_t dfdt(2);
    dfdt << x(1), u(0);
    return dfdt;
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation* preComp) override {
    VectorFunctionLinearApproximation linearDynamics;
    linearDynamics.f = computeFlowMap(t, x, u, preComp);
    linearDynamics.dfdx.resize(2, 2);
    linearDynamics.dfdx << 0, 1,  // clang-format off
                           0, 0;  // clang-format on
    linearDynamics.dfdu.resize(2, 1);
    linearDynamics.dfdu << 0, 1;
    return linearDynamics;
  }

  using ControlledSystemBase::computeFlowMap;
  using SystemDynamicsBase::linearApproximation;
};

TEST(testSystemDynamicsPreComputation, testIntermediateCallback) {
  DummySystem system;

  const scalar_t t = 0.0;
  const vector_t x = vector_t::Zero(2);
  const vector_t u = vector_t::Zero(1);

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);

  const auto flowMap = system.computeFlowMap(t, x, u);
  EXPECT_TRUE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);
  const auto flowMapApproximation = system.linearApproximation(t, x, u);
  EXPECT_TRUE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_TRUE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);
}

TEST(testSystemDynamicsPreComputation, testPreJumpCallback) {
  DummySystem system;

  const scalar_t t = 0.0;
  const vector_t x = vector_t::Zero(2);

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);

  const auto jumpMap = system.computeJumpMap(t, x);
  EXPECT_TRUE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_FALSE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);

  const auto jumpMapApproximation = system.jumpMapLinearApproximation(t, x);
  EXPECT_TRUE(DummyPreComputation::requestedFlags & PreComputation::Request::Dynamics);
  EXPECT_TRUE(DummyPreComputation::requestedFlags & PreComputation::Request::Approximation);
}

TEST(testSystemDynamicsPreComputation, testPreComputationRequestLogic) {
  using Request = PreComputation::Request;

  Request a = Request::Dynamics | Request::Approximation;
  EXPECT_TRUE(a & Request::Dynamics);
  EXPECT_TRUE(a & Request::Approximation);
  EXPECT_FALSE(a & Request::Cost);
  EXPECT_FALSE(a & Request::Constraint);
  EXPECT_TRUE((Request::Dynamics | Request::Dynamics) & Request::Dynamics);
}
