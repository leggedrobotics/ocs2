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

  void request(RequestSet request, scalar_t t, const vector_t& x, const vector_t& u) override { lastRequest = request; }

  void requestPreJump(RequestSet request, scalar_t t, const vector_t& x) override { lastRequest = request; }

  static void reset() { lastRequest = RequestSet(static_cast<Request>(0)); }

  static RequestSet lastRequest;
};

RequestSet DummyPreComputation::lastRequest = RequestSet(static_cast<Request>(0));

class DummySystem final : public SystemDynamicsBase {
 public:
  DummySystem() : SystemDynamicsBase(DummyPreComputation()) {}
  ~DummySystem() override = default;
  DummySystem* clone() const override { return new DummySystem(*this); }

  vector_t computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation&) override {
    vector_t dfdt(2);
    dfdt << x(1), u(0);
    return dfdt;
  }

  VectorFunctionLinearApproximation linearApproximation(scalar_t t, const vector_t& x, const vector_t& u,
                                                        const PreComputation& preComp) override {
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
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Approximation));

  const auto flowMap = system.ControlledSystemBase::computeFlowMap(t, x, u);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Approximation));

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Approximation));
  const auto flowMapApproximation = system.SystemDynamicsBase::linearApproximation(t, x, u);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(Request::Approximation));
}

TEST(testSystemDynamicsPreComputation, testPreJumpCallback) {
  DummySystem system;

  const scalar_t t = 0.0;
  const vector_t x = vector_t::Zero(2);

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Approximation));

  const auto jumpMap = system.computeJumpMap(t, x);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Approximation));

  DummyPreComputation::reset();
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_FALSE(DummyPreComputation::lastRequest.contains(Request::Approximation));

  const auto jumpMapApproximation = system.jumpMapLinearApproximation(t, x);
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(Request::Dynamics));
  EXPECT_TRUE(DummyPreComputation::lastRequest.contains(Request::Approximation));
}

TEST(testSystemDynamicsPreComputation, testPreComputationRequestLogic) {
  constexpr RequestSet a = Request::Cost + Request::Constraint;

  // Test for individual flags
  EXPECT_TRUE(a.contains(Request::Cost));
  EXPECT_TRUE(a.contains(Request::Constraint));
  EXPECT_FALSE(a.contains(Request::Approximation));
  EXPECT_FALSE(a.contains(Request::SoftConstraint));
  EXPECT_FALSE(a.contains(Request::Dynamics));
  // a.contains(int(42)); // this should not compile
  // a.contains(Request(Request::Cost)); // this should not compile

  constexpr RequestSet b = Request::Cost + Request::Approximation;
  constexpr RequestSet c = a + b;  // union

  // Test for subset
  EXPECT_TRUE(c.containsAll(a));
  EXPECT_TRUE(c.containsAll(b));
  EXPECT_TRUE(c.containsAll(c));
  EXPECT_FALSE(a.containsAll(b));
  EXPECT_FALSE(b.containsAll(a));

  // Test for non-empty intersection
  EXPECT_TRUE(c.containsAny(a));
  EXPECT_TRUE(c.containsAny(b));
  EXPECT_TRUE(c.containsAny(c));
  EXPECT_TRUE(a.containsAny(b));
  EXPECT_TRUE(b.containsAny(a));

  EXPECT_FALSE(c.contains(Request::Dynamics));
  EXPECT_FALSE(c.contains(Request::SoftConstraint));

  // Add same flag twice
  EXPECT_TRUE((Request::Dynamics + Request::Dynamics).contains(Request::Dynamics));
}
