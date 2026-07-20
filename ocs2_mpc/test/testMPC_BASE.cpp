#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <ocs2_mpc/MPC_BASE.h>

namespace ocs2 {
namespace {

class TestSolver final : public SolverBase {
 public:
  void reset() override {
    ++resetCount;
    finalTime = 0.0;
  }

  const OptimalControlProblem& getOptimalControlProblem() const override { return problem; }
  const PerformanceIndex& getPerformanceIndeces() const override { return performanceIndex; }
  size_t getNumIterations() const override { return 0; }
  const std::vector<PerformanceIndex>& getIterationsLog() const override { return iterationLog; }
  scalar_t getFinalTime() const override { return finalTime; }
  void getPrimalSolution(scalar_t, PrimalSolution*) const override {}
  const ProblemMetrics& getSolutionMetrics() const override { return problemMetrics; }
  ScalarFunctionQuadraticApproximation getValueFunction(scalar_t, const vector_t&) const override { return {}; }
  ScalarFunctionQuadraticApproximation getHamiltonian(scalar_t, const vector_t&, const vector_t&) override { return {}; }
  vector_t getStateInputEqualityConstraintLagrangian(scalar_t, const vector_t&) const override { return {}; }
  MultiplierCollection getIntermediateDualSolution(scalar_t) const override { return {}; }

  void setPerformance(scalar_t cost, scalar_t inequalityConstraintsSSE) {
    performanceIndex.cost = cost;
    performanceIndex.inequalityConstraintsSSE = inequalityConstraintsSSE;
  }

  int resetCount = 0;
  scalar_t finalTime = 0.0;

 private:
  void runImpl(scalar_t, const vector_t&, scalar_t) override {}
  void runImpl(scalar_t, const vector_t&, scalar_t, const ControllerBase*) override {}
  void runImpl(scalar_t, const vector_t&, scalar_t, const PrimalSolution&) override {}

  OptimalControlProblem problem;
  PerformanceIndex performanceIndex;
  std::vector<PerformanceIndex> iterationLog;
  ProblemMetrics problemMetrics;
};

class TestMpc final : public MPC_BASE {
 public:
  explicit TestMpc(scalar_t timeHorizon) : MPC_BASE(makeSettings(timeHorizon)) {}

  SolverBase* getSolverPtr() override { return &solver; }
  const SolverBase* getSolverPtr() const override { return &solver; }

  TestSolver solver;
  std::vector<scalar_t> runTimes;
  bool failNextRun = false;
  scalar_t target = 0.0;
  vector_t lastInput;

 private:
  static mpc::Settings makeSettings(scalar_t timeHorizon) {
    mpc::Settings settings;
    settings.timeHorizon_ = timeHorizon;
    return settings;
  }

  void calculateController(scalar_t initTime, const vector_t& initState, scalar_t finalTime) override {
    runTimes.push_back(initTime);
    if (failNextRun) {
      failNextRun = false;
      throw std::runtime_error("injected solve failure");
    }
    if (initState.size() == 0 || !initState.allFinite() || !std::isfinite(target)) {
      throw std::runtime_error("invalid replay state or target");
    }
    lastInput = vector_t::Constant(1, target - initState(0));
    const scalar_t cost = lastInput.squaredNorm();
    const scalar_t violation = std::max<scalar_t>(0.0, std::abs(lastInput(0)) - 1.0);
    solver.setPerformance(cost, violation * violation);
    solver.finalTime = finalTime;
  }
};

struct ReplayRecord final {
  size_t step = 0;
  std::string event;
  scalar_t time = 0.0;
  scalar_t state = 0.0;
  scalar_t target = 0.0;
  uint64_t resetEpoch = 0;
  uint64_t policySequence = 0;
  bool solveOutcome = false;
  bool stateFinite = false;
  bool inputFinite = false;
  scalar_t cost = 0.0;
  scalar_t inequalityConstraintsSSE = 0.0;
  int solverResetCount = 0;
};

std::vector<ReplayRecord> runLockstepReplay(double& elapsedMicroseconds) {
  TestMpc mpc(4.0);
  uint64_t resetEpoch = 0;
  uint64_t policySequence = 0;
  mpc.reset();
  ++resetEpoch;
  mpc.target = 1.0;

  struct ReplayInput final {
    const char* event;
    scalar_t time;
    scalar_t state;
    bool changesTarget;
    scalar_t nextTarget;
  };
  const std::vector<ReplayInput> inputs = {
      {"reset_observation", 1.0, 0.0, false, 0.0},
      {"observation", 2.0, 0.25, false, 0.0},
      {"target_change_observation", 3.0, 0.25, true, -0.5},
      {"time_discontinuity", 1.5, 0.0, false, 0.0},
  };

  std::vector<ReplayRecord> records;
  records.reserve(inputs.size());
  const auto start = std::chrono::steady_clock::now();
  for (size_t i = 0; i < inputs.size(); ++i) {
    const auto& input = inputs[i];
    if (input.changesTarget) {
      mpc.target = input.nextTarget;
    }
    vector_t state = vector_t::Constant(1, input.state);
    const bool solved = mpc.run(input.time, state);
    if (solved) {
      ++policySequence;
    }
    const auto& performance = mpc.solver.getPerformanceIndeces();
    records.push_back({i,
                       input.event,
                       input.time,
                       input.state,
                       mpc.target,
                       resetEpoch,
                       policySequence,
                       solved,
                       state.allFinite(),
                       mpc.lastInput.allFinite(),
                       performance.cost,
                       performance.inequalityConstraintsSSE,
                       mpc.solver.resetCount});
  }
  elapsedMicroseconds =
      std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - start).count();
  return records;
}

std::string replayCsv(const std::vector<ReplayRecord>& records) {
  std::ostringstream stream;
  stream << "step,event,time,state,target,reset_epoch,policy_sequence,solve_outcome,state_finite,input_finite,cost,"
            "inequality_constraints_sse,solver_reset_count\n";
  stream << std::fixed << std::setprecision(6);
  for (const auto& record : records) {
    stream << record.step << ',' << record.event << ',' << record.time << ',' << record.state << ',' << record.target
           << ',' << record.resetEpoch << ',' << record.policySequence << ',' << (record.solveOutcome ? 1 : 0) << ','
           << (record.stateFinite ? 1 : 0) << ',' << (record.inputFinite ? 1 : 0) << ',' << record.cost << ','
           << record.inequalityConstraintsSSE << ',' << record.solverResetCount << '\n';
  }
  return stream.str();
}

std::string readFile(const std::string& path) {
  std::ifstream input(path);
  if (!input) {
    throw std::runtime_error("Failed to open replay fixture: " + path);
  }
  return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

TEST(MpcBase, advancingObservationUsesWarmStart) {
  TestMpc mpc(4.0);

  EXPECT_TRUE(mpc.run(1.0, vector_t::Zero(1)));
  EXPECT_TRUE(mpc.run(2.0, vector_t::Zero(1)));

  EXPECT_EQ(mpc.solver.resetCount, 0);
  EXPECT_EQ(mpc.runTimes, (std::vector<scalar_t>{1.0, 2.0}));
  EXPECT_DOUBLE_EQ(mpc.solver.finalTime, 6.0);
}

TEST(MpcBase, observationBeyondPreviousHorizonColdResetsAndRuns) {
  TestMpc mpc(4.0);

  EXPECT_TRUE(mpc.run(1.0, vector_t::Zero(1)));
  EXPECT_TRUE(mpc.run(5.0, vector_t::Zero(1)));

  EXPECT_EQ(mpc.solver.resetCount, 1);
  EXPECT_EQ(mpc.runTimes, (std::vector<scalar_t>{1.0, 5.0}));
  EXPECT_DOUBLE_EQ(mpc.solver.finalTime, 9.0);
}

TEST(MpcBase, backwardObservationJumpColdResetsAndRuns) {
  TestMpc mpc(4.0);

  EXPECT_TRUE(mpc.run(5.0, vector_t::Zero(1)));
  EXPECT_TRUE(mpc.run(4.0, vector_t::Zero(1)));

  EXPECT_EQ(mpc.solver.resetCount, 1);
  EXPECT_EQ(mpc.runTimes, (std::vector<scalar_t>{5.0, 4.0}));
  EXPECT_DOUBLE_EQ(mpc.solver.finalTime, 8.0);
}

TEST(MpcBase, failedSolveRequiresExplicitReset) {
  TestMpc mpc(4.0);
  mpc.failNextRun = true;

  EXPECT_THROW(mpc.run(1.0, vector_t::Zero(1)), std::runtime_error);
  EXPECT_EQ(mpc.runTimes, (std::vector<scalar_t>{1.0}));

  EXPECT_THROW(mpc.run(2.0, vector_t::Zero(1)), std::runtime_error);
  EXPECT_EQ(mpc.runTimes, (std::vector<scalar_t>{1.0}));

  mpc.reset();
  EXPECT_TRUE(mpc.run(2.0, vector_t::Zero(1)));
  EXPECT_EQ(mpc.solver.resetCount, 1);
  EXPECT_EQ(mpc.runTimes, (std::vector<scalar_t>{1.0, 2.0}));
}

TEST(MpcBase, lockstepReplayIsFiniteOrderedAndRepeatable) {
  const std::string fixturePath = std::string(OCS2_MPC_TEST_DATA_DIR) + "/lockstep_replay_expected.csv";
  const std::string expectedCsv = readFile(fixturePath);

  double firstElapsedMicroseconds = 0.0;
  const auto firstRecords = runLockstepReplay(firstElapsedMicroseconds);
  ASSERT_EQ(firstRecords.size(), 4U);
  EXPECT_EQ(replayCsv(firstRecords), expectedCsv);
  RecordProperty("first_replay_elapsed_us", firstElapsedMicroseconds);

  for (size_t repetition = 1; repetition < 20; ++repetition) {
    double elapsedMicroseconds = 0.0;
    const auto records = runLockstepReplay(elapsedMicroseconds);
    EXPECT_EQ(replayCsv(records), expectedCsv) << "repetition=" << repetition;
  }
}

}  // namespace
}  // namespace ocs2
