#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include <ocs2_core/cost/StateInputCost.h>
#include <ocs2_ddp/search_strategy/LineSearchStrategy.h>

namespace ocs2 {
namespace {

struct RolloutActivity {
  std::atomic<int> active{0};
  std::atomic<int> peak{0};
  std::atomic<int> entered{0};
  std::mutex mutex;
  std::condition_variable condition;
  bool failCandidates = false;
};

// A two-knot rollout isolates the real line-search scheduler and Armijo
// selection from integration time and Riccati partitioning.
class ObservedRollout final : public RolloutBase {
public:
  explicit ObservedRollout(std::shared_ptr<RolloutActivity> activity)
      : RolloutBase(rollout::Settings{}), activity_(std::move(activity)) {}
  ObservedRollout *clone() const override { return new ObservedRollout(*this); }
  void abortRollout() override { aborted = true; }
  void reactivateRollout() override { aborted = false; }
  vector_t run(scalar_t start, const vector_t &state, scalar_t end,
               ControllerBase *controller, ModeSchedule &,
               scalar_array_t &times, size_array_t &events,
               vector_array_t &states, vector_array_t &inputs) override {
    const vector_t input = controller->computeInput(start, state);
    if (input(0) > 0.0) {
      const int active = ++activity_->active;
      int peak = activity_->peak;
      while (active > peak &&
             !activity_->peak.compare_exchange_weak(peak, active)) {
      }
      ++activity_->entered;
      activity_->condition.notify_all();
      // A finite rendezvous detects the 2-thread regression without hanging
      // when only the caller is accidentally scheduled.
      {
        std::unique_lock<std::mutex> lock(activity_->mutex);
        activity_->condition.wait_for(lock, std::chrono::milliseconds(50),
                                      [&] { return activity_->entered >= 2; });
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(2));
      --activity_->active;
      if (activity_->failCandidates) {
        throw std::runtime_error("injected candidate rollout failure");
      }
    }
    times = {start, end};
    events.clear();
    states = {state, state};
    inputs = {input, input};
    return state;
  }
  bool aborted = false;

private:
  std::shared_ptr<RolloutActivity> activity_;
};

class InputTargetCost final : public StateInputCost {
public:
  explicit InputTargetCost(double target) : target_(target) {}
  InputTargetCost *clone() const override { return new InputTargetCost(*this); }
  scalar_t getValue(scalar_t, const vector_t &, const vector_t &input,
                    const TargetTrajectories &,
                    const PreComputation &) const override {
    return 0.5 * std::pow(input(0) - target_, 2);
  }
  ScalarFunctionQuadraticApproximation
  getQuadraticApproximation(scalar_t time, const vector_t &state,
                            const vector_t &input,
                            const TargetTrajectories &target,
                            const PreComputation &preComp) const override {
    auto result =
        ScalarFunctionQuadraticApproximation::Zero(state.size(), input.size());
    result.f = getValue(time, state, input, target, preComp);
    result.dfdu(0) = input(0) - target_;
    result.dfduu(0, 0) = 1.0;
    return result;
  }

private:
  double target_;
};

TEST(LineSearchConcurrency,
     UsesAllThreadsAndPreservesLargestAcceptedStepAndFallback) {
  for (const int threads : {1, 2, 4}) {
    for (const int scenario : {0, 1, 2}) {
      SCOPED_TRACE("threads=" + std::to_string(threads) +
                   " scenario=" + std::to_string(scenario));
      ThreadPool pool(threads - 1);
      auto activity = std::make_shared<RolloutActivity>();
      activity->failCandidates = scenario == 2;
      std::vector<std::unique_ptr<ObservedRollout>> rollouts;
      std::vector<std::reference_wrapper<RolloutBase>> rolloutRefs;
      std::vector<OptimalControlProblem> problems(threads);
      std::vector<std::reference_wrapper<OptimalControlProblem>> problemRefs;
      TargetTrajectories target;
      for (int i = 0; i < threads; ++i) {
        rollouts.emplace_back(new ObservedRollout(activity));
        rolloutRefs.emplace_back(*rollouts.back());
        problems[i].targetTrajectoriesPtr = &target;
        problems[i].costPtr->add("input", std::make_unique<InputTargetCost>(
                                              scenario == 1 ? 0.0 : 0.4));
        problemRefs.emplace_back(problems[i]);
      }
      line_search::Settings settings;
      settings.minStepLength = 0.125;
      LineSearchStrategy search(
          search_strategy::Settings{}, settings, pool, rolloutRefs, problemRefs,
          [](const PerformanceIndex &p) { return p.cost; });
      LinearController controller;
      controller.timeStamp_ = {0.0, 1.0};
      controller.biasArray_ = {vector_t::Zero(1), vector_t::Zero(1)};
      controller.deltaBiasArray_ = {vector_t::Ones(1), vector_t::Ones(1)};
      controller.gainArray_ = {matrix_t::Zero(1, 1), matrix_t::Zero(1, 1)};
      for (int repeat = 0; repeat < 2; ++repeat) {
        activity->entered = 0;
        activity->peak = 0;
        search_strategy::Solution solution;
        solution.primalSolution.controllerPtr_ =
            std::make_unique<LinearController>();
        EXPECT_TRUE(search.run({0.0, 1.0}, vector_t::Zero(1), 0.0, controller,
                               DualSolution{}, ModeSchedule{}, solution));
        ASSERT_FALSE(solution.primalSolution.inputTrajectory_.empty());
        EXPECT_DOUBLE_EQ(solution.primalSolution.inputTrajectory_.front()(0),
                         scenario == 0 ? 0.5 : 0.0);
        EXPECT_EQ(activity->active, 0);
        EXPECT_GE(activity->peak, threads > 1 ? 2 : 1);
        for (const auto &rollout : rollouts) {
          EXPECT_FALSE(rollout->aborted);
        }
      }
    }
  }
}

} // namespace
} // namespace ocs2
