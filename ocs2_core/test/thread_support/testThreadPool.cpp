#include <atomic>
#include <chrono>
#include <condition_variable>
#include <future>
#include <gtest/gtest.h>
#include <mutex>
#include <ocs2_core/thread_support/ThreadPool.h>
#include <stdexcept>

using namespace ocs2;

TEST(testThreadPool, testCanExecuteTask) {
  ThreadPool pool(1);
  std::future<void> res;
  int answer = 0;

  res = pool.run([&answer](int) { answer = 42; });
  res.wait_for(std::chrono::seconds(1));

  EXPECT_EQ(answer, 42);
}

TEST(testThreadPool, testReturnType) {
  ThreadPool pool(1);
  std::future<int> res;

  res = pool.run([](int) -> int { return 42; });

  EXPECT_EQ(res.get(), 42);
}

TEST(testThreadPool, testPropagateException) {
  ThreadPool pool(1);
  std::function<void(int)> task;

  // send task to pool
  task = [](int) { throw std::string("exception"); };
  EXPECT_THROW(pool.run(task).get(), std::string);
}

TEST(testThreadPool, testCanExecuteMultipleTasks) {
  ThreadPool pool(2);
  std::function<void(int)> task;
  std::future<void> res1, res2;

  std::promise<void> barrier_promise;
  std::shared_future<void> barrier = barrier_promise.get_future();

  std::atomic_int state1{0};
  std::atomic_int state2{0};

  res1 = pool.run([&state1, barrier](int) {
    state1 = 1;
    barrier.wait();
    state1 = 2;
  });
  res2 = pool.run([&state2, barrier](int) {
    state2 = 1;
    barrier.wait();
    state2 = 2;
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // check that both threads are blocking
  EXPECT_EQ(state1, 1);
  EXPECT_EQ(state2, 1);

  // signal threads
  barrier_promise.set_value();

  // wait for pool to run tasks
  std::future_status status1 = res1.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status1, std::future_status::ready);

  std::future_status status2 = res2.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status2, std::future_status::ready);

  EXPECT_EQ(state1, 2);
  EXPECT_EQ(state2, 2);

  res1.get();
  res2.get();
}

TEST(testThreadPool, testRunMultiple) {
  ThreadPool pool(2);
  std::atomic_int counter;
  counter = 0;

  pool.runParallel([&](int) { counter++; }, 42);

  EXPECT_EQ(counter, 42);
}

TEST(testThreadPool, runParallelDrainsHelpersAfterCallerFailure) {
  ThreadPool pool(2);

  std::mutex mutex;
  std::condition_variable helpersStartedCondition;
  int helpersStarted = 0;
  std::atomic_int helpersFinished{0};
  std::promise<void> releaseHelpersPromise;
  const auto releaseHelpers = releaseHelpersPromise.get_future().share();

  auto result = std::async(std::launch::async, [&] {
    pool.runParallel(
        [&](int workerIndex) {
          if (workerIndex == static_cast<int>(pool.numThreads())) {
            throw std::runtime_error("caller failure");
          }

          {
            std::lock_guard<std::mutex> lock(mutex);
            ++helpersStarted;
          }
          helpersStartedCondition.notify_one();
          releaseHelpers.wait();
          ++helpersFinished;
        },
        3);
  });

  bool allHelpersStarted = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    allHelpersStarted = helpersStartedCondition.wait_for(
        lock, std::chrono::seconds(1), [&] { return helpersStarted == 2; });
  }
  const auto statusBeforeRelease =
      result.wait_for(std::chrono::milliseconds(50));
  releaseHelpersPromise.set_value();

  EXPECT_TRUE(allHelpersStarted);
  EXPECT_EQ(statusBeforeRelease, std::future_status::timeout);
  EXPECT_THROW(result.get(), std::runtime_error);
  EXPECT_EQ(helpersFinished, 2);

  std::atomic_int subsequentCalls{0};
  pool.runParallel([&](int) { ++subsequentCalls; }, 3);
  EXPECT_EQ(subsequentCalls, 3);
}

TEST(testThreadPool, runParallelDrainsLaterHelpersAfterHelperFailure) {
  ThreadPool pool(1);

  std::mutex mutex;
  std::condition_variable blockingHelperStartedCondition;
  bool blockingHelperStarted = false;
  std::atomic_int helperCalls{0};
  std::atomic_int helpersFinished{0};
  std::promise<void> releaseHelperPromise;
  const auto releaseHelper = releaseHelperPromise.get_future().share();

  auto result = std::async(std::launch::async, [&] {
    pool.runParallel(
        [&](int workerIndex) {
          if (workerIndex == static_cast<int>(pool.numThreads())) {
            std::unique_lock<std::mutex> lock(mutex);
            blockingHelperStartedCondition.wait(
                lock, [&] { return blockingHelperStarted; });
            return;
          }

          if (helperCalls++ == 0) {
            throw std::runtime_error("helper failure");
          }

          {
            std::lock_guard<std::mutex> lock(mutex);
            blockingHelperStarted = true;
          }
          blockingHelperStartedCondition.notify_all();
          releaseHelper.wait();
          ++helpersFinished;
        },
        3);
  });

  bool helperStarted = false;
  {
    std::unique_lock<std::mutex> lock(mutex);
    helperStarted = blockingHelperStartedCondition.wait_for(
        lock, std::chrono::seconds(1), [&] { return blockingHelperStarted; });
  }
  const auto statusBeforeRelease =
      result.wait_for(std::chrono::milliseconds(50));
  releaseHelperPromise.set_value();

  EXPECT_TRUE(helperStarted);
  EXPECT_EQ(statusBeforeRelease, std::future_status::timeout);
  EXPECT_THROW(result.get(), std::runtime_error);
  EXPECT_EQ(helperCalls, 2);
  EXPECT_EQ(helpersFinished, 1);

  std::atomic_int subsequentCalls{0};
  pool.runParallel([&](int) { ++subsequentCalls; }, 3);
  EXPECT_EQ(subsequentCalls, 3);
}

TEST(testThreadPool, runParallelPropagatesFailureFromEachHelperPosition) {
  ThreadPool pool(1);

  for (int failingHelper = 0; failingHelper < 3; ++failingHelper) {
    std::atomic_int helperCalls{0};
    std::atomic_int helpersFinished{0};

    EXPECT_THROW(
        pool.runParallel(
            [&](int workerIndex) {
              if (workerIndex == static_cast<int>(pool.numThreads())) {
                return;
              }

              const int helperCall = helperCalls++;
              if (helperCall == failingHelper) {
                throw std::runtime_error("helper failure");
              }
              ++helpersFinished;
            },
            4),
        std::runtime_error);

    EXPECT_EQ(helperCalls, 3);
    EXPECT_EQ(helpersFinished, 2);
  }
}

TEST(testThreadPool, testNoThreads) {
  ThreadPool pool(0);

  auto fut1 = pool.run([&](int) -> std::string { return "runs on main thread"; });
  EXPECT_EQ(fut1.get(), "runs on main thread");
}

TEST(testThreadPool, testRunMultipleNoThreads) {
  ThreadPool pool(0);
  std::atomic_int counter;
  counter = 0;

  pool.runParallel([&](int) { counter++; }, 42);

  EXPECT_EQ(counter, 42);
}

TEST(testThreadPool, testMoveOnlyTask) {
  ThreadPool pool(2);

  struct MoveOnlyTask {
    MoveOnlyTask() = default;
    ~MoveOnlyTask() = default;
    MoveOnlyTask(const MoveOnlyTask&) = delete;
    MoveOnlyTask& operator=(const MoveOnlyTask&) = delete;
    MoveOnlyTask(MoveOnlyTask&&) = default;
    MoveOnlyTask& operator=(MoveOnlyTask&&) = default;
    double operator()(int) { return 3.14; }
  };

  auto f = MoveOnlyTask();

  auto result = pool.run(std::move(f));

  EXPECT_EQ(result.get(), 3.14);
}
