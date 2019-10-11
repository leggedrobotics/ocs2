#include <gtest/gtest.h>
#include <future>
#include <ocs2_core/misc/ThreadPool.h>

TEST(testThreadPool, testCanExecuteTask) {
  ThreadPool pool(1);
  std::function<void(void)> task;

  std::promise<void> barrier;
  std::future<void> barrier_future = barrier.get_future();
  std::future_status status;

  // send task to pool
  task = [&] { barrier.set_value(); };
  pool.run(task);

  // wait for pool to run task with timeout
  status = barrier_future.wait_for(std::chrono::milliseconds(10));
  EXPECT_EQ(status, std::future_status::ready);
}
