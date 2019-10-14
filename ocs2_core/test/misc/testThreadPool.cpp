#include <gtest/gtest.h>
#include <ocs2_core/misc/ThreadPool.h>

TEST(testThreadPool, testCanExecuteTask) {
  ThreadPool pool(1);
  std::future<void> res;
  int answer = 0;

  // send task to pool
  res = pool.run([&answer] { answer = 42; });

  // wait for pool to run task with timeout
  res.wait_for(std::chrono::milliseconds(10));
  EXPECT_EQ(answer, 42);
}

TEST(testThreadPool, testCanExecuteMultipleTasks) {
  ThreadPool pool(2);
  std::function<void(void)> task;
  std::future<void> res1, res2;

  std::promise<void> barrier_promise;
  std::shared_future<void> barrier = barrier_promise.get_future();

  std::string data1, data2;

  // send task to pool
  res1 = pool.run([&data1, barrier] {
    data1 = "ready";
    barrier.wait();
    data1 = "done";
  });
  res2 = pool.run([&data2, barrier] {
    data2 = "ready";
    barrier.wait();
    data2 = "done";
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(data1, "ready");
  EXPECT_EQ(data2, "ready");

  // signal threads
  barrier_promise.set_value();

  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  EXPECT_EQ(data1, "done");
  EXPECT_EQ(data2, "done");

  // wait for pool to run tasks
  std::future_status status1 = res1.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status1, std::future_status::ready);

  std::future_status status2 = res2.wait_for(std::chrono::milliseconds(1));
  EXPECT_EQ(status2, std::future_status::ready);

  res1.get();
  res2.get();
}

TEST(testThreadPool, testPropagateException) {
  ThreadPool pool(1);
  std::function<void(void)> task;

  // send task to pool
  task = [] { throw std::string("exception"); };
  EXPECT_THROW(pool.run(task).get(), std::string);
}
