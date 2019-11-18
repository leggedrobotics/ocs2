#include <gtest/gtest.h>
#include <ocs2_core/misc/ThreadPool.h>

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

TEST(testThreadPool, testIdIsDifferent) {
  ThreadPool pool(1);
  int firstId, secondId;

  auto doNothing = [](int) { /* nop */ };
  std::ignore = pool.run(doNothing, firstId);
  std::ignore = pool.run(doNothing, secondId);

  EXPECT_TRUE(firstId != secondId);
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

  std::string data1, data2;

  res1 = pool.run([&data1, barrier](int) {
    data1 = "running";
    barrier.wait();
    data1 = "done";
  });
  res2 = pool.run([&data2, barrier](int) {
    data2 = "running";
    barrier.wait();
    data2 = "done";
  });

  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // check that both threads are blocking
  EXPECT_EQ(data1, "running");
  EXPECT_EQ(data2, "running");

  // signal threads
  barrier_promise.set_value();

  // wait for pool to run tasks
  std::future_status status1 = res1.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status1, std::future_status::ready);

  std::future_status status2 = res2.wait_for(std::chrono::seconds(1));
  ASSERT_EQ(status2, std::future_status::ready);

  EXPECT_EQ(data1, "done");
  EXPECT_EQ(data2, "done");

  res1.get();
  res2.get();
}

TEST(testThreadPool, testTaskDependency) {
  ThreadPool pool(2);
  // pool.enableDebug([](const std::string msg) { std::cout << "[ThreadPool]: " << msg << std::endl; });

  std::future<void> fut1, fut2, fut3;
  int id1, id2;

  std::promise<void> barrier_promise;
  std::shared_future<void> barrier = barrier_promise.get_future();

  std::string data = "";

  fut1 = pool.run(
      [&](int) {
        barrier.get();
        data = "first";
      },
      id1);
  fut2 = pool.runAfter(id1, [&](int) { data += " second"; }, id2);
  fut3 = pool.runAfter(id2, [&](int) { data += " third"; });

  // check that no thread has run
  EXPECT_EQ(data, "");

  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  barrier_promise.set_value();

  fut3.get();

  // check that order is correct
  EXPECT_EQ(data, "first second third");
}

TEST(testThreadPool, testDependsOnFinishedTask) {
  ThreadPool pool(1);
  std::future<void> fut;
  int firstId;

  auto doNothing = [](int) { /* nop */ };

  fut = pool.run(doNothing, firstId);
  fut.get();
  // first task has finihed here

  fut = pool.runAfter(firstId, doNothing);
  std::future_status status = fut.wait_for(std::chrono::seconds(1));
  // second task should run without delay

  ASSERT_EQ(status, std::future_status::ready);
  fut.get();
}

TEST(testThreadPool, testDependsOnNonexistingTask) {
  ThreadPool pool(1);
  std::future<void> fut;

  fut = pool.runAfter(0xdead, [](int) { /* nop */ });

  std::future_status status = fut.wait_for(std::chrono::seconds(1));

  // In case of an unknown ID dependency is regarded as fulfilled.
  // The task should run without delay.
  ASSERT_EQ(status, std::future_status::ready);
  fut.get();
}

TEST(testThreadPool, testRunMultiple) {
  ThreadPool pool(2);
  std::atomic_int counter;
  counter = 0;

  pool.runParallel([&](int) { counter++; }, 42);

  EXPECT_EQ(counter, 42);
}

TEST(testThreadPool, testNoThreads) {
  ThreadPool pool(0);
  int id;

  auto fut1 = pool.run([&](int) -> std::string { return "foo"; }, id);
  EXPECT_EQ(fut1.get(), "foo");

  auto fut2 = pool.runAfter(id, [&](int) -> std::string { return "ok"; });
  EXPECT_EQ(fut2.get(), "ok");
}
