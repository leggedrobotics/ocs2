#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

/**
 * Thread pool class to execute tasks in multiple threads.
 */
class ThreadPool {
 public:
  /**
   * Constructor
   *
   * @param [in] nThreads: Number of threads to launch in the pool
   */
  ThreadPool(size_t nThreads);

  /**
   * Destructor
   */
  ~ThreadPool();

  /**
   * Run a task asynchronously in another thread
   *
   * @param [in] taskFunction: task function to run in the pool
   * @return future object for synchronization
   */
  std::future<void> run(std::function<void(void)> taskFunction);

 private:
  using Task = std::packaged_task<void(void)>;

  /**
   * Thread worker loop
   *
   * @param [in] workerId: thread id number
   */
  void worker(int id);

  /**
   * Retrieve the next task in the queue
   *
   * @return next task from queue
   */
  Task nextTask();

  std::atomic<bool> stop_;
  std::vector<std::thread> workerThreads_;
  std::queue<Task> taskQueue_;
  std::condition_variable taskQueueCondition_;
  std::mutex taskQueueLock_;
};
