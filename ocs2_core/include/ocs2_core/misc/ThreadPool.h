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
  using Task = std::function<void(void)>;

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
   * @param nThreads: Number of threads to launch in the pool
   */
  void run(Task task);

 private:
  /**
   * Thread worker loop
   *
   * @param [in] workerId: therad id number
   */
  void worker(int id);

  /**
   * Retrieve the next task in the queue
   *
   * @param [out] task: task reference
   */
  void nextTask(Task& task);

  std::atomic<bool> active_;
  std::vector<std::thread> workerThreads_;
  std::queue<Task> taskQueue_;
  std::condition_variable taskQueueCondition_;
  std::mutex taskQueueLock_;
};
