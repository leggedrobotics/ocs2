#include <ocs2_core/misc/SetThreadPriority.h>
#include <ocs2_core/misc/ThreadPool.h>

namespace ocs2 {

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::ThreadPool(size_t nThreads, int priority) {
  workerThreads_.reserve(nThreads);
  for (size_t i = 0; i < nThreads; i++) {
    workerThreads_.emplace_back(&ThreadPool::worker, this, i);
    setThreadPriority(priority, workerThreads_.back());
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::~ThreadPool() {
  {  // set exit flag, wake up threads and join
    std::lock_guard<std::mutex> lock(taskQueueLock_);
    stop_ = true;
  }
  taskQueueCondition_.notify_all();
  for (auto& thread : workerThreads_) {
    if (thread.joinable()) {
      thread.join();
    }
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::worker(int workerIndex) {
  while (true) {
    std::unique_ptr<ThreadPool::TaskBase> taskPtr;
    {
      std::unique_lock<std::mutex> lock(taskQueueLock_);
      taskQueueCondition_.wait(lock, [this] { return !taskQueue_.empty() || stop_; });

      // exit condition
      if (stop_) {
        break;
      }

      // pop the first task
      if (!taskQueue_.empty()) {
        taskPtr = std::move(taskQueue_.front());
        taskQueue_.pop();
      }
    }

    if (taskPtr) {
      taskPtr->operator()(workerIndex);
    }
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::runTask(std::unique_ptr<TaskBase> taskPtr) {
  {
    std::lock_guard<std::mutex> lock(taskQueueLock_);
    taskQueue_.push(std::move(taskPtr));
  }
  taskQueueCondition_.notify_one();
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::runParallel(std::function<void(int)> taskFunction, int N) {
  // Launch tasks in helper threads
  std::vector<std::future<void>> futures;
  if (N > 1) {
    const int numHelpers = N - 1;
    futures.reserve(numHelpers);
    for (int i = 0; i < numHelpers; ++i) {
      futures.emplace_back(run(taskFunction));
    }
  }

  // Execute one instance in this thread.
  const auto workerId = static_cast<int>(numThreads());  // threadpool workers use ID 0 -> nThreads - 1
  taskFunction(workerId);

  // Wait for helpers to finish.
  for (auto&& fut : futures) {
    fut.get();
  }
}

}  // namespace ocs2
