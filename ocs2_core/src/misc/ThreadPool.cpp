#include <ocs2_core/misc/ThreadPool.h>

ThreadPool::ThreadPool(size_t nThreads) {
  stop_.store(false);
  workerThreads_.reserve(nThreads);
  for (size_t i = 0; i < nThreads; i++) {
    workerThreads_.push_back(std::thread(&ThreadPool::worker, this, i));
  }
}

ThreadPool::~ThreadPool() {
  // set exit flag, wake up threads and join
  {
    std::unique_lock<std::mutex> lock(taskQueueLock_);
    stop_.store(true);
  }
  taskQueueCondition_.notify_all();
  for (auto& thread : workerThreads_) {
    thread.join();
  }
}

std::future<void> ThreadPool::run(std::function<void(void)> taskFunction) {
  Task task(taskFunction);
  std::future<void> barrier = task.get_future();
  {
    std::unique_lock<std::mutex> lock(taskQueueLock_);
    taskQueue_.push(std::move(task));
  }
  taskQueueCondition_.notify_one();
  return barrier;
}

void ThreadPool::worker(int workerId) {
  while (1) {
    Task task;

    nextTask(task);

    // exit condition
    if (stop_.load()) {
      break;
    }

    task();
  }  // end while
}

void ThreadPool::nextTask(Task& task) {
  {
    {
      std::unique_lock<std::mutex> lock(taskQueueLock_);
      taskQueueCondition_.wait(lock, [&] { return stop_.load() || !taskQueue_.empty(); });
      if (!taskQueue_.empty()) {
        task = std::move(taskQueue_.front());
        taskQueue_.pop();
      }
    }
  }
}
