#include <ocs2_core/misc/SetThreadPriority.h>
#include <ocs2_core/misc/ThreadPool.h>

namespace ocs2 {

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::ThreadPool(int nThreads, int priority) {
  workerThreads_.reserve(nThreads);
  for (int i = 0; i < nThreads; i++) {
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
    thread.join();
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
std::unique_ptr<ThreadPool::TaskBase> ThreadPool::popReady() {
  std::unique_ptr<TaskBase> task;
  {
    std::unique_lock<std::mutex> lock(taskQueueLock_);
    taskQueueCondition_.wait(lock, [this] { return stop_ || !taskQueue_.empty(); });
    if (!taskQueue_.empty()) {
      task = std::move(taskQueue_.front());
      taskQueue_.pop();
    }
  }
  return std::move(task);
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::worker(int workerIndex) {
  while (true) {
    auto taskPtr = popReady();  // returns either a task or nullptr if stop_ is set

    // exit condition
    if (stop_) {
      break;
    }

    taskPtr->operator()(workerIndex);
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
  if (N <= 1) {
    taskFunction(0);
    return;
  }

  std::vector<std::future<void>> futures;
  futures.reserve(N);

  for (int i = 0; i < N; i++) {
    futures.push_back(std::move(run(taskFunction)));
  }

  for (auto&& fut : futures) {
    fut.get();
  }
}

}  // namespace ocs2
