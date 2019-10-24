#include <ocs2_core/misc/ThreadPool.h>

namespace ocs2 {

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::ThreadPool(size_t nThreads) : nextTaskId_(0), stop_(false) {
  workerThreads_.reserve(nThreads);
  for (size_t i = 0; i < nThreads; i++) {
    workerThreads_.push_back(std::thread(&ThreadPool::worker, this, i));
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::~ThreadPool() {
  {  // set exit flag, wake up threads and join
    std::lock_guard<std::mutex> lock(readyQueueLock_);
    stop_ = true;
  }
  readyQueueCondition_.notify_all();
  for (auto& thread : workerThreads_) {
    thread.join();
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::pushReady(int taskId) {
  {
    std::lock_guard<std::mutex> lock(readyQueueLock_);
    readyQueue_.push(taskId);
  }
  readyQueueCondition_.notify_one();
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
int ThreadPool::popReady() {
  int task;
  {
    std::unique_lock<std::mutex> lock(readyQueueLock_);
    readyQueueCondition_.wait(lock, [&] { return stop_ || !readyQueue_.empty(); });
    if (!readyQueue_.empty()) {
      task = readyQueue_.front();
      readyQueue_.pop();
    }
  }
  return task;
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::worker(int workerId) {
  while (1) {
    int taskId;

    taskId = popReady();  // can return if stop_ flag is set

    // exit condition
    if (stop_) {
      break;
    }

    std::shared_ptr<TaskBase> task;
    {
      std::lock_guard<std::mutex> lock(taskRegistryLock_);
      auto taskIter = taskRegistry_.find(taskId);
      task = taskIter->second;
    }

    task->operator()(workerId);

    {  // Atomically make dependent tasks ready and destroy finished task
      std::lock_guard<std::mutex> lock(taskRegistryLock_);

      for (int& child : task->children_) {
        pushReady(child);
      }

      auto taskIter = taskRegistry_.find(taskId);
      taskRegistry_.erase(taskIter);
    }
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
int ThreadPool::runTask(std::shared_ptr<TaskBase> task) {
  int taskId;

  {
    taskId = nextTaskId_++;
    std::lock_guard<std::mutex> lock(taskRegistryLock_);
    taskRegistry_.insert({taskId, std::move(task)});
  }

  pushReady(taskId);
  return taskId;
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
int ThreadPool::runTaskWithDependency(std::shared_ptr<TaskBase> task, int runsAfterID) {
  bool readyToRun = false;
  int taskId;

  {
    taskId = nextTaskId_++;
    // register task with id
    std::lock_guard<std::mutex> lock(taskRegistryLock_);
    taskRegistry_.insert({taskId, std::move(task)});

    // register task at parent
    auto parent = taskRegistry_.find(runsAfterID);
    if (parent != taskRegistry_.end()) {
      parent->second->children_.push_back(taskId);
    } else {
      readyToRun = true;
    }
  }

  if (readyToRun) {
    pushReady(taskId);
  }

  return taskId;
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::runMultiple(std::function<void(int)> taskFunction, size_t N) {
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
