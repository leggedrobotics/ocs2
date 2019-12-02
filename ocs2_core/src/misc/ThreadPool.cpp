#include <ocs2_core/misc/SetThreadPriority.h>
#include <ocs2_core/misc/ThreadPool.h>

#define DEBUG_MSG(...)         \
  if (debug_) {                \
    debugMessage(__VA_ARGS__); \
  }

namespace ocs2 {

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
ThreadPool::ThreadPool(size_t nThreads, int priority) : nextTaskId_(0), stop_(false), debug_(false) {
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
  while (true) {
    int taskId;

    taskId = popReady();  // can return if stop_ flag is set

    // exit condition
    if (stop_) {
      DEBUG_MSG("stop", workerId);
      break;
    }

    std::shared_ptr<TaskBase> task;
    {
      std::lock_guard<std::mutex> lock(taskRegistryLock_);
      auto taskIter = taskRegistry_.find(taskId);
      task = taskIter->second;
    }

    DEBUG_MSG("run  task " + std::to_string(taskId), workerId);
    task->operator()(workerId);
    DEBUG_MSG("done task " + std::to_string(taskId), workerId);

    {  // Atomically make dependent tasks ready and destroy finished task
      std::lock_guard<std::mutex> lock(taskRegistryLock_);

      for (int& child : task->children_) {
        DEBUG_MSG("ready  task " + std::to_string(child), workerId);
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
    std::lock_guard<std::mutex> lock(taskRegistryLock_);

    taskId = nextTaskId_++;
    taskRegistry_.insert({taskId, std::move(task)});
  }
  DEBUG_MSG("create task " + std::to_string(taskId));
  DEBUG_MSG("ready  task " + std::to_string(taskId));

  pushReady(taskId);
  return taskId;
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
int ThreadPool::runTaskWithDependency(std::shared_ptr<TaskBase> task, int runAfterId) {
  bool readyToRun = false;
  int taskId;

  {
    std::lock_guard<std::mutex> lock(taskRegistryLock_);

    taskId = nextTaskId_++;
    // register task with id
    taskRegistry_.insert({taskId, std::move(task)});

    // register task at parent
    auto parent = taskRegistry_.find(runAfterId);
    if (parent != taskRegistry_.end()) {
      parent->second->children_.push_back(taskId);
    } else {
      readyToRun = true;
    }
  }
  DEBUG_MSG("create task " + std::to_string(taskId) + " depends on " + std::to_string(runAfterId));

  if (readyToRun) {
    DEBUG_MSG("ready  task " + std::to_string(taskId));
    pushReady(taskId);
  }

  return taskId;
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::runParallel(std::function<void(int)> taskFunction, size_t N) {
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

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::debugMessage(const std::string msg, int workerId) {
  if (debug_) {
    std::string head;
    if (workerId >= 0) {
      head = "[Thread " + std::to_string(workerId) + "] ";
    }
    debugPrint_(head + msg);
  }
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
void ThreadPool::enableDebug(std::function<void(const std::string)> debugPrint) {
  debugPrint_ = debugPrint;
  debug_ = true;
}

}  // namespace ocs2
