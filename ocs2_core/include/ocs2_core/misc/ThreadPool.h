#pragma once

#include <condition_variable>
#include <future>
#include <list>
#include <map>
#include <mutex>
#include <queue>
#include <thread>
#include <vector>

namespace ocs2 {

/**
 * Task interface class for thread pool storage object.
 */
class TaskBase {
 public:
  TaskBase() = default;
  virtual ~TaskBase() = default;

 private:
  friend class ThreadPool;

  virtual void operator()(int id) = 0;

  std::list<int> children_;
};

/**
 * Task class for a specific function type.
 *
 * @tparam Functor: Type of callable task object (functor).
 */
template <typename Functor>
class Task : TaskBase {
 private:
  friend class ThreadPool;

  explicit Task(Functor taskFunction) : packagedTask_(taskFunction) {}
  ~Task() override = default;
  void operator()(int id) override { packagedTask_(id); }

  using ReturnType = typename std::result_of<Functor(int)>::type;
  std::packaged_task<ReturnType(int)> packagedTask_;
};

/**
 * Thread pool class to execute tasks in multiple threads.
 *
 * Note:
 * The task ID is assumed to be unique, so there can't be more than INT_MAX tasks pending.
 */
class ThreadPool {
 public:
  /**
   * Constructor
   *
   * @param [in] nThreads: Number of threads to launch in the pool
   */
  explicit ThreadPool(size_t nThreads = 1, int priority = 0);

  /**
   * Destructor
   */
  ~ThreadPool();

  /**
   * Run a task in another thread
   *
   * @param [in] taskFunction: task function to run in the pool
   * @param [out] thisTaskId: ID of the new task.
   * @return future object with taskFunction retrun value
   */
  template <typename Functor>
  std::future<typename std::result_of<Functor(int)>::type> run(Functor taskFunction, int& thisTaskId);
  template <typename Functor>
  std::future<typename std::result_of<Functor(int)>::type> run(Functor taskFunction);

  /**
   * Run a task that depends on antother task
   *
   * @param [in] taskFunction: task function to run in the pool.
   * @param [in] runAfterId: new task is delayed until task with given ID has finished.
   * @param [out] thisTaskId: new task ID.
   * @return future object with taskFunction retrun value
   */
  template <typename Functor>
  std::future<typename std::result_of<Functor(int)>::type> runAfter(int runAfterId, Functor taskFunction, int& thisTaskId);
  template <typename Functor>
  std::future<typename std::result_of<Functor(int)>::type> runAfter(int runAfterId, Functor taskFunction);

  /**
   * Helper function to run a task N times parallel on the pool
   *
   * @note this is a blocking operation, returns when all tasks are completed.
   *
   * @param [in] taskFunction: task function to run in the pool.
   * @param [in] N: number of times to run taskFunction, if N = 1 it is run on main thread.
   */
  void runParallel(std::function<void(int)> taskFunction, size_t N);

  /**
   * Enable debug log
   *
   * @param [in] debugPrint: re-entrant line printing function
   */
  void enableDebug(std::function<void(const std::string)> debugPrint);

 private:
  /**
   * Push task to ready queue
   *
   * @param [in] taskId next task from queue
   */
  void pushReady(int taskId);

  /**
   * Pop next task from ready queue
   *
   * @return next task from queue
   */
  int popReady();

  /**
   * Thread worker loop
   *
   * @param [in] workerId: thread id number
   */
  void worker(int id);

  /**
   * Run a task asynchronously in another thread
   *
   * @param [in] task: task object
   * @return task ID
   */
  int runTask(std::shared_ptr<TaskBase> task);

  /**
   * Run a task asynchronously in another thread with dependency
   *
   * @param [in] task: task object
   * @param [in] runAfterId: new task is delayed until task with given ID has finished.
   * @return task ID
   */
  int runTaskWithDependency(std::shared_ptr<TaskBase> task, int runAfterId);

  /**
   * Print debug message
   *
   * @param [in] workerId: thread id number
   * @param [in] msg: debug message
   */
  void debugMessage(const std::string msg, int workerId = -1);

  int nextTaskId_;  // protected by taskRegistryLock_
  std::map<int, std::shared_ptr<TaskBase>> taskRegistry_;
  std::mutex taskRegistryLock_;

  bool stop_;  // protected by readyQueueLock_
  std::vector<std::thread> workerThreads_;
  std::queue<int> readyQueue_;
  std::condition_variable readyQueueCondition_;
  std::mutex readyQueueLock_;

  bool debug_;
  std::function<void(const std::string)> debugPrint_;
};

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
template <typename Functor>
std::future<typename std::result_of<Functor(int)>::type> ThreadPool::run(Functor taskFunction) {
  int dummy;
  return run(taskFunction, dummy);
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
template <typename Functor>
std::future<typename std::result_of<Functor(int)>::type> ThreadPool::run(Functor taskFunction, int& thisTaskId) {
  using T = typename std::result_of<Functor(int)>::type;
  int taskId;
  std::future<T> future;

  auto task = new Task<Functor>(taskFunction);
  future = task->packagedTask_.get_future();

  std::shared_ptr<TaskBase> task_shared(static_cast<TaskBase*>(task));

  if (!workerThreads_.empty()) {
    taskId = runTask(std::move(task_shared));
  } else {
    taskId = nextTaskId_++;
    task_shared->operator()(0);
  }

  thisTaskId = taskId;
  return std::move(future);
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
template <typename Functor>
std::future<typename std::result_of<Functor(int)>::type> ThreadPool::runAfter(int runAfterId, Functor taskFunction) {
  int dummy;
  return runAfter(runAfterId, taskFunction, dummy);
}

/**************************************************************************************************/
/**************************************************************************************************/
/**************************************************************************************************/
template <typename Functor>
std::future<typename std::result_of<Functor(int)>::type> ThreadPool::runAfter(int runAfterId, Functor taskFunction, int& thisTaskId) {
  using T = typename std::result_of<Functor(int)>::type;
  int taskId;
  std::future<T> future;

  auto task = new Task<Functor>(taskFunction);
  future = task->packagedTask_.get_future();

  std::shared_ptr<TaskBase> task_shared(static_cast<TaskBase*>(task));

  if (!workerThreads_.empty()) {
    taskId = runTaskWithDependency(std::move(task_shared), runAfterId);
  } else {
    taskId = nextTaskId_++;
    task_shared->operator()(0);
  }

  thisTaskId = taskId;
  return std::move(future);
}

}  // namespace ocs2
