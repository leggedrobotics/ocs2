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

  Task(Functor taskFunction) : packaged_task_(taskFunction) {}
  ~Task() override{};
  void operator()(int id) override { packaged_task_(id); }

  using ReturnType = typename std::result_of<Functor(int)>::type;
  std::packaged_task<ReturnType(int)> packaged_task_;
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
  ThreadPool(size_t nThreads);

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

  int nextTaskId_;  // protected by taskRegistryLock_
  std::map<int, std::shared_ptr<TaskBase>> taskRegistry_;
  std::mutex taskRegistryLock_;

  bool stop_;  // protected by readyQueueLock_
  std::vector<std::thread> workerThreads_;
  std::queue<int> readyQueue_;
  std::condition_variable readyQueueCondition_;
  std::mutex readyQueueLock_;
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

  Task<Functor>* task = new Task<Functor>(taskFunction);
  future = task->packaged_task_.get_future();

  std::shared_ptr<TaskBase> task_shared(static_cast<TaskBase*>(task));
  taskId = runTask(std::move(task_shared));

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

  Task<Functor>* task = new Task<Functor>(taskFunction);
  future = task->packaged_task_.get_future();

  std::shared_ptr<TaskBase> task_shared(static_cast<TaskBase*>(task));
  taskId = runTaskWithDependency(std::move(task_shared), runAfterId);

  thisTaskId = taskId;
  return std::move(future);
}

}  // namespace ocs2
