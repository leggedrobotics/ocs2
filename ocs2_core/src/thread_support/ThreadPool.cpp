/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <ocs2_core/thread_support/SetThreadPriority.h>
#include <ocs2_core/thread_support/ThreadPool.h>

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
