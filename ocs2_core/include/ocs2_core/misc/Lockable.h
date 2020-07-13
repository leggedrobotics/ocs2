#pragma once

#include <memory>
#include <mutex>

namespace ocs2 {

/**
 * Adds a mutex interface to an object such that it can be locked with RAII.
 *
 * This object should not be destructed polymorphically. i.e. do not store a Lockable<T> as T& or T*
 *
 * Example usage:
 * Lockable<std::string> sharedString{std::string("Hello")}; // Create a lockable string.
 * {
 *    std::lock_guard<Lockable<std::string>> lock(sharedString); // Acquire the lock on sharedString
 *    std::cout << sharedString << std::endl; // Use the lockable object as a normal object
 * } // lock_guard goes out of scope -> unlocks the mutex on sharedString
 *
 * @tparam T : Object to be locked
 */
template <typename T>
class Lockable : public T {
 public:
  template <typename... Args>
  Lockable(Args&&... args) : T(std::forward<Args>(args)...){};

  Lockable(Lockable const&) = delete;
  Lockable& operator=(Lockable const&) = delete;

  void lock() { m_.lock(); }
  void unlock() { m_.unlock(); }
  bool try_lock() { return m_.try_lock(); }

 private:
  mutable std::mutex m_;
};

/**
 * Wraps a unique_ptr<T> and mutex into one class
 *
 * Example usage:
 * LockablePtr<T> lockableStringPtr = makeLockablePtr<std::string>("hello, world"); // Create a lockable string.
 * {
 *      std::lock_guard<LockablePtr<std::string>> lock(lockableStringPtr); // Acquire the lock on lockableStringPtr
 *      std::cout << *lockableStringPtr << std::endl; // Use with pointer as usual
 * } // lock_guard goes out of scope -> unlocks the mutex on lockableStringPtr
 *
 * @tparam T : Object to be wrapped
 */
template <typename T>
using LockablePtr = Lockable<std::unique_ptr<T>>;

template <typename T, typename... Args>
LockablePtr<T> makeLockablePtr(Args&&... args) {
  // TODO (rgrandia) : use make_unique after switching to cpp-14
  return LockablePtr<T>(new T(std::forward<Args>(args)...));
}

}  // namespace ocs2
