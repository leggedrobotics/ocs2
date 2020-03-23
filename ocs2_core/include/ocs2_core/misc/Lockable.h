#pragma once

#include <mutex>

namespace ocs2 {

/**
 * Adds a mutex interface to an object such that it can be locked with RAII.
 *
 * This object should not be destructed polymorphically. i.e. do not store a Lockable<T> as a std::unique_ptr<T>, but use instead
 * std::unique_ptr<Lockable<T>>.
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
  explicit Lockable(T t) : T(std::move(t)) {}

  void lock() { m_.lock(); }
  void unlock() { m_.unlock(); }
  bool try_lock() { return m_.try_lock(); }

 private:
  mutable std::mutex m_;
};

}  // namespace ocs2
