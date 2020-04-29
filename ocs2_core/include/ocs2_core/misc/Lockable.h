#pragma once

#include <memory>
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
class LockablePtr {
 public:
  using pointer = typename std::unique_ptr<T>::pointer;
  using lvalueReference = typename std::add_lvalue_reference<T>::type;

  /// Construct from unique_ptr.
  explicit LockablePtr(std::unique_ptr<T> p) : p_(std::move(p)){};

  /// Construct from raw ptr. Will assume unique ownership over the passed object.
  explicit LockablePtr(T* p) : p_(p){};

  /// Move constructor.
  LockablePtr(LockablePtr&& other) noexcept {
    std::lock_guard<std::mutex> lock(other.m_);
    p_ = std::move(other.p_);
  };

  /// Move assignment.
  LockablePtr& operator=(LockablePtr&& other) noexcept {
    std::lock(m_, other.m_);
    std::lock_guard<std::mutex> lockThis(m_, std::adopt_lock);
    std::lock_guard<std::mutex> lockOther(other.m_, std::adopt_lock);
    this->p_ = std::move(other.p_);
    return *this;
  };

  /* Delete copy operations */
  LockablePtr(const LockablePtr&) = delete;
  LockablePtr operator=(const LockablePtr&) = delete;

  /// Dereference the stored pointer.
  lvalueReference operator*() { return *p_; }

  /// Return the stored pointer.
  pointer operator->() const noexcept { return get(); }

  /// Return the stored pointer.
  pointer get() const noexcept { return p_.get(); }

  void lock() { m_.lock(); }
  void unlock() { m_.unlock(); }
  bool try_lock() { return m_.try_lock(); }

 private:
  std::unique_ptr<T> p_;
  mutable std::mutex m_;
};

template <typename T>
LockablePtr<T> makeLockablePtr(T&& t) {
  // TODO (rgrandia) : use make_unique after switching to cpp-14
  return LockablePtr<T>{new T(std::forward<T>(t))};
}

template <typename T>
using SharedLockablePPtr = std::shared_ptr<LockablePtr<T>>;

template <typename T>
SharedLockablePPtr<T> makeSharedLockablePPtr(T&& t) {
  // TODO (rgrandia) : use make_unique after switching to cpp-14
  return std::make_shared<LockablePtr<T>>(new T(std::forward<T>(t)));
}

template <typename T>
SharedLockablePPtr<T> makeSharedLockablePPtr(std::unique_ptr<T> tPtr) {
  return std::make_shared<LockablePtr<T>>(std::move(tPtr));
}

}  // namespace ocs2
