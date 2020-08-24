#pragma once

#include <memory>
#include <mutex>

#include <boost/thread/synchronized_value.hpp>

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

/**
 * Grants access to the object T and the unique_ptr that holds it.
 * Holds the SynchronizedPtr's mutex for the lifetime of the object.
 */
template <typename T>
class LockedPtr {
 public:
  LockedPtr(std::unique_ptr<T>& p, std::mutex& m) : p_(p), lk_(m) {}

  /// Access
  T* operator->() { return p_.get(); }
  const T* operator->() const { return p_.get(); }
  T& operator*() { return *p_; }
  const T& operator*() const { return *p_; }

  /// Resets the unique ptr within the SynchronizedPtr.
  void reset(std::unique_ptr<T> p) noexcept { p_ = std::move(p); }

  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  std::unique_ptr<T>& p_;
  std::unique_lock<std::mutex> lk_;
};

/**
 * Grants const access to the wrapped object T.
 * Holds the SynchronizedPtr's mutex for the lifetime of the object.
 */
template <typename T>
class LockedConstPtr {
 public:
  LockedConstPtr(const T* p, std::mutex& m) : p_(p), lk_(m) {}

  /** @return a pointer to the protected value */
  const T* operator->() const { return p_; }
  const T& operator*() const { return *p_; }

  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  const T* p_;
  std::unique_lock<std::mutex> lk_;
};

template <typename T>
class SynchronizedPtr {
 public:
  /// Default constructor : Owns nothing
  SynchronizedPtr() = default;

  /// Construct from unique_ptr
  explicit SynchronizedPtr(std::unique_ptr<T> p) noexcept : p_(std::move(p)) {}

  /// Destructor : Obtains the lock and destroys the owned object.
  ~SynchronizedPtr() {
    std::unique_lock<std::mutex> lk(m_);
    p_.reset();
  }

  // Disable copy operations
  SynchronizedPtr(const SynchronizedPtr&) = delete;
  SynchronizedPtr& operator=(const SynchronizedPtr&) = delete;

  /// Move construction
  SynchronizedPtr(SynchronizedPtr&& other) noexcept {
    // Current object is being created, no need to lock this->m_
    std::unique_lock<std::mutex> lkOther(other.m_);
    p_ = std::move(other.p_);
  }

  /// Move assignment
  SynchronizedPtr& operator=(SynchronizedPtr&& other) noexcept {
    // Both other and this object already exist. Need to lock both
    std::unique_lock<std::mutex> lkThis(m_, std::defer_lock);
    std::unique_lock<std::mutex> lkOther(other.m_, std::defer_lock);
    std::lock(lkOther, lkThis);
    p_ = std::move(other.p_);
    return *this;
  }

  /// reset
  void reset(std::unique_ptr<T> p) noexcept {
    std::unique_lock<std::mutex> lk(m_);
    p_ = std::move(p);
  }

  /// Synchronize for scoped operations. Lifetime of the returned value determines the lifetime of the lock.
  LockedPtr<T> synchronize() { return {p_, m_}; }
  LockedConstPtr<T> synchronize() const { return {p_.get(), m_}; }

  /// Apply operations directly on the held object. Holds the lock for the duration of the call
  LockedPtr<T> operator->() { return this->synchronize(); }
  LockedConstPtr<T> operator->() const { return this->synchronize(); }

  /// returns false if a nullptr is held, true otherwise.
  explicit operator bool() const noexcept {
    std::unique_lock<std::mutex> lk(m_);
    return p_.operator bool();
  }

  T* getUnsafe() { return p_.get(); }

 private:
  mutable std::mutex m_;
  std::unique_ptr<T> p_;
};

}  // namespace ocs2
