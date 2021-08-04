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

#pragma once

#include <memory>
#include <mutex>

namespace ocs2 {

/**
 * Provides an interface to a unique_ptr<T> and a mutex.
 * - The mutex is locked during construction and released during destruction.
 * - The object T can be called directly through this interface.
 * - The unique_ptr<T> can be swapped or reset.
 *
 * The user of this class has to make sure the unique_ptr<T> and the mutex outlive this interface.
 */
template <typename T>
class LockedPtr {
 public:
  LockedPtr(std::unique_ptr<T>& p, std::mutex& m) : p_(p), lk_(m) {}
  LockedPtr(std::unique_ptr<T>& p, std::mutex& m, std::adopt_lock_t) : p_(p), lk_(m, std::adopt_lock) {}

  /// Access
  T* operator->() { return p_.get(); }
  const T* operator->() const { return p_.get(); }
  T& operator*() { return *p_; }
  const T& operator*() const { return *p_; }

  /// Resets the wrapped unique ptr.
  void reset(std::unique_ptr<T> p) noexcept { p_ = std::move(p); }

  /// Swaps the wrapped unique ptr.
  void swap(std::unique_ptr<T>& p) noexcept { p_.swap(p); }

  /// Returns true if the wrapped unique ptr is not null.
  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  std::unique_ptr<T>& p_;
  std::unique_lock<std::mutex> lk_;
};

/**
 * Provides a const interface to a pointer T* and a mutex.
 * - The mutex is locked during construction and released during destruction.
 * - const methods on T* can be called directly through this interface.
 *
 * The user of this class has to make sure the original object T and the mutex outlive this interface.
 */
template <typename T>
class LockedConstPtr {
 public:
  LockedConstPtr(const T* p, std::mutex& m) : p_(p), lk_(m) {}
  LockedConstPtr(const T* p, std::mutex& m, std::adopt_lock_t) : p_(p), lk_(m, std::adopt_lock) {}

  /// const access
  const T* operator->() const { return p_; }
  const T& operator*() const { return *p_; }

  /// Returns true if the wrapped pointer is not null.
  explicit operator bool() const noexcept { return p_ != nullptr; }

 private:
  const T* p_;
  std::unique_lock<std::mutex> lk_;
};

/**
 * This class wraps a std::unique_ptr<T> with a mutex and provides an interface that automatically locks the mutex.
 *
 * - Calling the wrapped object:
 * 1) The operator->() of this class can be used to directly call a member function of T while holding the lock.
 * 2) lock() will return a custom pointer (LockedPtr/LockedConstPtr) to the wrapped object that holds the lock for the lifetime of that
 * pointer. This is useful if the lock should be held during multiple member function calls.
 *
 * - Wrapping a new object:
 * The wrapped object can be swapped or reset either directly through the Synchronized<T> or through a LockedPtr.
 *
 * - Locking multiple objects:
 * A helper function "synchronizeLock" is available to lock multiple Synchronized<T>s at the same time, similar to std::lock(.., ..)
 */
template <typename T>
class Synchronized {
 public:
  /// Default constructor : Owns nothing
  Synchronized() = default;

  /// Construct from unique_ptr
  explicit Synchronized(std::unique_ptr<T> p) noexcept : p_(std::move(p)) {}

  /// Destructor : Obtains the lock and destroys the owned object.
  ~Synchronized() {
    std::unique_lock<std::mutex> lk(m_);
    p_.reset();
  }

  // Disable copy operations
  Synchronized(const Synchronized&) = delete;
  Synchronized& operator=(const Synchronized&) = delete;

  /// Move construction
  Synchronized(Synchronized&& other) noexcept {
    // Current object is being created, no need to lock this->m_
    std::lock_guard<std::mutex> lkOther(other.m_);
    p_ = std::move(other.p_);
  }

  /// Move assignment
  Synchronized& operator=(Synchronized&& other) noexcept {
    // Both other and this object already exist. Need to lock both
    std::unique_lock<std::mutex> lkThis(m_, std::defer_lock);
    std::unique_lock<std::mutex> lkOther(other.m_, std::defer_lock);
    std::lock(lkOther, lkThis);
    p_.swap(other.p_);
    return *this;
  }

  /// Reset the wrapped object. The object is reseated while holding the lock.
  void reset(std::unique_ptr<T> p) noexcept {
    swap(p);  // by using swap, the old object is destroyed after the lock is released
  }

  /// Swaps the wrapped object. The object is swapped while holding the lock.
  void swap(std::unique_ptr<T>& p) noexcept {
    std::lock_guard<std::mutex> lk(m_);
    p_.swap(p);
  }

  /// Returns a pointer that holds the lock to the wrapped object. Lifetime of the LockedPtr<> determines the lifetime of the lock.
  LockedPtr<T> lock() { return {p_, m_}; }
  LockedConstPtr<T> lock() const { return {p_.get(), m_}; }

  /// Returns a pointer that adopts and holds the lock to the wrapped object. Lifetime of the LockedPtr<> determines the lifetime of the
  /// lock.
  LockedPtr<T> adoptLock() { return {p_, m_, std::adopt_lock}; }
  LockedConstPtr<T> adoptLock() const { return {p_.get(), m_, std::adopt_lock}; }

  /// Apply operations directly on the held object. Holds the lock for the duration of the call.
  LockedPtr<T> operator->() { return this->lock(); }
  LockedConstPtr<T> operator->() const { return this->lock(); }

  /// Get direct access to the internal mutex.
  std::mutex& getMutex() const noexcept { return m_; }

 private:
  mutable std::mutex m_;
  std::unique_ptr<T> p_;
};

/**
 * Helper function to lock multiple synchronized<T> objects simultaneously, preventing deadlocks.
 * @return a tuple of LockedPtr<T> / LockedConstPtr<T>.
 */
template <typename... SV>
auto synchronizeLock(SV&... sv) -> std::tuple<decltype(sv.adoptLock())...> {
  std::lock(sv.getMutex()...);
  using tuple_t = std::tuple<decltype(sv.adoptLock())...>;
  return tuple_t(sv.adoptLock()...);
}

}  // namespace ocs2
