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

#include <ocs2_core/thread_support/Synchronized.h>

namespace ocs2 {

/**
 * Wraps a value with a thread-safe buffer. A value remains constant and can be accessed through the get function until
 * updateFromBuffer is called.
 *
 * In the meantime, multiple threads can set new values to the buffer. The active value is not protected by a mutex, so
 * only one thread should access/modify the active value (i.e. not simultaneously calling get() and updateFromBuffer()).
 *
 * @tparam T : wrapped type
 */
template <typename T>
class BufferedValue {
 public:
  /**
   * Constructor initializes with a given value and an empty buffer.
   * @param value
   */
  explicit BufferedValue(T value) : activeValue_(std::move(value)), buffer_(nullptr){};

  /** Read the currently active value. */
  const T& get() const { return activeValue_; }

  /** Read/write the currently active value. */
  T& get() { return activeValue_; }

  /** Copy a new value into the buffer. */
  void setBuffer(const T& value) { buffer_.reset(std::unique_ptr<T>(new T(value))); }

  /** Move a new value into the buffer. */
  void setBuffer(T&& value) { buffer_.reset(std::unique_ptr<T>(new T(std::move(value)))); }

  /**
   * Replaces the active value with the value in the buffer.
   * The active value is not mutex protected so this method is NOT thread-safe w.r.t. get()
   * The buffer is mutex protected, so this method is thread-safe w.r.t. setBuffer()
   * @return True: the active value was updated, False: the active value was not updated.
   */
  bool updateFromBuffer() {
    // Read buffer with a pointer swap to minimize time under the lock. The swapped value will be null if there was no new value set.
    std::unique_ptr<T> updatedValuePtr(nullptr);
    buffer_.swap(updatedValuePtr);

    if (updatedValuePtr != nullptr) {
      activeValue_ = std::move(*updatedValuePtr);
      return true;
    } else {
      return false;
    }
  }

 private:
  T activeValue_;
  Synchronized<T> buffer_;
};

}  // namespace ocs2