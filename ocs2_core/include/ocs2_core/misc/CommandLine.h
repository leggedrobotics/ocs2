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

#include <atomic>
#include <chrono>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace ocs2 {

/**
 * Gets a command line string.
 * @praram [in] shouldTerminate: A function signals termination of the line reading thread. For example, one
 * can use this to terminate if the rosmaster is stopped.
 */
inline std::string getCommandLineString(bool (*shouldTerminate)() = []() { return false; }) {
  // Set up a thread to read user inputs
  std::string line{""};
  std::atomic_bool lineRead(false);
  std::thread thr([&line, &lineRead]() {
    lineRead = false;
    getline(std::cin, line);
    lineRead = true;
  });

  // wait till line is read or terminate if ROS is gone.
  const std::chrono::duration<double, std::ratio<1, 30>> hz30(1);  // 30Hz clock using fractional ticks
  while (!lineRead) {
    if (shouldTerminate()) {
      std::terminate();  // need to terminate thread that is still waiting for input
    }
    std::this_thread::sleep_for(hz30);
  }
  if (thr.joinable()) {
    thr.join();
  }
  return line;
}

/**
 * Transforms a line of words to a vector of words.
 */
inline std::vector<std::string> stringToWords(const std::string& str) {
  std::istringstream iss(str);
  std::vector<std::string> words{std::istream_iterator<std::string>{iss}, std::istream_iterator<std::string>{}};
  return words;
}

}  // namespace ocs2
