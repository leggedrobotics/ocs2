/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

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

#include "testLoopshapingConfigurations.h"

namespace ocs2 {

const std::string TestConfiguration_r_filter::fileName = "loopshaping_r.conf";
constexpr size_t TestConfiguration_r_filter::FULL_STATE_DIM;
constexpr size_t TestConfiguration_r_filter::FULL_INPUT_DIM;
constexpr size_t TestConfiguration_r_filter::SYSTEM_STATE_DIM;
constexpr size_t TestConfiguration_r_filter::SYSTEM_INPUT_DIM;
constexpr size_t TestConfiguration_r_filter::FILTER_STATE_DIM;
constexpr size_t TestConfiguration_r_filter::FILTER_INPUT_DIM;

const std::string TestConfiguration_r_simple_filter::fileName = "loopshaping_r_simple.conf";
constexpr size_t TestConfiguration_r_simple_filter::FULL_STATE_DIM;
constexpr size_t TestConfiguration_r_simple_filter::FULL_INPUT_DIM;
constexpr size_t TestConfiguration_r_simple_filter::SYSTEM_STATE_DIM;
constexpr size_t TestConfiguration_r_simple_filter::SYSTEM_INPUT_DIM;
constexpr size_t TestConfiguration_r_simple_filter::FILTER_STATE_DIM;
constexpr size_t TestConfiguration_r_simple_filter::FILTER_INPUT_DIM;

const std::string TestConfiguration_r_ballbot_filter::fileName = "loopshaping_r_ballbot.conf";
constexpr size_t TestConfiguration_r_ballbot_filter::FULL_STATE_DIM;
constexpr size_t TestConfiguration_r_ballbot_filter::FULL_INPUT_DIM;
constexpr size_t TestConfiguration_r_ballbot_filter::SYSTEM_STATE_DIM;
constexpr size_t TestConfiguration_r_ballbot_filter::SYSTEM_INPUT_DIM;
constexpr size_t TestConfiguration_r_ballbot_filter::FILTER_STATE_DIM;
constexpr size_t TestConfiguration_r_ballbot_filter::FILTER_INPUT_DIM;

const std::string TestConfiguration_s_filter::fileName = "loopshaping_s.conf";
constexpr size_t TestConfiguration_s_filter::FULL_STATE_DIM;
constexpr size_t TestConfiguration_s_filter::FULL_INPUT_DIM;
constexpr size_t TestConfiguration_s_filter::SYSTEM_STATE_DIM;
constexpr size_t TestConfiguration_s_filter::SYSTEM_INPUT_DIM;
constexpr size_t TestConfiguration_s_filter::FILTER_STATE_DIM;
constexpr size_t TestConfiguration_s_filter::FILTER_INPUT_DIM;

const std::string TestConfiguration_s_simple_filter::fileName = "loopshaping_s_simple.conf";
constexpr size_t TestConfiguration_s_simple_filter::FULL_STATE_DIM;
constexpr size_t TestConfiguration_s_simple_filter::FULL_INPUT_DIM;
constexpr size_t TestConfiguration_s_simple_filter::SYSTEM_STATE_DIM;
constexpr size_t TestConfiguration_s_simple_filter::SYSTEM_INPUT_DIM;
constexpr size_t TestConfiguration_s_simple_filter::FILTER_STATE_DIM;
constexpr size_t TestConfiguration_s_simple_filter::FILTER_INPUT_DIM;

const std::string TestConfiguration_s_eliminate_filter::fileName = "loopshaping_s_eliminate.conf";
constexpr size_t TestConfiguration_s_eliminate_filter::FULL_STATE_DIM;
constexpr size_t TestConfiguration_s_eliminate_filter::FULL_INPUT_DIM;
constexpr size_t TestConfiguration_s_eliminate_filter::SYSTEM_STATE_DIM;
constexpr size_t TestConfiguration_s_eliminate_filter::SYSTEM_INPUT_DIM;
constexpr size_t TestConfiguration_s_eliminate_filter::FILTER_STATE_DIM;
constexpr size_t TestConfiguration_s_eliminate_filter::FILTER_INPUT_DIM;

const std::string TestConfiguration_s_simple_eliminate_filter::fileName = "loopshaping_s_simple_eliminate.conf";
constexpr size_t TestConfiguration_s_simple_eliminate_filter::FULL_STATE_DIM;
constexpr size_t TestConfiguration_s_simple_eliminate_filter::FULL_INPUT_DIM;
constexpr size_t TestConfiguration_s_simple_eliminate_filter::SYSTEM_STATE_DIM;
constexpr size_t TestConfiguration_s_simple_eliminate_filter::SYSTEM_INPUT_DIM;
constexpr size_t TestConfiguration_s_simple_eliminate_filter::FILTER_STATE_DIM;
constexpr size_t TestConfiguration_s_simple_eliminate_filter::FILTER_INPUT_DIM;

}  // namespace ocs2
