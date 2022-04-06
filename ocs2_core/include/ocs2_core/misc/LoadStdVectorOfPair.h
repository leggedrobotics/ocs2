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

#include <string>
#include <utility>
#include <vector>

namespace ocs2 {
namespace loadData {

/**
 * An auxiliary function which loads a vector of string pairs from a file.
 *
 * It has the following format:	<br>
 * topicName	<br>
 * {	<br>
 *   [0] "value1, value2"	<br>
 *   [2] "value1, value2"	<br>
 *   [1] "value1, value2"	<br>
 * } 	<br>
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] topicName: The key name assigned in the config file.
 * @param [out] loadVector: The loaded vector of pairs of strings.
 */
extern void loadStdVectorOfPair(const std::string& filename, const std::string& topicName,
                                std::vector<std::pair<std::string, std::string>>& loadVector, bool verbose = true);

/**
 * An auxiliary function which loads a vector of integer pairs from a file.
 *
 * It has the following format:	<br>
 * topicName	<br>
 * {	<br>
 *   [0] "value1, value2"	<br>
 *   [2] "value1, value2"	<br>
 *   [1] "value1, value2"	<br>
 * } 	<br>
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] topicName: The key name assigned in the config file.
 * @param [out] loadVector: The loaded vector of pairs of integer.
 */
extern void loadStdVectorOfPair(const std::string& filename, const std::string& topicName,
                                std::vector<std::pair<size_t, size_t>>& loadVector, bool verbose = true);

}  // namespace loadData
}  // namespace ocs2
