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

#pragma once

#include <iostream>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2 {
namespace quadrotor {

template <typename SCALAR_T>
class QuadrotorParameters {
 public:
  QuadrotorParameters(SCALAR_T quadrotorMass = 1.0, SCALAR_T Thzz = 1.0, SCALAR_T Thxxyy = 1.0, SCALAR_T gravity = 9.8)
      : quadrotorMass_(quadrotorMass), Thzz_(Thzz), Thxxyy_(Thxxyy), gravity_(gravity) {}

  ~QuadrotorParameters() = default;

  inline void loadSettings(const std::string& filename, bool verbose = true) {
    boost::property_tree::ptree pt;
    boost::property_tree::read_info(filename, pt);

    if (verbose) std::cerr << "\n #### Quadrotor Parameters:" << std::endl;
    if (verbose) std::cerr << " #### =========================================" << std::endl;

    try {
      quadrotorMass_ = pt.get<SCALAR_T>("QuadrotorParameters.quadrotorMass");
      if (verbose) std::cerr << " #### quadrotorMass ......... " << quadrotorMass_ << std::endl;
    } catch (const std::exception& e) {
      if (verbose) std::cerr << " #### quadrotorMass ......... " << quadrotorMass_ << "\t(default)" << std::endl;
    }

    try {
      Thzz_ = pt.get<SCALAR_T>("QuadrotorParameters.Thzz");
      if (verbose) std::cerr << " #### Thzz .................. " << Thzz_ << std::endl;
    } catch (const std::exception& e) {
      if (verbose) std::cerr << " #### Thzz .................. " << Thzz_ << "\t(default)" << std::endl;
    }

    try {
      Thxxyy_ = pt.get<SCALAR_T>("QuadrotorParameters.Thxxyy");
      if (verbose) std::cerr << " #### Thxx/yy ............... " << Thxxyy_ << std::endl;
    } catch (const std::exception& e) {
      if (verbose) std::cerr << " #### Thxx/yy ............... " << Thxxyy_ << "\t(default)" << std::endl;
    }

    try {
      gravity_ = pt.get<SCALAR_T>("QuadrotorParameters.gravity");
      if (verbose) std::cerr << " #### gravity ............... " << gravity_ << std::endl;
    } catch (const std::exception& e) {
      if (verbose) std::cerr << " #### gravity ............... " << gravity_ << "\t(default)" << std::endl;
    }
  }

 public:
  // For safety, these parameters cannot be modified
  SCALAR_T quadrotorMass_;  // [kg]
  SCALAR_T Thzz_;
  SCALAR_T Thxxyy_;
  SCALAR_T gravity_;
};

}  // namespace quadrotor
}  // namespace ocs2
