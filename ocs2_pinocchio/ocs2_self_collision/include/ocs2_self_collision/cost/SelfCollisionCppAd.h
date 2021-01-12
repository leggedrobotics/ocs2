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

/*
 * SelfCollisionCostCppAd.h
 *
 *  Created on: 8 Sep 2020
 *      Author: perry
 */

#pragma once

#include <ocs2_pinocchio_interface/PinocchioInterface.h>
#include <ocs2_self_collision/PinocchioGeometryInterface.h>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>
#include <ocs2_core/cost/QuadraticGaussNewtonCostBaseAD.h>

namespace ocs2 {

class SelfCollisionCppAd {
 public:
  using base_t = Eigen::Matrix<scalar_t, 6, 1>;

  SelfCollisionCppAd(PinocchioInterface pinocchioInterface, PinocchioGeometryInterface geometryInterfaceSelfCollision,
                         scalar_t minimumDistance);
  ~SelfCollisionCppAd() = default;
  SelfCollisionCppAd(const SelfCollisionCppAd& rhs);

  SelfCollisionCppAd* clone() const { return new SelfCollisionCppAd(*this); }

  void initialize(const std::string& modelName, const std::string& modelFolder = "/tmp/ocs2", bool recompileLibraries = true,
                  bool verbose = true);

  /** Evaluate the cost */
  vector_t getValue(const base_t& base, const vector_t& joints);

  /** Evaluate the cost quadratic approximation */
  std::pair<vector_t, matrix_t> getLinearApproximation(const base_t& base, const vector_t& joints);

  ad_vector_t getCollisionObjectJointPoses();

 private:
  void setADInterfaces(const std::string& modelName, const std::string& modelFolder);
  void createModels(bool verbose);
  void loadModelsIfAvailable(bool verbose);

  // Number of params per result = 3 + 3 + 1 (nearest point 1, nearest point 2, sign indicator)
  const size_t numberOfParamsPerResult_ = 7;

  // From the current state of the robot, and the closest points in world frame, compute the positions of the points in link frame
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |3*2*number of collision pairs + 1 (for sign indicator)|
  ad_vector_t computeLinkPointsAd(ad_vector_t state, ad_vector_t points);
  // From the current state of the robot, and the closest points in link frames, calculate the distances wrt state
  // In this case : size of state = stateDim, size of points = 3*2*number of collision pairs + 1 (for sign indicator)
  // Returns a vector that is of length |collisionPairs|
  ad_vector_t distanceCalculationAd(ad_vector_t state, ad_vector_t points);

  std::unique_ptr<CppAdInterface> cppAdInterfaceDistanceCalculation_;
  std::unique_ptr<CppAdInterface> cppAdInterfaceLinkPoints_;

  PinocchioInterface pinocchioInterface_;
  PinocchioInterfaceCppAd pinocchioInterfaceAd_;
  PinocchioGeometryInterface pinocchioGeometrySelfCollisions_;

  scalar_t minimumDistance_;
};

} /* namespace ocs2 */
