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

#include <gtest/gtest.h>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include "ocs2_centroidal_model/FactoryFunctions.h"

#include "ocs2_centroidal_model/test/definitions.h"

using namespace ocs2;
using namespace centroidal_model;

TEST(SRBD_CentroidalModelInfo, InitModelFromUrdf) {
  PinocchioInterface pinocchioInterface = createPinocchioInterface(anymalUrdfFile);
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t numJoints = model.nq - 6;
  CentroidalModelInfo info = createCentroidalModelInfo(pinocchioInterface, CentroidalModelType::SingleRigidBodyDynamics,
                                                       getInitialState().tail(numJoints), anymal3DofContactNames, anymal6DofContactNames);

  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}

TEST(SRBD_CentroidalModelInfo, InitModelFromUrdfAD) {
  PinocchioInterface pinocchioInterface = createPinocchioInterface(anymalUrdfFile);
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t nq = model.nq;
  const size_t numJoints = nq - 6;
  CentroidalModelInfo info = createCentroidalModelInfo(pinocchioInterface, CentroidalModelType::SingleRigidBodyDynamics,
                                                       getInitialState().tail(numJoints), anymal3DofContactNames, anymal6DofContactNames);

  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}

TEST(FCD_CentroidalModelInfo, InitModelFromUrdf) {
  PinocchioInterface pinocchioInterface = createPinocchioInterface(anymalUrdfFile);
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t numJoints = model.nq - 6;
  CentroidalModelInfo info = createCentroidalModelInfo(pinocchioInterface, CentroidalModelType::FullCentroidalDynamics,
                                                       vector_t::Zero(numJoints), anymal3DofContactNames, anymal6DofContactNames);
  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}

TEST(FCD_CentroidalModelInfo, InitModelFromUrdfAD) {
  PinocchioInterface pinocchioInterface = createPinocchioInterface(anymalUrdfFile);
  const auto& model = pinocchioInterface.getModel();
  auto& data = pinocchioInterface.getData();
  const size_t nq = model.nq;
  const size_t numJoints = nq - 6;
  const CentroidalModelInfo info = createCentroidalModelInfo(pinocchioInterface, CentroidalModelType::FullCentroidalDynamics,
                                                             vector_t::Zero(numJoints), anymal3DofContactNames, anymal6DofContactNames);

  std::cerr << "nq " << model.nq << '\n';
  std::cerr << "nv " << model.nv << '\n';
  std::cerr << "STATE_DIM " << anymal::STATE_DIM << '\n';
  std::cerr << "INPUT_DIM " << anymal::INPUT_DIM << '\n';

  EXPECT_EQ(info.stateDim, anymal::STATE_DIM);
  EXPECT_EQ(info.inputDim, anymal::INPUT_DIM);
}
