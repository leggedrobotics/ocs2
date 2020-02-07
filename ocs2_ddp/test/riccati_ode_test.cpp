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

#include <memory>

#include <gtest/gtest.h>

#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/randomMatrices.h>
#include <ocs2_ddp/riccati_equations/ContinuousTimeRiccatiEquations.h>

template <typename riccati_t>
class RiccatiInitializer {
 public:
  using scalar_t = typename riccati_t::scalar_t;
  using size_array_t = typename riccati_t::size_array_t;
  using scalar_array_t = typename riccati_t::scalar_array_t;
  using state_matrix_t = typename riccati_t::state_matrix_t;
  using state_vector_t = typename riccati_t::state_vector_t;
  using dynamic_vector_t = typename riccati_t::dynamic_vector_t;
  using dynamic_matrix_t = typename riccati_t::dynamic_matrix_t;

  scalar_array_t timeStamp;
  ocs2::ModelDataBase::array_t projectedModelDataTrajectory;

  size_array_t eventsPastTheEndIndeces;
  ocs2::ModelDataBase::array_t modelDataEventTimesArray;

  ocs2::RiccatiModificationBase::array_t riccatiModificationTrajectory;

  RiccatiInitializer(const int state_dim, const int input_dim) {
    timeStamp = scalar_array_t{0.0, 1.0};

    ocs2::ModelDataBase projectedModelData;
    projectedModelData.stateDim_ = state_dim;
    projectedModelData.inputDim_ = input_dim;
    projectedModelData.dynamicsBias_ = dynamic_vector_t::Random(state_dim);
    projectedModelData.dynamicsStateDerivative_ = dynamic_matrix_t::Random(state_dim, state_dim);
    projectedModelData.dynamicsInputDerivative_ = dynamic_matrix_t::Random(state_dim, input_dim);
    projectedModelData.cost_ = dynamic_vector_t::Random(1)(0);
    projectedModelData.costStateDerivative_ = dynamic_vector_t::Random(state_dim);
    projectedModelData.costStateSecondDerivative_ = ocs2::LinearAlgebra::generateSPDmatrix<dynamic_matrix_t>(state_dim);
    projectedModelData.costInputDerivative_ = dynamic_vector_t::Random(input_dim);
    projectedModelData.costInputSecondDerivative_.setIdentity(
        input_dim, input_dim);  // Important: It is identity since it is a projected projectedModelData!
    projectedModelData.costInputStateDerivative_ = dynamic_matrix_t::Random(input_dim, state_dim);
    projectedModelData.numIneqConstr_ = 0;
    projectedModelData.numStateEqConstr_ = 0;
    projectedModelData.numStateInputEqConstr_ = input_dim;
    projectedModelData.stateInputEqConstr_.setZero(input_dim);
    projectedModelData.stateInputEqConstrStateDerivative_.setZero(input_dim, state_dim);
    projectedModelData.stateInputEqConstrInputDerivative_.setZero(input_dim, input_dim);

    projectedModelDataTrajectory = ocs2::ModelDataBase::array_t{projectedModelData, projectedModelData};

    ocs2::RiccatiModificationBase riccatiModification;
    riccatiModification.deltaQm_ = 0.1 * ocs2::LinearAlgebra::generateSPDmatrix<dynamic_matrix_t>(state_dim);
    riccatiModification.deltaGv_ = dynamic_vector_t::Zero(input_dim);
    riccatiModification.deltaGm_ = dynamic_matrix_t::Zero(input_dim, state_dim);
    riccatiModification.constraintRangeProjector_.setZero(input_dim, 0);
    ocs2::LinearAlgebra::computeInverseMatrixUUT(projectedModelData.costInputSecondDerivative_,
                                                 riccatiModification.constraintNullProjector_);

    riccatiModificationTrajectory = ocs2::RiccatiModificationBase::array_t{riccatiModification, riccatiModification};
  }

  void initialize(riccati_t& riccati) {
    riccati.setData(&timeStamp, &projectedModelDataTrajectory, &eventsPastTheEndIndeces, &modelDataEventTimesArray,
                    &riccatiModificationTrajectory);
  }
};

TEST(riccati_ode_test, compareImplementations) {
  constexpr int STATE_DIM = 48;
  constexpr int INPUT_DIM = 10;

  using riccati_t = ocs2::ContinuousTimeRiccatiEquations<STATE_DIM, INPUT_DIM>;

  riccati_t riccatiEquationPrecompute(true);
  riccati_t riccatiEquationNoPrecompute(false);

  RiccatiInitializer<riccati_t> ri(STATE_DIM, INPUT_DIM);
  ri.initialize(riccatiEquationPrecompute);
  ri.initialize(riccatiEquationNoPrecompute);

  riccati_t::s_vector_t S = riccati_t::s_vector_t::Random();
  riccati_t::s_vector_t dSdz_precompute, dSdz_noPrecompute;
  riccatiEquationPrecompute.computeFlowMap(0.6, S, dSdz_precompute);
  riccatiEquationNoPrecompute.computeFlowMap(0.6, S, dSdz_noPrecompute);

  EXPECT_LE((dSdz_precompute - dSdz_noPrecompute).array().abs().maxCoeff(), 1e-9);
}

TEST(riccati_ode_test, compareFixedAndDynamicSizedImplementation) {
  constexpr bool precompute = false;

  const int input_dim = 10;
  const int state_dim = 48;

  using riccati_static_t = ocs2::ContinuousTimeRiccatiEquations<state_dim, input_dim>;
  riccati_static_t riccati_static(precompute);
  srand(42);  // necessary to define a similar problem as dynamic case
  RiccatiInitializer<riccati_static_t> ris(state_dim, input_dim);
  ris.initialize(riccati_static);

  riccati_static_t::s_vector_t S_static = riccati_static_t::s_vector_t::Random();
  riccati_static_t::s_vector_t dSdz_static;
  riccati_static.computeFlowMap(0.6, S_static, dSdz_static);

  using riccati_dynamic_t = ocs2::ContinuousTimeRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;
  riccati_dynamic_t riccati_dynamic(precompute);
  srand(42);  // necessary to define a similar problem as static case
  RiccatiInitializer<riccati_dynamic_t> rid(state_dim, input_dim);
  rid.initialize(riccati_dynamic);

  riccati_dynamic_t::s_vector_t S = S_static;
  riccati_dynamic_t::s_vector_t dSdz;
  riccati_dynamic.computeFlowMap(0.6, S, dSdz);

  EXPECT_LE((dSdz - dSdz_static).array().abs().maxCoeff(), 1e-9);
}

TEST(riccati_ode_test, testFlattenSMatrix) {
  const int state_dim = 4;
  using riccati_t = ocs2::ContinuousTimeRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;

  riccati_t::s_vector_t allSs, allSs_expect;
  riccati_t::state_matrix_t Sm;
  riccati_t::state_vector_t Sv;
  riccati_t::scalar_t s;

  Sm.resize(state_dim, state_dim);
  Sm << 1, 2, 4, 7,  // clang-format off
        2, 3, 5, 8,
        4, 5, 6, 9,
        7, 8, 9, 10;  // clang-format on
  Sv.resize(state_dim);
  Sv << 11, 12, 13, 14;
  s = 15;

  allSs.resize(15);
  allSs_expect.resize(15);
  allSs_expect << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;

  riccati_t::convert2Vector(Sm, Sv, s, allSs);

  EXPECT_EQ(allSs, allSs_expect);
}

TEST(riccati_ode_test, testFlattenAndUnflattenSMatrix) {
  const int input_dim = 10;
  const int state_dim = 48;
  using riccati_t = ocs2::ContinuousTimeRiccatiEquations<state_dim, input_dim>;

  riccati_t::s_vector_t allSs;
  riccati_t::state_matrix_t Sm, Sm_out;
  riccati_t::state_vector_t Sv, Sv_out;
  riccati_t::scalar_t s, s_out;

  Sm.setRandom();
  Sm = (Sm + Sm.transpose()).eval();
  Sv.setRandom();
  s = riccati_t::dynamic_vector_t::Random(1)(0);

  riccati_t::convert2Vector(Sm, Sv, s, allSs);
  riccati_t::convert2Matrix(allSs, Sm_out, Sv_out, s_out);

  EXPECT_EQ(Sm, Sm_out);
  EXPECT_EQ(Sv, Sv_out);
  EXPECT_EQ(s, s_out);
}

TEST(riccati_ode_test, testFlattenAndUnflattenSMatrixDynamic) {
  const int state_dim = 42;
  using riccati_t = ocs2::ContinuousTimeRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;

  riccati_t::s_vector_t allSs;
  riccati_t::state_matrix_t Sm, Sm_out;
  riccati_t::state_vector_t Sv, Sv_out;
  riccati_t::scalar_t s, s_out;

  Sm.setRandom(state_dim, state_dim);
  Sm = (Sm + Sm.transpose()).eval();
  Sv.setRandom(state_dim);
  s = riccati_t::dynamic_vector_t::Random(1)(0);

  Sm_out.setZero(state_dim, state_dim);
  Sv_out.setZero(state_dim);
  s_out = 0.0;

  riccati_t::convert2Vector(Sm, Sv, s, allSs);
  riccati_t::convert2Matrix(allSs, Sm_out, Sv_out, s_out);

  EXPECT_EQ(Sm, Sm_out);
  EXPECT_EQ(Sv, Sv_out);
  EXPECT_EQ(s, s_out);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
