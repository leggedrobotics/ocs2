//
// Created by rgrandia on 29.07.19.
//


#include <gtest/gtest.h>
#include <ocs2_slq/riccati_equations/SequentialRiccatiEquationsNormalized.h>
#include <ocs2_core/misc/randomMatrices.h>
#include <ocs2_core/misc/LinearAlgebra.h>

using namespace ocs2;

TEST(testRiccatiEquations, compareImplementations) {
  constexpr size_t STATE_DIM = 4;
  constexpr size_t INPUT_DIM = 2;

  bool makePSD = false;
  double addedRiccatiDiagonal = 1e-5;

  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>
      riccatiEquationPrecompute(makePSD, addedRiccatiDiagonal, true);
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>
      riccatiEquationNoPrecompute(makePSD, addedRiccatiDiagonal, false);

  using state_matrix_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_matrix_t;
  using input_matrix_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::input_matrix_t;
  using state_input_matrix_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_input_matrix_t;
  using input_state_matrix_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::input_state_matrix_t;
  using dynamic_matrix_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::dynamic_matrix_t;
  using eigen_scalar_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::eigen_scalar_t;
  using state_vector_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_vector_t;
  using input_vector_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::input_vector_t;
  using s_vector_t = SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::s_vector_t;

  state_matrix_t A = state_matrix_t::Random();;
  state_input_matrix_t B = state_input_matrix_t::Random();
  eigen_scalar_t q_ = eigen_scalar_t::Random();
  state_vector_t qv = state_vector_t::Random();
  state_matrix_t Q = LinearAlgebra::generateSPDmatrix<state_matrix_t>();
  input_vector_t rv = input_vector_t::Random();
  input_state_matrix_t P = input_state_matrix_t::Random();
  input_matrix_t R = LinearAlgebra::generateSPDmatrix<input_matrix_t>();
  input_matrix_t RinvChol_;
  LinearAlgebra::computeLinvTLinv(R, RinvChol_);

  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::scalar_array_t timeStamp{0.0, 1.0};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_matrix_array_t Am{A, A};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_input_matrix_array_t Bm{B, B};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::eigen_scalar_array_t q{q_, q_};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_vector_array_t Qv{qv, qv};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_matrix_array_t Qm{Q, Q};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::input_vector_array_t Rv{rv, rv};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::dynamic_matrix_array_t RinvChol{RinvChol_, RinvChol_};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::input_state_matrix_array_t Pm{P, P};
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::size_array_t eventsPastTheEndIndeces;
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::eigen_scalar_array_t qFinal;
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_vector_array_t QvFinal;
  SequentialRiccatiEquationsNormalized<STATE_DIM, INPUT_DIM>::state_matrix_array_t QmFinal;

  riccatiEquationPrecompute.setData(&timeStamp,
                                    &Am, &Bm, &q, &Qv, &Qm, &Rv, &RinvChol, &Pm,
                                    &eventsPastTheEndIndeces, &qFinal, &QvFinal, &QmFinal);
  riccatiEquationNoPrecompute.setData(&timeStamp,
                                      &Am, &Bm, &q, &Qv, &Qm, &Rv, &RinvChol, &Pm,
                                      &eventsPastTheEndIndeces, &qFinal, &QvFinal, &QmFinal);

  s_vector_t S = s_vector_t::Random();
  s_vector_t dSdz_precompute, dSdz_noPrecompute;
  riccatiEquationPrecompute.computeFlowMap(0.6, S, dSdz_precompute);
  riccatiEquationNoPrecompute.computeFlowMap(0.6, S, dSdz_noPrecompute);

  ASSERT_LE((dSdz_precompute - dSdz_noPrecompute).array().abs().maxCoeff(), 1e-9);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
