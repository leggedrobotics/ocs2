
#include <gtest/gtest.h>
#include <ocs2_core/misc/LinearAlgebra.h>
#include <ocs2_core/misc/randomMatrices.h>
#include <ocs2_ddp/riccati_equations/SequentialRiccatiEquations.h>
#include <memory>

template <typename riccati_t>
class RiccatiInitializer {
 public:
  using state_matrix_t = typename riccati_t::state_matrix_t;
  using input_matrix_t = typename riccati_t::input_matrix_t;
  using state_input_matrix_t = typename riccati_t::state_input_matrix_t;
  using input_state_matrix_t = typename riccati_t::input_state_matrix_t;
  using eigen_scalar_t = typename riccati_t::eigen_scalar_t;
  using state_vector_t = typename riccati_t::state_vector_t;
  using input_vector_t = typename riccati_t::input_vector_t;

  using scalar_array_t = typename riccati_t::scalar_array_t;
  using state_matrix_array_t = typename riccati_t::state_matrix_array_t;
  using state_input_matrix_array_t = typename riccati_t::state_input_matrix_array_t;
  using eigen_scalar_array_t = typename riccati_t::eigen_scalar_array_t;
  using state_vector_array_t = typename riccati_t::state_vector_array_t;
  using input_vector_array_t = typename riccati_t::input_vector_array_t;
  using dynamic_matrix_array_t = typename riccati_t::dynamic_matrix_array_t;
  using input_state_matrix_array_t = typename riccati_t::input_state_matrix_array_t;
  using size_array_t = typename riccati_t::size_array_t;

  state_matrix_t A;
  state_input_matrix_t B;
  eigen_scalar_t q_;
  state_vector_t qv;
  state_matrix_t Q;
  input_vector_t rv;
  input_state_matrix_t P;
  input_matrix_t R;
  input_matrix_t RinvChol_;

  std::unique_ptr<scalar_array_t> timeStamp;
  std::unique_ptr<state_matrix_array_t> Am;
  std::unique_ptr<state_input_matrix_array_t> Bm;
  std::unique_ptr<eigen_scalar_array_t> q;
  std::unique_ptr<state_vector_array_t> Qv;
  std::unique_ptr<state_matrix_array_t> Qm;
  std::unique_ptr<input_vector_array_t> Rv;
  std::unique_ptr<dynamic_matrix_array_t> RinvChol;
  std::unique_ptr<input_state_matrix_array_t> Pm;

  size_array_t eventsPastTheEndIndeces;
  eigen_scalar_array_t qFinal;
  state_vector_array_t QvFinal;
  state_matrix_array_t QmFinal;

  RiccatiInitializer(const int state_dim, const int input_dim) {
    A = state_matrix_t::Random(state_dim, state_dim);
    B = state_input_matrix_t::Random(state_dim, input_dim);
    q_ = eigen_scalar_t::Random();
    qv = state_vector_t::Random(state_dim);
    Q = ocs2::LinearAlgebra::generateSPDmatrix<state_matrix_t>(state_dim);
    rv = input_vector_t::Random(input_dim);
    P = input_state_matrix_t::Random(input_dim, state_dim);
    R = ocs2::LinearAlgebra::generateSPDmatrix<input_matrix_t>(input_dim);
    RinvChol_.resize(input_dim, input_dim);
    ocs2::LinearAlgebra::computeLinvTLinv(R, RinvChol_);

    timeStamp = std::unique_ptr<scalar_array_t>(new scalar_array_t({0.0, 1.0}));
    Am = std::unique_ptr<state_matrix_array_t>(new state_matrix_array_t({A, A}));
    Bm = std::unique_ptr<state_input_matrix_array_t>(new state_input_matrix_array_t({B, B}));
    q = std::unique_ptr<eigen_scalar_array_t>(new eigen_scalar_array_t({q_, q_}));
    Qv = std::unique_ptr<state_vector_array_t>(new state_vector_array_t({qv, qv}));
    Qm = std::unique_ptr<state_matrix_array_t>(new state_matrix_array_t({Q, Q}));
    Rv = std::unique_ptr<input_vector_array_t>(new input_vector_array_t({rv, rv}));
    RinvChol = std::unique_ptr<dynamic_matrix_array_t>(new dynamic_matrix_array_t({RinvChol_, RinvChol_}));
    Pm = std::unique_ptr<input_state_matrix_array_t>(new input_state_matrix_array_t({P, P}));
  }

  void initialize(riccati_t& riccati) {
    riccati.setData(timeStamp.get(), Am.get(), Bm.get(), q.get(), Qv.get(), Qm.get(), Rv.get(), RinvChol.get(), Pm.get(),
                    &eventsPastTheEndIndeces, &qFinal, &QvFinal, &QmFinal);
  }
};

TEST(testRiccatiEquations, compareImplementations) {
  constexpr int STATE_DIM = 48;
  constexpr int INPUT_DIM = 10;
  constexpr bool makePSD = false;

  using riccati_t = ocs2::SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>;

  riccati_t riccatiEquationPrecompute(makePSD, true);
  riccati_t riccatiEquationNoPrecompute(makePSD, false);

  RiccatiInitializer<riccati_t> ri(STATE_DIM, INPUT_DIM);
  ri.initialize(riccatiEquationPrecompute);
  ri.initialize(riccatiEquationNoPrecompute);

  riccati_t::s_vector_t S;
  riccati_t::s_vector_t dSdz_precompute, dSdz_noPrecompute;

  S.setRandom();
  riccatiEquationPrecompute.computeFlowMap(0.6, S, dSdz_precompute);
  riccatiEquationNoPrecompute.computeFlowMap(0.6, S, dSdz_noPrecompute);

  EXPECT_LE((dSdz_precompute - dSdz_noPrecompute).array().abs().maxCoeff(), 1e-9);
}

TEST(testRiccatiEquations, compareFixedAndDynamicSizedImplementation) {
  constexpr bool makePSD = false;
  constexpr bool precompute = false;

  const int input_dim = 10;
  const int state_dim = 48;

  using riccati_static_t = ocs2::SequentialRiccatiEquations<state_dim, input_dim>;
  riccati_static_t riccati_static(makePSD, precompute);
  srand(42);
  RiccatiInitializer<riccati_static_t> ris(state_dim, input_dim);
  ris.initialize(riccati_static);

  riccati_static_t::s_vector_t S_static;
  riccati_static_t::s_vector_t dSdz_static;
  S_static.setRandom();
  riccati_static.computeFlowMap(0.6, S_static, dSdz_static);

  using riccati_dynamic_t = ocs2::SequentialRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;
  riccati_dynamic_t riccati_dynamic(makePSD, precompute);
  srand(42);
  RiccatiInitializer<riccati_dynamic_t> rid(state_dim, input_dim);
  rid.initialize(riccati_dynamic);

  riccati_dynamic_t::s_vector_t S;
  riccati_dynamic_t::s_vector_t dSdz;
  S = S_static;
  riccati_dynamic.computeFlowMap(0.6, S, dSdz);

  EXPECT_LE((dSdz - dSdz_static).array().abs().maxCoeff(), 1e-9);
}

TEST(testRiccatiEquations, testFlattenSMatrix) {
  const int state_dim = 4;
  using riccati_t = ocs2::SequentialRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;

  riccati_t::s_vector_t allSs, allSs_expect;
  riccati_t::state_matrix_t Sm;
  riccati_t::state_vector_t Sv;
  riccati_t::eigen_scalar_t s;

  Sm.resize(state_dim, state_dim);
  Sm << 1, 2, 4, 7,  // clang-format off
        2, 3, 5, 8,
        4, 5, 6, 9,
        7, 8, 9, 10;  // clang-format on
  Sv.resize(state_dim);
  Sv << 11, 12, 13, 14;
  s << 15;

  allSs.resize(15);
  allSs_expect.resize(15);
  allSs_expect << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15;

  riccati_t::convert2Vector(Sm, Sv, s, allSs);

  EXPECT_EQ(allSs, allSs_expect);
}

TEST(testRiccatiEquations, testFlattenAndUnflattenSMatrix) {
  const int input_dim = 10;
  const int state_dim = 48;
  using riccati_t = ocs2::SequentialRiccatiEquations<state_dim, input_dim>;

  riccati_t::s_vector_t allSs;
  riccati_t::state_matrix_t Sm, Sm_out;
  riccati_t::state_vector_t Sv, Sv_out;
  riccati_t::eigen_scalar_t s, s_out;

  Sm.setRandom();
  Sm = (Sm + Sm.transpose()).eval();
  Sv.setRandom();
  s.setRandom();

  riccati_t::convert2Vector(Sm, Sv, s, allSs);
  riccati_t::convert2Matrix(allSs, Sm_out, Sv_out, s_out);

  EXPECT_EQ(Sm, Sm_out);
  EXPECT_EQ(Sv, Sv_out);
  EXPECT_EQ(s, s_out);
}

TEST(testRiccatiEquations, testFlattenAndUnflattenSMatrixDynamic) {
  const int state_dim = 42;
  using riccati_t = ocs2::SequentialRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;

  riccati_t::s_vector_t allSs;
  riccati_t::state_matrix_t Sm, Sm_out;
  riccati_t::state_vector_t Sv, Sv_out;
  riccati_t::eigen_scalar_t s, s_out;

  Sm.setRandom(state_dim, state_dim);
  Sm = (Sm + Sm.transpose()).eval();
  Sv.setRandom(state_dim);
  s.setRandom();

  Sm_out.setZero(state_dim, state_dim);
  Sv_out.setZero(state_dim);
  s_out.setZero();

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
