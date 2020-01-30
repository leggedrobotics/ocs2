
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
  using state_vector_t = typename riccati_t::state_vector_t;
  using input_vector_t = typename riccati_t::input_vector_t;
  using dynamic_vector_t = typename riccati_t::dynamic_vector_t;
  using dynamic_matrix_t = typename riccati_t::dynamic_matrix_t;

  using scalar_t = typename riccati_t::scalar_t;
  using scalar_array_t = typename riccati_t::scalar_array_t;
  using state_matrix_array_t = typename riccati_t::state_matrix_array_t;
  using input_matrix_array_t = typename riccati_t::input_matrix_array_t;
  using state_input_matrix_array_t = typename riccati_t::state_input_matrix_array_t;
  using state_vector_array_t = typename riccati_t::state_vector_array_t;
  using input_vector_array_t = typename riccati_t::input_vector_array_t;
  using dynamic_matrix_array_t = typename riccati_t::dynamic_matrix_array_t;
  using input_state_matrix_array_t = typename riccati_t::input_state_matrix_array_t;
  using size_array_t = typename riccati_t::size_array_t;

  scalar_array_t timeStamp;
  dynamic_matrix_array_t RinvChol;
  ocs2::ModelDataBase::array_t modelDataTrajectory;

  size_array_t eventsPastTheEndIndeces;
  ocs2::ModelDataBase::array_t modelDataEventTimesArray;

  ocs2::RiccatiModificationBase::array_t riccatiModificationTrajectory;

  RiccatiInitializer(const int state_dim, const int input_dim) {
    timeStamp = scalar_array_t{0.0, 1.0};

    ocs2::ModelDataBase modelData;
    modelData.stateDim_ = state_dim;
    modelData.inputDim_ = input_dim;
    modelData.dynamicsBias_ = state_vector_t::Random(state_dim);
    modelData.dynamicsStateDerivative_ = state_matrix_t::Random(state_dim, state_dim);
    modelData.dynamicsInputDerivative_ = state_input_matrix_t::Random(state_dim, input_dim);
    modelData.cost_ = dynamic_vector_t::Random(1)(0);
    modelData.costStateDerivative_ = state_vector_t::Random(state_dim);
    modelData.costStateSecondDerivative_ = ocs2::LinearAlgebra::generateSPDmatrix<state_matrix_t>(state_dim);
    modelData.costInputDerivative_ = input_vector_t::Random(input_dim);
    modelData.costInputSecondDerivative_ = ocs2::LinearAlgebra::generateSPDmatrix<input_matrix_t>(input_dim);
    modelData.costInputStateDerivative_ = input_state_matrix_t::Random(input_dim, state_dim);
    modelData.numIneqConstr_ = 0;
    modelData.numStateEqConstr_ = 0;
    modelData.numStateInputEqConstr_ = 0;

    modelDataTrajectory = ocs2::ModelDataBase::array_t{modelData, modelData};

    ocs2::RiccatiModificationBase riccatiModification;
    riccatiModification.deltaQm_ = 0.1 * ocs2::LinearAlgebra::generateSPDmatrix<state_matrix_t>(state_dim);
    riccatiModification.deltaRm_ = input_matrix_t::Zero(input_dim, input_dim);
    riccatiModification.deltaPm_ = input_state_matrix_t::Zero(input_dim, state_dim);
    ocs2::LinearAlgebra::computeInverseMatrixUUT(modelData.costInputSecondDerivative_, riccatiModification.HmInverseConstrainedLowRank_);
    riccatiModification.HmInverseConstrained_ =
        riccatiModification.HmInverseConstrainedLowRank_ * riccatiModification.HmInverseConstrainedLowRank_.transpose();

    riccatiModificationTrajectory = ocs2::RiccatiModificationBase::array_t{riccatiModification, riccatiModification};
  }

  void initialize(riccati_t& riccati) {
    riccati.setData(&timeStamp, &modelDataTrajectory, &eventsPastTheEndIndeces, &modelDataEventTimesArray, &riccatiModificationTrajectory);
  }
};

TEST(riccati_ode_test, compareImplementations) {
  constexpr int STATE_DIM = 48;
  constexpr int INPUT_DIM = 10;

  using riccati_t = ocs2::SequentialRiccatiEquations<STATE_DIM, INPUT_DIM>;

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

  using riccati_static_t = ocs2::SequentialRiccatiEquations<state_dim, input_dim>;
  riccati_static_t riccati_static(precompute);
  srand(42);  // necessary to define a similar problem as dynamic case
  RiccatiInitializer<riccati_static_t> ris(state_dim, input_dim);
  ris.initialize(riccati_static);

  riccati_static_t::s_vector_t S_static = riccati_static_t::s_vector_t::Random();
  riccati_static_t::s_vector_t dSdz_static;
  riccati_static.computeFlowMap(0.6, S_static, dSdz_static);

  using riccati_dynamic_t = ocs2::SequentialRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;
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
  using riccati_t = ocs2::SequentialRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;

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
  using riccati_t = ocs2::SequentialRiccatiEquations<state_dim, input_dim>;

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
  using riccati_t = ocs2::SequentialRiccatiEquations<Eigen::Dynamic, Eigen::Dynamic>;

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
