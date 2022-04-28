#include "ocs2_oc/pre_condition/Scaling.h"

#include <atomic>
#include <numeric>

namespace ocs2 {
// Internal helper functions
namespace {

vector_t matrixInfNormRows(const Eigen::SparseMatrix<scalar_t>& mat) {
  vector_t infNorm;
  infNorm.setZero(mat.rows());

  for (int j = 0; j < mat.outerSize(); ++j) {
    for (Eigen::SparseMatrix<scalar_t>::InnerIterator it(mat, j); it; ++it) {
      int i = it.row();
      infNorm(i) = std::max(infNorm(i), std::abs(it.value()));
    }
  }
  return infNorm;
}

template <typename T>
vector_t matrixInfNormRows(const Eigen::MatrixBase<T>& mat) {
  if (mat.rows() == 0 || mat.cols() == 0) {
    return vector_t(0);
  } else {
    return mat.array().abs().matrix().rowwise().maxCoeff();
  }
}

template <typename T, typename... Rest>
vector_t matrixInfNormRows(const Eigen::MatrixBase<T>& mat, const Eigen::MatrixBase<Rest>&... rest) {
  vector_t temp = matrixInfNormRows(rest...);
  if (mat.rows() == 0 || mat.cols() == 0) {
    return temp;
  } else if (temp.rows() != 0) {
    return mat.array().abs().matrix().rowwise().maxCoeff().cwiseMax(temp);
  } else {
    return mat.array().abs().matrix().rowwise().maxCoeff();
  }
}

vector_t matrixInfNormCols(const Eigen::SparseMatrix<scalar_t>& mat) {
  vector_t infNorm;
  infNorm.setZero(mat.cols());

  for (int j = 0; j < mat.outerSize(); ++j) {
    for (Eigen::SparseMatrix<scalar_t>::InnerIterator it(mat, j); it; ++it) {
      infNorm(j) = std::max(infNorm(j), std::abs(it.value()));
    }
  }
  return infNorm;
}

template <typename T>
vector_t matrixInfNormCols(const Eigen::MatrixBase<T>& mat) {
  if (mat.rows() == 0 || mat.cols() == 0) {
    return vector_t(0);
  } else {
    return mat.array().abs().matrix().colwise().maxCoeff().transpose();
  }
}

template <typename T, typename... Rest>
vector_t matrixInfNormCols(const Eigen::MatrixBase<T>& mat, const Eigen::MatrixBase<Rest>&... rest) {
  vector_t temp = matrixInfNormCols(rest...);
  if (mat.rows() == 0 || mat.cols() == 0) {
    return temp;
  } else if (temp.rows() != 0) {
    return mat.array().abs().matrix().colwise().maxCoeff().transpose().cwiseMax(temp);
  } else {
    return mat.array().abs().matrix().colwise().maxCoeff().transpose();
  }
}

void scaleMatrixInPlace(const vector_t& rowScale, const vector_t& colScale, Eigen::SparseMatrix<scalar_t>& mat) {
  for (int j = 0; j < mat.outerSize(); ++j) {
    for (Eigen::SparseMatrix<scalar_t>::InnerIterator it(mat, j); it; ++it) {
      if (it.row() > rowScale.size() - 1 || it.col() > colScale.size() - 1) {
        throw std::runtime_error("[scaleMatrixInPlace] it.row() > rowScale.size() - 1 || it.col() > colScale.size() - 1");
      }
      it.valueRef() *= rowScale(it.row()) * colScale(j);
    }
  }
}

template <typename T>
void scaleMatrixInPlace(const vector_t* rowScale, const vector_t* colScale, Eigen::MatrixBase<T>& mat) {
  if (rowScale != nullptr) mat.array().colwise() *= rowScale->array();
  if (colScale != nullptr) mat *= colScale->asDiagonal();
}

void invSqrtInfNormInParallel(const std::vector<VectorFunctionLinearApproximation>& dynamics,
                              const std::vector<ScalarFunctionQuadraticApproximation>& cost, const vector_array_t& scalingVectors,
                              const int N, const std::function<void(vector_t&)>& limitScaling, vector_array_t& D, vector_array_t& E,
                              ThreadPool& threadPool) {
  // Helper function
  auto procedure = [&limitScaling](vector_t& dst, const vector_t& norm) {
    dst = norm;
    limitScaling(dst);
    dst.array() = dst.array().sqrt().inverse();
  };

  procedure(D[0], matrixInfNormCols(cost[0].dfduu, dynamics[0].dfdu));
  procedure(E[0], matrixInfNormRows(dynamics[0].dfdu, scalingVectors[0].asDiagonal().toDenseMatrix()));

  std::atomic_int timeStamp{1};
  auto task = [&](int workerId) {
    int k;
    while ((k = timeStamp++) < N) {
      procedure(D[2 * k - 1],
                matrixInfNormCols(cost[k].dfdxx, cost[k].dfdux, scalingVectors[k - 1].asDiagonal().toDenseMatrix(), dynamics[k].dfdx));
      procedure(D[2 * k], matrixInfNormCols(cost[k].dfdux.transpose().eval(), cost[k].dfduu, dynamics[k].dfdu));
      procedure(E[k], matrixInfNormRows(dynamics[k].dfdx, dynamics[k].dfdu, scalingVectors[k].asDiagonal().toDenseMatrix()));
    }
  };

  threadPool.runParallel(std::move(task), threadPool.numThreads() + 1U);

  procedure(D[2 * N - 1], matrixInfNormCols(cost[N].dfdxx, scalingVectors[N - 1].asDiagonal().toDenseMatrix()));
}

void scaleDataOneStepInPlaceInParallel(const vector_array_t& D, const vector_array_t& E, const int N,
                                       std::vector<VectorFunctionLinearApproximation>& dynamics,
                                       std::vector<ScalarFunctionQuadraticApproximation>& cost, std::vector<vector_t>& scalingVectors,
                                       ThreadPool& threadPool) {
  scaleMatrixInPlace(&(D[0]), &(D[0]), cost.front().dfduu);
  scaleMatrixInPlace(&(D[0]), nullptr, cost.front().dfdu);
  scaleMatrixInPlace(&(D[0]), nullptr, cost.front().dfdux);

  std::atomic_int timeStamp{1};
  auto scaleCost = [&](int workerId) {
    int k;
    while ((k = timeStamp++) < N) {
      auto& Q = cost[k].dfdxx;
      auto& R = cost[k].dfduu;
      auto& P = cost[k].dfdux;
      auto& q = cost[k].dfdx;
      auto& r = cost[k].dfdu;

      scaleMatrixInPlace(&(D[2 * k - 1]), &(D[2 * k - 1]), Q);
      scaleMatrixInPlace(&(D[2 * k - 1]), nullptr, q);
      scaleMatrixInPlace(&(D[2 * k]), &(D[2 * k - 1]), P);

      scaleMatrixInPlace(&(D[2 * k]), &(D[2 * k]), R);
      scaleMatrixInPlace(&(D[2 * k]), nullptr, r);
    }
  };

  threadPool.runParallel(std::move(scaleCost), threadPool.numThreads() + 1U);

  scaleMatrixInPlace(&(D.back()), &(D.back()), cost[N].dfdxx);
  scaleMatrixInPlace(&(D.back()), nullptr, cost[N].dfdx);

  // Scale G & g
  auto& B_0 = dynamics[0].dfdu;
  auto& C_0 = scalingVectors[0];
  /**
   * \f$
   * \tilde{B} = E * B * D,
   * scaling = E * I * D,
   * \tilde{g} = E * g,
   * \f$
   */
  scaleMatrixInPlace(&(E[0]), &(D[0]), B_0);
  C_0.array() *= E[0].array() * D[1].array();

  scaleMatrixInPlace(&(E[0]), nullptr, dynamics[0].dfdx);
  scaleMatrixInPlace(&(E[0]), nullptr, dynamics[0].f);

  timeStamp = 1;
  auto scaleConstraints = [&](int workerId) {
    int k;
    while ((k = timeStamp++) < N) {
      auto& A_k = dynamics[k].dfdx;
      auto& B_k = dynamics[k].dfdu;
      auto& b_k = dynamics[k].f;
      auto& C_k = scalingVectors[k];

      scaleMatrixInPlace(&(E[k]), &(D[2 * k - 1]), A_k);
      scaleMatrixInPlace(&(E[k]), &(D[2 * k]), B_k);
      C_k.array() *= E[k].array() * D[2 * k + 1].array();

      scaleMatrixInPlace(&(E[k]), nullptr, b_k);
    }
  };

  threadPool.runParallel(std::move(scaleConstraints), threadPool.numThreads() + 1U);
}
}  // namespace

void preConditioningSparseMatrixInPlace(Eigen::SparseMatrix<scalar_t>& H, vector_t& h, Eigen::SparseMatrix<scalar_t>& G,
                                        const int iteration, vector_t& DOut, vector_t& EOut, scalar_t& cOut) {
  const int nz = H.rows();
  const int nc = G.rows();

  // Init output
  cOut = 1.0;
  DOut.setOnes(nz);
  EOut.setOnes(nc);

  for (int i = 0; i < iteration; i++) {
    vector_t D, E;
    const vector_t infNormOfHCols = matrixInfNormCols(H);
    const vector_t infNormOfGCols = matrixInfNormCols(G);

    D = infNormOfHCols.array().max(infNormOfGCols.array());
    E = matrixInfNormRows(G);

    for (int i = 0; i < D.size(); i++) {
      if (D(i) > 1e+4) D(i) = 1e+4;
      if (D(i) < 1e-4) D(i) = 1.0;
    }

    for (int i = 0; i < E.size(); i++) {
      if (E(i) > 1e+4) E(i) = 1e+4;
      if (E(i) < 1e-4) E(i) = 1.0;
    }

    D = D.array().sqrt().inverse();
    E = E.array().sqrt().inverse();

    scaleMatrixInPlace(D, D, H);

    scaleMatrixInPlace(E, D, G);

    h.array() *= D.array();

    DOut = DOut.cwiseProduct(D);
    EOut = EOut.cwiseProduct(E);

    scalar_t infNormOfh = h.lpNorm<Eigen::Infinity>();
    if (infNormOfh < 1e-4) infNormOfh = 1.0;

    const vector_t infNormOfHColsUpdated = matrixInfNormCols(H);
    scalar_t c_temp = std::max(infNormOfHColsUpdated.mean(), infNormOfh);
    if (c_temp > 1e+4) c_temp = 1e+4;
    if (c_temp < 1e-4) c_temp = 1.0;

    scalar_t gamma = 1.0 / c_temp;

    H *= gamma;
    h *= gamma;

    cOut *= gamma;
  }
}

void preConditioningInPlaceInParallel(const vector_t& x0, const OcpSize& ocpSize, const int iteration,
                                      std::vector<VectorFunctionLinearApproximation>& dynamics,
                                      std::vector<ScalarFunctionQuadraticApproximation>& cost, vector_array_t& DOut, vector_array_t& EOut,
                                      vector_array_t& scalingVectors, scalar_t& cOut, ThreadPool& threadPool,
                                      const Eigen::SparseMatrix<scalar_t>& H, const vector_t& h, const Eigen::SparseMatrix<scalar_t>& G) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[preConditioningInPlaceInParallel] The number of stages cannot be less than 1.");
  }

  auto limitScaling = [](vector_t& vec) {
    for (int i = 0; i < vec.size(); i++) {
      if (vec(i) > 1e+4) vec(i) = 1e+4;
      if (vec(i) < 1e-4) vec(i) = 1.0;
    }
  };

  // Init output
  cOut = 1.0;
  DOut.resize(2 * N);
  EOut.resize(N);
  scalingVectors.resize(N);
  for (int i = 0; i < N; i++) {
    DOut[2 * i].setOnes(ocpSize.numInputs[i]);
    DOut[2 * i + 1].setOnes(ocpSize.numStates[i + 1]);
    EOut[i].setOnes(ocpSize.numStates[i + 1]);
    scalingVectors[i].setOnes(ocpSize.numStates[i + 1]);
  }

  // /**
  //  * Test start
  //  */
  // Eigen::SparseMatrix<scalar_t> HTest = H;
  // vector_t hTest = h;
  // Eigen::SparseMatrix<scalar_t> GTest = G;
  // /**
  //  * Test end
  //  */

  vector_array_t D(2 * N), E(N);
  for (int i = 0; i < iteration; i++) {
    invSqrtInfNormInParallel(dynamics, cost, scalingVectors, N, limitScaling, D, E, threadPool);
    scaleDataOneStepInPlaceInParallel(D, E, N, dynamics, cost, scalingVectors, threadPool);

    for (int k = 0; k < DOut.size(); k++) {
      DOut[k].array() *= D[k].array();
    }
    for (int k = 0; k < EOut.size(); k++) {
      EOut[k].array() *= E[k].array();
    }

    scalar_t infNormOfh = (cost.front().dfdu + cost.front().dfdux * x0).lpNorm<Eigen::Infinity>();
    for (int k = 1; k < N; k++) {
      infNormOfh = std::max(infNormOfh, std::max(cost[k].dfdx.lpNorm<Eigen::Infinity>(), cost[k].dfdu.lpNorm<Eigen::Infinity>()));
    }
    infNormOfh = std::max(infNormOfh, cost[N].dfdx.lpNorm<Eigen::Infinity>());
    if (infNormOfh < 1e-4) infNormOfh = 1.0;

    scalar_t sumOfInfNormOfH = matrixInfNormCols(cost[0].dfduu).derived().sum();
    for (int k = 1; k < N; k++) {
      sumOfInfNormOfH += matrixInfNormCols(cost[k].dfdxx, cost[k].dfdux).derived().sum();
      sumOfInfNormOfH += matrixInfNormCols(cost[k].dfdux.transpose().eval(), cost[k].dfduu).derived().sum();
    }
    sumOfInfNormOfH += matrixInfNormCols(cost[N].dfdxx).derived().sum();

    int numDecisionVariables = std::accumulate(ocpSize.numInputs.begin(), ocpSize.numInputs.end(),
                                               std::accumulate(ocpSize.numStates.begin() + 1, ocpSize.numStates.end(), (int)0));

    scalar_t c_temp = std::max(sumOfInfNormOfH / static_cast<scalar_t>(numDecisionVariables), infNormOfh);
    if (c_temp > 1e+4) c_temp = 1e+4;
    if (c_temp < 1e-4) c_temp = 1.0;

    scalar_t gamma = 1.0 / c_temp;

    for (int k = 0; k <= N; ++k) {
      cost[k].dfdxx *= gamma;
      cost[k].dfduu *= gamma;
      cost[k].dfdux *= gamma;
      cost[k].dfdx *= gamma;
      cost[k].dfdu *= gamma;
    }

    cOut *= gamma;

    // /**
    //  * Test start
    //  */
    // auto isEqual = [](scalar_t a, scalar_t b) -> bool {
    //   return std::abs(a - b) < std::numeric_limits<scalar_t>::epsilon() * std::min(std::abs(a), std::abs(b));
    // };
    // vector_t DTest, ETest;
    // vector_t infNormOfHCols = matrixInfNormCols(HTest);
    // vector_t infNormOfGCols = matrixInfNormCols(GTest);

    // DTest = infNormOfHCols.array().max(infNormOfGCols.array());
    // ETest = matrixInfNormRows(GTest);

    // limitScaling(DTest);
    // limitScaling(ETest);

    // std::cerr << "DTest: " << DTest.transpose() << "\n";
    // std::cerr << "ETest   : " << ETest.transpose() << "\n";
    // std::cerr << "G   :\n " << GTest.toDense() << "\n";

    // DTest = DTest.array().sqrt().inverse();
    // ETest = ETest.array().sqrt().inverse();

    // scaleMatrixInPlace(DTest, DTest, HTest);

    // scaleMatrixInPlace(ETest, DTest, GTest);

    // hTest.array() *= DTest.array();

    // scalar_t infNormOfhTest = hTest.lpNorm<Eigen::Infinity>();
    // if (infNormOfhTest < 1e-4) infNormOfhTest = 1.0;

    // const vector_t infNormOfHColsUpdated = matrixInfNormCols(HTest);
    // scalar_t c_tempTest = std::max(infNormOfHColsUpdated.mean(), infNormOfhTest);
    // if (c_tempTest > 1e+4) c_tempTest = 1e+4;
    // if (c_tempTest < 1e-4) c_tempTest = 1.0;

    // scalar_t gammaTest = 1.0 / c_tempTest;

    // HTest *= gamma;
    // hTest *= gamma;

    // ocs2::vector_t DStacked(H.cols()), EStacked(G.rows());
    // int curRow = 0;
    // for (auto& v : D) {
    //   DStacked.segment(curRow, v.size()) = v;
    //   curRow += v.size();
    // }
    // curRow = 0;
    // for (auto& v : E) {
    //   EStacked.segment(curRow, v.size()) = v;
    //   curRow += v.size();
    // }
    // if (!DStacked.isApprox(DTest)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "DStacked: " << DStacked.transpose() << "\n";
    //   std::cerr << "DTest   : " << DTest.transpose() << "\n";
    //   std::cerr << "diff    : " << (DStacked - DTest).transpose() << std::endl;
    //   std::cerr << "R: \n" << cost[0].dfduu << std::endl;
    //   std::cerr << "B: \n" << dynamics[0].dfdu << std::endl;
    // }
    // if (!EStacked.isApprox(ETest)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "EStacked: " << EStacked.transpose() << "\n";
    //   std::cerr << "ETest: " << ETest.transpose() << "\n";
    //   std::cerr << "diff: " << (ETest - EStacked).transpose() << std::endl;
    // }
    // if (!isEqual(infNormOfhTest, infNormOfh)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "infNormOfhTest: " << infNormOfhTest << "\n";
    //   std::cerr << "infNormOfh: " << infNormOfh << "\n";
    //   std::cerr << "diff: " << (infNormOfhTest - infNormOfh) << std::endl;
    // }
    // if (!isEqual(infNormOfHColsUpdated.sum(), sumOfInfNormOfH)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "infNormOfHColsUpdated.sum(): " << infNormOfHColsUpdated.sum() << "\n";
    //   std::cerr << "sumOfInfNormOfH: " << sumOfInfNormOfH << "\n";
    //   std::cerr << "diff: " << (infNormOfHColsUpdated.sum() - sumOfInfNormOfH) << std::endl;
    // }
    // if (!isEqual(c_temp, c_tempTest)) {
    //   std::cerr << "i: " << i << "\n";
    //   std::cerr << "c_temp: " << c_temp << "\n";
    //   std::cerr << "c_tempTest: " << c_tempTest << "\n";
    //   std::cerr << "diff: " << (c_temp - c_tempTest) << std::endl;
    // }
    // /**
    //  * Test end
    //  */
  }
}

void scaleDataInPlace(const OcpSize& ocpSize, const vector_t& D, const vector_t& E, const scalar_t c,
                      std::vector<VectorFunctionLinearApproximation>& dynamics, std::vector<ScalarFunctionQuadraticApproximation>& cost,
                      std::vector<vector_t>& scalingVectors) {
  const int N = ocpSize.numStages;
  if (N < 1) {
    throw std::runtime_error("[PIPG] The number of stages cannot be less than 1.");
  }

  scalingVectors.resize(N);

  // Scale H & h
  const int nu_0 = ocpSize.numInputs.front();
  // Scale row - Pre multiply D
  cost.front().dfduu.array().colwise() *= c * D.head(nu_0).array();
  // Scale col - Post multiply D
  cost.front().dfduu *= D.head(nu_0).asDiagonal();

  cost.front().dfdu.array() *= c * D.head(nu_0).array();

  cost.front().dfdux.array().colwise() *= c * D.head(nu_0).array();

  int currRow = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nx_k = ocpSize.numStates[k];
    const int nu_k = ocpSize.numInputs[k];
    auto& Q = cost[k].dfdxx;
    auto& R = cost[k].dfduu;
    auto& P = cost[k].dfdux;
    auto& q = cost[k].dfdx;
    auto& r = cost[k].dfdu;

    Q.array().colwise() *= c * D.segment(currRow, nx_k).array();
    Q *= D.segment(currRow, nx_k).asDiagonal();
    q.array() *= c * D.segment(currRow, nx_k).array();

    P *= D.segment(currRow, nx_k).asDiagonal();
    currRow += nx_k;

    R.array().colwise() *= c * D.segment(currRow, nu_k).array();
    R *= D.segment(currRow, nu_k).asDiagonal();
    r.array() *= c * D.segment(currRow, nu_k).array();

    P.array().colwise() *= c * D.segment(currRow, nu_k).array();

    currRow += nu_k;
  }

  const int nx_N = ocpSize.numStates[N];
  cost[N].dfdxx.array().colwise() *= c * D.tail(nx_N).array();
  cost[N].dfdxx *= D.tail(nx_N).asDiagonal();
  cost[N].dfdx.array() *= c * D.tail(nx_N).array();

  // Scale G & g
  auto& B_0 = dynamics[0].dfdu;
  auto& C_0 = scalingVectors[0];
  const int nx_1 = ocpSize.numStates[1];

  //  \tilde{B} = E * B * D,
  //  scaling = E * D,
  //  \tilde{g} = E * g
  B_0.array().colwise() *= E.head(nx_1).array();
  B_0 *= D.head(nu_0).asDiagonal();

  C_0 = E.head(nx_1).array() * D.segment(nu_0, nx_1).array();

  dynamics[0].dfdx.array().colwise() *= E.head(nx_1).array();
  dynamics[0].f.array() *= E.head(nx_1).array();

  currRow = nx_1;
  int currCol = nu_0;
  for (int k = 1; k < N; ++k) {
    const int nu_k = ocpSize.numInputs[k];
    const int nx_k = ocpSize.numStates[k];
    const int nx_next = ocpSize.numStates[k + 1];
    auto& A_k = dynamics[k].dfdx;
    auto& B_k = dynamics[k].dfdu;
    auto& b_k = dynamics[k].f;
    auto& C_k = scalingVectors[k];

    A_k.array().colwise() *= E.segment(currRow, nx_next).array();
    A_k *= D.segment(currCol, nx_k).asDiagonal();

    B_k.array().colwise() *= E.segment(currRow, nx_next).array();
    B_k *= D.segment(currCol + nx_k, nu_k).asDiagonal();

    C_k = E.segment(currRow, nx_next).array() * D.segment(currCol + nx_k + nu_k, nx_next).array();

    b_k.array() *= E.segment(currRow, nx_next).array();

    currRow += nx_next;
    currCol += nx_k + nu_k;
  }
}

}  // namespace ocs2