#include <benchmark/benchmark.h>

#include <algorithm>
#include <iostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

/* Old interpolation function for comparison */

/*
 * Declares an access function of name FIELD such as time, dynamics, dynamicsBias, ...
 * For example the signature of function for dynamics is:
 * const vector_t& dynamics(const std::vector<ocs2::ModelDataBase>* vec, size_t n) {
 *   return (*vec)[n].dynamic_;
 * }
 */
#define CREATE_INTERPOLATION_ACCESS_FUNCTION(FIELD)                                                                               \
  inline auto ModelData_##FIELD(const std::vector<ocs2::ModelDataBase>* vec, size_t ind)->const decltype((*vec)[ind].FIELD##_)& { \
    return (*vec)[ind].FIELD##_;                                                                                                  \
  }

#define CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(FIELD, SUBFIELD)                                \
  inline auto ModelData_##FIELD##_##SUBFIELD(const std::vector<ocs2::ModelDataBase>* vec, size_t ind) \
      ->const decltype((*vec)[ind].FIELD##_.SUBFIELD)& {                                              \
    return (*vec)[ind].FIELD##_.SUBFIELD;                                                             \
  }

// time
CREATE_INTERPOLATION_ACCESS_FUNCTION(time)

// dynamics
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(dynamics, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsBias)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(dynamics, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(dynamics, dfdu)
CREATE_INTERPOLATION_ACCESS_FUNCTION(dynamicsCovariance)

// cost
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdu)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdxx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfduu)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(cost, dfdux)

// state equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateEqConstr, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateEqConstr, dfdx)

// state-input equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateInputEqConstr, f)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateInputEqConstr, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTION_SUBFIELD(stateInputEqConstr, dfdu)

namespace ocs2 {
namespace LinearInterpolation {

/**
 * Helper access function for std::vector<Data_T, Alloc> where Data_T a simple data structure such as double, Eigen-Type.
 */
template <typename Data_T, class Alloc>
const Data_T& stdAccessFun(const std::vector<Data_T, Alloc>* vec, size_t ind) {
  return (*vec)[ind];
}

/**
 * Directly uses the index and interpolation coefficient provided by the user
 * @note If sizes in data array are not equal, the interpolation will snap to the data
 * point closest to the query time
 *
 *  - No data (nullptrs or zero size containers) implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] indexAlpha : index and interpolation coefficient (alpha) pair
 * @param [out] enquiryData : result of the interpolation
 * @param [in] dataPtr: Pointer to vector of data
 *
 * @tparam Data_T: Date type
 * @tparam Field_T: Data's subfield type.
 * @tparam Alloc: Specialized allocation class
 * @tparam AccessFun: A method to access the subfield of DATA_T
 */
template <typename Data_T, typename Field_T, class Alloc,
          const Field_T& (*AccessFun)(const std::vector<Data_T, Alloc>*, size_t) = stdAccessFun<Data_T, Alloc>>
void interpolateLegacy(index_alpha_t indexAlpha, Field_T& enquiryData, const std::vector<Data_T, Alloc>* dataPtr) {
  if (dataPtr != nullptr) {
    if (dataPtr->size() > 1) {
      // Normal interpolation case
      int index = indexAlpha.first;
      scalar_t alpha = indexAlpha.second;
      auto& lhs = AccessFun(dataPtr, index);
      auto& rhs = AccessFun(dataPtr, index + 1);
      if (areSameSize(rhs, lhs)) {
        enquiryData = alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
      } else {
        enquiryData = (alpha > 0.5) ? lhs : rhs;
      }
    } else if (dataPtr->size() == 1) {
      // Time vector has only 1 element -> Constant function
      enquiryData = AccessFun(dataPtr, 0);
    } else {
      // Time empty -> zero function
      enquiryData *= scalar_t(0.0);
    }
  } else {
    // No data set -> zero Function
    enquiryData *= scalar_t(0.0);
  }
}

}  // namespace LinearInterpolation
}  // namespace ocs2

ocs2::scalar_array_t getLinSpacedTimes(ocs2::scalar_t start, ocs2::scalar_t end, size_t N) {
  // assumes N > 1
  assert(N > 1);
  ocs2::scalar_array_t times(N);
  size_t count = 0;
  std::generate(times.begin(), times.end(), [&]() {
    ocs2::scalar_t t = start + count * (end - start) / (N - 1);
    count += 1;
    return t;
  });
  return times;
}

void interploate_Scalar(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::scalar_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::vector_t::Random(1)(0); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::scalar_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolate(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_Scalar);

void interploate_EigenVector(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::vector_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::vector_t::Random(10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::vector_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolate(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_EigenVector);

void interploate_EigenMatrix(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::matrix_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::matrix_t::Random(10, 10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::matrix_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolate(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_EigenMatrix);

void interploate_Scalar_AccessFunction(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::scalar_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::vector_t::Random(1)(0); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::scalar_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateLegacy(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_Scalar_AccessFunction);

void interploate_EigenVector_AccessFunction(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::vector_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::vector_t::Random(10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::vector_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateLegacy(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_EigenVector_AccessFunction);

void interploate_EigenMatrix_AccessFunction(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::matrix_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::matrix_t::Random(10, 10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::matrix_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateLegacy(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_EigenMatrix_AccessFunction);

void interploate_ModelDataScalar_AccessFunction(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<ocs2::ModelDataBase> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    ocs2::ModelDataBase md;
    md.cost_.f = 0.0;
    return md;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::ModelDataBase res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateLegacy<ocs2::ModelDataBase, ocs2::scalar_t, std::allocator<ocs2::ModelDataBase>,
                                                 ModelData_cost_f>(indexAlpha, res.cost_.f, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataScalar_AccessFunction);

void interploate_ModelDataVector_AccessFunction(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<ocs2::ModelDataBase> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    ocs2::ModelDataBase md;
    md.cost_.dfdx.setRandom(10);
    return md;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::ModelDataBase res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateLegacy<ocs2::ModelDataBase, ocs2::vector_t, std::allocator<ocs2::ModelDataBase>,
                                                 ModelData_cost_dfdx>(indexAlpha, res.cost_.dfdx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataVector_AccessFunction);

void interploate_ModelDataMatrix_AccessFunction(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<ocs2::ModelDataBase> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    ocs2::ModelDataBase md;
    md.cost_.dfdxx.setRandom(10, 10);
    return md;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::ModelDataBase res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateLegacy<ocs2::ModelDataBase, ocs2::matrix_t, std::allocator<ocs2::ModelDataBase>,
                                                 ModelData_cost_dfdxx>(indexAlpha, res.cost_.dfdxx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataMatrix_AccessFunction);

void interploate_ModelDataScalar(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<ocs2::ModelDataBase> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    ocs2::ModelDataBase md;
    md.cost_.f = 0.0;
    return md;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::ModelDataBase res;
  for (auto _ : state) {
    ocs2::ModelData::interpolate<ocs2::ModelData::cost_f>(indexAlpha, res.cost_.f, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataScalar);

void interploate_ModelDataVector(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<ocs2::ModelDataBase> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    ocs2::ModelDataBase md;
    md.cost_.dfdx.setRandom(10);
    return md;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::ModelDataBase res;
  for (auto _ : state) {
    ocs2::ModelData::interpolate<ocs2::ModelData::cost_dfdx>(indexAlpha, res.cost_.dfdx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataVector);

void interploate_ModelDataMatrix(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<ocs2::ModelDataBase> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    ocs2::ModelDataBase md;
    md.cost_.dfdxx.setRandom(10, 10);
    return md;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::ModelDataBase res;
  for (auto _ : state) {
    ocs2::ModelData::interpolate<ocs2::ModelData::cost_dfdxx>(indexAlpha, res.cost_.dfdxx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataMatrix);

BENCHMARK_MAIN();
