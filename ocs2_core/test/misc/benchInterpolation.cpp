#include <benchmark/benchmark.h>

#include <algorithm>
#include <iostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

/* Alternative interpolation function for comparison */

/*
 * Declares an access function class of name FIELD such as time, dynamics, dynamicsBias, ...
 * For example the signature of function for dynamics is:
 * struct dynamics {
 *   const vector_t& operator()(const ocs2::ModelDataBase& m) { return m.dynamics_; }
 * };
 */
#define CREATE_INTERPOLATION_ACCESS_FUNCTOR(FIELD)                                                      \
  struct ModelData_##FIELD {                                                                            \
    auto operator()(const ocs2::ModelDataBase& m) -> const decltype(m.FIELD##_)& { return m.FIELD##_; } \
  };

#define CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(FIELD, SUBFIELD)                                                     \
  struct ModelData_##FIELD##_##SUBFIELD {                                                                                 \
    auto operator()(const ocs2::ModelDataBase& m) -> const decltype(m.FIELD##_.SUBFIELD)& { return m.FIELD##_.SUBFIELD; } \
  };

// time
CREATE_INTERPOLATION_ACCESS_FUNCTOR(time)

// dynamics
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(dynamics, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR(dynamicsBias)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(dynamics, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(dynamics, dfdu)
CREATE_INTERPOLATION_ACCESS_FUNCTOR(dynamicsCovariance)

// cost
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdu)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdxx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfduu)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(cost, dfdux)

// state equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateEqConstr, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateEqConstr, dfdx)

// state-input equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateInputEqConstr, f)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateInputEqConstr, dfdx)
CREATE_INTERPOLATION_ACCESS_FUNCTOR_SUBFIELD(stateInputEqConstr, dfdu)

namespace ocs2 {
namespace LinearInterpolation {

/**
 * Default subfield access function, returns value itself.
 */
template <typename T>
struct Identity {
  const T& operator()(const T& value) const { return value; };
};

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
 * @tparam AccessFun: Subfield data access functor
 * @tparam Data: Data type
 * @tparam Field: Data's subfield type.
 * @tparam Alloc: Specialized allocation class
 */
template <class AccessFun, typename Data, typename Field, class Alloc>
void interpolateField(index_alpha_t indexAlpha, Field& enquiryData, const std::vector<Data, Alloc>* dataPtr) {
  if (dataPtr != nullptr) {
    AccessFun accessFun;
    if (dataPtr->size() > 1) {
      // Normal interpolation case
      int index = indexAlpha.first;
      scalar_t alpha = indexAlpha.second;
      auto& lhs = accessFun((*dataPtr)[index]);
      auto& rhs = accessFun((*dataPtr)[index + 1]);
      if (areSameSize(rhs, lhs)) {
        enquiryData = alpha * lhs + (scalar_t(1.0) - alpha) * rhs;
      } else {
        enquiryData = (alpha > 0.5) ? lhs : rhs;
      }
    } else if (dataPtr->size() == 1) {
      // Time vector has only 1 element -> Constant function
      enquiryData = accessFun((*dataPtr)[0]);
    } else {
      // Time empty -> zero function
      enquiryData *= scalar_t(0.0);
    }
  } else {
    // No data set -> zero Function
    enquiryData *= scalar_t(0.0);
  }
}

/**
 * Linearly interpolates at the given time. When duplicate values exist the lower range is selected s.t. ( ]
 * Example: t = [0.0, 1.0, 1.0, 2.0]
 * when querying tk = 1.0, the range (0.0, 1.0] is selected
 *
 *  - No data (nullptrs or zero size containers) implies the zero function
 *  - Single data point implies a constant function
 *  - Multiple data points are used for linear interpolation and zero order extrapolation
 *
 * @param [in] enquiryTime: The enquiry time for interpolation.
 * @param [out] enquiryData: The value of the trajectory at the requested time.
 * @param [in] timeStampPtr: Pointer to vector of times
 * @param [in] dataPtr: Pointer to vector of data
 * @return {index, alpha}: The greatest smaller time stamp index and the interpolation coefficient [1, 0]
 *
 * @tparam AccessFun: Subfield data access functor
 * @tparam Data: Data type
 * @tparam Field: Data's subfield type.
 * @tparam Alloc: Specialized allocation class
 */
template <class AccessFun, typename Data, typename Field, class Alloc>
index_alpha_t interpolateField(scalar_t enquiryTime, Field& enquiryData, const std::vector<scalar_t>* timeStampPtr,
                               const std::vector<Data, Alloc>* dataPtr) {
  auto indexAlpha = timeSegment(enquiryTime, timeStampPtr);
  interpolateField<AccessFun, Data, Field, Alloc>(indexAlpha, enquiryData, dataPtr);
  return indexAlpha;
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

void interpolate_Scalar(benchmark::State& state) {
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
BENCHMARK(interpolate_Scalar);

void interpolate_EigenVector(benchmark::State& state) {
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
BENCHMARK(interpolate_EigenVector);

void interpolate_EigenMatrix(benchmark::State& state) {
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
BENCHMARK(interpolate_EigenMatrix);

void interpolate_Scalar_AccessFunctor(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::scalar_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::vector_t::Random(1)(0); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::scalar_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateField<ocs2::LinearInterpolation::Identity<ocs2::scalar_t>>(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_Scalar_AccessFunctor);

void interpolate_EigenVector_AccessFunctor(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::vector_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::vector_t::Random(10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::vector_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateField<ocs2::LinearInterpolation::Identity<ocs2::vector_t>>(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_EigenVector_AccessFunctor);

void interpolate_EigenMatrix_AccessFunctor(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::matrix_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return ocs2::matrix_t::Random(10, 10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::matrix_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateField<ocs2::LinearInterpolation::Identity<ocs2::matrix_t>>(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_EigenMatrix_AccessFunctor);

void interpolate_ModelDataScalar(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolate(indexAlpha, res.cost_.f, &data, ocs2::ModelData::cost_f);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_ModelDataScalar);

void interpolate_ModelDataVector(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolate(indexAlpha, res.cost_.dfdx, &data, ocs2::ModelData::cost_dfdx);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_ModelDataVector);

void interpolate_ModelDataMatrix(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolate(indexAlpha, res.cost_.dfdxx, &data, ocs2::ModelData::cost_dfdxx);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_ModelDataMatrix);

void interpolate_ModelDataScalar_AccessFunctor(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolateField<ModelData_cost_f>(indexAlpha, res.cost_.f, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_ModelDataScalar_AccessFunctor);

void interpolate_ModelDataVector_AccessFunctor(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolateField<ModelData_cost_dfdx>(indexAlpha, res.cost_.dfdx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_ModelDataVector_AccessFunctor);

void interpolate_ModelDataMatrix_AccessFunctor(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolateField<ModelData_cost_dfdxx>(indexAlpha, res.cost_.dfdxx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interpolate_ModelDataMatrix_AccessFunctor);

BENCHMARK_MAIN();
