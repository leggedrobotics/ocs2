#include <benchmark/benchmark.h>

#include <algorithm>
#include <iostream>

#include <ocs2_core/Types.h>
#include <ocs2_core/misc/LinearInterpolation.h>
#include <ocs2_core/model_data/ModelDataLinearInterpolation.h>

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
  std::generate(data.begin(), data.end(), []() { return Eigen::VectorXd::Random(1)(0); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  for (auto _ : state) {
    ocs2::scalar_t res;
    ocs2::LinearInterpolation::interpolate(indexAlpha, res, &data);
  }
}
BENCHMARK(interploate_Scalar);

void interploate_EigenVector(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::vector_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return Eigen::VectorXd::Random(10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  for (auto _ : state) {
    ocs2::vector_t res;
    ocs2::LinearInterpolation::interpolate(indexAlpha, res, &data);
  }
}
BENCHMARK(interploate_EigenVector);

void interploate_EigenMatrix(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::matrix_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return Eigen::MatrixXd::Random(10, 10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  for (auto _ : state) {
    ocs2::matrix_t res;
    ocs2::LinearInterpolation::interpolate(indexAlpha, res, &data);
  }
}
BENCHMARK(interploate_EigenMatrix);

void interploate_ModelDataScalar(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<ocs2::ModelDataBase> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    ocs2::ModelDataBase md;
    md.cost_.f = 0.0;
    return md;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  for (auto _ : state) {
    ocs2::ModelDataBase res;
    ocs2::ModelData::interpolate(indexAlpha, res.cost_.f, &data, ocs2::ModelData::cost_f);
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

  for (auto _ : state) {
    ocs2::ModelDataBase res;
    ocs2::ModelData::interpolate(indexAlpha, res.cost_.dfdx, &data, ocs2::ModelData::cost_dfdx);
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

  for (auto _ : state) {
    ocs2::ModelDataBase res;
    ocs2::ModelData::interpolate(indexAlpha, res.cost_.dfdxx, &data, ocs2::ModelData::cost_dfdxx);
  }
}
BENCHMARK(interploate_ModelDataMatrix);

BENCHMARK_MAIN();
