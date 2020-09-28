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
  std::generate(data.begin(), data.end(), []() { return Eigen::VectorXd::Random(10); });

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
  std::generate(data.begin(), data.end(), []() { return Eigen::MatrixXd::Random(10, 10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::matrix_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolate(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_EigenMatrix);

void interploate_Scalar_DirectAccess(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::scalar_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return Eigen::VectorXd::Random(1)(0); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::scalar_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateSimple(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_Scalar_DirectAccess);

void interploate_EigenVector_DirectAccess(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::vector_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return Eigen::VectorXd::Random(10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::vector_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateSimple(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_EigenVector_DirectAccess);

void interploate_Scalar_AccessTrait(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::scalar_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return Eigen::VectorXd::Random(1)(0); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::scalar_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateTpl<ocs2::LinearInterpolation::IdentityAccess<ocs2::scalar_t>>(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_Scalar_AccessTrait);

void interploate_EigenVector_AccessTrait(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  ocs2::vector_array_t data(times.size());
  std::generate(data.begin(), data.end(), []() { return Eigen::VectorXd::Random(10); });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  ocs2::vector_t res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateTpl<ocs2::LinearInterpolation::IdentityAccess<ocs2::vector_t>>(indexAlpha, res, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_EigenVector_AccessTrait);

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
    ocs2::ModelData::interpolate(indexAlpha, res.cost_.f, &data, ocs2::ModelData::cost_f);
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
    ocs2::ModelData::interpolate(indexAlpha, res.cost_.dfdx, &data, ocs2::ModelData::cost_dfdx);
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
    ocs2::ModelData::interpolate(indexAlpha, res.cost_.dfdxx, &data, ocs2::ModelData::cost_dfdxx);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataMatrix);

struct ModelData_cost_f {
  const ocs2::scalar_t& operator()(const ocs2::ModelDataBase& md) { return md.cost_.f; }
};
struct ModelData_cost_dfdx {
  const ocs2::vector_t& operator()(const ocs2::ModelDataBase& md) { return md.cost_.dfdx; }
};
struct ModelData_cost_dfdxx {
  const ocs2::matrix_t& operator()(const ocs2::ModelDataBase& md) { return md.cost_.dfdxx; }
};

void interploate_ModelDataScalar_AccessTrait(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolateTpl<ModelData_cost_f>(indexAlpha, res.cost_.f, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataScalar_AccessTrait);

void interploate_ModelDataVector_AccessTrait(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolateTpl<ModelData_cost_dfdx>(indexAlpha, res.cost_.dfdx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataVector_AccessTrait);

void interploate_ModelDataMatrix_AccessTrait(benchmark::State& state) {
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
    ocs2::LinearInterpolation::interpolateTpl<ModelData_cost_dfdxx>(indexAlpha, res.cost_.dfdxx, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_ModelDataMatrix_AccessTrait);

struct MyStruct {
  ocs2::scalar_t scalar;
  ocs2::vector_t vector;
};

void interploate_MyStructScalar_MemberPointer(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<MyStruct> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    MyStruct s;
    s.scalar = 0.0;
    return s;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  MyStruct res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolate(indexAlpha, res.scalar, &data, &MyStruct::scalar);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_MyStructScalar_MemberPointer);

void interploate_MyStructVector_MemberPointer(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<MyStruct> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    MyStruct s;
    s.vector.setRandom(10);
    return s;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  MyStruct res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolate(indexAlpha, res.vector, &data, &MyStruct::vector);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_MyStructVector_MemberPointer);

struct MyStruct_scalar {
  const ocs2::scalar_t& operator()(const MyStruct& value) const { return value.scalar; }
};

struct MyStruct_vector {
  const ocs2::vector_t& operator()(const MyStruct& value) const { return value.vector; }
};

void interploate_MyStructScalar_AccessTrait(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<MyStruct> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    MyStruct s;
    s.scalar = 0.0;
    return s;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  MyStruct res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateTpl<MyStruct_scalar>(indexAlpha, res.scalar, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_MyStructScalar_AccessTrait);

void interploate_MyStructVector_AccessTrait(benchmark::State& state) {
  ocs2::scalar_array_t times = getLinSpacedTimes(0.0, 1.0, 100);
  std::vector<MyStruct> data(times.size());
  std::generate(data.begin(), data.end(), []() {
    MyStruct s;
    s.vector.setRandom(10);
    return s;
  });

  const auto indexAlpha = ocs2::LinearInterpolation::timeSegment(0.5, &times);

  MyStruct res;
  for (auto _ : state) {
    ocs2::LinearInterpolation::interpolateTpl<MyStruct_vector>(indexAlpha, res.vector, &data);
    benchmark::DoNotOptimize(res);
    benchmark::ClobberMemory();
  }
}
BENCHMARK(interploate_MyStructVector_AccessTrait);

BENCHMARK_MAIN();
