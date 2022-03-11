#include "ocs2_mpcnet/control/MpcnetOnnxController.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetOnnxController::loadPolicyModel(const std::string& policyFilePath) {
  policyFilePath_ = policyFilePath;
  // create session
  Ort::SessionOptions sessionOptions;
  sessionPtr_.reset(new Ort::Session(*onnxEnvironmentPtr_, policyFilePath_.c_str(), sessionOptions));
  // get input and output info
  inputNames_.clear();
  outputNames_.clear();
  inputShapes_.clear();
  outputShapes_.clear();
  Ort::AllocatorWithDefaultOptions allocator;
  for (int i = 0; i < sessionPtr_->GetInputCount(); i++) {
    inputNames_.push_back(sessionPtr_->GetInputName(i, allocator));
    inputShapes_.push_back(sessionPtr_->GetInputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }
  for (int i = 0; i < sessionPtr_->GetOutputCount(); i++) {
    outputNames_.push_back(sessionPtr_->GetOutputName(i, allocator));
    outputShapes_.push_back(sessionPtr_->GetOutputTypeInfo(i).GetTensorTypeAndShapeInfo().GetShape());
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MpcnetOnnxController::computeInput(const scalar_t t, const vector_t& x) {
  // create input tensor objects
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> time = getGeneralizedTime(t).cast<tensor_element_t>();
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> state = getRelativeState(t, x).cast<tensor_element_t>();
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  inputValues.push_back(
      Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, time.data(), time.size(), inputShapes_[0].data(), inputShapes_[0].size()));
  inputValues.push_back(
      Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, state.data(), state.size(), inputShapes_[1].data(), inputShapes_[1].size()));
  // run inference
  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues = sessionPtr_->Run(runOptions, inputNames_.data(), inputValues.data(), 2, outputNames_.data(), 1);
  // evaluate output tensor objects
  Eigen::Map<Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1>> input(outputValues[0].GetTensorMutableData<tensor_element_t>(),
                                                                       outputShapes_[0][1], outputShapes_[0][0]);
  return getInputTransformation(t, x) * input.cast<scalar_t>();
}

}  // namespace ocs2
