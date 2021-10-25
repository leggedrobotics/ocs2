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
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> tEigenData = getGeneralizedTime(t).cast<tensor_element_t>();
  Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1> xEigenData = getRelativeState(t, x).cast<tensor_element_t>();
  std::vector<tensor_element_t> tData(tEigenData.data(), tEigenData.data() + tEigenData.size());
  std::vector<tensor_element_t> xData(xEigenData.data(), xEigenData.data() + xEigenData.size());
  Ort::MemoryInfo memoryInfo = Ort::MemoryInfo::CreateCpu(OrtAllocatorType::OrtArenaAllocator, OrtMemType::OrtMemTypeDefault);
  std::vector<Ort::Value> inputValues;
  inputValues.push_back(
      Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, tData.data(), tData.size(), inputShapes_[0].data(), inputShapes_[0].size()));
  inputValues.push_back(
      Ort::Value::CreateTensor<tensor_element_t>(memoryInfo, xData.data(), xData.size(), inputShapes_[1].data(), inputShapes_[1].size()));
  // run inference
  Ort::RunOptions runOptions;
  std::vector<Ort::Value> outputValues = sessionPtr_->Run(runOptions, inputNames_.data(), inputValues.data(), 2, outputNames_.data(), 2);
  // evaluate output tensor objects (note that from u, p, U we only need u = U * p which is already evaluated by the model)
  Eigen::Map<Eigen::Matrix<tensor_element_t, Eigen::Dynamic, 1>> u(outputValues[0].GetTensorMutableData<tensor_element_t>(),
                                                                   outputShapes_[0][1], outputShapes_[0][0]);
  return getInputTransformation(t, x) * u.cast<scalar_t>();
}

}  // namespace ocs2
