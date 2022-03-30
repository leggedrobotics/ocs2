//
// Created by rgrandia on 25.03.22.
//

#pragma once

#include <ocs2_switched_model_interface/core/SwitchedModel.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

namespace switched_model {

template <typename SCALAR_T>
class ModelParentSkeleton {
 public:
  ModelParentSkeleton(ocs2::PinocchioInterfaceTpl<SCALAR_T> pinnochioInterface, std::vector<std::string> frames)
      : pinocchioInterface_(std::move(pinnochioInterface)) {
    const auto& model = pinocchioInterface_.getModel();
    for (const auto& name : frames) {
      frameIds_.push_back(model.getBodyId(name));
    }
  }

 private:
  ocs2::PinocchioInterfaceTpl<SCALAR_T> pinocchioInterface_;
  std::vector<size_t> frameIds_;
};

}  // namespace switched_model