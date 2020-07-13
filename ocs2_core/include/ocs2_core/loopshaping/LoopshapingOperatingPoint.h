//
// Created by ruben on 08.11.18.
//

#pragma once

#include <memory>

#include <ocs2_core/Types.h>
#include <ocs2_core/initialization/SystemOperatingTrajectoriesBase.h>
#include <ocs2_core/loopshaping/LoopshapingDefinition.h>

namespace ocs2 {
class LoopshapingOperatingPoint final : public SystemOperatingTrajectoriesBase {
 public:
  LoopshapingOperatingPoint(const SystemOperatingTrajectoriesBase& systembase, std::shared_ptr<LoopshapingDefinition> loopshapingDefinition)
      : SystemOperatingTrajectoriesBase(), systembase_(systembase.clone()), loopshapingDefinition_(std::move(loopshapingDefinition)) {}

  virtual ~LoopshapingOperatingPoint() = default;

  LoopshapingOperatingPoint(const LoopshapingOperatingPoint& obj)
      : SystemOperatingTrajectoriesBase(), systembase_(obj.systembase_->clone()), loopshapingDefinition_(obj.loopshapingDefinition_) {}

  LoopshapingOperatingPoint* clone() const override { return new LoopshapingOperatingPoint(*this); }

  void getSystemOperatingTrajectories(const vector_t& initialState, scalar_t startTime, scalar_t finalTime, scalar_array_t& timeTrajectory,
                                      vector_array_t& stateTrajectory, vector_array_t& inputTrajectory, bool concatOutput = false) override;

 private:
  std::unique_ptr<SystemOperatingTrajectoriesBase> systembase_;
  std::shared_ptr<LoopshapingDefinition> loopshapingDefinition_;
};
}  // namespace ocs2
