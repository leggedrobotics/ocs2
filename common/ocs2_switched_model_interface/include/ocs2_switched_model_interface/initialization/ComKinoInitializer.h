#pragma once

#include <ocs2_core/initialization/Initializer.h>

#include "ocs2_switched_model_interface/core/ComModelBase.h"
#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/logic/SwitchedModelModeScheduleManager.h"

namespace switched_model {

class ComKinoInitializer : public ocs2::Initializer {
 public:
  using com_model_t = ComModelBase<scalar_t>;

  ComKinoInitializer(const com_model_t& comModel, const SwitchedModelModeScheduleManager& modeScheduleManager);

  ~ComKinoInitializer() override = default;

  ComKinoInitializer* clone() const override;

  void compute(scalar_t time, const vector_t& state, scalar_t nextTime, vector_t& input, vector_t& nextState) override;

 protected:
  ComKinoInitializer(const ComKinoInitializer& rhs);

 private:
  std::unique_ptr<com_model_t> comModelPtr_;
  const SwitchedModelModeScheduleManager* modeScheduleManagerPtr_;
};

}  // end of namespace switched_model
