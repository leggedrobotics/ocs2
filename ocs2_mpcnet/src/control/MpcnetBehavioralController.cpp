#include "ocs2_mpcnet/control/MpcnetBehavioralController.h"

namespace ocs2 {

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
vector_t MpcnetBehavioralController::computeInput(scalar_t t, const vector_t& x) {
  if (optimalControllerPtr_ && learnedControllerPtr_) {
    return alpha_ * optimalControllerPtr_->computeInput(t, x) + (1 - alpha_) * learnedControllerPtr_->computeInput(t, x);
  } else {
    throw std::runtime_error(
        "MpcnetBehavioralController::computeInput cannot return input, since optimal and/or learned controller not set.");
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetBehavioralController::concatenate(const Base* otherController, int index, int length) {
  if (optimalControllerPtr_) {
    optimalControllerPtr_->concatenate(otherController, index, length);
  }
  if (learnedControllerPtr_) {
    learnedControllerPtr_->concatenate(otherController, index, length);
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
int MpcnetBehavioralController::size() const {
  if (optimalControllerPtr_ && learnedControllerPtr_) {
    return std::max(optimalControllerPtr_->size(), learnedControllerPtr_->size());
  } else if (optimalControllerPtr_) {
    return optimalControllerPtr_->size();
  } else if (learnedControllerPtr_) {
    return learnedControllerPtr_->size();
  } else {
    return 0;
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void MpcnetBehavioralController::clear() {
  if (optimalControllerPtr_) {
    optimalControllerPtr_->clear();
  }
  if (learnedControllerPtr_) {
    learnedControllerPtr_->clear();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
bool MpcnetBehavioralController::empty() const {
  if (optimalControllerPtr_ && learnedControllerPtr_) {
    return optimalControllerPtr_->empty() && learnedControllerPtr_->empty();
  } else if (optimalControllerPtr_) {
    return optimalControllerPtr_->empty();
  } else if (learnedControllerPtr_) {
    return learnedControllerPtr_->empty();
  } else {
    return true;
  }
}

}  // namespace ocs2
