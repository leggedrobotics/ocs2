#pragma once

#include <ocs2_comm_interfaces/ocs2_interfaces/MRT_BASE.h>
#include <ocs2_core/control/NetworkController.h>

namespace ocs2 {

template <size_t STATE_DIM, size_t INPUT_DIM>
class MRT_Network_Interface final : public MRT_BASE<STATE_DIM, INPUT_DIM> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using Base = MRT_BASE<STATE_DIM, INPUT_DIM>;
  using dim_t = Dimensions<STATE_DIM, INPUT_DIM>;
  using size_array_t = typename dim_t::size_array_t;
  using scalar_t = typename dim_t::scalar_t;
  using scalar_array_t = typename dim_t::scalar_array_t;
  using state_vector_t = typename dim_t::state_vector_t;
  using state_vector_array_t = typename dim_t::state_vector_array_t;
  using input_vector_t = typename dim_t::input_vector_t;
  using input_vector_array_t = typename dim_t::input_vector_array_t;

  explicit MRT_Network_Interface(const std::string& pathToPolicy) {
    this->currentPolicy_->mpcController_.reset(new NetworkController<STATE_DIM, INPUT_DIM>(pathToPolicy));
    this->currentPolicy_->mpcTimeTrajectory_ =
        scalar_array_t{std::numeric_limits<scalar_t>::lowest(), std::numeric_limits<scalar_t>::max()};
    this->currentPolicy_->mpcStateTrajectory_.resize(2, state_vector_t::Zero());

    this->mpcLinInterpolateState_.setData(&this->currentPolicy_->mpcTimeTrajectory_, &this->currentPolicy_->mpcStateTrajectory_);

    this->policyReceivedEver_ = true;
    this->policyUpdated_ = true;
  }

  virtual ~MRT_Network_Interface() = default;

  void resetMpcNode(const CostDesiredTrajectories<scalar_t>& initCostDesiredTrajectories) override {
    throw std::runtime_error("MRT_Network_Interface: Cannot reset MPC. There is no underlying MPC running.");
  }

  void setCurrentObservation(const SystemObservation<STATE_DIM, INPUT_DIM>& observation) override {}

  bool updatePolicyImpl() override { return true; }
};

}  // namespace ocs2
