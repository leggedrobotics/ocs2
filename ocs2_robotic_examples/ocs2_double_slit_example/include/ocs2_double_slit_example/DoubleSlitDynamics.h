#pragma once

#include <ocs2_core/dynamics/ControlledSystemBase.h>

#include "ocs2_double_slit_example/definitions.h"

namespace ocs2 {
namespace double_slit {

class DoubleSlitDynamics final : public ControlledSystemBase<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> {
 public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef std::shared_ptr<DoubleSlitDynamics> Ptr;
	typedef std::shared_ptr<const DoubleSlitDynamics> ConstPtr;

	typedef ControlledSystemBase<double_slit::STATE_DIM_, double_slit::INPUT_DIM_> BASE;
	typedef typename BASE::scalar_t               scalar_t;
	typedef typename BASE::state_vector_t         state_vector_t;
	typedef typename BASE::input_vector_t         input_vector_t;

	/**
	 * Constructor
	 */
	DoubleSlitDynamics() = default;

	/**
	 * Destructor
	 */
	~DoubleSlitDynamics() = default;

	virtual DoubleSlitDynamics* clone() const override { return new DoubleSlitDynamics(*this); }

	void computeFlowMap(const scalar_t& time, const state_vector_t& state, const input_vector_t& input, state_vector_t& stateDerivative) override {
		stateDerivative = input;
	}

};

} // namespace double_slit
}  // namespace ocs2
