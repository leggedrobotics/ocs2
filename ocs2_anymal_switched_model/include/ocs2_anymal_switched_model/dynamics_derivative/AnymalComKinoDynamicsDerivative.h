#pragma once

#include <ocs2_switched_model_interface/dynamics_derivative/ComKinoDynamicsDerivativeBase.h>

namespace anymal {

class AnymalComKinoDynamicsDerivative final : public switched_model::ComKinoDynamicsDerivativeBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::ComKinoDynamicsDerivativeBase<12> Base;

	AnymalComKinoDynamicsDerivative(const switched_model::Model_Settings& options = switched_model::Model_Settings());

	AnymalComKinoDynamicsDerivative(const AnymalComKinoDynamicsDerivative& rhs);

	~AnymalComKinoDynamicsDerivative() {}

	AnymalComKinoDynamicsDerivative* clone() const override;
};

} //end of namespace anymal
