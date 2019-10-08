#pragma once

#include <ocs2_switched_model_interface/dynamics/ComKinoDynamicsBase.h>

namespace anymal {

class AnymalComKinoDynamics : public switched_model::ComKinoDynamicsBase<12>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef switched_model::ComKinoDynamicsBase<12> Base;

	AnymalComKinoDynamics(const switched_model::Model_Settings& options = switched_model::Model_Settings());

	AnymalComKinoDynamics(const AnymalComKinoDynamics& rhs);

	~AnymalComKinoDynamics() {}

	AnymalComKinoDynamics* clone() const  override;
};

} //end of namespace anymal

