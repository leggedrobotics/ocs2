#ifndef OPTIONS_H_
#define OPTIONS_H_

namespace switched_model {

struct Options
{
	bool constrainedIntegration_;
	bool useCartesianContactForce_;
	double contactForceWeight_;
	double zDirectionPositionWeight_;
	double zDirectionVelocityWeight_;
	double swingLegLiftOff_;
	double mpcStrideLength_;

	size_t numPhasesInfullGaitCycle_;
	size_t defaultStartMode_;
	size_t defaultFinalMode_;
};

} // end of namespace switched_model

#endif /* end of include guard: OPTIONS_H_ */
