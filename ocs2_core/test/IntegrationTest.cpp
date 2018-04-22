/*
 * A unit test
 *
 *  Created on: Sept 25, 2017
 *      Author: farbodf
 */

#include <memory>
#include <fstream>

#include "ocs2_core/integration/Integrator.h"
#include "ocs2_core/integration/EventHandlerBase.h"
#include "ocs2_core/dynamics/ControlledSystemBase.h"
#include <ocs2_core/misc/FindActiveIntervalIndex.h>

#include <gtest/gtest.h>

using namespace ocs2;

class SecondOrderSystem : public ControlledSystemBase<2,1>
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	SecondOrderSystem() {}
	~SecondOrderSystem() {}

	void computeDerivative(
			const double& t,
			const Eigen::Matrix<double,2,1>& x,
			const Eigen::Matrix<double,1,1>& u,
			Eigen::Matrix<double,2,1>& dxdt) {

		Eigen::Matrix2d A;
		A << -2, -1, 1, 0;

		Eigen::Vector2d B;
		B << 1, 0;

		dxdt = A*x + B*u;
	}

	SecondOrderSystem* clone() const override {
		return new SecondOrderSystem(*this);
	}

private:

};


TEST(IntegrationTest, SecondOrderSystem_ODE45)
{
	bool resultsGood = true;

	std::shared_ptr<SecondOrderSystem> sys = std::make_shared<SecondOrderSystem>();

	SecondOrderSystem::scalar_array_t cntTimeStamp {0, 10};
	SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
	SecondOrderSystem::control_feedback_array_t k(2, SecondOrderSystem::control_feedback_t::Zero());

	SecondOrderSystem::controller_t controller;
	controller.time_ = cntTimeStamp;
	controller.uff_ = uff;
	controller.k_ = k;

	sys->setController(controller);

	std::shared_ptr<ControlledSystemBase<2, 1> > sysClone1(sys->clone());
	resultsGood = sysClone1.unique();

	ODE45<2> odeAdaptive(sys); // integrate adaptive
	ODE45<2> odeConst(sys); //integrate const
	ODE45<2> odeTime(sys); // integrate with a given time trajectory

	std::vector<double> timeTrajectory1, timeTrajectory2;
	std::vector<Eigen::Matrix<double,2,1>, Eigen::aligned_allocator<Eigen::Matrix<double,2,1>> > stateTrajectory1, stateTrajectory2, stateTrajectory3;

	Eigen::Matrix<double,2,1> x0, x02;
	x0.setZero();
	x02.setZero();

	odeAdaptive.integrate(x0, 0.0, 10.0, stateTrajectory1, timeTrajectory1); // integrate adaptive

	double dt = 0.05;
	odeConst.integrate(x02, 0.0, 10.0, dt, stateTrajectory2, timeTrajectory2); //integrate const

	odeTime.integrate(x0, timeTrajectory1.begin(), timeTrajectory1.end(), stateTrajectory3); // integrate with given time trajectory


	if(fabs(timeTrajectory1.back() - 10.0) > 1e-6)
		resultsGood = false;

	if(fabs(stateTrajectory1.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	if(fabs(timeTrajectory2.back() - 10.0) > 1e-6)
		resultsGood = false;

	if(fabs(stateTrajectory2.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	if(fabs(stateTrajectory3.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	ASSERT_TRUE(resultsGood);
}


TEST(IntegrationTest, SecondOrderSystem_AdamsBashfort)
{
	bool resultsGood = true;

	std::shared_ptr<SecondOrderSystem> sys = std::make_shared<SecondOrderSystem>();

	SecondOrderSystem::scalar_array_t cntTimeStamp {0, 10};
	SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
	SecondOrderSystem::control_feedback_array_t k(2, SecondOrderSystem::control_feedback_t::Zero());

	SecondOrderSystem::controller_t controller;
	controller.time_ = cntTimeStamp;
	controller.uff_ = uff;
	controller.k_ = k;

	sys->setController(controller);

	std::shared_ptr<ControlledSystemBase<2, 1> > sysClone1(sys->clone());
	resultsGood = sysClone1.unique();

	const size_t order = 5;
	IntegratorAdamsBashforth<2, order> odeAdaptive(sys); // integrate adaptive
	IntegratorAdamsBashforth<2, order> odeConst(sys); //integrate const
	IntegratorAdamsBashforth<2, order> odeTime(sys); // integrate with a given time trajectory

	std::vector<double> timeTrajectory1, timeTrajectory2;
	std::vector<Eigen::Matrix<double,2,1>, Eigen::aligned_allocator<Eigen::Matrix<double,2,1>> > stateTrajectory1, stateTrajectory2, stateTrajectory3;

	Eigen::Matrix<double,2,1> x0, x02;
	x0.setZero();
	x02.setZero();

	odeAdaptive.integrate(x0, 0.0, 10.0, stateTrajectory1, timeTrajectory1); // integrate adaptive

	double dt = 0.05;
	odeConst.integrate(x02, 0.0, 10.0, dt, stateTrajectory2, timeTrajectory2); //integrate const

	odeTime.integrate(x0, timeTrajectory1.begin(), timeTrajectory1.end(), stateTrajectory3); // integrate with given time trajectory


	if(fabs(timeTrajectory1.back() - 10.0) > 1e-6)
		resultsGood = false;

	if(fabs(stateTrajectory1.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	if(fabs(timeTrajectory2.back() - 10.0) > 1e-6)
		resultsGood = false;

	if(fabs(stateTrajectory2.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	if(fabs(stateTrajectory3.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	ASSERT_TRUE(resultsGood);
}


#if (BOOST_VERSION / 100000 == 1 && BOOST_VERSION / 100 % 1000 > 55)

TEST(IntegrationTest, SecondOrderSystem_AdamsBashfortMoulton)
{
	bool resultsGood = true;

	std::shared_ptr<SecondOrderSystem> sys = std::make_shared<SecondOrderSystem>();

	SecondOrderSystem::scalar_array_t cntTimeStamp {0, 10};
	SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
	SecondOrderSystem::control_feedback_array_t k(2, SecondOrderSystem::control_feedback_t::Zero());

	SecondOrderSystem::controller_t controller;
	controller.time_ = cntTimeStamp;
	controller.uff_ = uff;
	controller.k_ = k;

	sys->setController(controller);

	std::shared_ptr<ControlledSystemBase<2, 1> > sysClone1(sys->clone());
	resultsGood =  sysClone1.unique();

	const size_t order = 5;
	IntegratorAdamsBashforthMoulton<2, order> odeAdaptive(sys); // integrate adaptive
	IntegratorAdamsBashforthMoulton<2, order> odeConst(sys); //integrate const
	IntegratorAdamsBashforthMoulton<2, order> odeTime(sys); // integrate with a given time trajectory

	std::vector<double> timeTrajectory1, timeTrajectory2;
	std::vector<Eigen::Matrix<double,2,1>, Eigen::aligned_allocator<Eigen::Matrix<double,2,1>> > stateTrajectory1, stateTrajectory2, stateTrajectory3;

	Eigen::Matrix<double,2,1> x0, x02;
	x0.setZero();
	x02.setZero();

	odeAdaptive.integrate(x0, 0.0, 10.0, stateTrajectory1, timeTrajectory1); // integrate adaptive

	double dt = 0.05;
	odeConst.integrate(x02, 0.0, 10.0, dt, stateTrajectory2, timeTrajectory2); //integrate const

	odeTime.integrate(x0, timeTrajectory1.begin(), timeTrajectory1.end(), stateTrajectory3); // integrate with given time trajectory


	if(fabs(timeTrajectory1.back() - 10.0) > 1e-6)
		resultsGood = false;

	if(fabs(stateTrajectory1.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	if(fabs(timeTrajectory2.back() - 10.0) > 1e-6)
		resultsGood = false;

	if(fabs(stateTrajectory2.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	if(fabs(stateTrajectory3.back()(1) - 1.0) > 1e-3)
		resultsGood = false;

	ASSERT_TRUE(resultsGood);
}

#endif

int main(int argc, char** argv)
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}

