/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>

#include <memory>
#include <fstream>
#include "ocs2_core/integration/Integrator.h"
#include "ocs2_core/integration/EventHandlerBase.h"
#include "ocs2_core/dynamics/LinearSystemDynamics.h"

#include <ocs2_core/misc/FindActiveIntervalIndex.h>

using namespace ocs2;


TEST(IntegrationTest, SecondOrderSystem_ODE45)
{
	bool resultsGood = true;

	Eigen::Matrix2d A;
	A << -2, -1, 1, 0;
	Eigen::Vector2d B;
	B << 1, 0;

	typedef LinearSystemDynamics<2,1> SecondOrderSystem;
	ControlledSystemBase<2,1>::Ptr sys = ControlledSystemBase<2,1>::Ptr(new SecondOrderSystem(A, B));

	SecondOrderSystem::scalar_array_t cntTimeStamp {0, 10};
	SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
	SecondOrderSystem::input_state_matrix_array_t k(2, SecondOrderSystem::input_state_matrix_t::Zero());

	SecondOrderSystem::controller_t controller;
	controller.time_ = cntTimeStamp;
	controller.uff_ = uff;
	controller.k_ = k;

	sys->setController(controller);

	ControlledSystemBase<2, 1>::Ptr sysClone1(sys->clone());
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

	Eigen::Matrix2d A;
	A << -2, -1, 1, 0;
	Eigen::Vector2d B;
	B << 1, 0;

	typedef LinearSystemDynamics<2,1> SecondOrderSystem;
	ControlledSystemBase<2,1>::Ptr sys = ControlledSystemBase<2,1>::Ptr(new SecondOrderSystem(A, B));

	SecondOrderSystem::scalar_array_t cntTimeStamp {0, 10};
	SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
	SecondOrderSystem::input_state_matrix_array_t k(2, SecondOrderSystem::input_state_matrix_t::Zero());

	SecondOrderSystem::controller_t controller;
	controller.time_ = cntTimeStamp;
	controller.uff_ = uff;
	controller.k_ = k;

	sys->setController(controller);

	ControlledSystemBase<2, 1>::Ptr sysClone1(sys->clone());
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

	Eigen::Matrix2d A;
	A << -2, -1, 1, 0;
	Eigen::Vector2d B;
	B << 1, 0;

	typedef LinearSystemDynamics<2,1> SecondOrderSystem;
	ControlledSystemBase<2,1>::Ptr sys = ControlledSystemBase<2,1>::Ptr(new SecondOrderSystem(A, B));

	SecondOrderSystem::scalar_array_t cntTimeStamp {0, 10};
	SecondOrderSystem::input_vector_array_t uff(2, SecondOrderSystem::input_vector_t::Ones());
	SecondOrderSystem::input_state_matrix_array_t k(2, SecondOrderSystem::input_state_matrix_t::Zero());

	SecondOrderSystem::controller_t controller;
	controller.time_ = cntTimeStamp;
	controller.uff_ = uff;
	controller.k_ = k;

	sys->setController(controller);

	ControlledSystemBase<2, 1>::Ptr sysClone1(sys->clone());
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

