#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <algorithm>
#include <boost/numeric/odeint.hpp>

#include "ReferenceModel.h"
#include "Observer.h"
/*
 * 	This class constructs, contains a section of the reference, which is contained in
 * 	OverallReference and used in BouncingMass SLQ test.
 * 	The reference is based on the reference presented
 * 	in [On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints
 * by M. Rijnen]
 */
class Reference
{

public:

	/*
	 * Constructor
	 */
	Reference() = default;

	/*
	 * Constructor
	 *
	 * @param [in] t0: Time at which the first waypoint of the reference is defined
	 * @param [in] t1: Time at which the second waypoint of the reference is defined
	 * @param [in] p0: First waypoint of the reference [position,velocity,acceleration]
	 * @param [in] p1: Second waypoint of the reference
	 */
	Reference(double t0, double t1, Eigen::Vector3d p0, Eigen::Vector3d p1);

	/*
	 * Obtain reference input at the current time
	 *
	 * @param [in] time: current time
	 * @param [out] input: current reference input
	 */
	void getInput(const double time,double &input);

	/*
	 * Obtain reference state at current time
	 *
	 * @param [in] time: current time
	 * @param [out] x: current reference state
	 */
	void getState(const double time,Eigen::Vector3d &x);

	/*
	 * Extend the reference by integrating the input signal of the next
	 * and previous sections
	 *
	 * @param [in] delta: extension time
	 * @param [in] refPre: pointer to previous section of the reference
	 * @param [in] refPost: pointer to the next section of the reference
	 */
	void extendref(double delta, Reference* refPre, Reference* refPost);

	/*
	 * Display the reference
	 */
	void display();

	double t0_;
	double t1_;
	double delta_ = 0;
private:

	/*
	 * 	Find the polynomial coefficients corresponding to the posed constraints
	 * 	on position, velocity and accleration at the posed times
	 *
	 * 	@param [in]	t0:	Time moment of first waypoint
	 * 	@param [in] t1: time moment of second waypoint
	 * 	@param [in] p0: first waypoint
	 * 	@param [in] p1: second waypoint
	 */
	void Create5thOrdPol(double t0, double t1, Eigen::Vector3d p0, Eigen::Vector3d p1);

	/*
	 * 	Find the derivative of the polynomial described by a
	 * 	set of coefficients
	 *
	 * 	@param [in]	pol: coefficients describing the fifth order polynomial
	 */
	Eigen::Matrix<double, 6 , 1> polyder(Eigen::Matrix<double, 6 , 1> pol);

	/*
	 * 	Interpolate the extended reference signals to find the reference
	 * 	corresponding to the current time
	 *
	 * 	@param [in]	time:	current time
	 * 	@param [out] x: current state
	 */
	void interpolate_ext(double time, Eigen::Vector3d &x);

	Eigen::Matrix<double, 6, 1> polU_;
	Eigen::Matrix<double, 6, 1> polX_;
	Eigen::Matrix<double, 6, 1> polV_;

	std::vector<Eigen::Vector3d> xPre_;
	std::vector<double> tPre_;
	std::vector<Eigen::Vector3d> xPost_;
	std::vector<double> tPost_;

};

