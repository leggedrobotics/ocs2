#pragma once

#include "Reference.h"
/*
 * 	This class constructs, contains and provides the reference for the
 * 	BouncingMass SLQ test. The reference is based on the reference presented
 * 	in [On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints
 * by M. Rijnen]
 */
class OverallReference
{
	using dynamic_vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
	using dynamic_vector_array_t = std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t>>;

public:
	/*
	 * Constructor
	 */
	OverallReference() = default;

	/*
	 * Constructor
	 *
	 * @param [in] trajTimes: list of times at which the reference is defined
	 * @param [in] trajState: list of waypoints at which the reference is defined
	 */
	OverallReference(std::vector<double> trajTimes, std::vector<Eigen::Vector3d> trajState);

	/*
	 * Calculate the input at a certain time
	 *
	 * @param [in] time: time moment at which the input is calculated
	 * @param [out] input: input corresponding to time
	 */
	void getInput(double time, double &input);

	/*
	 * Calculate the input at a range of times
	 *
	 * @param [in] t0: First time element of output
	 * @param [in] t1: Last time element of output
	 * @param [in] dt: Time step of output vectors
	 * @param [out] time: Time vector
	 * @param [out] input: Input vector corresponding to time vector
	 */
	void getInput(const double t0, const double t1, double dt, std::vector<double>& time, std::vector<double>& input);

	/*
	 * Calculate the reference state at a certain time
	 *
	 * @param [in] time: time moment at which the input is calculated
	 * @param [out] state: state corresponding to time
	 */
	void getState(double time, Eigen::Vector3d &x);

	/*
	 * Calculate the reference state at a certain time and mode
	 *
	 * @param [in] idx: mode at which the input is calculated
	 * @param [in] time: time moment at which the input is calculated
	 * @param [out] state: state corresponding to time and mode
	 */
	void getState(int idx, double time, Eigen::Vector3d &x);

	/*
	 * Extend the reference past the event times, by integrating the input signal
	 * without applying the jump map
	 *
	 * @param [in] delta: length of the extensions
	 */
	void extendref(double delta);

	/*
	 * Display the reference
	 *
	 * @param [in] i: choice of which section of the reference is displayed
	 */
	void display(int i);

private:

	/*
	 * Find the index of the currently active reference
	 *
	 * @param [in] time: time moment at which the index is requested
	 *
	 * @return currently active index
	 */
	int getIndex(double time);

	/*
	 * Jump map of the system
	 *
	 * @param [in] x: State before event
	 *
	 * @return currently active index
	 */
	void jumpMap(Eigen::Vector3d &x);

	std::vector<Reference> 	References_;
	std::vector<double>	switchtimes_;
};

