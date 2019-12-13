#pragma once

#include "Reference.h"

class OverallReference
{
	using dynamic_vector_t = Eigen::Matrix<double, Eigen::Dynamic, 1>;
	using dynamic_vector_array_t = std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t>>;

	public:
	OverallReference() = default;
	OverallReference(std::vector<double> traj_times, std::vector<Eigen::Vector3d> traj_state);

	void getInput(double time, double &input);
	void getInput(const double t0, const double t1, double dt, std::vector<double>& time, std::vector<double>& input);

	void getState(double time, Eigen::Vector3d &x);
	void getState(int idx, double time, Eigen::Vector3d &x);

	void extendref(double delta);

	void display(int i);

	private:
	int getIndex(double time);
	void jumpMap(Eigen::Vector3d &x);

	std::vector<Reference> 	References_;
	std::vector<double>	switchtimes_;
};

