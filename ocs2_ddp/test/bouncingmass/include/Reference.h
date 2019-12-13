#pragma once

#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>
#include <algorithm>
#include <boost/numeric/odeint.hpp>

#include "ReferenceModel.h"
#include "Observer.h"

class Reference
{

public:

	Reference() = default;
	Reference(double t0, double t1, Eigen::Vector3d p0, Eigen::Vector3d p1);

	void getInput(double time,double &input);
	void getState(double time,Eigen::Vector3d &x);

	void extendref(double delta, Reference* ref_pre, Reference* ref_post);

	void display();

	double t0_;
	double t1_;
	double delta_ = 0;


private:

	void Create5thOrdPol(double t0, double t1, Eigen::Vector3d p0, Eigen::Vector3d p1);
	Eigen::Matrix<double, 6 , 1> polyder(Eigen::Matrix<double, 6 , 1> pol);
	void interpolate_ext(double time, Eigen::Vector3d &x);

	Eigen::Matrix<double, 6, 1> polU_;
	Eigen::Matrix<double, 6, 1> polX_;
	Eigen::Matrix<double, 6, 1> polV_;

	std::vector<Eigen::Vector3d> xPre_;
	std::vector<double> tPre_;
	std::vector<Eigen::Vector3d> xPost_;
	std::vector<double> tPost_;

};

