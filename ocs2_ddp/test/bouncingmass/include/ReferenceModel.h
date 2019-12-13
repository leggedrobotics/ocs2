#pragma once
#include <iostream>
#include "Reference.h"

typedef Eigen::Vector3d state_type;

class Reference;

class ReferenceModel
{
	public:

	ReferenceModel(Reference* ref);

	void operator()(const state_type &x, state_type &dxdt, const double t);

	private:
	Reference* ref_;
};
