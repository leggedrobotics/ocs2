#include "ReferenceModel.h"

ReferenceModel::ReferenceModel(Reference* ref): ref_(ref){}

void ReferenceModel::operator()(const state_type &x, state_type &dxdt, const double t)
	{
		double uref;
		ref_->getInput(t,uref);

		dxdt[0] = x[1];
		dxdt[1] = uref;		
		dxdt[2] = 0;
	}

