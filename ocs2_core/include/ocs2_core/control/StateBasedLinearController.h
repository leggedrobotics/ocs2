#pragma once

#include "ocs2_core/control/ControllerBase.h"
#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_core/control/LinearController.h"

#include <iomanip>

namespace ocs2{

template<size_t STATE_DIM, size_t INPUT_DIM>

class stateBasedLinearController final : public ControllerBase<STATE_DIM,INPUT_DIM>{

public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	using Base = ControllerBase<STATE_DIM, INPUT_DIM>;

	using dimensions_t = Dimensions<STATE_DIM, INPUT_DIM>;
	using size_array_t = typename dimensions_t::size_array_t;
	using scalar_t = typename dimensions_t::scalar_t;
	using scalar_array_t = typename dimensions_t::scalar_array_t;
	using float_array_t = typename Base::float_array_t;
	using state_vector_t = typename dimensions_t::state_vector_t;
	using input_vector_t = typename dimensions_t::input_vector_t;
	using input_vector_array_t = typename dimensions_t::input_vector_array_t;
	using input_state_matrix_t = typename dimensions_t::input_state_matrix_t;
	using input_state_matrix_array_t = typename dimensions_t::input_state_matrix_array_t;

	using logic_rules_t = HybridLogicRules;
	using controller_t =  ControllerBase<STATE_DIM,INPUT_DIM>;

	stateBasedLinearController() : CtrlPtr_(nullptr), CtrlEventTimes_(0),timeReference_(0),inputReference_(0){}

	void setController(controller_t* CtrlPtr)
	{
		CtrlPtr_ = CtrlPtr;
		CtrlEventTimes_.clear();
		CtrlPtr->getStateEvents(CtrlEventTimes_);

		if(CtrlEventTimes_.size()>0 && CtrlEventTimes_[0]!= 0)
		{
			CtrlEventTimes_.insert(CtrlEventTimes_.begin(),0);
		}
	}

	void setReference(scalar_array_t& timeReference,scalar_array_t& inputReference)
	{
		timeReference_ = timeReference;
		inputReference_ = inputReference;
	}

	~stateBasedLinearController() = default;

	input_vector_t computeInput(const scalar_t& t, const state_vector_t& x)
	{
		size_t CurrentMode = x.tail(1).value();
		size_t numEvents = CtrlEventTimes_.size();

		scalar_t tau_minus = (numEvents>CurrentMode) ? CtrlEventTimes_[CurrentMode] : CtrlEventTimes_.back();
		scalar_t tau = (numEvents>CurrentMode+1) ? CtrlEventTimes_[CurrentMode+1] : CtrlEventTimes_.back();
		scalar_t tau_plus = (numEvents>CurrentMode+2) ? CtrlEventTimes_[CurrentMode+2] : CtrlEventTimes_.back();


		bool pastAllEvents = tau_minus == tau && t>tau_minus;
		scalar_t eps = OCS2NumericTraits<scalar_t>::weakEpsilon();

		if((t>tau_minus && t<tau)  || pastAllEvents)
		{
			return CtrlPtr_->computeInput(t,x);
		}
		else if (t<tau_minus)
		{
			input_vector_t uRefT,uRefTau;
			// Compenstation of static reference input signal
			auto alpha = ocs2::LinearInterpolation<double>::interpolate(t,uRefT[0],&timeReference_,&inputReference_);
			alpha = ocs2::LinearInterpolation<double>::interpolate(tau_minus+2*eps,uRefTau[0],&timeReference_,&inputReference_);
			return CtrlPtr_->computeInput(tau_minus+2*eps,x) + uRefT - uRefTau;
		}
		else if (t>tau)
		{
			input_vector_t uRefT,uRefTau;
			// Compenstation of static reference input signal
			auto alpha = ocs2::LinearInterpolation<double>::interpolate(t,uRefT[0],&timeReference_,&inputReference_);
			alpha = ocs2::LinearInterpolation<double>::interpolate(tau-2*eps,uRefTau[0],&timeReference_,&inputReference_);
			return  CtrlPtr_->computeInput(tau-2*eps,x) + uRefT - uRefTau;
		}
	}

	void flatten(const scalar_array_t& timeArray, const std::vector<float_array_t*>& flatArray2) const override
	{
		CtrlPtr_->flatten(timeArray,flatArray2);
	}

	void unFlatten(const scalar_array_t& timeArray, const std::vector<float_array_t const*>& flatArray2) override
	{
		CtrlPtr_->unFlatten(timeArray,flatArray2);
	}

	void concatenate(const Base* nextController, int index, int length) override
	{
		CtrlPtr_->concatenate(nextController,index,length);
	}

	int size() const override
	{
		return CtrlPtr_->size();
	}

	ControllerType getType() const override
	{
		return CtrlPtr_->getType();
	}

	void clear() override
	{
		CtrlPtr_->clear();
	}

	void setZero() override
	{
		CtrlPtr_->setZero();
	}

	bool empty() const override
	{
		return CtrlPtr_->empty();
	}

	void display() const override
	{
		CtrlPtr_->display();
	}

	stateBasedLinearController* clone() const override
	{
		return new stateBasedLinearController(*this);
	}

private:
	controller_t* CtrlPtr_;
	scalar_array_t CtrlEventTimes_;

	std::vector<double> timeReference_;
	std::vector<double> inputReference_;
};



}
