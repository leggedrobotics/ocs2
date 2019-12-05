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

	stateBasedLinearController() = default;

	stateBasedLinearController(controller_t* CtrlPtr, scalar_array_t eventTimes): CtrlPtr_(CtrlPtr)
	{
		CurrentMode_ = 0;

		CtrlEventTimes_.clear();
		CtrlPtr->getStateEvents(CtrlEventTimes_);

		if(eventTimes.size()>0 && eventTimes[0]!= 0)
		{
			CtrlEventTimes_.insert(CtrlEventTimes_.begin(),0);
		}
	}

	~stateBasedLinearController() = default;

	input_vector_t computeInput(const scalar_t& t, const state_vector_t& x)
	{
		if(CtrlEventTimes_.size()==0)
		{
			return CtrlPtr_->computeInput(t,x);
		}

		CurrentMode_ = x[2];
		auto alpha = CurrentMode_;
		scalar_t tau_minus, tau, tau_plus;

		tau_minus = CtrlEventTimes_[alpha];

		if (CtrlEventTimes_.size()>alpha+1)
		{
			tau = CtrlEventTimes_[alpha+1];
		}
		else
		{
			tau = CtrlEventTimes_.back();
		}

		if (CtrlEventTimes_.size()>alpha+2)
		{
			tau_plus = CtrlEventTimes_[alpha+2];
		}
		else
		{
			tau_plus = CtrlEventTimes_.back();
		}

		bool tMuchSmaller = tau_minus -t > 1e-1;
		bool tMuchBigger =  t - tau > 1e-1;
		bool tMismatch = tMuchSmaller || tMuchBigger;

		bool pastAllEvents = tau_minus == tau && t>tau_minus;

		if((t>tau_minus && t<tau)  ||pastAllEvents)
		{
			return CtrlPtr_->computeInput(t,x);
		}
		else if (t<tau_minus)
		{
			return CtrlPtr_->computeInput(tau_minus+1e-9,x);
		}
		else if (t>tau)
		{
			return CtrlPtr_->computeInput(tau-1e-9,x);
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
	//size_array_t CtrlSubSequence_;

	std::shared_ptr<logic_rules_t> logicRulesPtr_;
	size_t CurrentMode_;
};



}
