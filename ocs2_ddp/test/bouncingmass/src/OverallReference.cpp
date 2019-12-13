#include <OverallReference.h>

using DIMENSIONS = ocs2::Dimensions<3, 1>;
	using scalar_t = typename DIMENSIONS::scalar_t;
	using scalar_array_t = typename DIMENSIONS::scalar_array_t;
	using state_vector_t = typename DIMENSIONS::state_vector_t;
	using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
	using input_vector_t = typename DIMENSIONS::input_vector_t;

OverallReference::OverallReference(std::vector<scalar_t> trajTimes, std::vector<state_vector_t> trajStates)
{

	References_.clear();
	References_.resize(trajTimes.size()-1);

	switchtimes_ = trajTimes;
	state_vector_t x0 = trajStates[0];

	for(int i = 0; i < trajTimes.size()-1; i++)
	{
		References_[i] = Reference(trajTimes[i], trajTimes[i+1], x0, trajStates[i+1]);
		x0 = trajStates[i+1];
		jumpMap(x0);
	}
}

int OverallReference::getIndex(scalar_t time)
{
	for(int i = 0; i<switchtimes_.size()-1; i++)
	{
		if(switchtimes_[i+1] >= time)
		{
			return i;
		}	
	}

	return -1;
}

void OverallReference::getInput(scalar_t time, input_vector_t &input)
{
	int idx = getIndex(time);
	if (idx>=0 && idx<References_.size())
	{
		References_[idx].getInput(time,input);
	}
	else
	{
		input.setZero();
	}
}

void OverallReference::getInput(scalar_t t0, scalar_t t1, scalar_t dt, std::vector<scalar_t>& time, std::vector<input_vector_t>& input)
{
	input.clear();

	input_vector_t inputT;
	for(scalar_t t = t0; t<=t1; t+=dt)
	{
		time.push_back(t);

		getInput(t,inputT);
		input.push_back(inputT);
	}

}

void OverallReference::getState(int idx, scalar_t time, state_vector_t &x)
{
	if (idx>=0 && idx<References_.size())
	{
		if(time<2)
		{
			References_[idx].getState(time,x);
		}
		else
		{
			getState(time,x);
		}
	}
	else
	{
		References_[0].getState(0,x);
	}
}

void OverallReference::getState(scalar_t time, state_vector_t &x)
{
	int idx = getIndex(time);
	if (idx>=0 && idx<References_.size())
	{
		References_[idx].getState(time,x);
	}
	else
	{
		References_[0].getState(0,x);
	}
}

void OverallReference::extendref(scalar_t delta)
{
	for(int i = 0; i<References_.size(); i++)
	{	
		Reference* pre;
		Reference* post;
		if (i>0)
		{ 
			pre = &References_[i-1];
		}
		else
		{
			pre = nullptr;
		}

		if (i <= References_.size() - 1)
		{ 
			post= &References_[i+1];
		}
		else
		{
			post = nullptr;
		}		
		References_[i].extendref(delta,pre,post);
	}


}

void OverallReference::display(int i)
{
	References_[i].display();
}

void OverallReference::jumpMap(state_vector_t &x)
{
	scalar_t e = 0.95;
	x[1] = x[1] - (1 + e) * x[1];
	x[2] = x[2] + 1;
}
