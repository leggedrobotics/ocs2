#include <OverallReference.h>

OverallReference::OverallReference(std::vector<double> trajTimes, std::vector<Eigen::Vector3d> trajStates)
{

	References_.clear();
	References_.resize(trajTimes.size()-1);

	switchtimes_ = trajTimes;
	Eigen::Vector3d x0 = trajStates[0];

	for(int i = 0; i < trajTimes.size()-1; i++)
	{
		References_[i] = Reference(trajTimes[i], trajTimes[i+1], x0, trajStates[i+1]);
		x0 = trajStates[i+1];
		jumpMap(x0);
	}
}

int OverallReference::getIndex(double time)
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

void OverallReference::getInput(double time, double &input)
{
	int idx = getIndex(time);
	if (idx>=0 && idx<References_.size())
	{
		References_[idx].getInput(time,input);
	}
	else
	{
		input  = 0;
	}
}

void OverallReference::getInput(double t0, double t1, double dt, std::vector<double>& time, std::vector<double>& input)
{
	input.clear();

	double inputT;
	for(double t = t0; t<=t1; t+=dt)
	{
		time.push_back(t);

		getInput(t,inputT);
		input.push_back(inputT);
	}

}

void OverallReference::getState(int idx, double time, Eigen::Vector3d &x)
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

void OverallReference::getState(double time, Eigen::Vector3d &x)
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

void OverallReference::extendref(double delta)
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

void OverallReference::jumpMap(Eigen::Vector3d &x)
{
	double e = 0.95;
	x[1] = x[1] - (1 + e) * x[1];
	x[2] = x[2] + 1;
}
