#include <Reference.h>
#include <algorithm>

Reference::Reference(double t0, double t1, Eigen::Vector3d p0, Eigen::Vector3d p1)
{
	Create5thOrdPol(t0,t1,p0,p1);
	polV_ = polyder(polX_);
	polU_ = polyder(polV_);

	t0_ = t0;
	t1_ = t1;
}

void Reference::getInput(double time,double &input)
{
	input = 0;
	for(int i = 0; i<polU_.size(); i++)
	{

		input += polU_[i] * std::pow(time,i);

	}
}

void Reference:: getState(double time,Eigen::Vector3d &x)
{
	if (time <= t1_ && time>= t0_)
	{
		x[0] =0;
		x[1] =0;
		x[2] =0;

		for(int i = 0; i<polU_.size(); i++)
		{
			x[0] += polX_[i] * std::pow(time,i);
			x[1] += polV_[i] * std::pow(time,i);
		}
	}
	else
	{
		interpolate_ext(time,x);
	}
}

void Reference::extendref(double delta, Reference* refPre, Reference* refPost)
{
	delta_ = delta;
	boost::numeric::odeint::runge_kutta_dopri5
	<Eigen::Vector3d,double,Eigen::Vector3d,double,boost::numeric::odeint::vector_space_algebra> stepper;

	// pre-part of extension
	if(refPre != nullptr)
	{	
	ReferenceModel preModel(refPre);

	Eigen::Vector3d x0;
	getState(t0_,x0); 	
	double t0 = t0_;
	double t1 = t0-delta;
	double dt = -1e-3;

	boost::numeric::odeint::integrate_adaptive(stepper,preModel,x0, t0, t1,dt,Observer(&tPre_,&xPre_));
	std::reverse(std::begin(tPre_),std::end(tPre_));
	std::reverse(std::begin(xPre_),std::end(xPre_));
	}

	//post-part of extension
	if(refPost != nullptr)
	{
	ReferenceModel postModel(refPost);

	Eigen::Vector3d x0;
	getState(t1_,x0); 	
	double t0 = t1_;
	double t1 = t0+delta;
	double dt = 1e-3;
	boost::numeric::odeint::integrate_adaptive(stepper,postModel,x0, t0, t1,dt,Observer(&tPost_,&xPost_));
	}


}

void Reference::Create5thOrdPol(double t0, double t1, Eigen::Vector3d p0, Eigen::Vector3d p1)
{

Eigen::Matrix<double, 6 , 6> A;
Eigen::Matrix<double, 6 , 6> Ainv;

A << 1, 	t0, 	std::pow(t0,2), 	std::pow(t0,3), 	std::pow(t0,4), 	std::pow(t0,5),
     0, 	1, 		2*t0, 				3*std::pow(t0,2), 	4*std::pow(t0,3), 	5*std::pow(t0,4),
     0,		0,		2,					6*t0,				12*std::pow(t0,2),	20*std::pow(t0,3),
     1,		t1,		std::pow(t1,2),		std::pow(t1,3),		std::pow(t1,4),		std::pow(t1,5),
     0,		1,		2*t1,				3*std::pow(t1,2),	4*std::pow(t1,3),	5*std::pow(t1,4),
     0,		0,		2,					6*t1,				12*std::pow(t1,2),	20*std::pow(t1,3);

Ainv = A.inverse();

Eigen::Matrix<double, 6, 1> x;
x<<p0,p1;
polX_ = Ainv*x;
}

void Reference::interpolate_ext(double time, Eigen::Vector3d &x)
{
	std::vector<double>* tVec;
	std::vector<Eigen::Vector3d>* xVec;
	if (time<t0_)
	{	tVec = &tPre_;
		xVec = &xPre_;
		x     = xPre_.front();	}
	else
	{	tVec = &tPost_;
		xVec = &xPost_;
		x     = xPost_.back();	}

	int idx;
	for (int i = 0; i<tVec->size()-1; i++)
	{
		if(time > tVec->at(i) && time<tVec->at(i+1))
		{
			idx = i;
			double fac = (time-tVec->at(idx))/(tVec->at(idx+1)-tVec->at(idx));
			x = fac*xVec->at(idx) + (1-fac)*xVec->at(idx+1);
			return;
		}		
	}
	
	
	
	
}

void Reference::display()
{
	std::cout<<"#########################"<<std::endl;
	std::cout<<"#Pre-Extended-Trajectory#"<<std::endl;
	std::cout<<"#########################"<<std::endl;	
	for(int i =0; i<tPre_.size(); i++)
	{
		std::cout<<tPre_[i]<<";"<<xPre_[i][0]<<";"<<xPre_[i][1]<<std::endl;
	}

	std::cout<<"#########################"<<std::endl;
	std::cout<<"####Normal-Trajectory####"<<std::endl;
	std::cout<<"#########################"<<std::endl;	

	float dt = 0.01;
	for(int i = 0; i<(t1_-t0_)/dt; i++)
	{
		double t = t0_ + dt*i;
		Eigen::Vector3d x;
		double u;
		getInput(t,u);
		getState(t,x);
		
		std::cout<<t<<";"<<x[0]<<";"<<x[1]<<std::endl;
	}

	std::cout<<"##########################"<<std::endl;
	std::cout<<"#Post-Extended-Trajectory#"<<std::endl;
	std::cout<<"##########################"<<std::endl;	

	for(int i =0; i<tPost_.size(); i++)
	{
		std::cout<<tPost_[i]<<";"<<xPost_[i][0]<<";"<<xPost_[i][1]<<std::endl;
	}
}


Eigen::Matrix<double, 6 , 1> Reference::polyder(Eigen::Matrix<double, 6 , 1> pol)
{
	Eigen::Matrix<double, 6 , 1> polOld = pol;

	for(int i = 0; i< pol.size(); i++)
	{
		if (i<pol.size()-1)
		{
			pol[i] = (i+1)*polOld[i+1];
		}
		else
		{
			pol[i] = 0;
		}

	}

	return pol;
}
