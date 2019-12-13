#include <vector>
#include <Eigen/Dense>

//typedef Eigen::Vector3d state_type;

class Observer
{
	public:

	Observer(std::vector<double>* tStore, std::vector<Eigen::Vector3d>* xStore)
	: tStore_(tStore), xStore_(xStore)
	{}


	void operator()(const Eigen::Vector3d &x, const double &t)
	{
		tStore_->push_back(t);
		xStore_->push_back(x);
	}


	private:
	std::vector<Eigen::Vector3d>* xStore_;
	std::vector<double>* tStore_;
};
