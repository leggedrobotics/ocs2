#include <vector>
#include <Eigen/Dense>

/*
 *  Observer used to obtain state trajectory while integrating the reference input
 * 	to extend the reference past event times
 */
class Observer
{
	public:

	/*
	 * Constructor
	 *
	 * @param [in] tStore: Pointer to Vector in which to store times
	 * @param [in] xStore: Pointer to Vector in which to store statetrajectory
	 */
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
