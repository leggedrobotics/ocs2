/*
 * ZMPConstraint.h
 *
 *  Created on: Sep 12, 2016
 *      Author: farbod
 */

#ifndef HYQ_ZMPCONSTRAINT_H_
#define HYQ_ZMPCONSTRAINT_H_

#include <array>
#include <Eigen/Dense>

namespace hyq {

class ZMPConstraint
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	ZMPConstraint() {}
	~ZMPConstraint() {}

	static double ConstraintValue(const std::array<bool,4>& stanceLegs, const std::array<Eigen::Vector3d,4>& com_com2StanceFeet)  {
		Eigen::Vector3d com_com2StanceFeetCenter;
		CalculateFeetCenter(stanceLegs, com_com2StanceFeet, com_com2StanceFeetCenter);
		return com_com2StanceFeetCenter.head<2>().norm();
	}


	static Eigen::Vector3d ConstraintDerivative(const std::array<bool,4>& stanceLegs, const std::array<Eigen::Vector3d,4>& com_com2StanceFeet) {
		Eigen::Vector3d com_com2StanceFeetCenter;
		CalculateFeetCenter(stanceLegs, com_com2StanceFeet, com_com2StanceFeetCenter);
		com_com2StanceFeetCenter(2) = 0.0;
		return com_com2StanceFeetCenter / com_com2StanceFeetCenter.norm();
	}

private:
	static void CalculateFeetCenter(const std::array<bool,4>& stanceLegs, const std::array<Eigen::Vector3d,4>& com_com2StanceFeet,
			Eigen::Vector3d& com_com2StanceFeetCenter)  {
		com_com2StanceFeetCenter.setZero();
		for (size_t j=0; j<4; j++)  if (stanceLegs[j]==true)  com_com2StanceFeetCenter += com_com2StanceFeet[j];
		com_com2StanceFeetCenter /= (stanceLegs[0]+stanceLegs[1]+stanceLegs[2]+stanceLegs[3]);

//		for (size_t j=0; j<4; j++)
//			std::cout << "foot " << j << " is active " << stanceLegs[j] << " with position " << com_com2StanceFeet[j].transpose() << std::endl;
//		std::cout << "Feet center: " << com_com2StanceFeetCenter.head<2>().transpose() << std::endl;
	}

};

}  // end of hyq namespace


#endif /* HYQ_ZMPCONSTRAINT_H_ */
