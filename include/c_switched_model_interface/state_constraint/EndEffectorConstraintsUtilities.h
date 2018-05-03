/*
 * EndEffectorConstraintsUtilities.h
 *
 *  Created on: Jul 21, 2016
 *      Author: farbod
 */

#ifndef ENDEFFECTORCONSTRAINTSUTILITIES_H_
#define ENDEFFECTORCONSTRAINTSUTILITIES_H_

#include <memory>
#include <string>
#include <iostream>
#include <Eigen/Dense>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

#include "c_switched_model_interface/state_constraint/EndEffectorConstraintBase.h"
#include "c_switched_model_interface/state_constraint/PotentialFieldConstraint.h"

namespace switched_model {


inline void loadGaps(const std::string& filename, std::vector<EndEffectorConstraintBase::ConstPtr>& gapIndicatorPtrs, bool verbose = true)
{
	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	gapIndicatorPtrs.clear();

	std::cerr <<"EndEffectorConstraints Options: " << std::endl;
	std::cerr <<"=====================================================================" << std::endl;

	size_t cnt = 0;
	while (true)  {

		std::string gapName = "endeffector_constraint_" + std::to_string(cnt);

		// check that the topic exist
		boost::property_tree::ptree::const_assoc_iterator it = pt.find(gapName);
		if(it == pt.not_found())  break;

		std::string type;
		Eigen::Vector2d xLimits, yLimits;

		try	{
			type = pt.get<std::string>(gapName + ".type");
			if (verbose)  std::cerr << " #### EndEffectorConstraint " << cnt << ": option 'type'        " << type << std::endl;
		}
		catch (const std::exception& e){
			type = "GAP";
			yLimits << EndEffectorConstraintBase::minusInf_, EndEffectorConstraintBase::plusInf_;
			if (verbose)  std::cerr << " #### EndEffectorConstraint " << cnt << ": option 'type'        " << type << "\t(default)" << std::endl;
		}

		try	{
			xLimits(0) = pt.get<double>(gapName + ".xMin");
			xLimits(1) = pt.get<double>(gapName + ".xMax");
			if (verbose)  std::cerr << " #### EndEffectorConstraint " << cnt << ": option 'xLimits'     [" << xLimits(0) << ", " << xLimits(1) << "]" << std::endl;
		}
		catch (const std::exception& e){
			xLimits << EndEffectorConstraintBase::minusInf_, EndEffectorConstraintBase::plusInf_;
			if (verbose)  std::cerr << " #### EndEffectorConstraint " << cnt << ": option 'xLimits'     [-inf, +inf] \t(default)" << std::endl;
		}

		try	{
			yLimits(0) = pt.get<double>(gapName + ".yMin");
			yLimits(1) = pt.get<double>(gapName + ".yMax");
			if (verbose)  std::cerr << " #### EndEffectorConstraint " << cnt << ": option 'yLimits'     [" << yLimits(0) << ", " << yLimits(1) << "]" << std::endl;
		}
		catch (const std::exception& e){
			yLimits << EndEffectorConstraintBase::minusInf_, EndEffectorConstraintBase::plusInf_;
			if (verbose)  std::cerr << " #### EndEffectorConstraint " << cnt << ": option 'yLimits'     [-inf, +inf] \t(default)" << std::endl;
		}

		bool isRepeller = (type.compare("STONE")==0) ? false : true;
		EndEffectorConstraintBase::Ptr gapIndicatorPtr;
		if (isRepeller==true)
			gapIndicatorPtr = std::shared_ptr<EllipticalConstraint>( new EllipticalConstraint(isRepeller, xLimits, yLimits) );
		else
			gapIndicatorPtr = std::shared_ptr<PotentialFieldConstraint>( new PotentialFieldConstraint(isRepeller, xLimits, yLimits) );

		gapIndicatorPtrs.push_back(gapIndicatorPtr);

		cnt++;
		std::cerr << std::endl;

	}  // end of while loop

	std::cerr << std::endl;
}


inline void publishGapMessages(const std::vector<EndEffectorConstraintBase::Ptr>& gapIndicatorPtrs, ros::Publisher& gapPublisher)  {

	// wait until rviz subscribes
	while (gapPublisher.getNumSubscribers() < 1)
		sleep(0.1);

	// standard defintion of gap massage
	visualization_msgs::Marker gap;
	gap.header.frame_id = "world";
	gap.header.stamp = ros::Time();
	gap.ns = "gap";
	gap.type = visualization_msgs::Marker::SPHERE;

	gap.action = visualization_msgs::Marker::ADD;
	gap.lifetime = ros::Duration();
	gap.pose.position.x = 0.0;
	gap.pose.position.y = 0.0;
	gap.pose.position.z = 0.0;
	gap.pose.orientation.x = 0.0;
	gap.pose.orientation.y = 0.0;
	gap.pose.orientation.z = 0.0;
	gap.pose.orientation.w = 1.0;
	gap.scale.x = 0.001;
	gap.scale.y = 0.001;
	gap.scale.z = 0.001;

	// clear all the gaps
	gap.id = 100;
	gap.action = 3u;  // delete all
	gapPublisher.publish(gap);

	for (size_t i=0; i<gapIndicatorPtrs.size(); i++) {

		// gap i features
		gap.id = 100+i;
		gap.action = visualization_msgs::Marker::ADD;
		gap.pose.position.x = 0.5*(gapIndicatorPtrs[i]->getXLimits()(1)+gapIndicatorPtrs[i]->getXLimits()(0));
		gap.pose.position.y = 0.5*(gapIndicatorPtrs[i]->getYLimits()(1)+gapIndicatorPtrs[i]->getYLimits()(0));
		gap.scale.x = gapIndicatorPtrs[i]->getXLimits()(1)-gapIndicatorPtrs[i]->getXLimits()(0);
		gap.scale.y = gapIndicatorPtrs[i]->getYLimits()(1)-gapIndicatorPtrs[i]->getYLimits()(0);

		// color
		bool isRepeller = gapIndicatorPtrs[i]->isRepeller();
		if (isRepeller==true) {
			gap.color.a = 0.6; // Don't forget to set the alpha!
			gap.color.r = 0.6;
			gap.color.g = 0.6;
			gap.color.b = 0.6;

		} else {
			gap.color.a = 0.6; // Don't forget to set the alpha!
			gap.color.r = 0.0;
			gap.color.g = 0.6;
			gap.color.b = 0.0;
		}

		// publish box
		gapPublisher.publish(gap);

	}  // end of i loop

}


}  // end of switched_model namespace




#endif /* ENDEFFECTORCONSTRAINTSUTILITIES_H_ */
