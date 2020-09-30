/*
 * GeometryInterfaceVisualization.h
 *
 *  Created on: 4 Sep 2020
 *      Author: perry
 */

#pragma once

#include <ros/ros.h>

#include <visualization_msgs/MarkerArray.h>

#include <ocs2_pinocchio/PinocchioGeometryInterface.hpp>

namespace ocs2 {

class GeometryInterfaceVisualization {
 public:
  GeometryInterfaceVisualization(const PinocchioGeometryInterface& geometryInterface, ros::NodeHandle& nh,
                                 std::string pinocchioWorldFrame = "world");
  virtual ~GeometryInterfaceVisualization() = default;

  void publishDistances(const ocs2::vector_t&);

 private:
  PinocchioGeometryInterface geometryInterface_;

  ros::Publisher markerPublisher_;

  std::string pinocchioWorldFrame_;
};

}  // namespace ocs2
