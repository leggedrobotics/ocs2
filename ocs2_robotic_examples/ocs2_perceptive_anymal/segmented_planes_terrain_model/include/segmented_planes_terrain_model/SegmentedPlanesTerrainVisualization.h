//
// Created by rgrandia on 24.06.20.
//

#pragma once

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <ocs2_ros_interfaces/visualization/VisualizationColors.h>
#include <ocs2_switched_model_interface/terrain/ConvexTerrain.h>

namespace switched_model {

visualization_msgs::msg::MarkerArray getConvexTerrainMarkers(const ConvexTerrain& convexTerrain, ocs2::Color color, double linewidth,
                                                        double normalLength);

}  // namespace switched_model
