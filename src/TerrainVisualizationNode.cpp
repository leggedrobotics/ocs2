

#include "ros/ros.h"
#include "geometry_msgs/PolygonStamped.h"

#include <ocs2_switched_model_interface/ground/TerrainModel.h>
#include <visualization_msgs/MarkerArray.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "TerrainVisualizationNode");
  ros::NodeHandle n;
  ros::Rate loop_rate(1);

  ros::Publisher markerPublisher = n.advertise<visualization_msgs::MarkerArray>("TerrainMarkers", 1);

  switched_model::TerrainModel terrainModel;
  terrainModel.loadTestTerrain();
  auto& polytopes = terrainModel.getPolytopes();

  int count = 0;
  while (ros::ok())
  {
    visualization_msgs::MarkerArray arrayMsg;
    for (int i=0; i<polytopes.size(); i++){
      arrayMsg.markers.push_back(switched_model::toRosMarker(polytopes[i], "world", i));
    }

    markerPublisher.publish(arrayMsg);

    ros::spinOnce();
    loop_rate.sleep();

    ++count;
  }

  return 0;
}