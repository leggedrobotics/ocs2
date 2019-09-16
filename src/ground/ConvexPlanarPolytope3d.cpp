

#include "ocs2_switched_model_interface/ground/ConvexPlanarPolytope3d.h"

namespace switched_model {

geometry_msgs::PolygonStamped toRos(const ConvexPlanarPolytope3d& polytope, std::string frame_id) {
  geometry_msgs::PolygonStamped msg;
  msg.header.frame_id = std::move(frame_id);
  for (const auto& point : polytope) {
    geometry_msgs::Point32 p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    msg.polygon.points.push_back(p);
  }
  return msg;
}

visualization_msgs::Marker toRosMarker(const ConvexPlanarPolytope3d& polytope, std::string frame_id, int objectId) {
  visualization_msgs::Marker msg;
  msg.header.frame_id = std::move(frame_id);
  msg.id = objectId;
  msg.type = visualization_msgs::Marker::LINE_STRIP;
  msg.frame_locked = true;
  msg.scale.x = 0.005; // used for line width
  msg.color.b = 1.0;
  msg.color.a = 1.0;
  auto& points = msg.points;
  for (const auto& point : polytope) {
    geometry_msgs::Point p;
    p.x = point[0];
    p.y = point[1];
    p.z = point[2];
    points.push_back(p);
  }
  // push last point again to close the polygon
  geometry_msgs::Point p;
  auto& point = polytope[0];
  p.x = point[0];
  p.y = point[1];
  p.z = point[2];
  points.push_back(p);
  return msg;
}

Eigen::Matrix<double, 4, 1> getSingleHalveSpace(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& normal,
                                                const Eigen::Vector3d& controlPoint) {
  // w is perpendicular to p01 and the normal vector
  Eigen::Vector3d w = (p1 - p0).cross(normal);
  w.normalize();
  // offset such that w*p0 + b = 0
  auto b = -w.dot(p0);
  // Adapt sign through control point for which we need w*p + b >= 0
  if ((w.dot(controlPoint) + b) < 0) {  // signs are wrong --> flip it
    w = -w;
    b = -b;
  }
  return (Eigen::Matrix<double, 4, 1>() << w, b).finished(); // return [w, b]
}

Eigen::MatrixXd toHalfSpaces(const ConvexPlanarPolytope3d& polytope) {
  if (polytope.size() < 3) {
    throw std::runtime_error("[ConvexPlanarPolytope3d:toHalfSpaces] need at least three points to define a polytope");
  }
  // Extract surface normal from first three points
  Eigen::Vector3d n = (polytope[1] - polytope[0]).cross(polytope[2] - polytope[1]);

  auto numConstraints = static_cast<int>(polytope.size());
  Eigen::MatrixXd Ab(numConstraints, 4);
  for (int i = 0; i < polytope.size(); i++) {
    int nextPoint = (i + 1) % numConstraints;  // Loop around to connect last and first point
    int checkPoint = (i + 2) % numConstraints;
    Ab.row(i) = getSingleHalveSpace(polytope[i], polytope[nextPoint], n, polytope[checkPoint]);
  }
  return Ab;
}

ConvexPlanarPolytope3d createPolytope(int numPoints, double scale, double rotation, Eigen::Vector3d offset){
  ConvexPlanarPolytope3d polytope;
  polytope.reserve(numPoints);
  for (int i=0; i<numPoints; i++){
    Eigen::Vector3d point{scale*sin(rotation), scale*cos(rotation), 0.0};
    point += offset;
    polytope.push_back(point);
    rotation += 2.0 * M_PI / numPoints;
  }
  return polytope;
}


}  // namespace switched_model