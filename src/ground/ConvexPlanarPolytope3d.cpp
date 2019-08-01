//
// Created by rgrandia on 08.07.19.
//

#include "ocs2_switched_model_interface/ground/ConvexPlanarPolytope3d.h"

namespace switched_model {

geometry_msgs::PolygonStamped toRos(const ConvexPlanarPolytope3d& polytope, std::string frame_id) {
  geometry_msgs::PolygonStamped msg;
  msg.header.frame_id = std::move(frame_id);
  for (const auto& point : polytope) {
    geometry_msgs::Point32 p;
    p.x = (float)point[0];
    p.y = (float)point[1];
    p.z = (float)point[2];
    msg.polygon.points.push_back(p);
  }
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

ConvexPlanarPolytope3d createSquare(double scale, Eigen::Vector3d offset) {
  ConvexPlanarPolytope3d square;
  square.reserve(4);
  Eigen::Vector3d point20{-scale, -scale, 0.0};
  Eigen::Vector3d point21{scale, -scale, 0.0};
  Eigen::Vector3d point22{scale, scale, 0.0};
  Eigen::Vector3d point23{-scale, scale, 0.0};
  point20 += offset;
  point21 += offset;
  point22 += offset;
  point23 += offset;
  square.push_back(point20);
  square.push_back(point21);
  square.push_back(point22);
  square.push_back(point23);
  return square;
}

}  // namespace switched_model