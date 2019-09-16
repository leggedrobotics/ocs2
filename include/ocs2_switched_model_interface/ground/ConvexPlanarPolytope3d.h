

#ifndef OCS2_CTRL_CONVEXPLANARPOLYTOPE3D_H
#define OCS2_CTRL_CONVEXPLANARPOLYTOPE3D_H

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <geometry_msgs/PolygonStamped.h>
#include <visualization_msgs/Marker.h>
#include <ros/ros.h>

namespace switched_model {

/**
 *  Data type to store ordered set of 3D points defining a Convex Polytope.
 *  No checks are made if the points are co-planar or if the polytope is indeed convex
 */
using ConvexPlanarPolytope3d = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;
using ConvexPlanarPolytope3dArray = std::vector<ConvexPlanarPolytope3d, Eigen::aligned_allocator<ConvexPlanarPolytope3d>>;

/**
 * Convert polytope to ROS PolygonsStamped message
 * @param polytope
 * @param frame_id
 * @return ROS PolygonsStamped message
 */
geometry_msgs::PolygonStamped toRos(const ConvexPlanarPolytope3d& polytope, std::string frame_id);

/**
 * Convert polytope to ROS Marker message
 * @param polytope
 * @param frame_id
 * @return ROS Marker message
 */
visualization_msgs::Marker toRosMarker(const ConvexPlanarPolytope3d& polytope, std::string frame_id, int objectId);

/**
 * Returns a single halvespace definition [w | b] such that
 * - Both points are on the separating plane
 *      w.dot(p0) + b = 0
 *      w.dot(p1) + b = 0
 * - The control point is on the interior
 *      w.dot(pControl) + b >= 0
 * - w is perpendicular to the normal vector
 * - w is normalized to norm(w) = 1
 * @param p0
 * @param p1
 * @param normal
 * @param controlPoint
 * @return halvespace [w | b]
 */
Eigen::Matrix<double, 4, 1> getSingleHalveSpace(const Eigen::Vector3d& p0, const Eigen::Vector3d& p1, const Eigen::Vector3d& normal,
                                                const Eigen::Vector3d& controlPoint);

/**
 * Returns Matrix [A | b], such that a point p is inside the polytope iff A*p + b >= 0
 * Each inequality has the same units as the polytope
 * @param polytope
 * @return Matrix [A | b]
 */
Eigen::MatrixXd toHalfSpaces(const ConvexPlanarPolytope3d& polytope);

ConvexPlanarPolytope3d createPolytope(int numPoints, double scale, double rotation, Eigen::Vector3d offset);

}  // namespace switched_model

#endif  // OCS2_CTRL_CONVEXPLANARPOLYTOPE3D_H
