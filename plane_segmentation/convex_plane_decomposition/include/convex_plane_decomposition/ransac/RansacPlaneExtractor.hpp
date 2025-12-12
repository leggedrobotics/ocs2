#pragma once

#include <vector>

#include <Eigen/Core>

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Point_with_normal_3.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/property_map.h>

#include "RansacPlaneExtractorParameters.h"

namespace ransac_plane_extractor {

// Point with normal related type declarations.
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point3D = Kernel::Point_3;
using Vector3D = Kernel::Vector_3;
using PointWithNormal = std::pair<Kernel::Point_3, Kernel::Vector_3>;
using PwnVector = std::vector<PointWithNormal>;

// RANSAC plane extractor related type declarations.
using PointMap = CGAL::First_of_pair_property_map<PointWithNormal>;
using NormalMap = CGAL::Second_of_pair_property_map<PointWithNormal>;
using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, PwnVector, PointMap, NormalMap>;
using EfficientRansac = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using Plane = CGAL::Shape_detection::Plane<Traits>;
using Shape = CGAL::Shape_detection::Shape_base<Traits>;

class RansacPlaneExtractor {
 public:
  RansacPlaneExtractor(const RansacPlaneExtractorParameters& parameters);

  void setParameters(const RansacPlaneExtractorParameters& parameters);

  void detectPlanes(std::vector<PointWithNormal>& points_with_normal);

  /// Return {plane normal, support vector} for the detected shape
  static std::pair<Eigen::Vector3d, Eigen::Vector3d> getPlaneParameters(Shape* shapePtr);

  /// Returns an iterator range. Data is still in the ransac_object
  EfficientRansac::Shape_range getDetectedPlanes() const { return ransac_.shapes(); };

  /// Returns an iterator range. Data is still in the ransac_object
  EfficientRansac::Point_index_range getUnassignedPointIndices() { return ransac_.indices_of_unassigned_points(); }

 private:
  EfficientRansac ransac_;
  EfficientRansac::Parameters cgalRansacParameters_;
};

}  // namespace ransac_plane_extractor
