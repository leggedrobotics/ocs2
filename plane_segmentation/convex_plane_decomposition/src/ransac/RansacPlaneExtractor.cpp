#include "convex_plane_decomposition/ransac/RansacPlaneExtractor.hpp"

namespace ransac_plane_extractor {

RansacPlaneExtractor::RansacPlaneExtractor(const RansacPlaneExtractorParameters& parameters) {
  setParameters(parameters);
  ransac_.add_shape_factory<Plane>();
}

void RansacPlaneExtractor::setParameters(const RansacPlaneExtractorParameters& parameters) {
  cgalRansacParameters_.probability = parameters.probability;
  cgalRansacParameters_.min_points = parameters.min_points;
  cgalRansacParameters_.epsilon = parameters.epsilon / 3.0;  // CGAL ransac puts the inlier tolerance at 3 times epsilon
  cgalRansacParameters_.cluster_epsilon = parameters.cluster_epsilon;
  cgalRansacParameters_.normal_threshold = std::cos(parameters.normal_threshold * M_PI / 180.0);
}

void RansacPlaneExtractor::detectPlanes(std::vector<PointWithNormal>& points_with_normal) {
  ransac_.set_input(points_with_normal);
  ransac_.detect(cgalRansacParameters_);
}

std::pair<Eigen::Vector3d, Eigen::Vector3d> RansacPlaneExtractor::getPlaneParameters(Shape* shapePtr) {
  const auto* planePtr = static_cast<Plane*>(shapePtr);

  // Get Normal, pointing upwards
  Eigen::Vector3d normalVector(planePtr->plane_normal().x(), planePtr->plane_normal().y(), planePtr->plane_normal().z());
  if (normalVector.z() < 0.0) {
    normalVector = -normalVector;
  }

  // Project origin to get a point on the plane.
  const auto support = planePtr->projection({0.0, 0.0, 0.0});
  const Eigen::Vector3d supportVector(support.x(), support.y(), support.z());

  return {normalVector, supportVector};
}

}  // namespace ransac_plane_extractor
