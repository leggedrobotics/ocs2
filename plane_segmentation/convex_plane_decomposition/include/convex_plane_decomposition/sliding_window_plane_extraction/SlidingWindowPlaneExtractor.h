#pragma once

#include <Eigen/Core>

#include <grid_map_core/GridMap.hpp>
#include <opencv2/core/mat.hpp>

#include "convex_plane_decomposition/SegmentedPlanesMap.h"
#include "convex_plane_decomposition/ransac/RansacPlaneExtractor.hpp"

#include "SlidingWindowPlaneExtractorParameters.h"

namespace convex_plane_decomposition {
namespace sliding_window_plane_extractor {

class SlidingWindowPlaneExtractor {
 public:
  SlidingWindowPlaneExtractor(const SlidingWindowPlaneExtractorParameters& parameters,
                              const ransac_plane_extractor::RansacPlaneExtractorParameters& ransacParameters);

  void runExtraction(const grid_map::GridMap& map, const std::string& layer_height);

  const SegmentedPlanesMap& getSegmentedPlanesMap() const { return segmentedPlanesMap_; }

  const cv::Mat& getBinaryLabeledImage() const { return binaryImagePatch_; }

  /** Can be run after extraction for debugging purpose */
  void addSurfaceNormalToMap(grid_map::GridMap& map, const std::string& layerPrefix) const;

 private:
  bool isGloballyPlanar(const Eigen::Vector3d& normalVectorPlane, const Eigen::Vector3d& supportVectorPlane,
                        const std::vector<ransac_plane_extractor::PointWithNormal>& pointsWithNormal) const;
  bool isWithinInclinationLimit(const Eigen::Vector3d& normalVectorPlane) const;

  std::pair<Eigen::Vector3d, double> computeNormalAndErrorForWindow(const Eigen::MatrixXf& windowData) const;
  bool isLocallyPlanar(const Eigen::Vector3d& localNormal, double meanSquaredError) const;

  int getLinearIndex(int row, int col) const { return row + col * mapRows_; };

  void computePlaneParametersForLabel(int label, std::vector<ransac_plane_extractor::PointWithNormal>& pointsWithNormal);
  void refineLabelWithRansac(int label, std::vector<ransac_plane_extractor::PointWithNormal>& pointsWithNormal);

  void extractPlaneParametersFromLabeledImage();

  void runSegmentation();

  void runSlidingWindowDetector();

  void setToBackground(int label);

  SlidingWindowPlaneExtractorParameters parameters_;
  ransac_plane_extractor::RansacPlaneExtractorParameters ransacParameters_;

  const grid_map::GridMap* map_;
  std::string elevationLayer_;
  int mapRows_;

  std::vector<Eigen::Vector3d> surfaceNormals_;
  std::vector<ransac_plane_extractor::PointWithNormal> pointsWithNormal_;

  cv::Mat binaryImagePatch_;
  SegmentedPlanesMap segmentedPlanesMap_;
};
}  // namespace sliding_window_plane_extractor
}  // namespace convex_plane_decomposition
