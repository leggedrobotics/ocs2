#include "convex_plane_decomposition/PlaneDecompositionPipeline.h"

#include <opencv2/core/eigen.hpp>

namespace convex_plane_decomposition {

PlaneDecompositionPipeline::PlaneDecompositionPipeline(const Config& config)
    : preprocessing_(config.preprocessingParameters),
      slidingWindowPlaneExtractor_(config.slidingWindowPlaneExtractorParameters, config.ransacPlaneExtractorParameters),
      contourExtraction_(config.contourExtractionParameters),
      postprocessing_(config.postprocessingParameters) {}

void PlaneDecompositionPipeline::update(grid_map::GridMap&& gridMap, const std::string& elevationLayer) {
  // Clear / Overwrite old result
  planarTerrain_.planarRegions.clear();
  planarTerrain_.gridMap = std::move(gridMap);

  preprocessTimer_.startTimer();
  preprocessing_.preprocess(planarTerrain_.gridMap, elevationLayer);
  preprocessTimer_.endTimer();

  slidingWindowTimer_.startTimer();
  slidingWindowPlaneExtractor_.runExtraction(planarTerrain_.gridMap, elevationLayer);
  slidingWindowTimer_.endTimer();

  contourExtractionTimer_.startTimer();
  planarTerrain_.planarRegions = contourExtraction_.extractPlanarRegions(slidingWindowPlaneExtractor_.getSegmentedPlanesMap());
  contourExtractionTimer_.endTimer();

  postprocessTimer_.startTimer();
  // Add binary map
  const std::string planeClassificationLayer{"plane_classification"};
  planarTerrain_.gridMap.add(planeClassificationLayer);
  auto& traversabilityMask = planarTerrain_.gridMap.get(planeClassificationLayer);
  cv::cv2eigen(slidingWindowPlaneExtractor_.getBinaryLabeledImage(), traversabilityMask);

  postprocessing_.postprocess(planarTerrain_, elevationLayer, planeClassificationLayer);
  postprocessTimer_.endTimer();
}

void PlaneDecompositionPipeline::getSegmentation(grid_map::GridMap::Matrix& segmentation) const {
  cv::cv2eigen(slidingWindowPlaneExtractor_.getSegmentedPlanesMap().labeledImage, segmentation);
}

}  // namespace convex_plane_decomposition