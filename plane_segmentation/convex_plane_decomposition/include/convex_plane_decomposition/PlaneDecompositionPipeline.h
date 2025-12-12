//
// Created by rgrandia on 16.03.22.
//

#pragma once

#include "convex_plane_decomposition/GridMapPreprocessing.h"
#include "convex_plane_decomposition/PlanarRegion.h"
#include "convex_plane_decomposition/Postprocessing.h"
#include "convex_plane_decomposition/Timer.h"
#include "convex_plane_decomposition/contour_extraction/ContourExtraction.h"
#include "convex_plane_decomposition/sliding_window_plane_extraction/SlidingWindowPlaneExtractor.h"

namespace convex_plane_decomposition {

/**
 * Encloses the full plane decomposition pipeline:
 *
 * Input:
 *   - Elevation map
 * Output:
 *   - Planar terrain (planes + filtered elevation map)
 *   - Segmented elevation map
 */
class PlaneDecompositionPipeline {
 public:
  /** Collection of all parameters of steps in the pipeline */
  struct Config {
    PreprocessingParameters preprocessingParameters;
    sliding_window_plane_extractor::SlidingWindowPlaneExtractorParameters slidingWindowPlaneExtractorParameters;
    ransac_plane_extractor::RansacPlaneExtractorParameters ransacPlaneExtractorParameters;
    contour_extraction::ContourExtractionParameters contourExtractionParameters;
    PostprocessingParameters postprocessingParameters;
  };

  /**
   * Constructor
   * @param config : configuration containing all parameters of the pipeline
   */
  PlaneDecompositionPipeline(const Config& config);

  /**
   * Trigger update of the pipeline
   * @param gridMap : gridmap to process. Will be modified and added to the resulting planar terrain.
   * @param elevationLayer : Name of the elevation layer.
   */
  void update(grid_map::GridMap&& gridMap, const std::string& elevationLayer);

  /// Access to the Pipeline result.
  PlanarTerrain& getPlanarTerrain() { return planarTerrain_; }

  /// Fills in the resulting segmentation into a gridmap layer data.
  void getSegmentation(grid_map::GridMap::Matrix& segmentation) const;

  // Timers
  const Timer& getPrepocessTimer() const { return preprocessTimer_; }
  const Timer& getSlidingWindowTimer() const { return slidingWindowTimer_; }
  const Timer& getContourExtractionTimer() const { return contourExtractionTimer_; }
  const Timer& getPostprocessTimer() const { return postprocessTimer_; }

 private:
  PlanarTerrain planarTerrain_;

  // Pipeline
  GridMapPreprocessing preprocessing_;
  sliding_window_plane_extractor::SlidingWindowPlaneExtractor slidingWindowPlaneExtractor_;
  contour_extraction::ContourExtraction contourExtraction_;
  Postprocessing postprocessing_;

  // Timing
  Timer preprocessTimer_;
  Timer slidingWindowTimer_;
  Timer contourExtractionTimer_;
  Timer postprocessTimer_;
};

}  // namespace convex_plane_decomposition
