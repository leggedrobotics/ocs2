//
// Created by rgrandia on 26.10.21.
//

#pragma once

#include "convex_plane_decomposition/SegmentedPlanesMap.h"

namespace convex_plane_decomposition {
namespace contour_extraction {

/**
 * Upsamples an image such that all pixels are turned into 9 pixels, with the original pixel in the middle.
 * Around the edges, each pixel is only upsamples in the inward direction.
 *
 * @param image : source image
 * @return upsamples image
 */
cv::Mat upSample(const cv::Mat& image);

/**
 * Upsamples a segmented terrain such that the resulting terrain has 1/3 of the input resolution. (Each cell is split into 9 cells)
 * This specific upsampling ratio makes it possible to keep labels in their exact original location in world frame.
 *
 * @param mapIn : source terrain
 * @return upsampled terrain
 */
SegmentedPlanesMap upSample(const SegmentedPlanesMap& mapIn);

}  // namespace contour_extraction
}  // namespace convex_plane_decomposition