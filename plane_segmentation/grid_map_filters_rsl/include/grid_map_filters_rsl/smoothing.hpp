/**
 * @file        smoothing.hpp
 * @authors     Fabian Jenelten
 * @date        18.05, 2021
 * @affiliation ETH RSL
 * @brief       Smoothing and outlier rejection filters.
 */

#pragma once

// grid map.
#include <grid_map_core/grid_map_core.hpp>

namespace grid_map {
namespace smoothing {

/**
 * @brief Sequential median filter (open-cv function). In-place operation (layerIn = layerOut) is supported.
 * @param map               grid map
 * @param layerIn           reference layer (filter is applied wrt this layer)
 * @param layerOut          output layer (filtered map is written into this layer)
 * @param kernelSize        size of the smoothing window (must be an odd number)
 * @param deltaKernelSize   kernel size is increased by this value, if numberOfRepeats > 1
 * @param numberOfRepeats   number of sequentially applied median filters (approaches to gaussian blurring if increased)
 */
void median(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut, int kernelSize, int deltaKernelSize = 2,
            int numberOfRepeats = 1);

/**
 * @brief Sequential box blur filter (open cv-function). In-place operation (layerIn = layerOut) is supported.
 * @param map               grid map
 * @param layerIn           reference layer (filter is applied wrt this layer)
 * @param layerOut          output layer (filtered map is written into this layer)
 * @param kernelSize        size of the smoothing window (should be an odd number, otherwise, introduces offset)
 * @param numberOfRepeats   number of sequentially applied blurring filters (approaches to gaussian blurring if increased)
 */
void boxBlur(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut, int kernelSize, int numberOfRepeats = 1);

/**
 * @brief Gaussian blur filter (open cv-function). In-place operation (layerIn = layerOut) is supported.
 * @param map               grid map
 * @param layerIn           reference layer (filter is applied wrt this layer)
 * @param layerOut          output layer (filtered map is written into this layer)
 * @param kernelSize        size of the smoothing window (should be an odd number, otherwise, introduces offset)
 * @param sigma             variance
 */
void gaussianBlur(grid_map::GridMap& map, const std::string& layerIn, const std::string& layerOut, int kernelSize, double sigma);

}  // namespace smoothing
}  // namespace grid_map
