//
// Created by rgrandia on 26.10.21.
//

#include "convex_plane_decomposition/contour_extraction/Upsampling.h"

namespace convex_plane_decomposition {
namespace contour_extraction {

namespace {
// Helper function that upsamples a single column. Writes into a target that is already allocated with the right size
void upSampleColumn(const cv::Mat& column, cv::Mat& target, int col, int rows, int upsampledRows) {
  for (int row = 0; row < rows; ++row) {
    const auto value = column.at<float>(row);
    if (row == 0) {
      target.at<float>(0, col) = value;
      target.at<float>(1, col) = value;
    } else if (row + 1 == rows) {
      target.at<float>(upsampledRows - 2, col) = value;
      target.at<float>(upsampledRows - 1, col) = value;
    } else {
      const int targetRow = 2 + 3 * (row - 1);
      target.at<float>(targetRow, col) = value;
      target.at<float>(targetRow + 1, col) = value;
      target.at<float>(targetRow + 2, col) = value;
    }
  }
}
}  // namespace

cv::Mat upSample(const cv::Mat& image) {
  const int rows = image.rows;
  const int cols = image.cols;
  assert(rows >= 2);
  assert(cols >= 2);
  const int upsampledRows = 4 + 3 * (rows - 2);
  const int upsampledCols = 4 + 3 * (cols - 2);

  cv::Mat result(upsampledRows, upsampledCols, image.type());

  for (int col = 0; col < cols; ++col) {
    const auto& column = image.col(col);
    if (col == 0) {
      upSampleColumn(column, result, 0, rows, upsampledRows);
      result.col(0).copyTo(result.col(1));
    } else if (col + 1 == cols) {
      upSampleColumn(column, result, upsampledCols - 2, rows, upsampledRows);
      result.col(upsampledCols - 2).copyTo(result.col(upsampledCols - 1));
    } else {
      const int targetCol = 2 + 3 * (col - 1);
      upSampleColumn(column, result, targetCol, rows, upsampledRows);
      result.col(targetCol).copyTo(result.col(targetCol + 1));
      result.col(targetCol + 1).copyTo(result.col(targetCol + 2));
    }
  }

  return result;
}

SegmentedPlanesMap upSample(const SegmentedPlanesMap& mapIn) {
  SegmentedPlanesMap mapOut;
  mapOut.labelPlaneParameters = mapIn.labelPlaneParameters;
  mapOut.labeledImage = upSample(mapIn.labeledImage);
  mapOut.resolution = mapIn.resolution / 3.0;
  mapOut.mapOrigin = mapIn.mapOrigin;
  mapOut.highestLabel = mapIn.highestLabel;
  return mapOut;
}

}  // namespace contour_extraction
}  // namespace convex_plane_decomposition