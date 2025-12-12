//
// Created by rgrandia on 26.10.21.
//

#include <gtest/gtest.h>

#include "convex_plane_decomposition/contour_extraction/Upsampling.h"

TEST(TestUpsampling, upsampleImage) {
  // clang-format off
  cv::Mat M = (cv::Mat_<float>(3, 3) <<
      1, 2, 3,
      4, 5, 6,
      7, 8, 9);
  cv::Mat MoutCheck = (cv::Mat_<float>(7, 7) <<
      1, 1, 2, 2, 2, 3, 3,
      1, 1, 2, 2, 2, 3, 3,
      4, 4, 5, 5, 5, 6, 6,
      4, 4, 5, 5, 5, 6, 6,
      4, 4, 5, 5, 5, 6, 6,
      7, 7, 8, 8, 8, 9, 9,
      7, 7, 8, 8, 8, 9, 9);
  // clang-format on

  const auto Mout = convex_plane_decomposition::contour_extraction::upSample(M);

  ASSERT_TRUE(std::equal(MoutCheck.begin<float>(), MoutCheck.end<float>(), Mout.begin<float>()));
}