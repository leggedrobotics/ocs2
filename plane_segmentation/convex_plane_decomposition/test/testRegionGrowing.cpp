//
// Created by rgrandia on 02.02.22.
//


#include <gtest/gtest.h>

#include "convex_plane_decomposition/ConvexRegionGrowing.h"

#include <convex_plane_decomposition/GeometryUtils.h>

using namespace convex_plane_decomposition;

TEST(TestRegionGrowing, center_on_border) {
  // Rare case where the region algorithm go stuck
  const int numberOfVertices = 16;  // Multiple of 4 is nice for symmetry.
  const double growthFactor = 1.05;
  CgalPoint2d center(0.0, 1.0);

  CgalPolygonWithHoles2d parentShape;
  parentShape.outer_boundary().container() = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

  const auto convexInnerApprox = growConvexPolygonInsideShape(parentShape, center, numberOfVertices, growthFactor);
  ASSERT_TRUE(convexInnerApprox.is_convex());
  for (auto it = convexInnerApprox.vertices_begin(); it!=convexInnerApprox.vertices_end(); ++it) {
    ASSERT_TRUE(isInside(*it, parentShape));
  }
}

TEST(TestRegionGrowing, debug_case) {
  // Rare case where the region algorithm got stuck
  const int numberOfVertices = 16;
  const double growthFactor = 1.05;
  CgalPoint2d center(-0.147433, 0.800114);

  CgalPolygonWithHoles2d parentShape;
  parentShape.outer_boundary().container() = {
      {1.03923, -0.946553},  {1.03923, 0.840114},   {0.7859, 0.840114},    {0.772567, 0.853447},  {0.759233, 0.853447},
      {0.7459, 0.86678},     {0.7459, 0.880114},    {0.732567, 0.893447},  {0.719233, 0.893447},  {0.7059, 0.90678},
      {0.7059, 1.05345},     {0.652567, 1.05345},   {0.652567, 0.90678},   {0.639233, 0.893447},  {0.6259, 0.893447},
      {0.612567, 0.880114},  {0.612567, 0.86678},   {0.599233, 0.853447},  {0.5859, 0.853447},    {0.572567, 0.840114},
      {0.532567, 0.840114},  {0.532567, 0.82678},   {0.519233, 0.813447},  {0.5059, 0.813447},    {0.492567, 0.800114},
      {0.3059, 0.800114},    {0.292567, 0.813447},  {0.279233, 0.813447},  {0.2659, 0.82678},     {0.2659, 0.840114},
      {0.252567, 0.853447},  {0.239233, 0.853447},  {0.2259, 0.86678},     {0.2259, 0.920114},    {0.212567, 0.933447},
      {0.199233, 0.933447},  {0.1859, 0.94678},     {0.1859, 1.05345},     {0.132567, 1.05345},   {0.132567, 0.86678},
      {0.119233, 0.853447},  {0.1059, 0.853447},    {0.0925666, 0.840114}, {0.0925666, 0.82678},  {0.0792332, 0.813447},
      {0.0658999, 0.813447}, {0.0525666, 0.800114}, {-0.1341, 0.800114},   {-0.147433, 0.813447}, {-0.160767, 0.813447},
      {-0.1741, 0.82678},    {-0.1741, 0.840114},   {-0.2141, 0.840114},   {-0.227433, 0.853447}, {-0.240767, 0.853447},
      {-0.2541, 0.86678},    {-0.2541, 0.880114},   {-0.267433, 0.893447}, {-0.280767, 0.893447}, {-0.2941, 0.90678},
      {-0.2941, 1.05345},    {-0.960767, 1.05345},  {-0.960767, -0.946553}};

  CgalPolygon2d hole;
  hole.container() = {{0.5459, -0.266553},   {0.532566, -0.279886}, {0.3059, -0.279886},   {0.292566, -0.266553}, {0.279233, -0.266553},
                      {0.2659, -0.25322},    {0.2659, -0.239886},   {0.252566, -0.226553}, {0.239233, -0.226553}, {0.2259, -0.21322},
                      {0.2259, 0.320114},    {0.239233, 0.333447},  {0.252566, 0.333447},  {0.2659, 0.34678},     {0.532567, 0.34678},
                      {0.5459, 0.333447},    {0.559233, 0.333447},  {0.572567, 0.320114},  {0.572567, 0.30678},   {0.5859, 0.293447},
                      {0.599233, 0.293447},  {0.612567, 0.280114},  {0.612566, 0.0667803}, {0.6259, 0.053447},    {0.639233, 0.053447},
                      {0.652566, 0.0401136}, {0.652566, -0.17322},  {0.639233, -0.186553}, {0.6259, -0.186553},   {0.612566, -0.199886},
                      {0.612566, -0.21322},  {0.599233, -0.226553}, {0.5859, -0.226553},   {0.572566, -0.239886}, {0.572566, -0.25322},
                      {0.559233, -0.266553}};
  parentShape.holes().push_back(std::move(hole));

  const auto convexInnerApprox = growConvexPolygonInsideShape(parentShape, center, numberOfVertices, growthFactor);

  ASSERT_TRUE(convexInnerApprox.is_convex());
  for (auto it = convexInnerApprox.vertices_begin(); it!=convexInnerApprox.vertices_end(); ++it) {
    ASSERT_TRUE(isInside(*it, parentShape));
  }
}