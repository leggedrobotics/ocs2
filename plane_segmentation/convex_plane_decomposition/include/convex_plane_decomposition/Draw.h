//
// Created by rgrandia on 09.06.20.
//

#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/imgproc/imgproc.hpp"

#include "PolygonTypes.h"

namespace convex_plane_decomposition {

cv::Vec3b randomColor();

std::vector<cv::Point> toCv(const CgalPolygon2d& polygon);

void drawContour(cv::Mat& img, const CgalPoint2d& point, double radius = 1, const cv::Vec3b* color = nullptr);
void drawContour(cv::Mat& img, const CgalPolygon2d& polygon, const cv::Vec3b* color = nullptr);
void drawContour(cv::Mat& img, const CgalPolygonWithHoles2d& polygonWithHoles2d, const cv::Vec3b* color = nullptr);

CgalPolygon2d scaleShape(const CgalPolygon2d& polygon, double scale);
CgalPolygonWithHoles2d scaleShape(const CgalPolygonWithHoles2d& polygonWithHoles, double scale);

}  // namespace convex_plane_decomposition
