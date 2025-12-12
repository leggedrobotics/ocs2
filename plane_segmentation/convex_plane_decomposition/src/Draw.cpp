//
// Created by rgrandia on 09.06.20.
//

#include "convex_plane_decomposition/Draw.h"

namespace convex_plane_decomposition {

cv::Vec3b randomColor() {
  return cv::Vec3b((rand() & 255), (rand() & 255), (rand() & 255));
}

std::vector<cv::Point> toCv(const CgalPolygon2d& polygon) {
  std::vector<cv::Point> contour;
  contour.reserve(polygon.size());
  for (const auto& point : polygon) {
    contour.emplace_back(point.x(), point.y());
  }
  return contour;
}

void drawContour(cv::Mat& img, const CgalPoint2d& point, double radius, const cv::Vec3b* color) {
  const cv::Vec3b contourColor = (color == nullptr) ? randomColor() : *color;
  cv::Point cvPoint(point.x(), point.y());
  cv::circle(img, cvPoint, radius, contourColor);
}

void drawContour(cv::Mat& img, const CgalPolygon2d& polygon, const cv::Vec3b* color) {
  const cv::Vec3b contourColor = (color == nullptr) ? randomColor() : *color;
  std::vector<std::vector<cv::Point>> contours{toCv(polygon)};
  drawContours(img, contours, 0, contourColor);
}

void drawContour(cv::Mat& img, const CgalPolygonWithHoles2d& polygonWithHoles2d, const cv::Vec3b* color) {
  const cv::Vec3b contourColor = (color == nullptr) ? randomColor() : *color;

  drawContour(img, polygonWithHoles2d.outer_boundary(), &contourColor);
  for (const auto& hole : polygonWithHoles2d.holes()) {
    drawContour(img, hole, &contourColor);
  }
}

CgalPolygon2d scaleShape(const CgalPolygon2d& polygon, double scale) {
  CgalPolygon2d scaledShape;
  scaledShape.container().reserve(polygon.size());
  for (const auto& point : polygon) {
    scaledShape.push_back({scale * point.x(), scale * point.y()});
  }
  return scaledShape;
}
CgalPolygonWithHoles2d scaleShape(const CgalPolygonWithHoles2d& polygonWithHoles, double scale) {
  CgalPolygonWithHoles2d scaledShape(scaleShape(polygonWithHoles.outer_boundary(), scale));

  for (const auto& hole : polygonWithHoles.holes()) {
    scaledShape.add_hole(scaleShape(hole, scale));
  }
  return scaledShape;
}

}