//
// Created by rgrandia on 07.06.20.
//

#pragma once

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>

namespace convex_plane_decomposition {

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using CgalPoint2d = K::Point_2;
using CgalCircle2d = K::Circle_2;
using CgalPolygon2d = CGAL::Polygon_2<K>;
using CgalSegment2d = CgalPolygon2d::Segment_2;
using CgalPolygonWithHoles2d = CGAL::Polygon_with_holes_2<K>;
using CgalBbox2d = CGAL::Bbox_2;

}  // namespace convex_plane_decomposition
