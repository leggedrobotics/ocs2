//
// Created by rgrandia on 09.06.20.
//

#pragma once

#include "PolygonTypes.h"

namespace convex_plane_decomposition {

CgalPolygon2d createRegularPolygon(const CgalPoint2d& center, double radius, int numberOfVertices);

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygon2d& parentShape, CgalPoint2d center, int numberOfVertices, double growthFactor);

CgalPolygon2d growConvexPolygonInsideShape(const CgalPolygonWithHoles2d& parentShape, CgalPoint2d center, int numberOfVertices,
                                           double growthFactor);

void updateMean(CgalPoint2d& mean, const CgalPoint2d& oldValue, const CgalPoint2d& updatedValue, int N);

}  // namespace convex_plane_decomposition
