//
// Created by rgrandia on 21.04.20.
//

#include "ocs2_switched_model_interface/terrain/TerrainPlane.h"

#include "ocs2_switched_model_interface/core/Rotations.h"

#include <ocs2_core/misc/LoadData.h>

namespace switched_model {

TerrainPlane loadTerrainPlane(const std::string& filename, bool verbose) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  if (verbose) {
    std::cerr << "\n #### terrain plane:" << std::endl;
    std::cerr << " #### ==================================================" << std::endl;
  }

  TerrainPlane plane;
  const std::string positionPrefix{"terrainPlane.position."};
  ocs2::loadData::loadPtreeValue(pt, plane.positionInWorld.x(), positionPrefix + "x", verbose);
  ocs2::loadData::loadPtreeValue(pt, plane.positionInWorld.y(), positionPrefix + "y", verbose);
  ocs2::loadData::loadPtreeValue(pt, plane.positionInWorld.z(), positionPrefix + "z", verbose);

  vector3_t eulerAnglesXYZ(0, 0, 0);
  const std::string orientationprefix{"terrainPlane.orientation."};
  ocs2::loadData::loadPtreeValue(pt, eulerAnglesXYZ.x(), orientationprefix + "roll", verbose);
  ocs2::loadData::loadPtreeValue(pt, eulerAnglesXYZ.y(), orientationprefix + "pitch", verbose);
  ocs2::loadData::loadPtreeValue(pt, eulerAnglesXYZ.z(), orientationprefix + "yaw", verbose);
  plane.orientationWorldToTerrain = rotationMatrixOriginToBase(eulerAnglesXYZ);

  if (verbose) {
    std::cerr << " #### ==================================================" << std::endl;
  }
  return plane;
}

}  // namespace switched_model
