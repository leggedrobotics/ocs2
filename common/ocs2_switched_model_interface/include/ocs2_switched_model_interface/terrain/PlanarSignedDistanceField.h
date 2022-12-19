#pragma once

#include "ocs2_switched_model_interface/core/SwitchedModel.h"
#include "ocs2_switched_model_interface/terrain/SignedDistanceField.h"
#include "ocs2_switched_model_interface/terrain/TerrainModel.h"

namespace switched_model {

/**
 * Implements a flat terrain signed distance field
 */
class PlanarSignedDistanceField : public SignedDistanceField {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit PlanarSignedDistanceField(TerrainPlane terrainPlane);

  ~PlanarSignedDistanceField() override = default;
  PlanarSignedDistanceField* clone() const override { return new PlanarSignedDistanceField(*this); };

  scalar_t value(const vector3_t& position) const override;

  vector3_t derivative(const vector3_t& position) const override;

  std::pair<scalar_t, vector3_t> valueAndDerivative(const vector3_t& position) const override;

 protected:
  PlanarSignedDistanceField(const PlanarSignedDistanceField& other);

 private:
  TerrainPlane terrainPlane_;
};

}  // namespace switched_model
