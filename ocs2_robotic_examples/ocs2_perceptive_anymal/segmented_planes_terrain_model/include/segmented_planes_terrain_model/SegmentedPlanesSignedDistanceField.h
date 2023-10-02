//
// Created by rgrandia on 17.03.22.
//

#pragma once

#include <ocs2_switched_model_interface/terrain/SignedDistanceField.h>

#include <grid_map_sdf/SignedDistanceField.hpp>

namespace switched_model {

/**
 * Simple wrapper class to implement the switched_model::SignedDistanceField interface.
 * See the forwarded function for documentation.
 */
class SegmentedPlanesSignedDistanceField : public SignedDistanceField {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  SegmentedPlanesSignedDistanceField(const grid_map::GridMap& gridMap, const std::string& elevationLayer, double minHeight,
                                     double maxHeight)
      : sdf_(gridMap, elevationLayer, minHeight, maxHeight) {}

  ~SegmentedPlanesSignedDistanceField() override = default;
  SegmentedPlanesSignedDistanceField* clone() const override { return new SegmentedPlanesSignedDistanceField(*this); };

  switched_model::scalar_t value(const switched_model::vector3_t& position) const override { return sdf_.value(position); }

  switched_model::vector3_t derivative(const switched_model::vector3_t& position) const override { return sdf_.derivative(position); }

  std::pair<switched_model::scalar_t, switched_model::vector3_t> valueAndDerivative(
      const switched_model::vector3_t& position) const override {
    return sdf_.valueAndDerivative(position);
  }

  grid_map::SignedDistanceField& asGridmapSdf() { return sdf_; }
  const grid_map::SignedDistanceField& asGridmapSdf() const { return sdf_; }

 protected:
  SegmentedPlanesSignedDistanceField(const SegmentedPlanesSignedDistanceField& other) : sdf_(other.sdf_){};

 private:
  grid_map::SignedDistanceField sdf_;
};

}  // namespace switched_model
