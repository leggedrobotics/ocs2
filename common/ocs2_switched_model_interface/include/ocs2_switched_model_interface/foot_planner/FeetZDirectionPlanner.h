/*
 * FeetZDirectionPlanner.h
 *
 *  Created on: Jul 5, 2016
 *      Author: farbod
 */

#pragma once

#include "ocs2_switched_model_interface/foot_planner/FeetPlannerBase.h"

namespace switched_model {

class FeetZDirectionPlanner : public FeetPlannerBase {
 public:
  using Base = FeetPlannerBase;
  using Base::bool_array_t;
  using Base::cpg_t;
  using Base::feet_cpg_ptr_t;
  using Base::scalar_array_t;
  using Base::scalar_t;
  using Base::size_array_t;

  /**
   * default constructor
   */
  FeetZDirectionPlanner() : FeetZDirectionPlanner(0.15, 1.0) {}

  /**
   * Constructor
   *
   * @param [in] swingLegLiftOff: Maximum swing leg lift-off
   * @param [in] swingTimeScale: The scaling factor for adapting swing leg's lift-off
   */
  explicit FeetZDirectionPlanner(scalar_t swingLegLiftOff, scalar_t swingTimeScale = 1.0, scalar_t liftOffVelocity = 0.0,
                                 scalar_t touchDownVelocity = 0.0);

  /** Destructor. */
  ~FeetZDirectionPlanner() override = default;

  /** clone the class */
  FeetZDirectionPlanner* clone() const override;

  feet_cpg_ptr_t planSingleMode(size_t index, const size_array_t& phaseIDsStock, const scalar_array_t& eventTimes) override;

 private:
  void checkThatIndicesAreValid(int leg, int index, int startIndex, int finalIndex) const;

  scalar_t adaptiveSwingLegLiftOff(scalar_t startTime, scalar_t finalTime, scalar_t swingTimeScale) {
    return std::min(1.0, (finalTime - startTime) / swingTimeScale);
  }

  scalar_t swingLegLiftOff_ = 0.15;
  scalar_t swingTimeScale_ = 1.0;
  const scalar_t liftOffVelocity_ = 0.0;
  const scalar_t touchDownVelocity_ = 0.0;

  size_array_t phaseIDsStock_;
  scalar_array_t eventTimes_;
  std::array<bool_array_t, 4> eesContactFlagStocks_;
  std::array<int_array_t, 4> startTimesIndices_;
  std::array<int_array_t, 4> finalTimesIndices_;
};

}  // namespace switched_model
