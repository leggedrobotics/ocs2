/*
 * FeetPlannerBase.h
 *
 *  Created on: Feb 5, 2018
 *      Author: farbod
 */

#pragma once

#include <array>
#include <memory>
#include <vector>

#include "SplineCpg.h"
#include "ocs2_core/logic/ModeSchedule.h"
#include "ocs2_switched_model_interface/core/MotionPhaseDefinition.h"

namespace switched_model {

class FeetPlannerBase {
 public:
  using Dims_t = ocs2::Dimensions<0, 0>;
  using scalar_t = Dims_t::scalar_t;
  using bool_array_t = std::vector<bool>;
  using int_array_t = std::vector<int>;
  using size_array_t = Dims_t::size_array_t;
  using scalar_array_t = Dims_t::scalar_array_t;

  using cpg_t = SplineCpg;
  using feet_cpg_ptr_t = std::array<std::unique_ptr<cpg_t>, NUM_CONTACT_POINTS>;

  virtual ~FeetPlannerBase() = default;

  /** clone the class */
  virtual FeetPlannerBase* clone() const = 0;

  /**
   * Plans the CPG for the swing legs in the indexed mode.
   *
   * @param [in] index: The index of the subsystem for which the CPG should be designed.
   * @param [in] modeschedule: containing event times and subsystem definitions
   * @param [in] eventTimes: The event times.
   * @return plannedCPG: An array of the CPG class for each endeffector.
   */
  virtual feet_cpg_ptr_t planSingleMode(size_t index, const ocs2::ModeSchedule& modeSchedule) = 0;

  /**
   * based on the input phaseIDsStock finds the start subsystem and final subsystem of the swing
   * phases of the a foot in each subsystem.
   *
   * startTimeIndexStock: eventTimes[startTimesIndex] will be the take-off time for the requested leg.
   * finalTimeIndexStock: eventTimes[finalTimesIndex] will be the touch-down time for the requested leg.
   *
   * @param [in] footIndex: Foot index
   * @param [in] phaseIDsStock: The sequence of the motion phase IDs.
   * @param [in] contactFlagStock: The sequence of the contact status for the requested leg.
   * @return { startTimeIndexStock, finalTimeIndexStock}
   */
  static std::pair<int_array_t, int_array_t> updateFootSchedule(size_t footIndex, const size_array_t& phaseIDsStock,
                                                                const bool_array_t& contactFlagStock);

  /**
   * Extracts for each leg the contact sequence over the motion phase sequence.
   * @param phaseIDsStock
   * @return contactFlagStock
   */
  static std::array<bool_array_t, NUM_CONTACT_POINTS> extractContactFlags(const size_array_t& phaseIDsStock);

  /**
   * Finds the take-off and touch-down times indices for a specific leg.
   *
   * @param index
   * @param contactFlagStock
   * @return {The take-off time index for swing legs, touch-down time index for swing legs}
   */
  static std::pair<int, int> findIndex(size_t index, const bool_array_t& contactFlagStock);
};

}  // namespace switched_model
