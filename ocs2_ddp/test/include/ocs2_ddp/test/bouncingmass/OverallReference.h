#pragma once

#include "Reference.h"
/*
 * 	This class constructs, contains and provides the reference for the
 * 	BouncingMass SLQ test. The reference is based on the reference presented
 * 	in [On Optimal Trajectory Tracking for Mechanical Systems with Unilateral Constraints
 * by M. Rijnen]
 */
class OverallReference {
  using DIMENSIONS = ocs2::Dimensions<3, 1>;
  using scalar_t = typename DIMENSIONS::scalar_t;
  using scalar_array_t = typename DIMENSIONS::scalar_array_t;
  using state_vector_t = typename DIMENSIONS::state_vector_t;
  using state_vector_array_t = typename DIMENSIONS::state_vector_array_t;
  using input_vector_t = typename DIMENSIONS::input_vector_t;

  using dynamic_vector_t = Eigen::Matrix<scalar_t, Eigen::Dynamic, 1>;
  using dynamic_vector_array_t = std::vector<dynamic_vector_t, Eigen::aligned_allocator<dynamic_vector_t>>;

 public:
  /*
   * Constructor
   */
  OverallReference() = default;

  /*
   * Constructor
   *
   * @param [in] trajTimes: list of times at which the reference is defined
   * @param [in] trajState: list of waypoints at which the reference is defined
   */
  OverallReference(std::vector<scalar_t> trajTimes, std::vector<state_vector_t> trajState);

  /*
   * Calculate the input at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @param [out] input: input corresponding to time
   */
  void getInput(scalar_t time, input_vector_t& input);

  /*
   * Calculate the reference state at a certain time
   *
   * @param [in] time: time moment at which the input is calculated
   * @param [out] state: state corresponding to time
   */
  void getState(scalar_t time, state_vector_t& x);

  /*
   * Calculate the reference state at a certain time and mode
   *
   * @param [in] idx: mode at which the input is calculated
   * @param [in] time: time moment at which the input is calculated
   * @param [out] state: state corresponding to time and mode
   */
  void getState(int idx, scalar_t time, state_vector_t& x);

  /*
   * Extend the reference past the event times, by integrating the input signal
   * without applying the jump map
   *
   * @param [in] delta: length of the extensions
   */
  void extendref(scalar_t delta);

  /*
   * Display the reference
   *
   * @param [in] i: choice of which section of the reference is displayed
   */
  void display(int i);

 private:
  /*
   * Find the index of the currently active reference
   *
   * @param [in] time: time moment at which the index is requested
   *
   * @return currently active index
   */
  int getIndex(scalar_t time);

  /*
   * Jump map of the system
   *
   * @param [in] x: State before event
   *
   * @return currently active index
   */
  void jumpMap(state_vector_t& x);

  std::vector<Reference> References_;
  std::vector<scalar_t> switchtimes_;
};
