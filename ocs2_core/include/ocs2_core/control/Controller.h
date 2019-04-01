#pragma once

#include <Eigen/Dense>

#include "ocs2_core/Dimensions.h"

namespace ocs2 {

/**
  * The base class for all controllers.
  *
 * @tparam STATE_DIM: Dimension of the state space.
 * @tparam INPUT_DIM: Dimension of the control input space.
 */
//TODO(jcarius) do we need the LOGIC_RULES_T template?

template <size_t STATE_DIM, size_t INPUT_DIM>
class Controller
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  using dimensions_t = Dimensions<STATE_DIM,INPUT_DIM>;
  using scalar_t = typename dimensions_t::scalar_t;
  using state_vector_t = typename dimensions_t::state_vector_t;
  using input_vector_t = typename dimensions_t::input_vector_t;

  /**
   * Default constructor.
   */
  Controller() = default;

  /**
   * Default destructor.
   */
  virtual ~Controller() = default;

  /**
   * Computes the control command at a given time and state.
   *
   * @param [in] t: Current time.
   * @param [in] x: Current state.
   * @return Current input.
   */
  virtual input_vector_t computeInput(
                  const scalar_t& t,
                  const state_vector_t& x) = 0;
};

}
