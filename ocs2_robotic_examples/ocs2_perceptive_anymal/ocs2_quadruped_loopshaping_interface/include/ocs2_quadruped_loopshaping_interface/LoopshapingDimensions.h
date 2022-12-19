//
// Created by rgrandia on 18.02.20.
//

#pragma once

#include <Eigen/Dense>

#include <ocs2_core/automatic_differentiation/CppAdInterface.h>

namespace switched_model_loopshaping {

static constexpr size_t STATE_DIM = 48;
static constexpr size_t INPUT_DIM = 24;
static constexpr size_t SYSTEM_STATE_DIM = 24;
static constexpr size_t SYSTEM_INPUT_DIM = 24;
static constexpr size_t FILTER_STATE_DIM = 24;
static constexpr size_t FILTER_INPUT_DIM = 24;

using scalar_t = ocs2::scalar_t;
using ad_scalar_t = ocs2::ad_scalar_t;
using scalar_array_t = ocs2::scalar_array_t;
using vector_t = ocs2::vector_t;
using vector_array_t = ocs2::vector_array_t;

template <typename SCALAR_T>
using filter_state_s_t = Eigen::Matrix<SCALAR_T, FILTER_STATE_DIM, 1>;
using filter_state_t = filter_state_s_t<scalar_t>;
using filter_state_ad_t = filter_state_s_t<ad_scalar_t>;

template <typename SCALAR_T>
using filter_input_s_t = Eigen::Matrix<SCALAR_T, FILTER_INPUT_DIM, 1>;
using filter_input_t = filter_input_s_t<scalar_t>;
using filter_input_ad_t = filter_input_s_t<ad_scalar_t>;

}  // namespace switched_model_loopshaping
