//
// Created by rgrandia on 18.09.19.
//

#include "ocs2_anymal_switched_model/core/implementation/AnymalKinematics.h"

// Explicit instantiation
template class anymal::tpl::AnymalKinematics<double>;
template class anymal::tpl::AnymalKinematics<ocs2::CppAdInterface<double>::ad_scalar_t>;