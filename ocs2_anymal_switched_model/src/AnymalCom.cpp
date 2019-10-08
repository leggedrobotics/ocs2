/*
 * AnymalCom.h
 *
 *  Created on: Nov, 2018
 *      Author: farbod
 */

#include "ocs2_anymal_switched_model/dynamics/implementation/AnymalCom.h"

// Explicit instantiation
template class anymal::tpl::AnymalCom<double>;
template class anymal::tpl::AnymalCom<ocs2::CppAdInterface<double>::ad_scalar_t>;

