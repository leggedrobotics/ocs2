#ifndef IIT_ROBCOGEN__CAMEL_MISCELLANEOUS_H_
#define IIT_ROBCOGEN__CAMEL_MISCELLANEOUS_H_

#include "inertia_properties.h"
#include "transforms.h"
#include <iit/rbd/traits/TraitSelector.h>

namespace iit {
namespace camel {

/** \name Center of mass calculation
 * Computes the Center Of Mass (COM) position of the whole robot, in
 * base coordinates.
 *
 * Common parameters are the inertia properties of the robot and the set
 * of homogeneous coordinate transforms. If a joint status variable is
 * also passed, then the transforms are updated accordingly; otherwise,
 * they are not modified before being used.
 */
///@{
/**
 * \param inertia the inertia properties of the links of the robot
 * \param transforms the homogeneous coordinate transforms of the robot
 * \return the position of the Center Of Mass of the whole robot, expressed
 *         in base coordinates
 */
template<typename SCALAR_T>
typename iit::rbd::Core<SCALAR_T>::Vector3 getWholeBodyCOM(
    const dyn::tpl::InertiaProperties<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait>& inertia,
    const tpl::HomogeneousTransforms<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait>& transforms);
/**
 * \param inertia the inertia properties of the links of the robot
 * \param q the joint status vector describing the configuration of the robot
 * \param transforms the homogeneous coordinate transforms of the robot
 * \return the position of the Center Of Mass of the whole robot, expressed
 *         in base coordinates
 */
template<typename SCALAR_T>
typename iit::rbd::Core<SCALAR_T>::Vector3 getWholeBodyCOM(
    const dyn::tpl::InertiaProperties<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait>& inertia,
    const tpl::JointState<SCALAR_T>& q,
    tpl::HomogeneousTransforms<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait>& transforms);
///@}

}
}

#include "miscellaneous.cpp"

#endif
