#ifndef IIT_ROBOGEN__ANYMAL_TRAITS_H_
#define IIT_ROBOGEN__ANYMAL_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace ANYmal {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename ANYmal::JointIdentifiers JointID;
    typedef typename ANYmal::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename ANYmal::tpl::JointState<SCALAR> JointState;



    typedef typename ANYmal::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename ANYmal::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename ANYmal::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename ANYmal::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::ANYmal::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::ANYmal::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::ANYmal::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::ANYmal::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = ANYmal::jointsCount;
    static const int links_count  = ANYmal::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return ANYmal::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return ANYmal::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
