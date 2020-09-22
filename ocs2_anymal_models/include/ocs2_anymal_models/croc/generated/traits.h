#ifndef IIT_ROBOGEN__CROC_TRAITS_H_
#define IIT_ROBOGEN__CROC_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace croc {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename croc::JointIdentifiers JointID;
    typedef typename croc::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename croc::tpl::JointState<SCALAR> JointState;



    typedef typename croc::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename croc::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename croc::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename croc::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::croc::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::croc::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::croc::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::croc::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = croc::jointsCount;
    static const int links_count  = croc::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return croc::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return croc::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
