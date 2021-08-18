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
namespace chip {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename chip::JointIdentifiers JointID;
    typedef typename chip::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename chip::tpl::JointState<SCALAR> JointState;



    typedef typename chip::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename chip::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename chip::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename chip::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::chip::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::chip::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::chip::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::chip::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = chip::jointsCount;
    static const int links_count  = chip::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return chip::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return chip::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
