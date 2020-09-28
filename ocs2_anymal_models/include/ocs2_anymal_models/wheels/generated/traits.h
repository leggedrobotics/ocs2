#ifndef IIT_ROBOGEN__WHEELS_TRAITS_H_
#define IIT_ROBOGEN__WHEELS_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace wheels {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename wheels::JointIdentifiers JointID;
    typedef typename wheels::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename wheels::tpl::JointState<SCALAR> JointState;



    typedef typename wheels::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename wheels::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename wheels::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename wheels::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::wheels::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::wheels::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::wheels::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::wheels::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = wheels::jointsCount;
    static const int links_count  = wheels::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return wheels::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return wheels::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
