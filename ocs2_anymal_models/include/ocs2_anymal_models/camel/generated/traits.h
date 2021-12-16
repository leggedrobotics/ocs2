#ifndef IIT_ROBOGEN__CAMEL_TRAITS_H_
#define IIT_ROBOGEN__CAMEL_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace camel {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename camel::JointIdentifiers JointID;
    typedef typename camel::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename camel::tpl::JointState<SCALAR> JointState;



    typedef typename camel::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename camel::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename camel::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename camel::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::camel::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::camel::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::camel::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::camel::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = camel::jointsCount;
    static const int links_count  = camel::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return camel::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return camel::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
