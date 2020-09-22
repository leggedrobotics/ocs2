#ifndef IIT_ROBOGEN__BEAR_TRAITS_H_
#define IIT_ROBOGEN__BEAR_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace bear {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename bear::JointIdentifiers JointID;
    typedef typename bear::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename bear::tpl::JointState<SCALAR> JointState;



    typedef typename bear::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename bear::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename bear::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename bear::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::bear::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::bear::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::bear::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::bear::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = bear::jointsCount;
    static const int links_count  = bear::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return bear::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return bear::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
