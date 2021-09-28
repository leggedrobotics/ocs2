#ifndef IIT_ROBOGEN__BALLBOT_TRAITS_H_
#define IIT_ROBOGEN__BALLBOT_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace Ballbot {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename Ballbot::JointIdentifiers JointID;
    typedef typename Ballbot::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename Ballbot::tpl::JointState<SCALAR> JointState;



    typedef typename Ballbot::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename Ballbot::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename Ballbot::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename Ballbot::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::Ballbot::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::Ballbot::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::Ballbot::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::Ballbot::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = Ballbot::jointsCount;
    static const int links_count  = Ballbot::linksCount;
    static const bool floating_base = false;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return Ballbot::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return Ballbot::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
