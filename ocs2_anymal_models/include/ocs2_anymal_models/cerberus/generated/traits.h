#ifndef IIT_ROBOGEN__CERBERUS_TRAITS_H_
#define IIT_ROBOGEN__CERBERUS_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace cerberus {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename cerberus::JointIdentifiers JointID;
    typedef typename cerberus::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename cerberus::tpl::JointState<SCALAR> JointState;



    typedef typename cerberus::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename cerberus::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename cerberus::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename cerberus::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::cerberus::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::cerberus::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::cerberus::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::cerberus::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = cerberus::jointsCount;
    static const int links_count  = cerberus::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return cerberus::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return cerberus::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
