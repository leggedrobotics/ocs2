#ifndef IIT_ROBOGEN__CHIMERA_TRAITS_H_
#define IIT_ROBOGEN__CHIMERA_TRAITS_H_

#include "declarations.h"
#include "transforms.h"
#include "inverse_dynamics.h"
#include "forward_dynamics.h"
#include "jsim.h"
#include "inertia_properties.h"
#include "jacobians.h"
#include <iit/rbd/traits/TraitSelector.h>


namespace iit {
namespace chimera {

namespace tpl {

template <typename SCALAR>
struct Traits {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef SCALAR S;

    typedef typename chimera::JointIdentifiers JointID;
    typedef typename chimera::LinkIdentifiers  LinkID;
    typedef typename iit::rbd::tpl::TraitSelector<SCALAR>::Trait Trait;

    typedef typename chimera::tpl::JointState<SCALAR> JointState;



    typedef typename chimera::tpl::HomogeneousTransforms<Trait> HomogeneousTransforms;
    typedef typename chimera::tpl::MotionTransforms<Trait> MotionTransforms;
    typedef typename chimera::tpl::ForceTransforms<Trait> ForceTransforms;
    typedef typename chimera::tpl::Jacobians<Trait> Jacobians;

    typedef typename iit::chimera::dyn::tpl::InertiaProperties<Trait> InertiaProperties;
    typedef typename iit::chimera::dyn::tpl::ForwardDynamics<Trait> FwdDynEngine;
    typedef typename iit::chimera::dyn::tpl::InverseDynamics<Trait> InvDynEngine;
    typedef typename iit::chimera::dyn::tpl::JSIM<Trait> JSIM;

    static const int joints_count = chimera::jointsCount;
    static const int links_count  = chimera::linksCount;
    static const bool floating_base = true;

    static inline const JointID* orderedJointIDs();
    static inline const LinkID*  orderedLinkIDs();
};

template <typename SCALAR>
inline const typename Traits<SCALAR>::JointID*  Traits<SCALAR>::orderedJointIDs() {
    return chimera::orderedJointIDs;
}
template <typename SCALAR>
inline const typename Traits<SCALAR>::LinkID*  Traits<SCALAR>::orderedLinkIDs() {
    return chimera::orderedLinkIDs;
}

}

typedef tpl::Traits<double> Traits; // default instantiation - backward compatibility...

}
}

#endif
