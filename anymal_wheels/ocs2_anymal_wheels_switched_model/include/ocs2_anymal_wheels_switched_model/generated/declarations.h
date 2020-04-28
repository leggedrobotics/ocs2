#ifndef IIT_ROBOT_ANYMAL_DECLARATIONS_H_
#define IIT_ROBOT_ANYMAL_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace ANYmal {

static const int JointSpaceDimension = 16;
static const int jointsCount = 16;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 17;

namespace tpl {
template <typename SCALAR>
using Column16d = iit::rbd::PlainMatrix<SCALAR, 16, 1>;

template <typename SCALAR>
using JointState = Column16d<SCALAR>;
}

using Column16d = tpl::Column16d<double>;
typedef Column16d JointState;

enum JointIdentifiers {
    LF_HAA = 0
    , LF_HFE
    , LF_KFE
    , LF_WHEEL
    , RF_HAA
    , RF_HFE
    , RF_KFE
    , RF_WHEEL
    , LH_HAA
    , LH_HFE
    , LH_KFE
    , LH_WHEEL
    , RH_HAA
    , RH_HFE
    , RH_KFE
    , RH_WHEEL
};

enum LinkIdentifiers {
    BASE = 0
    , LF_HIP
    , LF_THIGH
    , LF_SHANK
    , LF_WHEEL_L
    , RF_HIP
    , RF_THIGH
    , RF_SHANK
    , RF_WHEEL_L
    , LH_HIP
    , LH_THIGH
    , LH_SHANK
    , LH_WHEEL_L
    , RH_HIP
    , RH_THIGH
    , RH_SHANK
    , RH_WHEEL_L
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {LF_HAA,LF_HFE,LF_KFE,LF_WHEEL,RF_HAA,RF_HFE,RF_KFE,RF_WHEEL,LH_HAA,LH_HFE,LH_KFE,LH_WHEEL,RH_HAA,RH_HFE,RH_KFE,RH_WHEEL};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,LF_HIP,LF_THIGH,LF_SHANK,LF_WHEEL_L,RF_HIP,RF_THIGH,RF_SHANK,RF_WHEEL_L,LH_HIP,LH_THIGH,LH_SHANK,LH_WHEEL_L,RH_HIP,RH_THIGH,RH_SHANK,RH_WHEEL_L};

}
}
#endif
