#ifndef IIT_ROBOT_CAMEL_DECLARATIONS_H_
#define IIT_ROBOT_CAMEL_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace camel {

static const int JointSpaceDimension = 12;
static const int jointsCount = 12;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 13;

namespace tpl {
template <typename SCALAR>
using Column12d = iit::rbd::PlainMatrix<SCALAR, 12, 1>;

template <typename SCALAR>
using JointState = Column12d<SCALAR>;
}

using Column12d = tpl::Column12d<double>;
typedef Column12d JointState;

enum JointIdentifiers {
    LF_HAA = 0
    , LF_HFE
    , LF_KFE
    , RF_HAA
    , RF_HFE
    , RF_KFE
    , LH_HAA
    , LH_HFE
    , LH_KFE
    , RH_HAA
    , RH_HFE
    , RH_KFE
};

enum LinkIdentifiers {
    BASE = 0
    , LF_HIP
    , LF_THIGH
    , LF_SHANK
    , RF_HIP
    , RF_THIGH
    , RF_SHANK
    , LH_HIP
    , LH_THIGH
    , LH_SHANK
    , RH_HIP
    , RH_THIGH
    , RH_SHANK
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {LF_HAA,LF_HFE,LF_KFE,RF_HAA,RF_HFE,RF_KFE,LH_HAA,LH_HFE,LH_KFE,RH_HAA,RH_HFE,RH_KFE};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {BASE,LF_HIP,LF_THIGH,LF_SHANK,RF_HIP,RF_THIGH,RF_SHANK,LH_HIP,LH_THIGH,LH_SHANK,RH_HIP,RH_THIGH,RH_SHANK};

}
}
#endif
