#ifndef IIT_ROBOT_BALLBOT_DECLARATIONS_H_
#define IIT_ROBOT_BALLBOT_DECLARATIONS_H_

#include <iit/rbd/rbd.h>

namespace iit {
namespace Ballbot {

static const int JointSpaceDimension = 5;
static const int jointsCount = 5;
/** The total number of rigid bodies of this robot, including the base */
static const int linksCount  = 6;

namespace tpl {
template <typename SCALAR>
using Column5d = iit::rbd::PlainMatrix<SCALAR, 5, 1>;

template <typename SCALAR>
using JointState = Column5d<SCALAR>;
}

using Column5d = tpl::Column5d<double>;
typedef Column5d JointState;

enum JointIdentifiers {
    JBALL_X = 0
    , JBALL_Y
    , JBASE_Z
    , JBASE_Y
    , JBASE_X
};

enum LinkIdentifiers {
    WORLD = 0
    , DUMMY_BALL1
    , BALL
    , DUMMY_BASE1
    , DUMMY_BASE2
    , BASE
};

static const JointIdentifiers orderedJointIDs[jointsCount] =
    {JBALL_X,JBALL_Y,JBASE_Z,JBASE_Y,JBASE_X};

static const LinkIdentifiers orderedLinkIDs[linksCount] =
    {WORLD,DUMMY_BALL1,BALL,DUMMY_BASE1,DUMMY_BASE2,BASE};

}
}
#endif
