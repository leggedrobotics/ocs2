#ifndef BALLBOT_JACOBIANS_H_
#define BALLBOT_JACOBIANS_H_

		#include <iit/rbd/rbd.h>
#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace Ballbot {

template<typename SCALAR, int COLS, class M>
class JacobianT : public iit::rbd::JacobianBase<tpl::JointState<SCALAR>, COLS, M>
{};

namespace tpl {

/**
 *
 */
template <typename TRAIT>
class Jacobians {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef typename TRAIT::Scalar Scalar;
        typedef iit::rbd::Core<Scalar> CoreS;

        typedef JointState<Scalar> JState;

        class Type_fr_base0_J_fr_control : public JacobianT<Scalar, 5, Type_fr_base0_J_fr_control>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base0_J_fr_control();
            const Type_fr_base0_J_fr_control& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_base0_J_fr_control fr_base0_J_fr_control;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
