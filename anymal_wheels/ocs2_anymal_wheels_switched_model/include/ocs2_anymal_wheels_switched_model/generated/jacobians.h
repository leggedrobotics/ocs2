#ifndef ANYMAL_JACOBIANS_H_
#define ANYMAL_JACOBIANS_H_

		#include <iit/rbd/rbd.h>
#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace ANYmal {

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

        class Type_fr_base_J_fr_LF_WHEEL_L : public JacobianT<Scalar, 4, Type_fr_base_J_fr_LF_WHEEL_L>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_WHEEL_L();
            const Type_fr_base_J_fr_LF_WHEEL_L& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_WHEEL_L : public JacobianT<Scalar, 4, Type_fr_base_J_fr_LH_WHEEL_L>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_WHEEL_L();
            const Type_fr_base_J_fr_LH_WHEEL_L& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_WHEEL_L : public JacobianT<Scalar, 4, Type_fr_base_J_fr_RF_WHEEL_L>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_WHEEL_L();
            const Type_fr_base_J_fr_RF_WHEEL_L& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_WHEEL_L : public JacobianT<Scalar, 4, Type_fr_base_J_fr_RH_WHEEL_L>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_WHEEL_L();
            const Type_fr_base_J_fr_RH_WHEEL_L& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_base_J_fr_LF_WHEEL_L fr_base_J_fr_LF_WHEEL_L;
        Type_fr_base_J_fr_LH_WHEEL_L fr_base_J_fr_LH_WHEEL_L;
        Type_fr_base_J_fr_RF_WHEEL_L fr_base_J_fr_RF_WHEEL_L;
        Type_fr_base_J_fr_RH_WHEEL_L fr_base_J_fr_RH_WHEEL_L;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
