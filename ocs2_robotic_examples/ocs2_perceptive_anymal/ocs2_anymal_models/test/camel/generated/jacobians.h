#ifndef CAMEL_JACOBIANS_H_
#define CAMEL_JACOBIANS_H_

		#include <iit/rbd/rbd.h>
#include <iit/rbd/TransformsBase.h>
#include <iit/rbd/traits/DoubleTrait.h>
#include "declarations.h"
#include "kinematics_parameters.h"

namespace iit {
namespace camel {

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

        class Type_fr_base_J_fr_LF_FOOT : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LF_FOOT>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_FOOT();
            const Type_fr_base_J_fr_LF_FOOT& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_FOOT : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LH_FOOT>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_FOOT();
            const Type_fr_base_J_fr_LH_FOOT& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_FOOT : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RF_FOOT>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_FOOT();
            const Type_fr_base_J_fr_RF_FOOT& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_FOOT : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RH_FOOT>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_FOOT();
            const Type_fr_base_J_fr_RH_FOOT& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_base_J_fr_LF_FOOT fr_base_J_fr_LF_FOOT;
        Type_fr_base_J_fr_LH_FOOT fr_base_J_fr_LH_FOOT;
        Type_fr_base_J_fr_RF_FOOT fr_base_J_fr_RF_FOOT;
        Type_fr_base_J_fr_RH_FOOT fr_base_J_fr_RH_FOOT;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
