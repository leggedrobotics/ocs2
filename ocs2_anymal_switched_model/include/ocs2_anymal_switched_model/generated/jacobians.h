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

        class Type_fr_base_J_fr_LF_HIP : public JacobianT<Scalar, 1, Type_fr_base_J_fr_LF_HIP>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_HIP();
            const Type_fr_base_J_fr_LF_HIP& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LF_THIGH : public JacobianT<Scalar, 2, Type_fr_base_J_fr_LF_THIGH>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_THIGH();
            const Type_fr_base_J_fr_LF_THIGH& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LF_SHANK : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LF_SHANK>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_SHANK();
            const Type_fr_base_J_fr_LF_SHANK& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_HIP : public JacobianT<Scalar, 1, Type_fr_base_J_fr_RF_HIP>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_HIP();
            const Type_fr_base_J_fr_RF_HIP& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_THIGH : public JacobianT<Scalar, 2, Type_fr_base_J_fr_RF_THIGH>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_THIGH();
            const Type_fr_base_J_fr_RF_THIGH& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_SHANK : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RF_SHANK>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_SHANK();
            const Type_fr_base_J_fr_RF_SHANK& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_HIP : public JacobianT<Scalar, 1, Type_fr_base_J_fr_LH_HIP>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_HIP();
            const Type_fr_base_J_fr_LH_HIP& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_THIGH : public JacobianT<Scalar, 2, Type_fr_base_J_fr_LH_THIGH>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_THIGH();
            const Type_fr_base_J_fr_LH_THIGH& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_SHANK : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LH_SHANK>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_SHANK();
            const Type_fr_base_J_fr_LH_SHANK& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_HIP : public JacobianT<Scalar, 1, Type_fr_base_J_fr_RH_HIP>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_HIP();
            const Type_fr_base_J_fr_RH_HIP& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_THIGH : public JacobianT<Scalar, 2, Type_fr_base_J_fr_RH_THIGH>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_THIGH();
            const Type_fr_base_J_fr_RH_THIGH& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_SHANK : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RH_SHANK>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_SHANK();
            const Type_fr_base_J_fr_RH_SHANK& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LF_ADAPTER : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LF_ADAPTER>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_ADAPTER();
            const Type_fr_base_J_fr_LF_ADAPTER& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LF_FOOT : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LF_FOOT>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_FOOT();
            const Type_fr_base_J_fr_LF_FOOT& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LF_HIP_COM : public JacobianT<Scalar, 1, Type_fr_base_J_fr_LF_HIP_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_HIP_COM();
            const Type_fr_base_J_fr_LF_HIP_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LF_SHANK_COM : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LF_SHANK_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_SHANK_COM();
            const Type_fr_base_J_fr_LF_SHANK_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LF_THIGH_COM : public JacobianT<Scalar, 2, Type_fr_base_J_fr_LF_THIGH_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LF_THIGH_COM();
            const Type_fr_base_J_fr_LF_THIGH_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_ADAPTER : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LH_ADAPTER>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_ADAPTER();
            const Type_fr_base_J_fr_LH_ADAPTER& update(const JState&);
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
        
        class Type_fr_base_J_fr_LH_HIP_COM : public JacobianT<Scalar, 1, Type_fr_base_J_fr_LH_HIP_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_HIP_COM();
            const Type_fr_base_J_fr_LH_HIP_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_SHANK_COM : public JacobianT<Scalar, 3, Type_fr_base_J_fr_LH_SHANK_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_SHANK_COM();
            const Type_fr_base_J_fr_LH_SHANK_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_LH_THIGH_COM : public JacobianT<Scalar, 2, Type_fr_base_J_fr_LH_THIGH_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_LH_THIGH_COM();
            const Type_fr_base_J_fr_LH_THIGH_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_ADAPTER : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RF_ADAPTER>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_ADAPTER();
            const Type_fr_base_J_fr_RF_ADAPTER& update(const JState&);
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
        
        class Type_fr_base_J_fr_RF_HIP_COM : public JacobianT<Scalar, 1, Type_fr_base_J_fr_RF_HIP_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_HIP_COM();
            const Type_fr_base_J_fr_RF_HIP_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_SHANK_COM : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RF_SHANK_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_SHANK_COM();
            const Type_fr_base_J_fr_RF_SHANK_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RF_THIGH_COM : public JacobianT<Scalar, 2, Type_fr_base_J_fr_RF_THIGH_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RF_THIGH_COM();
            const Type_fr_base_J_fr_RF_THIGH_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_ADAPTER : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RH_ADAPTER>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_ADAPTER();
            const Type_fr_base_J_fr_RH_ADAPTER& update(const JState&);
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
        
        class Type_fr_base_J_fr_RH_HIP_COM : public JacobianT<Scalar, 1, Type_fr_base_J_fr_RH_HIP_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_HIP_COM();
            const Type_fr_base_J_fr_RH_HIP_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_SHANK_COM : public JacobianT<Scalar, 3, Type_fr_base_J_fr_RH_SHANK_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_SHANK_COM();
            const Type_fr_base_J_fr_RH_SHANK_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_base_J_fr_RH_THIGH_COM : public JacobianT<Scalar, 2, Type_fr_base_J_fr_RH_THIGH_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_base_J_fr_RH_THIGH_COM();
            const Type_fr_base_J_fr_RH_THIGH_COM& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_base_J_fr_LF_HIP fr_base_J_fr_LF_HIP;
        Type_fr_base_J_fr_LF_THIGH fr_base_J_fr_LF_THIGH;
        Type_fr_base_J_fr_LF_SHANK fr_base_J_fr_LF_SHANK;
        Type_fr_base_J_fr_RF_HIP fr_base_J_fr_RF_HIP;
        Type_fr_base_J_fr_RF_THIGH fr_base_J_fr_RF_THIGH;
        Type_fr_base_J_fr_RF_SHANK fr_base_J_fr_RF_SHANK;
        Type_fr_base_J_fr_LH_HIP fr_base_J_fr_LH_HIP;
        Type_fr_base_J_fr_LH_THIGH fr_base_J_fr_LH_THIGH;
        Type_fr_base_J_fr_LH_SHANK fr_base_J_fr_LH_SHANK;
        Type_fr_base_J_fr_RH_HIP fr_base_J_fr_RH_HIP;
        Type_fr_base_J_fr_RH_THIGH fr_base_J_fr_RH_THIGH;
        Type_fr_base_J_fr_RH_SHANK fr_base_J_fr_RH_SHANK;
        Type_fr_base_J_fr_LF_ADAPTER fr_base_J_fr_LF_ADAPTER;
        Type_fr_base_J_fr_LF_FOOT fr_base_J_fr_LF_FOOT;
        Type_fr_base_J_fr_LF_HIP_COM fr_base_J_fr_LF_HIP_COM;
        Type_fr_base_J_fr_LF_SHANK_COM fr_base_J_fr_LF_SHANK_COM;
        Type_fr_base_J_fr_LF_THIGH_COM fr_base_J_fr_LF_THIGH_COM;
        Type_fr_base_J_fr_LH_ADAPTER fr_base_J_fr_LH_ADAPTER;
        Type_fr_base_J_fr_LH_FOOT fr_base_J_fr_LH_FOOT;
        Type_fr_base_J_fr_LH_HIP_COM fr_base_J_fr_LH_HIP_COM;
        Type_fr_base_J_fr_LH_SHANK_COM fr_base_J_fr_LH_SHANK_COM;
        Type_fr_base_J_fr_LH_THIGH_COM fr_base_J_fr_LH_THIGH_COM;
        Type_fr_base_J_fr_RF_ADAPTER fr_base_J_fr_RF_ADAPTER;
        Type_fr_base_J_fr_RF_FOOT fr_base_J_fr_RF_FOOT;
        Type_fr_base_J_fr_RF_HIP_COM fr_base_J_fr_RF_HIP_COM;
        Type_fr_base_J_fr_RF_SHANK_COM fr_base_J_fr_RF_SHANK_COM;
        Type_fr_base_J_fr_RF_THIGH_COM fr_base_J_fr_RF_THIGH_COM;
        Type_fr_base_J_fr_RH_ADAPTER fr_base_J_fr_RH_ADAPTER;
        Type_fr_base_J_fr_RH_FOOT fr_base_J_fr_RH_FOOT;
        Type_fr_base_J_fr_RH_HIP_COM fr_base_J_fr_RH_HIP_COM;
        Type_fr_base_J_fr_RH_SHANK_COM fr_base_J_fr_RH_SHANK_COM;
        Type_fr_base_J_fr_RH_THIGH_COM fr_base_J_fr_RH_THIGH_COM;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
