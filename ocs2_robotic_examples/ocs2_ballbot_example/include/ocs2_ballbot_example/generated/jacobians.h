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

        class Type_fr_world_J_fr_dummy_ball1 : public JacobianT<Scalar, 1, Type_fr_world_J_fr_dummy_ball1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_dummy_ball1();
            const Type_fr_world_J_fr_dummy_ball1& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_ball : public JacobianT<Scalar, 2, Type_fr_world_J_fr_ball>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_ball();
            const Type_fr_world_J_fr_ball& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_dummy_base1 : public JacobianT<Scalar, 3, Type_fr_world_J_fr_dummy_base1>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_dummy_base1();
            const Type_fr_world_J_fr_dummy_base1& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_dummy_base2 : public JacobianT<Scalar, 4, Type_fr_world_J_fr_dummy_base2>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_dummy_base2();
            const Type_fr_world_J_fr_dummy_base2& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_base : public JacobianT<Scalar, 5, Type_fr_world_J_fr_base>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_base();
            const Type_fr_world_J_fr_base& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_ball_COM : public JacobianT<Scalar, 2, Type_fr_world_J_fr_ball_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_ball_COM();
            const Type_fr_world_J_fr_ball_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_base_COM : public JacobianT<Scalar, 5, Type_fr_world_J_fr_base_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_base_COM();
            const Type_fr_world_J_fr_base_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_dummy_ball1_COM : public JacobianT<Scalar, 1, Type_fr_world_J_fr_dummy_ball1_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_dummy_ball1_COM();
            const Type_fr_world_J_fr_dummy_ball1_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_dummy_base1_COM : public JacobianT<Scalar, 3, Type_fr_world_J_fr_dummy_base1_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_dummy_base1_COM();
            const Type_fr_world_J_fr_dummy_base1_COM& update(const JState&);
        protected:
        };
        
        class Type_fr_world_J_fr_dummy_base2_COM : public JacobianT<Scalar, 4, Type_fr_world_J_fr_dummy_base2_COM>
        {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
            Type_fr_world_J_fr_dummy_base2_COM();
            const Type_fr_world_J_fr_dummy_base2_COM& update(const JState&);
        protected:
        };
        
    public:
        Jacobians();
        void updateParameters();
    public:
        Type_fr_world_J_fr_dummy_ball1 fr_world_J_fr_dummy_ball1;
        Type_fr_world_J_fr_ball fr_world_J_fr_ball;
        Type_fr_world_J_fr_dummy_base1 fr_world_J_fr_dummy_base1;
        Type_fr_world_J_fr_dummy_base2 fr_world_J_fr_dummy_base2;
        Type_fr_world_J_fr_base fr_world_J_fr_base;
        Type_fr_world_J_fr_ball_COM fr_world_J_fr_ball_COM;
        Type_fr_world_J_fr_base_COM fr_world_J_fr_base_COM;
        Type_fr_world_J_fr_dummy_ball1_COM fr_world_J_fr_dummy_ball1_COM;
        Type_fr_world_J_fr_dummy_base1_COM fr_world_J_fr_dummy_base1_COM;
        Type_fr_world_J_fr_dummy_base2_COM fr_world_J_fr_dummy_base2_COM;

    protected:

};

} //namespace tpl

using Jacobians = tpl::Jacobians<rbd::DoubleTrait>;

#include "jacobians.impl.h"


}
}

#endif
