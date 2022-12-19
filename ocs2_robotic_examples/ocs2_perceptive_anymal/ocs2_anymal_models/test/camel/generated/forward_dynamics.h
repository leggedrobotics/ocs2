#ifndef IIT_ROBOT_CAMEL_FORWARD_DYNAMICS_H_
#define IIT_ROBOT_CAMEL_FORWARD_DYNAMICS_H_

#include <iit/rbd/rbd.h>
#include <iit/rbd/utils.h>
#include <iit/rbd/robcogen_commons.h>

#include "declarations.h"
#include "transforms.h"
#include "inertia_properties.h"
#include "link_data_map.h"

namespace iit {
namespace camel {
namespace dyn {

/**
 * The Forward Dynamics routine for the robot camel.
 *
 * The parameters common to most of the methods are the joint status \c q, the
 * joint velocities \c qd and the joint forces \c tau. The accelerations \c qdd
 * will be filled with the computed values. Overloaded methods without the \c q
 * parameter use the current configuration of the robot; they are provided for
 * the sake of efficiency, in case the kinematics transforms of the robot have
 * already been updated elsewhere with the most recent configuration (eg by a
 * call to setJointStatus()), so that it would be useless to compute them again.
 */

namespace tpl {

template <typename TRAIT>
class ForwardDynamics {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Convenient type aliases:

    typedef typename TRAIT::Scalar Scalar;

    typedef iit::rbd::Core<Scalar> CoreS;

    typedef typename CoreS::ForceVector Force;
    typedef typename CoreS::VelocityVector Velocity;
    typedef typename CoreS::VelocityVector Acceleration;
    typedef typename CoreS::Column6D Column6DS;
    typedef typename CoreS::Matrix66 Matrix66S;
    typedef LinkDataMap<Force> ExtForces;
    typedef typename iit::camel::tpl::JointState<Scalar> JointState;
    typedef iit::camel::tpl::MotionTransforms<TRAIT> MTransforms;

public:
    /**
     * Default constructor
     * \param in the inertia properties of the links
     * \param tr the container of all the spatial motion transforms of
     *     the robot camel, which will be used by this instance
     *     to compute the dynamics.
     */
    ForwardDynamics(InertiaProperties<TRAIT>& in, MTransforms& tr);
    /** \name Forward dynamics
     * The Articulated-Body-Algorithm to compute the joint accelerations
     */ ///@{
    /**
     * \param qdd the joint accelerations vector (output parameter).
     * \param base_a
     * \param base_v
     * \param g the gravity acceleration vector, expressed in the
     *          base coordinates
     * \param q the joint status vector
     * \param qd the joint velocities vector
     * \param tau the joint forces (torque or force)
     * \param fext the external forces, optional. Each force must be
     *              expressed in the reference frame of the link it is
     *              exerted on.
     */
    void fd(
       JointState& qdd, Acceleration& base_a, // output parameters,
       const Velocity& base_v, const Acceleration& g,
       const JointState& q, const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    void fd(
        JointState& qdd, Acceleration& base_a, // output parameters,
        const Velocity& base_v, const Acceleration& g,
        const JointState& qd, const JointState& tau, const ExtForces& fext = zeroExtForces);
    ///@}

    /** Updates all the kinematics transforms used by this instance. */
    void setJointStatus(const JointState& q) const;

private:
    InertiaProperties<TRAIT>* inertiaProps;
    MTransforms* motionTransforms;

    Matrix66S vcross; // support variable
    Matrix66S Ia_r;   // support variable, articulated inertia in the case of a revolute joint
    // Link 'base'
    Matrix66S base_AI;
    Force base_p;

    // Link 'LF_HIP' :
    Matrix66S LF_HIP_AI;
    Velocity LF_HIP_a;
    Velocity LF_HIP_v;
    Velocity LF_HIP_c;
    Force    LF_HIP_p;

    Column6DS LF_HIP_U;
    Scalar LF_HIP_D;
    Scalar LF_HIP_u;
    // Link 'LF_THIGH' :
    Matrix66S LF_THIGH_AI;
    Velocity LF_THIGH_a;
    Velocity LF_THIGH_v;
    Velocity LF_THIGH_c;
    Force    LF_THIGH_p;

    Column6DS LF_THIGH_U;
    Scalar LF_THIGH_D;
    Scalar LF_THIGH_u;
    // Link 'LF_SHANK' :
    Matrix66S LF_SHANK_AI;
    Velocity LF_SHANK_a;
    Velocity LF_SHANK_v;
    Velocity LF_SHANK_c;
    Force    LF_SHANK_p;

    Column6DS LF_SHANK_U;
    Scalar LF_SHANK_D;
    Scalar LF_SHANK_u;
    // Link 'RF_HIP' :
    Matrix66S RF_HIP_AI;
    Velocity RF_HIP_a;
    Velocity RF_HIP_v;
    Velocity RF_HIP_c;
    Force    RF_HIP_p;

    Column6DS RF_HIP_U;
    Scalar RF_HIP_D;
    Scalar RF_HIP_u;
    // Link 'RF_THIGH' :
    Matrix66S RF_THIGH_AI;
    Velocity RF_THIGH_a;
    Velocity RF_THIGH_v;
    Velocity RF_THIGH_c;
    Force    RF_THIGH_p;

    Column6DS RF_THIGH_U;
    Scalar RF_THIGH_D;
    Scalar RF_THIGH_u;
    // Link 'RF_SHANK' :
    Matrix66S RF_SHANK_AI;
    Velocity RF_SHANK_a;
    Velocity RF_SHANK_v;
    Velocity RF_SHANK_c;
    Force    RF_SHANK_p;

    Column6DS RF_SHANK_U;
    Scalar RF_SHANK_D;
    Scalar RF_SHANK_u;
    // Link 'LH_HIP' :
    Matrix66S LH_HIP_AI;
    Velocity LH_HIP_a;
    Velocity LH_HIP_v;
    Velocity LH_HIP_c;
    Force    LH_HIP_p;

    Column6DS LH_HIP_U;
    Scalar LH_HIP_D;
    Scalar LH_HIP_u;
    // Link 'LH_THIGH' :
    Matrix66S LH_THIGH_AI;
    Velocity LH_THIGH_a;
    Velocity LH_THIGH_v;
    Velocity LH_THIGH_c;
    Force    LH_THIGH_p;

    Column6DS LH_THIGH_U;
    Scalar LH_THIGH_D;
    Scalar LH_THIGH_u;
    // Link 'LH_SHANK' :
    Matrix66S LH_SHANK_AI;
    Velocity LH_SHANK_a;
    Velocity LH_SHANK_v;
    Velocity LH_SHANK_c;
    Force    LH_SHANK_p;

    Column6DS LH_SHANK_U;
    Scalar LH_SHANK_D;
    Scalar LH_SHANK_u;
    // Link 'RH_HIP' :
    Matrix66S RH_HIP_AI;
    Velocity RH_HIP_a;
    Velocity RH_HIP_v;
    Velocity RH_HIP_c;
    Force    RH_HIP_p;

    Column6DS RH_HIP_U;
    Scalar RH_HIP_D;
    Scalar RH_HIP_u;
    // Link 'RH_THIGH' :
    Matrix66S RH_THIGH_AI;
    Velocity RH_THIGH_a;
    Velocity RH_THIGH_v;
    Velocity RH_THIGH_c;
    Force    RH_THIGH_p;

    Column6DS RH_THIGH_U;
    Scalar RH_THIGH_D;
    Scalar RH_THIGH_u;
    // Link 'RH_SHANK' :
    Matrix66S RH_SHANK_AI;
    Velocity RH_SHANK_a;
    Velocity RH_SHANK_v;
    Velocity RH_SHANK_c;
    Force    RH_SHANK_p;

    Column6DS RH_SHANK_U;
    Scalar RH_SHANK_D;
    Scalar RH_SHANK_u;
private:
    static const ExtForces zeroExtForces;
};

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::setJointStatus(const JointState& q) const {
    (motionTransforms-> fr_LF_HIP_X_fr_base)(q);
    (motionTransforms-> fr_LF_THIGH_X_fr_LF_HIP)(q);
    (motionTransforms-> fr_LF_SHANK_X_fr_LF_THIGH)(q);
    (motionTransforms-> fr_RF_HIP_X_fr_base)(q);
    (motionTransforms-> fr_RF_THIGH_X_fr_RF_HIP)(q);
    (motionTransforms-> fr_RF_SHANK_X_fr_RF_THIGH)(q);
    (motionTransforms-> fr_LH_HIP_X_fr_base)(q);
    (motionTransforms-> fr_LH_THIGH_X_fr_LH_HIP)(q);
    (motionTransforms-> fr_LH_SHANK_X_fr_LH_THIGH)(q);
    (motionTransforms-> fr_RH_HIP_X_fr_base)(q);
    (motionTransforms-> fr_RH_THIGH_X_fr_RH_HIP)(q);
    (motionTransforms-> fr_RH_SHANK_X_fr_RH_THIGH)(q);
}

template <typename TRAIT>
inline void ForwardDynamics<TRAIT>::fd(
    JointState& qdd, Acceleration& base_a, // output parameters,
    const Velocity& base_v, const Acceleration& g,
    const JointState& q,
    const JointState& qd,
    const JointState& tau,
    const ExtForces& fext/* = zeroExtForces */)
{
    setJointStatus(q);
    fd(qdd, base_a, base_v, g, qd, tau, fext);
}

}

typedef tpl::ForwardDynamics<iit::rbd::DoubleTrait> ForwardDynamics;

}
}
}

#include "forward_dynamics.impl.h"

#endif
