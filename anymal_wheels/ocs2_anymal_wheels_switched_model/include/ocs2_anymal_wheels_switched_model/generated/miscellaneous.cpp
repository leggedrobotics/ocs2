#include <iit/rbd/utils.h>
#include "miscellaneous.h"

using namespace iit::ANYmal;
using namespace iit::ANYmal::dyn;

iit::rbd::Vector3d iit::ANYmal::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const HomogeneousTransforms& ht)
{
    iit::rbd::Vector3d tmpSum(iit::rbd::Vector3d::Zero());

    tmpSum += inertiaProps.getCOM_base() * inertiaProps.getMass_base();

    HomogeneousTransforms::MatrixType tmpX(HomogeneousTransforms::MatrixType::Identity());
    HomogeneousTransforms::MatrixType base_X_LF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_LH_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RF_HAA_chain;
    HomogeneousTransforms::MatrixType base_X_RH_HAA_chain;
    
    
    base_X_LF_HAA_chain = tmpX * ht.fr_base_X_fr_LF_HIP;
    tmpSum += inertiaProps.getMass_LF_HIP() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_HIP()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_HIP_X_fr_LF_THIGH;
    tmpSum += inertiaProps.getMass_LF_THIGH() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_THIGH()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_THIGH_X_fr_LF_SHANK;
    tmpSum += inertiaProps.getMass_LF_SHANK() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_SHANK()));
    
    base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_SHANK_X_fr_LF_WHEEL_L;
    tmpSum += inertiaProps.getMass_LF_WHEEL_L() *
            ( iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_WHEEL_L()));
    
    base_X_LH_HAA_chain = tmpX * ht.fr_base_X_fr_LH_HIP;
    tmpSum += inertiaProps.getMass_LH_HIP() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_HIP()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_HIP_X_fr_LH_THIGH;
    tmpSum += inertiaProps.getMass_LH_THIGH() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_THIGH()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_THIGH_X_fr_LH_SHANK;
    tmpSum += inertiaProps.getMass_LH_SHANK() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_SHANK()));
    
    base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_SHANK_X_fr_LH_WHEEL_L;
    tmpSum += inertiaProps.getMass_LH_WHEEL_L() *
            ( iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_WHEEL_L()));
    
    base_X_RF_HAA_chain = tmpX * ht.fr_base_X_fr_RF_HIP;
    tmpSum += inertiaProps.getMass_RF_HIP() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_HIP()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_HIP_X_fr_RF_THIGH;
    tmpSum += inertiaProps.getMass_RF_THIGH() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_THIGH()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_THIGH_X_fr_RF_SHANK;
    tmpSum += inertiaProps.getMass_RF_SHANK() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_SHANK()));
    
    base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_SHANK_X_fr_RF_WHEEL_L;
    tmpSum += inertiaProps.getMass_RF_WHEEL_L() *
            ( iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_WHEEL_L()));
    
    base_X_RH_HAA_chain = tmpX * ht.fr_base_X_fr_RH_HIP;
    tmpSum += inertiaProps.getMass_RH_HIP() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_HIP()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_HIP_X_fr_RH_THIGH;
    tmpSum += inertiaProps.getMass_RH_THIGH() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_THIGH()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_THIGH_X_fr_RH_SHANK;
    tmpSum += inertiaProps.getMass_RH_SHANK() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_SHANK()));
    
    base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_SHANK_X_fr_RH_WHEEL_L;
    tmpSum += inertiaProps.getMass_RH_WHEEL_L() *
            ( iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_WHEEL_L()));
    

    return tmpSum / inertiaProps.getTotalMass();
}

iit::rbd::Vector3d iit::ANYmal::getWholeBodyCOM(
    const InertiaProperties& inertiaProps,
    const JointState& q,
    HomogeneousTransforms& ht)
{
    // First updates the coordinate transforms that will be used by the routine
    ht.fr_base_X_fr_LF_HIP(q);
    ht.fr_base_X_fr_LH_HIP(q);
    ht.fr_base_X_fr_RF_HIP(q);
    ht.fr_base_X_fr_RH_HIP(q);
    ht.fr_LF_HIP_X_fr_LF_THIGH(q);
    ht.fr_LF_THIGH_X_fr_LF_SHANK(q);
    ht.fr_LF_SHANK_X_fr_LF_WHEEL_L(q);
    ht.fr_RF_HIP_X_fr_RF_THIGH(q);
    ht.fr_RF_THIGH_X_fr_RF_SHANK(q);
    ht.fr_RF_SHANK_X_fr_RF_WHEEL_L(q);
    ht.fr_LH_HIP_X_fr_LH_THIGH(q);
    ht.fr_LH_THIGH_X_fr_LH_SHANK(q);
    ht.fr_LH_SHANK_X_fr_LH_WHEEL_L(q);
    ht.fr_RH_HIP_X_fr_RH_THIGH(q);
    ht.fr_RH_THIGH_X_fr_RH_SHANK(q);
    ht.fr_RH_SHANK_X_fr_RH_WHEEL_L(q);

    // The actual calculus
    return getWholeBodyCOM(inertiaProps, ht);
}
