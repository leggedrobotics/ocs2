#include <iit/rbd/utils.h>
#include <iit/rbd/traits/TraitSelector.h>
#include "inertia_properties.h"
#include "transforms.h"
//#include "miscellaneous.h"

namespace iit {
namespace camel {

    template<typename SCALAR_T>
    typename iit::rbd::Core<SCALAR_T>::Vector3 getWholeBodyCOM(
        const dyn::tpl::InertiaProperties<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait> &inertiaProps,
        const tpl::HomogeneousTransforms<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait> &ht) {

        using Vec3 = typename iit::rbd::Core<SCALAR_T>::Vector3;
        Vec3 tmpSum = Vec3::Zero();

        tmpSum += inertiaProps.getCOM_base() * inertiaProps.getMass_base();

        using HomogeneousTransformsScalar_t = typename tpl::HomogeneousTransforms<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait>;
        typename HomogeneousTransformsScalar_t::MatrixType tmpX(HomogeneousTransformsScalar_t::MatrixType::Identity());
        typename HomogeneousTransformsScalar_t::MatrixType base_X_LF_HAA_chain;
        typename HomogeneousTransformsScalar_t::MatrixType base_X_RF_HAA_chain;
        typename HomogeneousTransformsScalar_t::MatrixType base_X_LH_HAA_chain;
        typename HomogeneousTransformsScalar_t::MatrixType base_X_RH_HAA_chain;

        base_X_LF_HAA_chain = tmpX * ht.fr_base_X_fr_LF_HIP;
        tmpSum += inertiaProps.getMass_LF_HIP() *
            (iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_HIP()));

        base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_HIP_X_fr_LF_THIGH;
        tmpSum += inertiaProps.getMass_LF_THIGH() *
            (iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_THIGH()));

        base_X_LF_HAA_chain = base_X_LF_HAA_chain * ht.fr_LF_THIGH_X_fr_LF_SHANK;
        tmpSum += inertiaProps.getMass_LF_SHANK() *
            (iit::rbd::Utils::transform(base_X_LF_HAA_chain, inertiaProps.getCOM_LF_SHANK()));

        base_X_RF_HAA_chain = tmpX * ht.fr_base_X_fr_RF_HIP;
        tmpSum += inertiaProps.getMass_RF_HIP() *
            (iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_HIP()));

        base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_HIP_X_fr_RF_THIGH;
        tmpSum += inertiaProps.getMass_RF_THIGH() *
            (iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_THIGH()));

        base_X_RF_HAA_chain = base_X_RF_HAA_chain * ht.fr_RF_THIGH_X_fr_RF_SHANK;
        tmpSum += inertiaProps.getMass_RF_SHANK() *
            (iit::rbd::Utils::transform(base_X_RF_HAA_chain, inertiaProps.getCOM_RF_SHANK()));

        base_X_LH_HAA_chain = tmpX * ht.fr_base_X_fr_LH_HIP;
        tmpSum += inertiaProps.getMass_LH_HIP() *
            (iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_HIP()));

        base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_HIP_X_fr_LH_THIGH;
        tmpSum += inertiaProps.getMass_LH_THIGH() *
            (iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_THIGH()));

        base_X_LH_HAA_chain = base_X_LH_HAA_chain * ht.fr_LH_THIGH_X_fr_LH_SHANK;
        tmpSum += inertiaProps.getMass_LH_SHANK() *
            (iit::rbd::Utils::transform(base_X_LH_HAA_chain, inertiaProps.getCOM_LH_SHANK()));

        base_X_RH_HAA_chain = tmpX * ht.fr_base_X_fr_RH_HIP;
        tmpSum += inertiaProps.getMass_RH_HIP() *
            (iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_HIP()));

        base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_HIP_X_fr_RH_THIGH;
        tmpSum += inertiaProps.getMass_RH_THIGH() *
            (iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_THIGH()));

        base_X_RH_HAA_chain = base_X_RH_HAA_chain * ht.fr_RH_THIGH_X_fr_RH_SHANK;
        tmpSum += inertiaProps.getMass_RH_SHANK() *
            (iit::rbd::Utils::transform(base_X_RH_HAA_chain, inertiaProps.getCOM_RH_SHANK()));

        tmpSum *= SCALAR_T(1.0)/ inertiaProps.getTotalMass();
        return tmpSum;
    }

    template<typename SCALAR_T>
    typename iit::rbd::Core<SCALAR_T>::Vector3 getWholeBodyCOM(
        const dyn::tpl::InertiaProperties<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait> &inertiaProps,
        const tpl::JointState<SCALAR_T> &q,
        tpl::HomogeneousTransforms<typename iit::rbd::tpl::TraitSelector<SCALAR_T>::Trait> &ht) {
        // First updates the coordinate transforms that will be used by the routine
        ht.fr_base_X_fr_LF_HIP(q);
        ht.fr_base_X_fr_RF_HIP(q);
        ht.fr_base_X_fr_LH_HIP(q);
        ht.fr_base_X_fr_RH_HIP(q);
        ht.fr_LF_HIP_X_fr_LF_THIGH(q);
        ht.fr_LF_THIGH_X_fr_LF_SHANK(q);
        ht.fr_RF_HIP_X_fr_RF_THIGH(q);
        ht.fr_RF_THIGH_X_fr_RF_SHANK(q);
        ht.fr_LH_HIP_X_fr_LH_THIGH(q);
        ht.fr_LH_THIGH_X_fr_LH_SHANK(q);
        ht.fr_RH_HIP_X_fr_RH_THIGH(q);
        ht.fr_RH_THIGH_X_fr_RH_SHANK(q);

        // The actual calculus
        return getWholeBodyCOM<SCALAR_T>(inertiaProps, ht);
    }


} // namespace camel
} // namespace iit