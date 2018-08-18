#ifndef INCLUDE_EXTERNAL_HYQ_TRAITS_TRAITSELECTOR_HPP_
#define INCLUDE_EXTERNAL_HYQ_TRAITS_TRAITSELECTOR_HPP_

#include "FloatTrait.h"
#include "DoubleTrait.h"
#include "CppADCodegenTrait.h"

#include <cppad/cg.hpp>
#include <cppad/cppad.hpp>
#include <cppad/cg/cppadcg.hpp>

namespace iit {
namespace rbd {
namespace tpl {

template <typename SCALAR>
struct TraitSelector
{

};

template <>
struct TraitSelector<double>
{
 	typedef DoubleTrait Trait;
};

template <>
struct TraitSelector<float>
{
 	typedef FloatTrait Trait;
};

template <>
struct TraitSelector<CppAD::AD<CppAD::cg::CG<double>>>
{
    typedef CppADCodegenTrait Trait;
};

} //namespace tpl
} // namespace rbd
} // namespace iit


#endif //INCLUDE_EXTERNAL_HYQ_TRAITS_TRAITSELECTOR_HPP_

