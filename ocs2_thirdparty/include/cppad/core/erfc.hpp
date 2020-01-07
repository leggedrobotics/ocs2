# ifndef CPPAD_CORE_ERFC_HPP
# define CPPAD_CORE_ERFC_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

/*
-------------------------------------------------------------------------------
$begin erfc$$
$spell
    erf
    erfc
    Vec
    CppAD
$$
$section The Complementary Error Function: erfc$$

$head Syntax$$
$icode%y% = erfc(%x%)%$$

$head Description$$
Returns the value of the complementary error function which is defined by
$icode%y% == 1 - erf(%x%)%$$.

$head x, y$$
See the $cref/possible types/unary_standard_math/Possible Types/$$
for a unary standard math function.

$head CPPAD_USE_CPLUSPLUS_2011$$

$subhead true$$
If this preprocessor symbol is true ($code 1$$),
and $icode x$$ is an AD type,
this is an $cref/atomic operation/glossary/Operation/Atomic/$$.

$subhead false$$
If this preprocessor symbol is false ($code 0$$),
CppAD uses the representation
$latex \[
\R{erfc} (x) = 1 - \R{erf}(x)
\] $$
to compute this function.

$head Example$$
$children%
    example/general/erfc.cpp
%$$
The file
$cref erfc.cpp$$
contains an example and test of this function.

$end
-------------------------------------------------------------------------------
*/
# include <cppad/configure.hpp>
# if ! CPPAD_USE_CPLUSPLUS_2011

// BEGIN CppAD namespace
namespace CppAD {

template <class Type>
Type erfc_template(const Type &x)
{   return Type(1) - CppAD::erf(x);
}

inline float erfc(const float &x)
{   return erfc_template(x); }

inline double erfc(const double &x)
{   return erfc_template(x); }

template <class Base>
AD<Base> erfc(const AD<Base> &x)
{   return erfc_template(x); }

template <class Base>
AD<Base> erfc(const VecAD_reference<Base> &x)
{   return erfc_template( x.ADBase() ); }


} // END CppAD namespace

# endif // CPPAD_USE_CPLUSPLUS_2011
# endif // CPPAD_ERFC_INCLUDED
