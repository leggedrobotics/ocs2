# ifndef CPPAD_LOCAL_PARAMETER_OP_HPP
# define CPPAD_LOCAL_PARAMETER_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-16 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */


namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file parameter_op.hpp
Zero order forward mode for ParOp
*/


/*!
Compute zero order forward mode Taylor coefficient for result of op = ParOp.

The C++ source code corresponding to this operation is one of the following
\verbatim
    ADFun<Base> f(x, y)
    f.Dependent(x, y)
\endverbatim
where some of the components of the vector y are parameters.

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base .

\param i_z
variable index corresponding to the result for this operation;
i.e. the row index in taylor corresponding to the component of y
that is a parameter.

\param arg
 arg[0]
\n
index corresponding to the parameter value for this operator.

\param num_par
is the number of parameters in parameter.

\param parameter
\b Input: parameter[ arg[0] ] is the value of a component
of y that is a parameter.

\param cap_order
number of colums in the matrix containing all the Taylor coefficients.

\param taylor
\b Output: taylor [ i_z * cap_order + 0 ]
is the zero order Taylor coefficient corresponding to z.

\par Checked Assertions where op is the unary operator with one result:
\li NumArg(op) == 1
\li NumRes(op) == 1
\li size_t(arg[0]) < num_par
\li 0 < cap_order
*/
template <class Base>
void forward_par_op_0(
    size_t        i_z         ,
    const addr_t* arg         ,
    size_t        num_par     ,
    const Base*   parameter   ,
    size_t        cap_order   ,
    Base*         taylor      )
{
    // check assumptions
    CPPAD_ASSERT_UNKNOWN( NumArg(ParOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( NumRes(ParOp) == 1 );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_par );
    CPPAD_ASSERT_UNKNOWN( 0 < cap_order );

    Base* z = taylor + i_z * cap_order;

    z[0]  = parameter[ arg[0] ];
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
