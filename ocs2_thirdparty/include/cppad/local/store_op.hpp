# ifndef CPPAD_LOCAL_STORE_OP_HPP
# define CPPAD_LOCAL_STORE_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file store_op.hpp
Changing the current value of a VecAD element.
*/
/*
==============================================================================
<!-- define preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_v_x = index_by_ind[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->
==============================================================================
*/
/*!
Shared documentation for zero order forward implementation of
op = StppOp, StpvOp, StvpOp, or StvvOp (not called).

<!-- replace preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_v_x = index_by_ind[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type Base.

\param i_z
is the index corresponding to the previous variable on the tape
(only used for error checking).

\param arg
\n
arg[0]
\n
is the offset of this VecAD vector relative to the beginning
of the isvar_by_ind and index_by_ind arrays.
\n
\n
arg[1]
\n
If this is a StppOp or StpvOp operation (if x is a parameter),
i_vec is defined by
\verbatim
    i_vec = arg[1]
\endverbatim
If this is a StvpOp or StvvOp operation (if x is a variable),
i_vec is defined by
\verbatim
    i_vec = floor( taylor[ size_t(arg[1]) * cap_order + 0 ] )
\endverbatim
where floor(c) is the greatest integer less that or equal c.
\n
\n
arg[2]
\n
index corresponding to the third operand for this operator;
i.e. the index corresponding to y.

\param num_par
is the total number of parameters on the tape
(only used for error checking).

\param cap_order
number of columns in the matrix containing the Taylor coefficients.

\param taylor
In StvpOp and StvvOp cases, <code><taylor[ size_t(arg[1]) * cap_order + 0 ]</code>
is used to compute the index in the definition of i_vec above.

\param isvar_by_ind
If y is a varable (StpvOp and StvvOp cases),
<code>isvar_by_ind[ arg[0] + i_vec ] </code> is set to true.
Otherwise y is a paraemter (StppOp and StvpOp cases) and
<code>isvar_by_ind[ arg[0] + i_vec ] </code> is set to false.

\param index_by_ind
<code>index_by_ind[ arg[0] - 1 ]</code>
is the number of elements in the user vector containing this element.
The value <code>index_by_ind[ arg[0] + i_vec]</code>
is set equal to arg[2].

\par Check User Errors
\li Check that the index is with in range; i.e.
<code>i_vec < index_by_ind[ arg[0] - 1 ]</code>
Note that, if x is a parameter,
the corresponding vector index and it does not change.
In this case, the error above should be detected during tape recording.

\par Checked Assertions
\li NumArg(op) == 3
\li NumRes(op) == 0
\li 0 <  arg[0]
\li if y is a parameter, arg[2] < num_par
*/
template <class Base>
void forward_store_op_0(
    size_t         i_z         ,
    const addr_t*  arg         ,
    size_t         num_par     ,
    size_t         cap_order   ,
    Base*          taylor      ,
    bool*          isvar_by_ind   ,
    size_t*        index_by_ind   )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}
/*!
Shared documnetation for sparsity operations corresponding to
op = StpvOp or StvvOp (not called).

\tparam Vector_set
is the type used for vectors of sets. It can be either
sparse_pack or sparse_list.

\param op
is the code corresponding to this operator;
i.e., StpvOp, StvpOp, or StvvOp.

\param arg
\n
 arg[0]
is the offset corresponding to this VecAD vector in the combined array.
\n
\n
 arg[2]
\n
The set with index arg[2] in var_sparsity
is the sparsity pattern corresponding to y.
(Note that arg[2] > 0 because y is a variable.)

\param num_combined
is the total number of elements in the VecAD address array.

\param combined
 combined [ arg[0] - 1 ]
is the index of the set in vecad_sparsity corresponding
to the sparsity pattern for the vector v.
We use the notation i_v below which is defined by
\verbatim
    i_v = combined[ arg[0] - 1 ]
\endverbatim

\param var_sparsity
The set  with index arg[2] in var_sparsity
is the sparsity pattern for y.
This is an input for forward mode operations.
For reverse mode operations:
The sparsity pattern for v is added to the spartisy pattern for y.

\param vecad_sparsity
The set with index i_v in vecad_sparsity
is the sparsity pattern for v.
This is an input for reverse mode operations.
For forward mode operations, the sparsity pattern for y is added
to the sparsity pattern for the vector v.

\par Checked Assertions
\li NumArg(op) == 3
\li NumRes(op) == 0
\li 0 <  arg[0]
\li arg[0] < num_combined
\li arg[2] < var_sparsity.n_set()
\li i_v       < vecad_sparsity.n_set()
*/
template <class Vector_set>
void sparse_store_op(
    OpCode         op             ,
    const addr_t*  arg            ,
    size_t         num_combined   ,
    const size_t*  combined       ,
    Vector_set&    var_sparsity   ,
    Vector_set&    vecad_sparsity )
{
    // This routine is only for documentaiton, it should not be used
    CPPAD_ASSERT_UNKNOWN( false );
}


/*!
Zero order forward mode implementation of op = StppOp.

\copydetails CppAD::local::forward_store_op_0
*/
template <class Base>
void forward_store_pp_op_0(
    size_t         i_z         ,
    const addr_t*  arg         ,
    size_t         num_par     ,
    size_t         cap_order   ,
    Base*          taylor      ,
    bool*          isvar_by_ind   ,
    size_t*        index_by_ind   )
{   addr_t i_vec = arg[1];

    // Because the index is a parameter, this indexing error should be
    // caught and reported to the user when the tape is recording.
    CPPAD_ASSERT_UNKNOWN( size_t(i_vec) < index_by_ind[ arg[0] - 1 ] );

    CPPAD_ASSERT_UNKNOWN( NumArg(StppOp) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(StppOp) == 0 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < num_par );

    isvar_by_ind[ arg[0] + i_vec ]  = false;
    index_by_ind[ arg[0] + i_vec ]  = size_t(arg[2]);
}

/*!
Zero order forward mode implementation of op = StpvOp.

\copydetails CppAD::local::forward_store_op_0
*/
template <class Base>
void forward_store_pv_op_0(
    size_t         i_z         ,
    const addr_t*  arg         ,
    size_t         num_par     ,
    size_t         cap_order   ,
    Base*          taylor      ,
    bool*          isvar_by_ind   ,
    size_t*        index_by_ind   )
{   addr_t i_vec = arg[1];

    // Because the index is a parameter, this indexing error should be
    // caught and reported to the user when the tape is recording.
    CPPAD_ASSERT_UNKNOWN( size_t(i_vec) < index_by_ind[ arg[0] - 1 ] );

    CPPAD_ASSERT_UNKNOWN( NumArg(StpvOp) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(StpvOp) == 0 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );

    isvar_by_ind[ arg[0] + i_vec ]  = true;
    index_by_ind[ arg[0] + i_vec ]  = size_t(arg[2]);
}

/*!
Zero order forward mode implementation of op = StvpOp.

\copydetails CppAD::local::forward_store_op_0
*/
template <class Base>
void forward_store_vp_op_0(
    size_t         i_z         ,
    const addr_t*  arg         ,
    size_t         num_par     ,
    size_t         cap_order   ,
    Base*          taylor      ,
    bool*          isvar_by_ind   ,
    size_t*        index_by_ind   )
{
    addr_t i_vec = addr_t(Integer( taylor[ size_t(arg[1]) * cap_order + 0 ] ));
    CPPAD_ASSERT_KNOWN(
        size_t(i_vec) < index_by_ind[ arg[0] - 1 ] ,
        "VecAD: index during zero order forward sweep is out of range"
    );

    CPPAD_ASSERT_UNKNOWN( NumArg(StvpOp) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(StvpOp) == 0 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < num_par );

    isvar_by_ind[ arg[0] + i_vec ]  = false;
    index_by_ind[ arg[0] + i_vec ]  = size_t(arg[2]);
}

/*!
Zero order forward mode implementation of op = StvvOp.

\copydetails CppAD::local::forward_store_op_0
*/
template <class Base>
void forward_store_vv_op_0(
    size_t         i_z         ,
    const addr_t*  arg         ,
    size_t         num_par     ,
    size_t         cap_order   ,
    Base*          taylor      ,
    bool*          isvar_by_ind   ,
    size_t*        index_by_ind   )
{
    addr_t i_vec = addr_t(Integer( taylor[ size_t(arg[1]) * cap_order + 0 ] ));
    CPPAD_ASSERT_KNOWN(
        size_t(i_vec) < index_by_ind[ arg[0] - 1 ] ,
        "VecAD: index during zero order forward sweep is out of range"
    );

    CPPAD_ASSERT_UNKNOWN( NumArg(StvpOp) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(StvpOp) == 0 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );

    isvar_by_ind[ arg[0] + i_vec ]  = true;
    index_by_ind[ arg[0] + i_vec ]  = size_t(arg[2]);
}

/*!
Forward mode sparsity operations for StpvOp and StvvOp

<!-- replace preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_v_x = index_by_ind[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->

\param dependency
is this a dependency (or sparsity) calculation.

\copydetails CppAD::local::sparse_store_op
*/
template <class Vector_set>
void forward_sparse_store_op(
    bool                dependency     ,
    OpCode              op             ,
    const addr_t*       arg            ,
    size_t              num_combined   ,
    const size_t*       combined       ,
    Vector_set&         var_sparsity   ,
    Vector_set&         vecad_sparsity )
{
    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_combined );
    size_t i_v = combined[ arg[0] - 1 ];
    CPPAD_ASSERT_UNKNOWN( i_v < vecad_sparsity.n_set() );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < var_sparsity.n_set() );

    if( dependency & ( (op == StvvOp) | (op == StvpOp) ) )
        vecad_sparsity.binary_union(i_v, i_v, size_t(arg[1]), var_sparsity);

    if( (op == StpvOp) | (op == StvvOp ) )
        vecad_sparsity.binary_union(i_v, i_v, size_t(arg[2]), var_sparsity);

    return;
}

/*!
Reverse mode sparsity operations for StpvOp, StvpOp, and StvvOp

<!-- replace preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_v_x = index_by_ind[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->

This routine is given the sparsity patterns for
G(v[x], y , w , u ... ) and it uses them to compute the
sparsity patterns for
\verbatim
    H(y , w , u , ... ) = G[ v[x], y , w , u , ... ]
\endverbatim

\param dependency
is this a dependency (or sparsity) calculation.

\copydetails CppAD::local::sparse_store_op
*/
template <class Vector_set>
void reverse_sparse_jacobian_store_op(
    bool               dependency      ,
    OpCode             op              ,
    const addr_t*      arg             ,
    size_t             num_combined    ,
    const size_t*      combined        ,
    Vector_set&        var_sparsity    ,
    Vector_set&        vecad_sparsity  )
{
    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_combined );
    size_t i_v = combined[ arg[0] - 1 ];
    CPPAD_ASSERT_UNKNOWN( i_v < vecad_sparsity.n_set() );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < var_sparsity.n_set() );

    if( dependency & ( (op == StvpOp) | (op == StvvOp) ) )
        var_sparsity.binary_union( size_t(arg[1]), size_t(arg[1]), i_v, vecad_sparsity);
    if( (op == StpvOp) | (op == StvvOp) )
        var_sparsity.binary_union( size_t(arg[2]), size_t(arg[2]), i_v, vecad_sparsity);

    return;
}

/*!
Reverse mode sparsity operations for StpvOp and StvvOp

<!-- replace preamble -->
The C++ source code corresponding to this operation is
\verbatim
    v[x] = y
\endverbatim
where v is a VecAD<Base> vector, x is an AD<Base> object,
and y is AD<Base> or Base objects.
We define the index corresponding to v[x] by
\verbatim
    i_v_x = index_by_ind[ arg[0] + i_vec ]
\endverbatim
where i_vec is defined under the heading arg[1] below:
<!-- end preamble -->

This routine is given the sparsity patterns for
G(v[x], y , w , u ... )
and it uses them to compute the sparsity patterns for
\verbatim
    H(y , w , u , ... ) = G[ v[x], y , w , u , ... ]
\endverbatim

\copydetails CppAD::local::sparse_store_op

\param var_jacobian
 var_jacobian[ arg[2] ]
is false (true) if the Jacobian of G with respect to y is always zero
(may be non-zero).

\param vecad_jacobian
 vecad_jacobian[i_v]
is false (true) if the Jacobian with respect to x is always zero
(may be non-zero).
On input, it corresponds to the function G,
and on output it corresponds to the function H.
*/
template <class Vector_set>
void reverse_sparse_hessian_store_op(
    OpCode             op           ,
    const addr_t*      arg          ,
    size_t             num_combined ,
    const size_t*      combined     ,
    Vector_set&        var_sparsity ,
    Vector_set&        vecad_sparsity ,
    bool*              var_jacobian   ,
    bool*              vecad_jacobian )
{
    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 3 );
    CPPAD_ASSERT_UNKNOWN( NumRes(op) == 0 );
    CPPAD_ASSERT_UNKNOWN( 0 < arg[0] );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[0]) < num_combined );
    size_t i_v = combined[ arg[0] - 1 ];
    CPPAD_ASSERT_UNKNOWN( i_v < vecad_sparsity.n_set() );
    CPPAD_ASSERT_UNKNOWN( size_t(arg[2]) < var_sparsity.n_set() );

    var_sparsity.binary_union( size_t(arg[2]), size_t(arg[2]), i_v, vecad_sparsity);

    var_jacobian[ arg[2] ] |= vecad_jacobian[i_v];

    return;
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
