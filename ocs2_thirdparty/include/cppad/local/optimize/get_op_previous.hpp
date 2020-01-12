# ifndef CPPAD_LOCAL_OPTIMIZE_GET_OP_PREVIOUS_HPP
# define CPPAD_LOCAL_OPTIMIZE_GET_OP_PREVIOUS_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*!
\file get_cexp_info.hpp
Create operator information tables
*/

# include <cppad/local/optimize/match_op.hpp>
# include <cppad/local/optimize/usage.hpp>

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize {

/*!
Get mapping from each variable to a previous variable
that can be used to replace it (if one exists).

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD< Base > and computations by this routine are done using type
 Base.

\param play
This is the old operation sequence.

\param random_itr
This is a random iterator for the old operation sequence.

\param cexp_set
set[i] is a set of elements for the i-th operator.
Suppose that e is an element of set[i], j = e / 2, k = e % 2.
If the comparision for the j-th conditional expression is equal to bool(k),
the i-th operator can be skipped (is not used by any of the results).
Note the the j indexs the CExpOp operators in the operation sequence.
On input, cexp_set is does not count previous optimization.
On output, it does count previous optimization.

\param op_previous
The input size of this vector must be zero.
Upon return it has size equal to the number of operators
in the operation sequence; i.e., num_op = play->nun_var_rec().
Let j = op_previous[i]. It j = 0, no replacement was found for i-th operator.
Otherwise, j < i, op_previous[j] == 0, op_usage[j] == usage_t(yes_usage),
i-th operator has NumArg(op) <= 3, 0 < NumRes(op), is not one of the following:
    - PriOp, ParOp, InvOp, EndOp, CexpOp, BeginOp.

    - it is not one of the load store op
    LtpvOp, LtvpOp, LtvvOp, StppOp, StpvOp, StvpOp, StvvOp.

    - it is not a atomic function fucntion op
    AFunOp, FunapOp, FunavOp, FunrpOp, FunrvOp.

\param op_usage
The size of this vector is the number of operators in the
operation sequence.i.e., play->nun_var_rec().
On input, op_usage[i] is the usage for
the i-th operator in the operation sequence not counting previous
optimization.
On output, it is the usage counting previous operator optimization.
*/

template <class Addr, class Base>
void get_op_previous(
    const player<Base>*                         play                ,
    const play::const_random_iterator<Addr>&    random_itr          ,
    sparse_list&                                cexp_set            ,
    pod_vector<addr_t>&                         op_previous         ,
    pod_vector<usage_t>&                        op_usage            )
{
    // number of operators in the tape
    const size_t num_op = random_itr.num_op();
    CPPAD_ASSERT_UNKNOWN( op_previous.size() == 0 );
    CPPAD_ASSERT_UNKNOWN( op_usage.size() == num_op );
    op_previous.resize( num_op );
    //
    // number of conditional expressions in the tape
    //
    // initialize mapping from variable index to operator index
    CPPAD_ASSERT_UNKNOWN(
        size_t( std::numeric_limits<addr_t>::max() ) >= num_op
    );
    // ----------------------------------------------------------------------
    // compute op_previous
    // ----------------------------------------------------------------------
    sparse_list  hash_table_op;
    hash_table_op.resize(CPPAD_HASH_TABLE_SIZE, num_op);
    //
    pod_vector<bool> work_bool;
    pod_vector<addr_t> work_addr_t;
    for(size_t i_op = 0; i_op < num_op; ++i_op)
    {   op_previous[i_op] = 0;

        if( op_usage[i_op] == usage_t(yes_usage) )
        switch( random_itr.get_op(i_op) )
        {
            // ----------------------------------------------------------------
            // these operators never match pevious operators
            case BeginOp:
            case CExpOp:
            case CSkipOp:
            case CSumOp:
            case EndOp:
            case InvOp:
            case LdpOp:
            case LdvOp:
            case ParOp:
            case PriOp:
            case StppOp:
            case StpvOp:
            case StvpOp:
            case StvvOp:
            case AFunOp:
            case FunapOp:
            case FunavOp:
            case FunrpOp:
            case FunrvOp:
            break;

            // ----------------------------------------------------------------
            // check for a previous match
            case AbsOp:
            case AcosOp:
            case AcoshOp:
            case AddpvOp:
            case AddvvOp:
            case AsinOp:
            case AsinhOp:
            case AtanOp:
            case AtanhOp:
            case CosOp:
            case CoshOp:
            case DisOp:
            case DivpvOp:
            case DivvpOp:
            case DivvvOp:
            case EqpvOp:
            case EqvvOp:
            case ErfOp:
            case ExpOp:
            case Expm1Op:
            case LepvOp:
            case LevpOp:
            case LevvOp:
            case LogOp:
            case Log1pOp:
            case LtpvOp:
            case LtvpOp:
            case LtvvOp:
            case MulpvOp:
            case MulvvOp:
            case NepvOp:
            case NevvOp:
            case PowpvOp:
            case PowvpOp:
            case PowvvOp:
            case SignOp:
            case SinOp:
            case SinhOp:
            case SqrtOp:
            case SubpvOp:
            case SubvpOp:
            case SubvvOp:
            case TanOp:
            case TanhOp:
            case ZmulpvOp:
            case ZmulvpOp:
            case ZmulvvOp:
            match_op(
                random_itr,
                op_previous,
                i_op,
                hash_table_op,
                work_bool,
                work_addr_t
            );
            if( op_previous[i_op] != 0 )
            {   // like a unary operator that assigns i_op equal to previous.
                size_t previous = size_t( op_previous[i_op] );
                bool sum_op = false;
                CPPAD_ASSERT_UNKNOWN( previous < i_op );
                op_inc_arg_usage(
                    play, sum_op, i_op, previous, op_usage, cexp_set
                );
            }
            break;

            // ----------------------------------------------------------------
            default:
            CPPAD_ASSERT_UNKNOWN(false);
            break;
        }
    }
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
