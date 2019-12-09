# ifndef CPPAD_LOCAL_OPTIMIZE_MATCH_OP_HPP
# define CPPAD_LOCAL_OPTIMIZE_MATCH_OP_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/optimize/hash_code.hpp>
/*!
\file match_op.hpp
Check if current operator matches a previous operator.
*/
// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {
/*!
Search for a previous operator that matches the current one.

If an argument for the current operator is a variable,
and the argument has previous match,
the previous match for the argument is used when checking for a match
for the current operator.

\param random_itr
is a random iterator for the old operation sequence.

\param op_previous
Mapping from operator index to previous operator that can replace this one.
The input value of op_previous[current] is assumed to be zero.
If a match if found,
the output value of op_previous[current] is set to the
matching operator index, otherwise it is left as is.
Note that op_previous[current] < current and
op_previous[ op_previous[current] ] = 0.

\param current
is the index of the current operator which must be an unary
or binary operator with NumRes(op) > 0.
Note that NumArg(ErfOp) == 3 but it is effectivey
a unary operator and is allowed otherwise
NumArg( random_itr.get_op[current]) < 3.
It is assumed that hash_table_op is initialized as a vector of emtpy
sets. After this initialization, the value of current inceases with
each call to match_op.

\li
This must be a unary or binary
operator; hence, NumArg( random_itr.get_op[current] ) is one or two.
There is one exception, NumRes( ErfOp ) == 3, but arg[0]
is the only true arguments (the others are always the same).

\li
This must not be a VecAD load or store operation; i.e.,
LtpvOp, LtvpOp, LtvvOp, StppOp, StpvOp, StvpOp, StvvOp.
It also must not be an independent variable operator InvOp.

\param hash_table_op
is a vector of sets,
hash_table_op.n_set() == CPPAD_HASH_TABLE_SIZE and
hash_table_op.end() == op_previous.size().
If i_op is an element of set[j],
then the operation op_previous[i_op] has hash code j,
and op_previous[i_op] does not match any other element of set[j].
An entry to set[j] is added each time match_op is called
and a match for the current operator is not found.

\param work_bool
work space that is used by match_op between calls to increase speed.
Should be empty on first call for this forward passs of the operation
sequence and not modified untill forward pass is done

\param work_addr_t
work space that is used by match_op between calls to increase speed.
Should be empty on first call for this forward passs of the operation
sequence and not modified untill forward pass is done

*/
template <class Addr>
void match_op(
    const play::const_random_iterator<Addr>&    random_itr     ,
    pod_vector<addr_t>&                         op_previous    ,
    size_t                                      current        ,
    sparse_list&                                hash_table_op  ,
    pod_vector<bool>&                           work_bool      ,
    pod_vector<addr_t>&                         work_addr_t    )
{   //
    // num_op
    size_t num_op = random_itr.num_op();
    //
    // num_var
    size_t num_var = random_itr.num_var();
    //
    // variable is a reference to, and better name for, work_bool
    pod_vector<bool>&  variable(work_bool);
    //
    // var2previous_var is a reference to, and better name for, work_addr_t
    pod_vector<addr_t>&  var2previous_var(work_addr_t);
    if( var2previous_var.size() == 0 )
    {   var2previous_var.resize(num_var);
        for(size_t i = 0; i < num_var; ++i)
            var2previous_var[i] = addr_t(i);
    }
    //
    CPPAD_ASSERT_UNKNOWN( var2previous_var.size() == num_var );
    CPPAD_ASSERT_UNKNOWN( num_op == op_previous.size() );
    CPPAD_ASSERT_UNKNOWN( op_previous[current] == 0 );
    CPPAD_ASSERT_UNKNOWN(
        hash_table_op.n_set() == CPPAD_HASH_TABLE_SIZE
    );
    CPPAD_ASSERT_UNKNOWN( hash_table_op.end() == num_op );
    CPPAD_ASSERT_UNKNOWN( current < num_op );
    //
    // op, arg, i_var
    OpCode        op;
    const addr_t* arg;
    size_t        i_var;
    random_itr.op_info(current, op, arg, i_var);
    CPPAD_ASSERT_UNKNOWN( 0 < NumArg(op) );
    //
    // num_arg
    size_t num_arg = NumArg(op);
    CPPAD_ASSERT_UNKNOWN( num_arg <= 3 );
    //
    arg_is_variable(op, arg, variable);
    CPPAD_ASSERT_UNKNOWN( variable.size() == num_arg );
    //
    // If j-th argument to this operator is a variable, and a previous
    // variable will be used in its place, use the previous variable for
    // hash coding and matching.
    addr_t arg_match[3];
    for(size_t j = 0; j < num_arg; ++j)
    {   arg_match[j] = arg[j];
        if( variable[j] )
            arg_match[j] = var2previous_var[ arg[j] ];
    }
    //
    size_t code = optimize_hash_code(opcode_t(op), num_arg, arg_match);
    //
    // iterator for the set with this hash code
    sparse_list_const_iterator itr(hash_table_op, code);
    //
    // check for a match
    size_t count = 0;
    while( *itr != num_op )
    {   ++count;
        //
        // candidate previous for current operator
        size_t  candidate  = *itr;
        CPPAD_ASSERT_UNKNOWN( candidate < current );
        CPPAD_ASSERT_UNKNOWN( op_previous[candidate] == 0 );
        //
        OpCode        op_c;
        const addr_t* arg_c;
        size_t        i_var_c;
        random_itr.op_info(candidate, op_c, arg_c, i_var_c);
        //
        // check for a match
        bool match = op == op_c;
        size_t j   = 0;
        while( match & (j < num_arg) )
        {   if( variable[j] )
                match &= arg_match[j] == var2previous_var[ arg_c[j] ];
            else
                match &= arg_match[j] == arg_c[j];
            ++j;
        }
        if( match )
        {   op_previous[current] = static_cast<addr_t>( candidate );
            if( NumRes(op) > 0 )
            {   CPPAD_ASSERT_UNKNOWN( i_var_c < i_var );
                var2previous_var[i_var] = addr_t( i_var_c );
            }
            return;
        }
        ++itr;
    }

    // special case where operator is commutative
    if( (op == AddvvOp) | (op == MulvvOp ) )
    {   CPPAD_ASSERT_UNKNOWN( NumArg(op) == 2 );
        std::swap( arg_match[0], arg_match[1] );
        //
        code      = optimize_hash_code(opcode_t(op), num_arg, arg_match);
        sparse_list_const_iterator itr_swap(hash_table_op, code);
        while( *itr_swap != num_op )
        {
            size_t candidate  = *itr_swap;
            CPPAD_ASSERT_UNKNOWN( candidate < current );
            CPPAD_ASSERT_UNKNOWN( op_previous[candidate] == 0 );
            //
            OpCode        op_c;
            const addr_t* arg_c;
            size_t        i_var_c;
            random_itr.op_info(candidate, op_c, arg_c, i_var_c);
            //
            bool match = op == op_c;
            size_t j   = 0;
            while( match & (j < num_arg) )
            {   CPPAD_ASSERT_UNKNOWN( variable[j] )
                match &= arg_match[j] == var2previous_var[ arg_c[j] ];
                ++j;
            }
            if( match )
            {   op_previous[current] = static_cast<addr_t>(candidate);
                if( NumRes(op) > 0 )
                {   CPPAD_ASSERT_UNKNOWN( i_var_c < i_var );
                    var2previous_var[i_var] = addr_t( i_var_c );
                }
                return;
            }
            ++itr_swap;
        }
    }
    CPPAD_ASSERT_UNKNOWN( count < 11 );
    if( count == 10 )
    {   // restart the list
        hash_table_op.clear(code);
    }
    // no match was found, add this operator the the set for this hash code
    hash_table_op.add_element(code, current);
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
