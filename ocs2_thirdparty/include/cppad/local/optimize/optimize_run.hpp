# ifndef CPPAD_LOCAL_OPTIMIZE_OPTIMIZE_RUN_HPP
# define CPPAD_LOCAL_OPTIMIZE_OPTIMIZE_RUN_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */

# include <stack>
# include <iterator>
# include <cppad/local/optimize/get_op_usage.hpp>
# include <cppad/local/optimize/get_par_usage.hpp>
# include <cppad/local/optimize/get_dyn_previous.hpp>
# include <cppad/local/optimize/get_op_previous.hpp>
# include <cppad/local/optimize/get_cexp_info.hpp>
# include <cppad/local/optimize/size_pair.hpp>
# include <cppad/local/optimize/csum_stacks.hpp>
# include <cppad/local/optimize/cexp_info.hpp>
# include <cppad/local/optimize/record_pv.hpp>
# include <cppad/local/optimize/record_vp.hpp>
# include <cppad/local/optimize/record_vv.hpp>
# include <cppad/local/optimize/record_csum.hpp>

/*!
\file optimize_run.hpp
Convert a player object to an optimized recorder object
*/

// BEGIN_CPPAD_LOCAL_OPTIMIZE_NAMESPACE
namespace CppAD { namespace local { namespace optimize  {

/*!
Convert a player object to an optimized recorder object

\tparam Addr
Type to use with player random iterators. Must correspond to the
result for play->addr_type().

\tparam Base
base type for the operator; i.e., this operation was recorded
using AD<Base> and computations by this routine are done using type
Base.

\param options
\li
If the sub-string "no_conditional_skip" appears,
conditional skip operations will not be generated.
This may make the optimize routine use significantly less memory
and take significantly less time.
\li
If the sub-string "no_compare_op" appears,
then comparison operators will be removed from the optimized tape.
These operators are necessary for the compare_change function to be
be meaningful in the resulting recording.
On the other hand, they are not necessary and take extra time
when compare_change is not used.
\li
If the sub-string "no_print_for" appears,
then print forward (PriOp) operators will be removed from the optimized tape.
These operators are useful for reporting problems evaluating derivatives
at independent variable values different from those used to record a function.

\param n
is the number of independent variables on the tape.

\param dep_taddr
On input this vector contains the indices for each of the dependent
variable values in the operation sequence corresponding to play.
Upon return it contains the indices for the same variables but in
the operation sequence corresponding to rec.

\param play
This is the operation sequence that we are optimizing.
It is const except for the fact that play->setup_random is called.

\param rec
The input contents of this recording must be empty; i.e.,
it corresponds to directly after the default constructor.
Upon return, it contains an optimized verison of the
operation sequence corresponding to play.
*/

template <class Addr, class Base>
void optimize_run(
    const std::string&                         options    ,
    size_t                                     n          ,
    pod_vector<size_t>&                        dep_taddr  ,
    player<Base>*                              play       ,
    recorder<Base>*                            rec        )
{   // check that recorder is empty
    CPPAD_ASSERT_UNKNOWN( rec->num_op_rec() == 0 );
    //
    // get a random iterator for this player
    play->template setup_random<Addr>();
    local::play::const_random_iterator<Addr> random_itr =
        play->template get_random<Addr>();

    bool conditional_skip = true;
    bool compare_op       = true;
    bool print_for_op     = true;
    size_t index = 0;
    while( index < options.size() )
    {   while( index < options.size() && options[index] == ' ' )
            ++index;
        std::string option;
        while( index < options.size() && options[index] != ' ' )
            option += options[index++];
        if( option != "" )
        {   if( option == "no_conditional_skip" )
                conditional_skip = false;
            else if( option == "no_compare_op" )
                compare_op = false;
            else if( option == "no_print_for_op" )
                print_for_op = false;
            else
            {   option += " is not a valid optimize option";
                CPPAD_ASSERT_KNOWN( false , option.c_str() );
            }
        }
    }
    // number of operators in the player
    const size_t num_op = play->num_op_rec();
    CPPAD_ASSERT_UNKNOWN(
        num_op < size_t( std::numeric_limits<addr_t>::max() )
    );

    // number of variables in the player
    const size_t num_var = play->num_var_rec();

    // number of parameter in the player
    const size_t num_par = play->num_par_rec();

    // number of  VecAD indices
    size_t num_vecad_ind   = play->num_vec_ind_rec();

    // number of VecAD vectors
    size_t num_vecad_vec   = play->num_vecad_vec_rec();

    // number of independent dynamic parameters
    size_t num_dynamic_ind = play->num_dynamic_ind();

    // number of dynamic parameters
    size_t num_dynamic_par = play->num_dynamic_par();

    // mapping from dynamic parameter index to paramemter index
    const pod_vector<addr_t>& dyn_ind2par_ind( play->dyn_ind2par_ind() );

    // number of dynamic parameters
    CPPAD_ASSERT_UNKNOWN( num_dynamic_ind <= play->num_dynamic_par () );

    // -----------------------------------------------------------------------
    // operator information
    pod_vector<addr_t>        cexp2op;
    sparse_list               cexp_set;
    pod_vector<bool>          vecad_used;
    pod_vector<usage_t>       op_usage;
    get_op_usage(
        conditional_skip,
        compare_op,
        print_for_op,
        play,
        random_itr,
        dep_taddr,
        cexp2op,
        cexp_set,
        vecad_used,
        op_usage
    );
    pod_vector<addr_t>        op_previous;
    get_op_previous(
        play,
        random_itr,
        cexp_set,
        op_previous,
        op_usage
    );
    size_t num_cexp = cexp2op.size();
    CPPAD_ASSERT_UNKNOWN( conditional_skip || num_cexp == 0 );
    vector<struct_cexp_info>  cexp_info; // struct_cexp_info not POD
    sparse_list               skip_op_true;
    sparse_list               skip_op_false;
    //
    if( cexp2op.size() > 0 ) get_cexp_info(
        play,
        random_itr,
        op_previous,
        op_usage,
        cexp2op,
        cexp_set,
        cexp_info,
        skip_op_true,
        skip_op_false
    );

    // We no longer need cexp_set, and cexp2op, so free their memory
    cexp_set.resize(0, 0);
    cexp2op.clear();
    // -----------------------------------------------------------------------
    // dynamic parameter information
    pod_vector<bool> par_usage;
    get_par_usage(
        play,
        random_itr,
        op_usage,
        vecad_used,
        par_usage
    );
    pod_vector<addr_t> dyn_previous;
    get_dyn_previous(
        play                ,
        random_itr          ,
        par_usage           ,
        dyn_previous
    );
    // -----------------------------------------------------------------------

    // nan with type Base
    Base base_nan = Base( std::numeric_limits<double>::quiet_NaN() );

    // -------------------------------------------------------------
    // conditional expression information
    //
    // Size of the conditional expression information structure.
    // This is equal to the number of conditional expressions when
    // conditional_skip is true, otherwise it is zero.
    //
    // sort the conditional expression information by max_left_right
    // this is the conditional skip order
    vector<size_t> cskip_order(num_cexp);
    if( num_cexp > 0 )
    {   vector<size_t> keys(num_cexp);
        for(size_t i = 0; i < num_cexp; i++)
            keys[i] = size_t( cexp_info[i].max_left_right );
        CppAD::index_sort(keys, cskip_order);
    }
    // initial index in conditional skip order
    size_t cskip_order_next = 0;
    //
    // initialize index in conditional expression order
    size_t cexp_next = 0;

    // mapping from conditional expression index to conditional skip
    // information on new tape
    pod_vector<struct_cskip_new> cskip_new(num_cexp);
    //
    // flag used to indicate that there is no conditional skip
    // for this conditional expression
    for(size_t i = 0; i < num_cexp; i++)
        cskip_new[i].i_arg = 0;
    // =======================================================================
    // Create new recording
    // =======================================================================
    //
    // dynamic parameter information in player
    const pod_vector<bool>&     dyn_par_is( play->dyn_par_is() );
    const pod_vector<opcode_t>& dyn_par_op( play->dyn_par_op() );
    const pod_vector<addr_t>&   dyn_par_arg( play->dyn_par_arg() );
    //
    // start mapping from old parameter indices to new parameter indices
    // for all parameters that get used.
    pod_vector<addr_t> new_par( num_par );
    addr_t addr_t_max = std::numeric_limits<addr_t>::max();
    for(size_t i_par = 0; i_par < num_par; ++i_par)
        new_par[i_par] = addr_t_max; // initialize as not used
    //
    // start new recording
    CPPAD_ASSERT_UNKNOWN( rec->num_op_rec() == 0 );
    rec->set_num_dynamic_ind(num_dynamic_ind);
    rec->set_abort_op_index(0);
    rec->set_record_compare( compare_op );

    // copy parameters with index 0
    CPPAD_ASSERT_UNKNOWN( ! dyn_par_is[0] && isnan( play->GetPar(0) ) );
    rec->put_con_par( play->GetPar(0) );
    new_par[0] = 0;

    // set new_par for the independent dynamic parameters
    for(size_t i_par = 1; i_par <= num_dynamic_ind; i_par++)
    {   CPPAD_ASSERT_UNKNOWN( dyn_par_is[i_par] );
        addr_t i = rec->put_dyn_par(play->GetPar(i_par), ind_dyn);
        CPPAD_ASSERT_UNKNOWN( size_t(i) == i_par );
        new_par[i_par] = i;
    }

    // set new_par for the constant parameters that are used
    for(size_t i_par = num_dynamic_ind + 1; i_par < num_par; ++i_par)
    if( ! dyn_par_is[i_par] )
    {   CPPAD_ASSERT_UNKNOWN( i_par == 0 || num_dynamic_ind < i_par );
        if( par_usage[i_par] )
        {   // value of this parameter
            Base par       = play->GetPar(i_par);
            new_par[i_par] = rec->put_con_par(par);
        }
    }

    // set new_par for the dependent dynamic parameters
    size_t i_dyn = num_dynamic_ind;  // dynamic parmaeter index
    size_t i_arg = 0;                // dynamic parameter argument index
    pod_vector<addr_t> arg_vec;
    for(size_t i_par = num_dynamic_ind + 1; i_par < num_par; ++i_par)
    if( dyn_par_is[i_par] )
    {   // operator for this dynamic parameter
        op_code_dyn op = op_code_dyn( dyn_par_op[i_dyn] );
        //
        // number of arguments for this dynamic parameter
        size_t n_arg   = num_arg_dyn(op);
        //
        // number of dynamic parameter results for this operator
        size_t n_dyn   = 1;
        //
        if( op == call_dyn )
        {   size_t atom_index = size_t( dyn_par_arg[i_arg + 0]  );
            size_t atom_n     = size_t( dyn_par_arg[i_arg + 1]  );
            size_t atom_m     = size_t( dyn_par_arg[i_arg + 2]  );
            n_dyn             = size_t( dyn_par_arg[i_arg + 3]  );
            n_arg             = 5 + atom_n + atom_m;
            //
            // check if any dynamic parameter result for this operator is used
            bool call_used = false;
# ifndef NDEBUG
            bool found_i_par = false;
            for(size_t i = 0; i < atom_m; ++i)
            {   size_t j_par = size_t( dyn_par_arg[i_arg + 4 + atom_n + i] );
                if( dyn_par_is[j_par] )
                {   call_used |= par_usage[j_par];
                    CPPAD_ASSERT_UNKNOWN( j_par == i_par || found_i_par );
                    // j_par > i_par corresponds to result_dyn operator
                    CPPAD_ASSERT_UNKNOWN( j_par >= i_par );
                    found_i_par |= j_par == i_par;
                }
            }
            CPPAD_ASSERT_UNKNOWN( found_i_par );
# else
            for(size_t i = 0; i < atom_m; ++i)
            {   size_t j_par = size_t( dyn_par_arg[i_arg + 4 + atom_n + i] );
                if( dyn_par_is[j_par] )
                    call_used |= par_usage[j_par];
            }
# endif
            if( call_used )
            {   arg_vec.resize(0);
                arg_vec.push_back( addr_t( atom_index ) );
                arg_vec.push_back( addr_t( atom_n ) );
                arg_vec.push_back( addr_t( atom_m ) );
                arg_vec.push_back( addr_t( n_dyn ) );
                for(size_t j = 0; j < atom_n; ++j)
                {   addr_t arg_j = dyn_par_arg[i_arg + 4 + j];
                    if( arg_j > 0 && par_usage[arg_j] )
                        arg_vec.push_back( new_par[ arg_j ] );
                    else
                        arg_vec.push_back(0);
                }
                bool first_dynamic_result = true;
                for(size_t i = 0; i < atom_m; ++i)
                {   addr_t res_i = dyn_par_arg[i_arg + 4 + atom_n + i];
                    CPPAD_ASSERT_UNKNOWN( dyn_par_is[res_i] || res_i == 0 );
                    //
                    if( dyn_par_is[res_i] )
                    {   Base par = play->GetPar( size_t(res_i) );
                        if( first_dynamic_result )
                        {   first_dynamic_result = false;
                            new_par[res_i] = rec->put_dyn_par(par, call_dyn);
                        }
                        else
                            new_par[res_i] = rec->put_dyn_par(par, result_dyn);
                        arg_vec.push_back( new_par[res_i] );
                    }
                    else
                    {   // this result is a constant parameter
                        if( new_par[res_i] != addr_t_max )
                            arg_vec.push_back( new_par[res_i] );
                        else
                        {   // this constant parameter is not used
                            arg_vec.push_back(0); // phantom parameter
                        }
                    }
                }
                arg_vec.push_back( addr_t(5 + atom_n + atom_m ) );
                rec->put_dyn_arg_vec( arg_vec );
            }
        }
        else if( par_usage[i_par] & (op != result_dyn) )
        {   size_t j_dyn = size_t( dyn_previous[i_dyn] );
            if( j_dyn != num_dynamic_par )
            {   size_t j_par = size_t( dyn_ind2par_ind[j_dyn] );
                CPPAD_ASSERT_UNKNOWN( j_par < i_par );
                new_par[i_par] = new_par[j_par];
            }
            else
            {
                // value of this parameter
                Base par       = play->GetPar(i_par);
                //
                if( op == cond_exp_dyn )
                {   // cond_exp_dyn
                    CPPAD_ASSERT_UNKNOWN( num_dynamic_ind <= i_par );
                    CPPAD_ASSERT_UNKNOWN( n_arg = 5 );
                    new_par[i_par] = rec->put_dyn_cond_exp(
                        par                                ,   // par
                        CompareOp( dyn_par_arg[i_arg + 0] ),   // cop
                        new_par[ dyn_par_arg[i_arg + 1] ]  ,   // left
                        new_par[ dyn_par_arg[i_arg + 2] ]  ,   // right
                        new_par[ dyn_par_arg[i_arg + 3] ]  ,   // if_true
                        new_par[ dyn_par_arg[i_arg + 4] ]      // if_false
                    );
                }
                else if(  op == dis_dyn )
                {   // dis_dyn
                    CPPAD_ASSERT_UNKNOWN( n_arg = 2 );
                    new_par[i_par] = rec->put_dyn_par(
                        par                               ,  // par
                        op                                ,  // op
                        dyn_par_arg[i_arg + 0]            ,  // index
                        new_par[ dyn_par_arg[i_arg + 1] ]    // parameter
                    );
                }
                else if( n_arg == 1 )
                {   // cases with one argument
                    CPPAD_ASSERT_UNKNOWN( num_dynamic_ind <= i_par );
                    new_par[i_par] = rec->put_dyn_par( par, op,
                        new_par[ dyn_par_arg[i_arg + 0] ]
                    );
                }
                else if( n_arg == 2 )
                {   // cases with two arguments
                    CPPAD_ASSERT_UNKNOWN( num_dynamic_ind <= i_par );
                    new_par[i_par] = rec->put_dyn_par( par, op,
                        new_par[ dyn_par_arg[i_arg + 0] ],
                        new_par[ dyn_par_arg[i_arg + 1] ]
                    );
                }
                else
                {   // independent dynamic parmaeter case
                    CPPAD_ASSERT_UNKNOWN( op == ind_dyn )
                    CPPAD_ASSERT_UNKNOWN( i_par <= num_dynamic_ind );
                    CPPAD_ASSERT_UNKNOWN( n_arg == 0 );
                    new_par[i_par] = rec->put_dyn_par( par, op);
                }
            }
        }
        ++i_dyn;
        i_arg += n_arg;
    }
    // -----------------------------------------------------------------------
    // There is an additional constant parameter for each cumulative summation
    // (that does not have a corresponding old parameter index).
    // ------------------------------------------------------------------------
    // initialize mapping from old VecAD index to new VecAD index
    CPPAD_ASSERT_UNKNOWN(
        size_t( std::numeric_limits<addr_t>::max() ) >= num_vecad_ind
    );
    pod_vector<addr_t> new_vecad_ind(num_vecad_ind);
    for(size_t i = 0; i < num_vecad_ind; i++)
        new_vecad_ind[i] = addr_t( num_vecad_ind ); // invalid index
    {
        size_t j = 0;     // index into the old set of indices
        for(size_t i = 0; i < num_vecad_vec; i++)
        {   // length of this VecAD
            size_t length = play->GetVecInd(j);
            if( vecad_used[i] )
            {   // Put this VecAD vector in new recording
                CPPAD_ASSERT_UNKNOWN(length < num_vecad_ind);
                new_vecad_ind[j] = rec->PutVecInd( addr_t(length) );
                for(size_t k = 1; k <= length; k++) new_vecad_ind[j+k] =
                    rec->PutVecInd(
                        new_par[ play->GetVecInd(j+k) ]
                );
            }
            // start of next VecAD
            j       += length + 1;
        }
        CPPAD_ASSERT_UNKNOWN( j == num_vecad_ind );
    }

    // temporary buffer for new argument values
    addr_t new_arg[6];

    // temporary work space used by record_csum
    // (decalared here to avoid realloaction of memory)
    struct_csum_stacks csum_work;

    // tempory used to hold a size_pair
    struct_size_pair size_pair;
    //
    // Mapping from old operator index to new variable index,
    // zero is invalid except for new_var[0].
    pod_vector<addr_t> new_var(num_op);
    //
    // Mapping from old operator index to new operator index will share
    // memory with op_previous. Must get op_previous[i_op] for this operator
    // before over writting it with new_op[i_op].
    pod_vector<addr_t>& new_op( op_previous );
    CPPAD_ASSERT_UNKNOWN( new_op.size() == num_op );
    // -------------------------------------------------------------
    // information for current operator
    size_t          i_op;   // index
    OpCode          op;     // operator
    const addr_t*   arg;    // arguments
    size_t          i_var;  // variable index of primary (last) result
    //
    // information about atomic function
    enum_atom_state atom_state = start_atom;
    size_t atom_i              = 0;
    size_t atom_j              = 0;
    //
    i_var      = 0;
    for(i_op = 0; i_op < num_op; ++i_op)
    {   // if non-zero, use previous result in place of this operator.
        // Must get this information before writing new_op[i_op].
        size_t previous = size_t( op_previous[i_op] );
        //
        // zero is invalid except for new_op[0].
        new_op[i_op] = 0;
        //
        // Zero is invalid except for new_var[0] and previous is zero unless
        // this operator is replace by a previous operator.
        new_var[i_op] = 0;
        if( op_usage[i_op] == usage_t(yes_usage) )
            new_var[i_op] = new_var[previous];
        //
        // temporary used in some switch cases
        addr_t mask;
        //
        // this operator information
        size_t i_tmp;
        random_itr.op_info(i_op, op, arg, i_tmp);
        if( NumRes(op) > 0 )
            i_var = i_tmp;
        //
        // determine if we should insert a conditional skip here
        bool skip  = conditional_skip;
        if( skip )
        {   skip      &= cskip_order_next < num_cexp;
            skip      &= op != BeginOp;
            skip      &= op != InvOp;
            skip      &= atom_state == start_atom;
            if( skip )
            {   size_t j = cskip_order[cskip_order_next];
                if( NumRes(op) > 0 )
                    skip &= size_t( cexp_info[j].max_left_right ) < i_var;
                else
                    skip &= size_t( cexp_info[j].max_left_right ) <= i_var;
            }
            if( skip )
            {   size_t j = cskip_order[cskip_order_next];
                cskip_order_next++;
                size_t n_true   = skip_op_true.number_elements(j);
                size_t n_false  = skip_op_false.number_elements(j);
                skip &= n_true > 0 || n_false > 0;
                if( skip )
                {   CPPAD_ASSERT_UNKNOWN( NumRes(CSkipOp) == 0 );
                    size_t n_arg   = 7 + size_t(n_true) + size_t(n_false);
                    // reserve space for the arguments to this operator but
                    // delay setting them until we have all the new addresses
                    cskip_new[j].i_arg = rec->ReserveArg(n_arg);
                    // i_arg == 0 is used to check if conditional expression
                    // has been skipped.
                    CPPAD_ASSERT_UNKNOWN( cskip_new[j].i_arg > 0 );
                    // There is no corresponding old operator in this case
                    rec->PutOp(CSkipOp);
                }
            }
        }
        //
        CPPAD_ASSERT_UNKNOWN(
            size_t( std::numeric_limits<addr_t>::max() ) >= rec->num_op_rec()
        );
        //
        // For each call, first and second AFunOp will have same op_usage
        skip  = op_usage[i_op] != usage_t( yes_usage );
        skip &= atom_state != arg_atom && atom_state != ret_atom;
        if( skip )
        {   if( op == CExpOp )
                ++cexp_next;
            //
            if( op == AFunOp )
            {   if( atom_state == start_atom )
                    atom_state = end_atom;
                else
                {   CPPAD_ASSERT_UNKNOWN( atom_state == end_atom );
                    atom_state = start_atom;
                }
            }
        }
        else switch( op )
        {   // op_usage[i_op] == usage_t(yes_usage)

            case BeginOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            // Put BeginOp at beginning of recording
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            new_var[i_op] = rec->PutOp(BeginOp);
            rec->PutArg(arg[0]);
            break;

            // --------------------------------------------------------------
            // Unary operators, argument a variable, one result
            case AbsOp:
            case AcosOp:
            case AcoshOp:
            case AsinOp:
            case AsinhOp:
            case AtanOp:
            case AtanhOp:
            case CosOp:
            case CoshOp:
            case ErfOp:
            case ExpOp:
            case Expm1Op:
            case LogOp:
            case Log1pOp:
            case SignOp:
            case SinOp:
            case SinhOp:
            case SqrtOp:
            case TanOp:
            case TanhOp:
            if( previous == 0 )
            {   //
                new_arg[0]   = new_var[ random_itr.var2op(size_t(arg[0])) ];
                rec->PutArg( new_arg[0] );
                //
                new_op[i_op]  = addr_t( rec->num_op_rec() );
                new_var[i_op] = rec->PutOp(op);
                CPPAD_ASSERT_UNKNOWN(
                    new_arg[0] < new_var[random_itr.var2op(i_var)]
                );
                if( op == ErfOp )
                {   CPPAD_ASSERT_NARG_NRES(op, 3, 5);
                    // Error function is a special case
                    // second argument is always the parameter 0
                    // third argument is always the parameter 2 / sqrt(pi)
                    CPPAD_ASSERT_UNKNOWN( NumArg(ErfOp) == 3 );
                    rec->PutArg( rec->put_con_par( Base(0.0) ) );
                    rec->PutArg( rec->put_con_par(
                        Base( 1.0 / std::sqrt( std::atan(1.0) ) )
                    ) );
                }
                else
                {   // some of these operators have an auxillary result;
                    // e.g. sine and cosine are computed together.
                    CPPAD_ASSERT_UNKNOWN( NumArg(op) == 1 );
                    CPPAD_ASSERT_UNKNOWN( NumRes(op) ==1 || NumRes(op) == 2 );
                }
            }
            break;
            // ---------------------------------------------------
            // Binary operators, left variable, right parameter, one result
            case SubvpOp:
            // check if this is the top of a csum connection
            if( op_usage[ random_itr.var2op(size_t(arg[0])) ] == usage_t(csum_usage) )
            {   CPPAD_ASSERT_UNKNOWN( previous == 0 );
                //
                // convert to a sequence of summation operators
                size_pair = record_csum(
                    play                ,
                    random_itr          ,
                    op_usage            ,
                    new_par             ,
                    new_var             ,
                    i_var               ,
                    rec                 ,
                    csum_work
                );
                new_op[i_op]  = addr_t( size_pair.i_op );
                new_var[i_op] = addr_t( size_pair.i_var );
                // abort rest of this case
                break;
            }
            case DivvpOp:
            case PowvpOp:
            case ZmulvpOp:
            if( previous == 0 )
            {   //
                size_pair = record_vp(
                    play                ,
                    random_itr          ,
                    new_par             ,
                    new_var             ,
                    i_op                ,
                    rec
                );
                new_op[i_op]  = addr_t( size_pair.i_op );
                new_var[i_op] = addr_t( size_pair.i_var );
            }
            break;
            // ---------------------------------------------------
            // Binary operators, left index, right variable, one result
            case DisOp:
            CPPAD_ASSERT_NARG_NRES(op, 2, 1);
            if( previous == 0 )
            {   //
                new_arg[0] = arg[0];
                new_arg[1] = new_var[ random_itr.var2op(size_t(arg[1])) ];
                rec->PutArg( new_arg[0], new_arg[1] );
                //
                new_op[i_op]  = addr_t( rec->num_op_rec() );
                new_var[i_op] = rec->PutOp(op);
                CPPAD_ASSERT_UNKNOWN(
                    new_arg[1] < new_var[random_itr.var2op(i_var)]
                );
            }
            break;

            // ---------------------------------------------------
            // Binary operators, left parameter, right variable, one result
            case SubpvOp:
            case AddpvOp:
            // check if this is the top of a csum connection
            if( op_usage[ random_itr.var2op(size_t(arg[1])) ] == usage_t(csum_usage) )
            {   CPPAD_ASSERT_UNKNOWN( previous == 0 );
                //
                // convert to a sequence of summation operators
                size_pair = record_csum(
                    play                ,
                    random_itr          ,
                    op_usage            ,
                    new_par             ,
                    new_var             ,
                    i_var               ,
                    rec                 ,
                    csum_work
                );
                new_op[i_op]  = addr_t( size_pair.i_op );
                new_var[i_op] = addr_t( size_pair.i_var );
                // abort rest of this case
                break;
            }
            case DivpvOp:
            case MulpvOp:
            case PowpvOp:
            case ZmulpvOp:
            if( previous == 0 )
            {   //
                size_pair = record_pv(
                    play                ,
                    random_itr          ,
                    new_par             ,
                    new_var             ,
                    i_op                ,
                    rec
                );
                new_op[i_op]  = addr_t( size_pair.i_op );
                new_var[i_op] = addr_t( size_pair.i_var );
            }
            break;
            // ---------------------------------------------------
            // Binary operator, left and right variables, one result
            case AddvvOp:
            case SubvvOp:
            // check if this is the top of a csum connection
            if(
                op_usage[ random_itr.var2op(size_t(arg[0])) ] == usage_t(csum_usage)
                ||
                op_usage[ random_itr.var2op(size_t(arg[1])) ] == usage_t(csum_usage)
            )
            {   CPPAD_ASSERT_UNKNOWN( previous == 0 );
                //
                // convert to a sequence of summation operators
                size_pair = record_csum(
                    play                ,
                    random_itr          ,
                    op_usage            ,
                    new_par             ,
                    new_var             ,
                    i_var               ,
                    rec                 ,
                    csum_work
                );
                new_op[i_op]  = addr_t( size_pair.i_op );
                new_var[i_op] = addr_t( size_pair.i_var );
                // abort rest of this case
                break;
            }
            case DivvvOp:
            case MulvvOp:
            case PowvvOp:
            case ZmulvvOp:
            if( previous == 0 )
            {   //
                size_pair = record_vv(
                    play                ,
                    random_itr          ,
                    new_var             ,
                    i_op                ,
                    rec
                );
                new_op[i_op]  = addr_t( size_pair.i_op );
                new_var[i_op] = addr_t( size_pair.i_var );
            }
            break;
            // ---------------------------------------------------
            // Conditional expression operators
            case CExpOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 6, 1);
            new_arg[0] = arg[0];
            new_arg[1] = arg[1];
            mask = 1;
            for(size_t i = 2; i < 6; i++)
            {   if( arg[1] & mask )
                {   new_arg[i] = new_var[ random_itr.var2op(size_t(arg[i])) ];
                    CPPAD_ASSERT_UNKNOWN(
                        size_t(new_arg[i]) < num_var
                    );
                }
                else
                    new_arg[i] = new_par[ arg[i] ];
                mask = mask << 1;
            }
            rec->PutArg(
                new_arg[0] ,
                new_arg[1] ,
                new_arg[2] ,
                new_arg[3] ,
                new_arg[4] ,
                new_arg[5]
            );
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            new_var[i_op] = rec->PutOp(op);
            //
            // The new addresses for left and right are used during
            // fill in the arguments for the CSkip operations. This does not
            // affect max_left_right which is used during this sweep.
            if( conditional_skip )
            {   CPPAD_ASSERT_UNKNOWN( cexp_next < num_cexp );
                CPPAD_ASSERT_UNKNOWN(
                    size_t( cexp_info[cexp_next].i_op ) == i_op
                );
                cskip_new[ cexp_next ].left  = size_t( new_arg[2] );
                cskip_new[ cexp_next ].right = size_t( new_arg[3] );
                ++cexp_next;
            }
            break;
            // ---------------------------------------------------
            // Operations with no arguments and no results
            case EndOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 0, 0);
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(op);
            break;
            // ---------------------------------------------------
            // Comparison operations: two arguments and no results
            case LepvOp:
            case LtpvOp:
            case EqpvOp:
            case NepvOp:
            CPPAD_ASSERT_UNKNOWN( compare_op );
            CPPAD_ASSERT_NARG_NRES(op, 2, 0);
            if( previous == 0 )
            {   new_arg[0] = new_par[ arg[0] ];
                new_arg[1] = new_var[ random_itr.var2op(size_t(arg[1])) ];
                rec->PutArg(new_arg[0], new_arg[1]);
                new_op[i_op] = addr_t( rec->num_op_rec() );
                rec->PutOp(op);
            }
            break;
            //
            case LevpOp:
            case LtvpOp:
            CPPAD_ASSERT_UNKNOWN( compare_op );
            CPPAD_ASSERT_NARG_NRES(op, 2, 0);
            if( previous == 0 )
            {   new_arg[0] = new_var[ random_itr.var2op(size_t(arg[0])) ];
                new_arg[1] = new_par[ arg[1] ];
                rec->PutArg(new_arg[0], new_arg[1]);
                new_op[i_op] = addr_t( rec->num_op_rec() );
                rec->PutOp(op);
            }
            break;
            //
            case LevvOp:
            case LtvvOp:
            case EqvvOp:
            case NevvOp:
            CPPAD_ASSERT_UNKNOWN( compare_op );
            CPPAD_ASSERT_NARG_NRES(op, 2, 0);
            if( previous == 0 )
            {   new_arg[0] = new_var[ random_itr.var2op(size_t(arg[0])) ];
                new_arg[1] = new_var[ random_itr.var2op(size_t(arg[1])) ];
                rec->PutArg(new_arg[0], new_arg[1]);
                new_op[i_op] = addr_t( rec->num_op_rec() );
                rec->PutOp(op);
            }
            break;

            // ---------------------------------------------------
            // Operations with no arguments and one result
            case InvOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 0, 1);
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            new_var[i_op] = rec->PutOp(op);
            break;

            // ---------------------------------------------------
            // Unary operators, argument a parameter, one result
            case ParOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 1, 1);
            new_arg[0] = new_par[ arg[0] ];
            rec->PutArg( new_arg[0] );
            //
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            new_var[i_op] = rec->PutOp(op);
            break;

            // ---------------------------------------------------
            // print forward operator
            case PriOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 5, 0);
            // arg[0]
            new_arg[0] = arg[0];
            //
            // arg[1]
            if( arg[0] & 1 )
            {   new_arg[1] = new_var[ random_itr.var2op(size_t(arg[1])) ];
                CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
            }
            else
            {   new_arg[1] = new_par[ arg[1] ];
            }
            //
            // arg[3]
            if( arg[0] & 2 )
            {   new_arg[3] = new_var[ random_itr.var2op(size_t(arg[3])) ];
                CPPAD_ASSERT_UNKNOWN( size_t(new_arg[3]) < num_var );
            }
            else
            {   new_arg[3] = new_par[ arg[3] ];
            }
            new_arg[2] = rec->PutTxt( play->GetTxt(size_t(arg[2])) );
            new_arg[4] = rec->PutTxt( play->GetTxt(size_t(arg[4])) );
            //
            rec->PutArg(
                new_arg[0] ,
                new_arg[1] ,
                new_arg[2] ,
                new_arg[3] ,
                new_arg[4]
            );
            // new operator
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            // no new variable
            rec->PutOp(op);
            break;

            // ---------------------------------------------------
            // VecAD operators

            // Load using a parameter index
            case LdpOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 3, 1);
            new_arg[0] = new_vecad_ind[ arg[0] ];
            new_arg[1] = arg[1];
            CPPAD_ASSERT_UNKNOWN(
                size_t( std::numeric_limits<addr_t>::max() ) >= rec->num_load_op_rec()
            );
            new_arg[2] = addr_t( rec->num_load_op_rec() );
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
            rec->PutArg(
                new_arg[0],
                new_arg[1],
                new_arg[2]
            );
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            new_var[i_op] = rec->PutLoadOp(op);
            break;

            // Load using a variable index
            case LdvOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 3, 1);
            new_arg[0] = new_vecad_ind[ arg[0] ];
            new_arg[1] = new_var[ random_itr.var2op(size_t(arg[1])) ];
            CPPAD_ASSERT_UNKNOWN(
                size_t( std::numeric_limits<addr_t>::max() ) >= rec->num_load_op_rec()
            );
            new_arg[2] = addr_t( rec->num_load_op_rec() );
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
            rec->PutArg(
                new_arg[0],
                new_arg[1],
                new_arg[2]
            );
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            new_var[i_op] = rec->PutLoadOp(op);
            break;

            // Store a parameter using a parameter index
            case StppOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 3, 0);
            new_arg[0] = new_vecad_ind[ arg[0] ];
            new_arg[1] = new_par[ arg[1] ];
            new_arg[2] = new_par[ arg[2] ];
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
            rec->PutArg(
                new_arg[0],
                new_arg[1],
                new_arg[2]
            );
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(op);
            break;

            // Store a parameter using a variable index
            case StvpOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 3, 0);
            new_arg[0] = new_vecad_ind[ arg[0] ];
            new_arg[1] = new_var[ random_itr.var2op(size_t(arg[1])) ];
            new_arg[2] = new_par[ arg[2] ];
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
            rec->PutArg(
                new_arg[0],
                new_arg[1],
                new_arg[2]
            );
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(op);
            break;

            // Store a variable using a parameter index
            case StpvOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 3, 0);
            new_arg[0] = new_vecad_ind[ arg[0] ];
            new_arg[1] = new_par[ arg[1] ];
            new_arg[2] = new_var[ random_itr.var2op(size_t(arg[2])) ];
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[2]) < num_var );
            rec->PutArg(
                new_arg[0],
                new_arg[1],
                new_arg[2]
            );
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(op);
            break;

            // Store a variable using a variable index
            case StvvOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 3, 0);
            new_arg[0] = new_vecad_ind[ arg[0] ];
            new_arg[1] = new_var[ random_itr.var2op(size_t(arg[1])) ];
            new_arg[2] = new_var[ random_itr.var2op(size_t(arg[2])) ];
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[0]) < num_vecad_ind );
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[1]) < num_var );
            CPPAD_ASSERT_UNKNOWN( size_t(new_arg[2]) < num_var );
            rec->PutArg(
                new_arg[0],
                new_arg[1],
                new_arg[2]
            );
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(op);
            break;

            // -----------------------------------------------------------
            // atomic function call operators

            case AFunOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 4, 0);
            // atom_index, atom_old, atom_n, atom_m
            rec->PutArg(arg[0], arg[1], arg[2], arg[3]);
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(AFunOp);
            if( atom_state == start_atom )
            {   atom_state = arg_atom;
                atom_j     = size_t( arg[2] ); // just for counting arguments
                atom_i     = size_t( arg[3] ); // just for counting results
                CPPAD_ASSERT_UNKNOWN( atom_j > 0 );
                CPPAD_ASSERT_UNKNOWN( atom_i > 0 );
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( atom_state == end_atom );
                atom_state = start_atom;
            }
            break;

            case FunapOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            new_arg[0] = new_par[ arg[0] ];
            if( new_arg[0] != addr_t_max )
                rec->PutArg(new_arg[0]);
            else
                rec->PutArg(0); // argument not used
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(FunapOp);
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            --atom_j;
            if( atom_j == 0 )
                atom_state = ret_atom;
            break;

            case FunavOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            new_arg[0] = new_var[ random_itr.var2op(size_t(arg[0])) ];
            if( size_t(new_arg[0]) < num_var )
            {   rec->PutArg(new_arg[0]);
                new_op[i_op] = addr_t( rec->num_op_rec() );
                rec->PutOp(FunavOp);
            }
            else
            {   // This argument does not affect the result and
                // has been optimized out so use nan in its place.
                new_arg[0] = rec->put_con_par( base_nan );
                rec->PutArg(new_arg[0]);
                new_op[i_op] = addr_t( rec->num_op_rec() );
                rec->PutOp(FunapOp);
            }
            CPPAD_ASSERT_UNKNOWN( atom_state == arg_atom );
            --atom_j;
            if( atom_j == 0 )
                atom_state = ret_atom;
            break;

            case FunrpOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 1, 0);
            new_arg[0] = new_par[ arg[0] ];
            if( new_arg[0] != addr_t_max )
            {   // This parameter is used, but may not by this operation
                rec->PutArg(new_arg[0]);
            }
            else
            {   // This parameter is not used here or anywhere.
                CPPAD_ASSERT_UNKNOWN( op_usage[i_op] == usage_t(no_usage) );
                rec->PutArg(0); // result not used
            }
            new_op[i_op] = addr_t( rec->num_op_rec() );
            rec->PutOp(FunrpOp);
            CPPAD_ASSERT_UNKNOWN( atom_state == ret_atom );
            --atom_i;
            if( atom_i == 0 )
                atom_state = end_atom;
            break;

            case FunrvOp:
            CPPAD_ASSERT_UNKNOWN( previous == 0 );
            CPPAD_ASSERT_NARG_NRES(op, 0, 1);
            new_op[i_op]  = addr_t( rec->num_op_rec() );
            if( op_usage[i_op] == usage_t(yes_usage) )
                new_var[i_op] = rec->PutOp(FunrvOp);
            else
            {   // change FunrvOp -> FunrpOp to avoid creating new variable
                CPPAD_ASSERT_UNKNOWN( op_usage[i_op] == usage_t(no_usage) );
                CPPAD_ASSERT_NARG_NRES(FunrpOp, 1, 0);
                rec->PutArg(0); // result not used
                rec->PutOp(FunrpOp);
            }

            --atom_i;
            if( atom_i == 0 )
                atom_state = end_atom;
            break;
            // ---------------------------------------------------

            // all cases should be handled above
            default:
            CPPAD_ASSERT_UNKNOWN(false);

        }
    }
    // modify the dependent variable vector to new indices
    for(size_t i = 0; i < dep_taddr.size(); i++ )
    {   dep_taddr[i] = size_t(new_var[ random_itr.var2op(dep_taddr[i]) ]);
        CPPAD_ASSERT_UNKNOWN( size_t(dep_taddr[i]) < num_var );
    }

# ifndef NDEBUG
    for(i_op = 0; i_op < num_op; i_op++)
    {   random_itr.op_info(i_op, op, arg, i_var);
        if( NumRes(op) > 0 )
            CPPAD_ASSERT_UNKNOWN(
                size_t(new_op[i_op]) < rec->num_op_rec()
            );
    }
# endif
    // make sure that all the conditional expressions have been
    // checked to see if they are still present
    CPPAD_ASSERT_UNKNOWN( cskip_order_next == num_cexp );
    // fill in the arguments for the CSkip operations
    for(size_t i = 0; i < num_cexp; i++)
    {   // if cskip_new[i].i_arg == 0, this conditional expression was skipped
        if( cskip_new[i].i_arg > 0 )
        {   // size_t i_arg
            struct_cexp_info info = cexp_info[i];
            addr_t n_true  = addr_t( skip_op_true.number_elements(i) );
            addr_t n_false = addr_t( skip_op_false.number_elements(i) );
            i_arg          = cskip_new[i].i_arg;
            addr_t left    = addr_t( cskip_new[i].left );
            addr_t right   = addr_t( cskip_new[i].right );
            rec->ReplaceArg(i_arg++, addr_t(info.cop)   );
            rec->ReplaceArg(i_arg++, addr_t(info.flag)  );
            rec->ReplaceArg(i_arg++, left  );
            rec->ReplaceArg(i_arg++, right );
            rec->ReplaceArg(i_arg++, n_true     );
            rec->ReplaceArg(i_arg++, n_false    );
            sparse_list::const_iterator itr_true(skip_op_true, i);
            while( *itr_true != skip_op_true.end() )
            {   i_op = *itr_true;
                // op_usage[i_op] == usage_t(yes_usage)
                CPPAD_ASSERT_UNKNOWN( new_op[i_op] != 0 );
                rec->ReplaceArg(i_arg++, new_op[i_op] );
                //
                ++itr_true;
            }
            sparse_list::const_iterator itr_false(skip_op_false, i);
            while( *itr_false != skip_op_false.end() )
            {   i_op   = *itr_false;
                // op_usage[i_op] == usage_t(yes_usage)
                CPPAD_ASSERT_UNKNOWN( new_op[i_op] != 0 );
                rec->ReplaceArg(i_arg++, new_op[i_op] );
                //
                ++itr_false;
            }
            rec->ReplaceArg(i_arg++, n_true + n_false);
# ifndef NDEBUG
            size_t n_arg   = 7 + size_t(n_true) + size_t(n_false);
            CPPAD_ASSERT_UNKNOWN( cskip_new[i].i_arg + n_arg == i_arg );
# endif
        }
    }
}

} } } // END_CPPAD_LOCAL_OPTIMIZE_NAMESPACE

# endif
