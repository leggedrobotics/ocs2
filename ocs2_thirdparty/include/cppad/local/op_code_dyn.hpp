# ifndef CPPAD_LOCAL_OP_CODE_DYN_HPP
# define CPPAD_LOCAL_OP_CODE_DYN_HPP
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
\file op_code_dyn.hpp
Defines the op_code_dyn enum type

*/

/// Parameter only op codes, at least one operand is a dynamic parameter.
/// The following dynamic parameter operators as in the OpCode enum type:
/// EqppOp, NeppOp, LtppOp, LeppOp
enum op_code_dyn {
    abs_dyn,       // abs(parameter)
    acos_dyn,      // acos(parameter)
    acosh_dyn,     // acosh(parameter)
    add_dyn,       // parameter + parameter
    asin_dyn,      // asin(parameter)
    asinh_dyn,     // asinh(parameter)
    atan_dyn,      // atan(parameter)
    atanh_dyn,     // atanh(parameter)
    call_dyn,      // atomic function call
    // arg[0]     = atom_index for this function; see call_atomic.
    // arg[1]     = n ia number of arguments to this atomic function
    // arg[2]     = m is  number of results for this atomic function
    // arg[3]     = n_dyn is number of results that are dynamic parameters
    // arg[4+j]   = atomic argument parameter index for j = 0, ..., n
    // arg[4+n+i] = atomic result parameter index for i = 0, ..., m
    // arg[4+n+m] = 5 + n + m = number arguments to this operator
    cond_exp_dyn,  // cond_exp(cop, left, right, if_true, if_false)
    cos_dyn,       // cos(parameter)
    cosh_dyn,      // cosh(parameter)
    dis_dyn,       // discrete function (index, parameter)
    div_dyn,       // parameter / parameter
    erf_dyn,       // erf(parameter)
    exp_dyn,       // exp(parameter)
    expm1_dyn,     // expm1(parameter)
    fabs_dyn,      // fabs(parameter)
    ind_dyn,       // independent parameter
    log_dyn,       // log(parameter)
    log1p_dyn,     // log1p(parameter)
    mul_dyn,       // parameter * parameter
    pow_dyn,       // pow(parameter,    parameter)
    result_dyn,    // atomic function result
    sign_dyn,      // sign(parameter)
    sin_dyn,       // sin(parameter)
    sinh_dyn,      // sinh(parameter)
    sqrt_dyn,      // sqrt(parameter)
    sub_dyn,       // parameter - parameter
    tan_dyn,       // tan(parameter)
    tanh_dyn,      // tan(parameter)
    zmul_dyn,      // azmul(parameter, parameter)
    number_dyn     // number of operator codes (not an operator)
};
// 2DO: Following operators in OpCode need to be extended to parameter only:
// CExpOp, AFunOp


/// number of arguments for each dynamic parameter operator
inline size_t num_arg_dyn(op_code_dyn op)
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;

    static const size_t num_arg_table[] = {
        1, // abs_dyn
        1, // acos_dyn
        1, // acosh_dyn
        2, // add_dyn
        1, // asin_dyn
        1, // asinh_dyn
        1, // atan_dyn
        1, // atanh_dyn
        0, // call_dyn: this operator has a variable number of arguments
        5, // cond_exp_dyn
        1, // cos_dyn
        1, // cosh_dyn
        2, // dis_dyn
        2, // div_dyn
        1, // erf_dyn
        1, // exp_dyn
        1, // expm1_dyn
        1, // fabs_dyn
        0, // ind_dyn
        1, // log_dyn
        1, // log1p_dyn
        2, // mul_dyn
        2, // pow_dyn
        0, // result_dyn
        1, // sign_dyn
        1, // sin_dyn
        1, // sinh_dyn
        1, // sqrt_dyn
        2, // sub_dyn
        1, // tan_dyn
        1, // tanh_dyn
        2, // zmul_dyn
        0  // number_dyn (not used)
    };
    static bool first = true;
    if( first )
    {   CPPAD_ASSERT_UNKNOWN(
        size_t(number_dyn)+1 == sizeof(num_arg_table)/sizeof(num_arg_table[0])
        );
        first = false;
    }
    return num_arg_table[op];
}

/// name for each operator
inline const char* op_name_dyn(op_code_dyn op)
{   CPPAD_ASSERT_FIRST_CALL_NOT_PARALLEL;

    static const char* op_name_table[] = {
        "abs",
        "acos",
        "acosh",
        "add",
        "asin",
        "asinh",
        "atan",
        "atanh",
        "call",
        "cond_exp",
        "cos",
        "cosh",
        "dis",
        "div",
        "erf",
        "exp",
        "expm1",
        "fabs",
        "ind",
        "log",
        "log1p",
        "mul",
        "pow",
        "result",
        "sign",
        "sin",
        "sinh",
        "sqrt",
        "sub",
        "tan",
        "tanh",
        "zmul",
        "number"
    };
    static bool first = true;
    if( first )
    {   CPPAD_ASSERT_UNKNOWN(
        size_t(number_dyn)+1 == sizeof(op_name_table)/sizeof(op_name_table[0])
        );
        first = false;
    }
    return op_name_table[op];
}

} } // END_CPPAD_LOCAL_NAMESPACE

# endif
