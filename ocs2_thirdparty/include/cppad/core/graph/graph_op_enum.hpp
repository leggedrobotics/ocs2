# ifndef CPPAD_CORE_GRAPH_GRAPH_OP_ENUM_HPP
# define CPPAD_CORE_GRAPH_GRAPH_OP_ENUM_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

  CppAD is distributed under the terms of the
               Eclipse Public License Version 2.0.

  This Source Code may also be made available under the following
  Secondary License when the conditions for such availability set forth
  in the Eclipse Public License, Version 2.0 are satisfied:
        GNU General Public License, Version 2.0 or later.
-------------------------------------------------------------------------- */

# include <cstddef>
# include <string>
# include <map>

# include <cppad/utility/vector.hpp>
# include <cppad/configure.hpp>

/*
$begin graph_op_enum$$
$spell
    Cond
    asinh
    acosh
    atanh
    erf
    erfc
    expm
    vec
    enum
    Exp
    Gt
    Le
$$

$section C++ AD Graph Operator Enum Type$$

$head Unary$$
The unary operators have one argument and one result node.
The argument is a node index and the result is the next node.

$subhead Require C++11$$
The following unary operators require a compiler that supports c++11:
$code asinh$$, $code acosh$$, $code atanh$$,
$code erf$$, $code erfc$$,
$code expm1$$, $code log1p$$.

$head Binary$$
The binary operators have two arguments and one result node.
The arguments are node indices and the result is the next node.
The first (second) argument is the left (right) operand node index.

$head Conditional Expression$$
The conditional expression operators have four arguments and one result node.
The arguments are node indices and the result is the next node.
The first argument is $cref/left/CondExp/left/$$,
the second is $cref/right/CondExp/right/$$,
the third is $cref/if_true/CondExp/if_true/$$,
the fourth is $cref/if_false/CondExp/if_false/$$,
the result is given by
$codei%
    if( %left% %cop% %right%)
        %result% = %if_true%;
    else
        %result% = %if_false%;
%$$
where $icode cop$$ is given in the comment after the enum type values below.

$subhead Other Comparisons$$
Note that
$codei%
    CondExpGt(%left%, %right%, %if_true%, %if_false%)
%$$
is equivalent to
$codei%
    CondExpLe(%left%, %right%, %if_false%, %if_true%)
%$$
Similar conversions can be used for all the possible
$cref/conditional expressions/CondExp/$$.

$head Comparison$$
The comparison operators have two arguments and no result node.
The first (second) argument is the left (right) operand node index.
The comparison result was true for the value of the independent
dynamic parameters and independent variables at which this graph was created.

$subhead Other Comparisons$$
The comparison result true for $icode%left% > %right%$$
is equivalent to the comparison result true for $icode%right% < %left%$$.
The comparison result false for $icode%left% > %right%$$
is equivalent to the comparison result true for $icode%left% <= %right%$$.
In a similar fashion, all the possible comparisons results
can be converted to a true result for one of the comparisons above.

$head Summation$$
The summation operator has one node result and a variable
number of arguments.
The first argument is the number of nodes in the summation,
and the other arguments are the indices of the nodes to be summed.
The total number of arguments for this operator
is one plus the number of nodes in the summation.

$head Atomic Function$$
The atomic function operator has a variable number of arguments
and a variable number of node results.
The total number of arguments for this operator is three plus the number
of arguments for the function being called.
$list number$$
The first argument is the index in
$cref/atomic_name_vec/cpp_ad_graph/atomic_name_vec/$$ for the
$cref/name/atomic_three_ctor/atomic_three/name/$$
of the $code atomic_three$$ function that is called.
$lnext
The second argument is the number of result for this function call.
The order of the results is determined by function being called.
$lnext
The third argument is the number of arguments
for this function call.
$lnext
The other arguments are the indices of nodes for each argument to the
function call.  The order of the arguments is determined by function
being called.
$lend

$head Enum Values$$
$srcfile%include/cppad/core/graph/graph_op_enum.hpp%
    0%// BEGIN_SORT_THIS_LINE_PLUS_2%// END_SORT_THIS_LINE_MINUS_3%1
%$$

$head Examples$$

$childtable%
    example/graph/unary_op.cpp%
    example/graph/add_op.cpp%
    example/graph/div_op.cpp%
    example/graph/mul_op.cpp%
    example/graph/sub_op.cpp%
    example/graph/sum_op.cpp%
    example/graph/comp_op.cpp%
    example/graph/cexp_op.cpp%
    example/graph/atom_op.cpp
%$$

$end
*/
// BEGIN_SORT_THIS_LINE_PLUS_2
namespace CppAD { namespace graph {
    enum graph_op_enum {
        abs_graph_op,      // unary
        acos_graph_op,     // unary
        acosh_graph_op,    // unary
        add_graph_op,      // binary
        asin_graph_op,     // unary
        asinh_graph_op,    // unary
        atan_graph_op,     // unary
        atanh_graph_op,    // unary
        atom_graph_op,     // atomic function
        cexp_eq_graph_op,  // conditional expression ==
        cexp_le_graph_op,  // conditional expression <=
        cexp_lt_graph_op,  // conditional expression <
        comp_eq_graph_op,  // comparison  ==
        comp_le_graph_op,  // comparison  <=
        comp_lt_graph_op,  // comparison  <
        comp_ne_graph_op,  // comparison  !=
        cos_graph_op,      // unary
        cosh_graph_op,     // unary
        div_graph_op,      // binary
        erf_graph_op,      // unary
        erfc_graph_op,     // unary
        exp_graph_op,      // unary
        expm1_graph_op,    // unary
        log1p_graph_op,    // unary
        log_graph_op,      // unary
        mul_graph_op,      // binary
        sign_graph_op,     // unary
        sin_graph_op,      // unary
        sinh_graph_op,     // unary
        sqrt_graph_op,     // unary
        sub_graph_op,      // binary
        sum_graph_op,      // summation
        tan_graph_op,      // unary
        tanh_graph_op,     // unary
        n_graph_op         // number of operators defined so far
    };
} }
// END_SORT_THIS_LINE_MINUS_3


# endif
