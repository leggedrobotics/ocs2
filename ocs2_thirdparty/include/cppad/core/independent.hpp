# ifndef CPPAD_CORE_INDEPENDENT_HPP
# define CPPAD_CORE_INDEPENDENT_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
/*
---------------------------------------------------------------------------

$begin Independent$$
$spell
    op
    alloc
    num
    Cpp
    bool
    const
    var
    typename
$$

$section Declare Independent Variables and Start Recording$$

$head Syntax$$
$codei%Independent(%x%)
%$$
$codei%Independent(%x%, %abort_op_index%)
%$$
$codei%Independent(%x%, %abort_op_index%, %record_compare%)
%$$
$codei%Independent(%x%, %abort_op_index%, %record_compare%, %dynamic%)
%$$

$head Start Recording$$
The syntax above starts recording
$cref/AD of Base/glossary/AD of Base/$$ operations
with $icode x$$ as the independent variable vector.
Once the
$cref/operation sequence/glossary/Operation/Sequence/$$ is completed,
it must be transferred to a function object or aborted; see below.

$head Stop Recording$$
The recording is stopped,
and the operation sequence is transferred to the AD function object $icode f$$,
using either the $cref/function constructor/FunConstruct/$$
$codei%
    ADFun<%Base%> %f%(%x%, %y%)
%$$
or the $cref/dependent variable specifier/Dependent/$$
$codei%
    %f%.Dependent(%x%, %y%)
%$$
The only other way to stop a recording is using
$cref abort_recording$$.
Between when the recording is started and when it stopped,
we refer to the elements of $icode x$$,
and the values that depend on the elements of $icode x$$,
as $codei%AD<%Base%>%$$ variables.

$head x$$
The vector $icode x$$ has prototype
$codei%
    %ADVector% &%x%
%$$
(see $icode ADVector$$ below).
The size of the vector $icode x$$, must be greater than zero,
and is the number of independent variables for this
AD operation sequence.

$head abort_op_index$$
If this argument is present,
it specifies the operator index at which the execution will aborted
by calling the CppAD $cref/error handler/ErrorHandler/$$.
When this error handler leads to an assert, the user
can inspect the call stack to see the source code corresponding to
this operator index; see
$cref/purpose/compare_change/op_index/Purpose/$$ for $icode op_index$$
and $cref/NDEBUG/Faq/Speed/NDEBUG/$$.
No abort will occur if $icode abort_op_index$$ is zero.
If this argument is not present, the default value zero is used
for $icode abort_index$$.

$head record_compare$$
This argument has prototype
$codei%
    bool %record_compare%
%$$
If it is present,
it specifies if AD $cref compare$$  operations are recorded.
It takes extra time and memory to record these operations.
On the other hand, they can be useful for detecting when and why
a functions recording would change; see $icode abort_op_index$$ above and
$cref compare_change$$.
If this argument is not present, the default value $code true$$ is used
for $icode record_compare$$.
If this argument is false, $icode abort_op_index$$ must be zero.

$head dynamic$$
If this argument is present, it has prototype
$codei%
    const %ADVector%& %dynamic%
%$$
(see $icode Vector$$ below).
It specifies the independent
$cref/dynamic/glossary/Parameter/Dynamic/$$ parameters.
The value of these parameters,
in the $cref ADFun$$ object $icode f$$,
that can be changed using $cref new_dynamic$$.

$subhead Efficiency$$
Any operations that use dynamic parameters will be recorded.
We use other dynamic parameters to denote parameters that depend on
the independent dynamic parameters $icode dynamic$$,
and do not depend on $icode x$$.
It is more efficient to compute other dynamic parameters before calling
$code Independent$$ and include them in the
independent dynamic parameter vector $icode dynamic$$.

$head ADVector$$
The type $icode ADVector$$ must be a $cref SimpleVector$$ class with
$cref/elements of type/SimpleVector/Elements of Specified Type/$$
$codei%AD<%Base%>%$$.
The routine $cref CheckSimpleVector$$ will generate an error message
if this is not the case.

$head Parallel Mode$$
Each thread can have one, and only one, active recording.
A call to $code Independent$$ starts the recording for the current thread.
The recording must be stopped by a corresponding call to
$codei%
    ADFun<%Base%> %f%( %x%, %y%)
%$$
or
$codei%
    %f%.Dependent( %x%, %y%)
%$$
or $cref abort_recording$$
preformed by the same thread; i.e.,
$cref/thread_alloc::thread_num/ta_thread_num/$$ must be the same.

$head Example$$
$children%
    example/general/independent.cpp
%$$
The file
$cref independent.cpp$$
contains an example and test of this operation.

$end
-----------------------------------------------------------------------------
*/
# include <cppad/local/independent.hpp>
/*!
\file core/independent.hpp
Declare the independent variables
*/

namespace CppAD { // BEGIN_CPPAD_NAMESPACE

/*!
Declaration of independent variables.

\tparam ADVector
This is simple vector type with elements of type AD<Base>.

\param x
Vector of the independent variablers.

\param abort_op_index
operator index at which execution will be aborted (during  the recording
of operations). The value zero corresponds to not aborting (will not match).

\param record_compare
should comparison operators be recorded.

\param dynamic
is the dynamic parameter vector.
*/
template <class ADVector>
void Independent(
    ADVector&  x              ,
    size_t     abort_op_index ,
    bool       record_compare ,
    ADVector&  dynamic        )
{   CPPAD_ASSERT_KNOWN(
        abort_op_index == 0 || record_compare,
        "Independent: abort_op_index is non-zero and record_compare is false."
    );
    typedef typename ADVector::value_type ADBase;
    typedef typename ADBase::value_type   Base;
    CPPAD_ASSERT_KNOWN(
        ADBase::tape_ptr() == CPPAD_NULL,
        "Independent: cannot create a new tape because\n"
        "a previous tape is still active (for this thread).\n"
        "AD<Base>::abort_recording() would abort this previous recording."
    );
    local::ADTape<Base>* tape = ADBase::tape_manage(new_tape_manage);
    tape->Independent(x, abort_op_index, record_compare, dynamic);
}
// ---------------------------------------------------------------------------
/*!
Declare independent variables using default for dynamic.

\tparam ADVector
This is simple vector type with elements of type AD<Base>.

\param x
Vector of the independent variablers.

\param abort_op_index
operator index at which execution will be aborted (during  the recording
of operations). The value zero corresponds to not aborting (will not match).

\param record_compare
should comparison operators be recorded.
*/
template <class ADVector>
void Independent(ADVector &x, size_t abort_op_index, bool record_compare)
{   ADVector dynamic(0); // empty vector
    Independent(x, abort_op_index, record_compare, dynamic);
}
// ---------------------------------------------------------------------------
/*!
Declare independent variables using default for record_compare and dynamic.

\tparam ADVector
This is simple vector type with elements of type AD<Base>.

\param x
Vector of the independent variablers.

\param abort_op_index
operator index at which execution will be aborted (during  the recording
of operations). The value zero corresponds to not aborting (will not match).
*/
template <class ADVector>
void Independent(ADVector &x, size_t abort_op_index)
{   bool     record_compare = true;
    ADVector dynamic(0); // empty vector
    Independent(x, abort_op_index, record_compare, dynamic);
}
// ---------------------------------------------------------------------------
/*!
Declare independent variables using default for
record_compare, abort_op_index, and dynamic.

\tparam ADVector
This is simple vector type with elements of type AD<Base>.

\param x
Vector of the independent variablers.
*/
template <class ADVector>
void Independent(ADVector &x)
{   size_t   abort_op_index = 0;
    bool     record_compare = true;
    ADVector dynamic(0); // empty vector
    Independent(x, abort_op_index, record_compare, dynamic);
}

} // END_CPPAD_NAMESPACE
# endif
