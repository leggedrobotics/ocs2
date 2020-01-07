# ifndef CPPAD_CONFIGURE_HPP
# define CPPAD_CONFIGURE_HPP
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
 \file configure.hpp
Replacement for config.h so that all preprocessor symbols begin with CPPAD_
*/

/*!
\def CPPAD_COMPILER_HAS_CONVERSION_WARN
is the compiler a variant of g++ and has conversion warnings
*/
# define CPPAD_COMPILER_HAS_CONVERSION_WARN 1

/*!
\def CPPAD_DISABLE_SOME_MICROSOFT_COMPILER_WARNINGS
This macro is only used to document the pragmas that disables the
follow warnings:

\li C4100
unreferenced formal parameter.
\li C4127
conditional expression is constant.
*/
# define CPPAD_DISABLE_SOME_MICROSOFT_COMPILER_WARNINGS 1
# if _MSC_VER
# pragma warning( disable : 4100 )
# pragma warning( disable : 4127 )
# endif
# undef CPPAD_DISABLE_SOME_MICROSOFT_COMPILER_WARNINGS

/*!
\def CPPAD_USE_CPLUSPLUS_2011
Should CppAD use C++11 features. This will be true if the current
compiler flags request C++11 features and the install procedure
determined that all the necessary features are avaiable.
*/
# if     _MSC_VER
# define    CPPAD_USE_CPLUSPLUS_2011 1
# else   //
# if         __cplusplus >= 201100
# define         CPPAD_USE_CPLUSPLUS_2011 1
# else       //
# define         CPPAD_USE_CPLUSPLUS_2011 0
# endif      //
# endif //

/*!
\def CPPAD_PACKAGE_STRING
cppad-yyyymmdd as a C string where yyyy is year, mm is month, and dd is day.
*/
# define CPPAD_PACKAGE_STRING "cppad-20190200.5"

/*!
def CPPAD_HAS_ADOLC
Was a adolc_prefix specified on the cmake command line.
*/
# define CPPAD_HAS_ADOLC 0

/*!
def CPPAD_HAS_COLPACK
Was a colpack_prefix specified on the cmake command line.
*/
# define CPPAD_HAS_COLPACK 0

/*!
def CPPAD_HAS_EIGEN
Was a eigen_prefix specified on the cmake command line.
*/
# define CPPAD_HAS_EIGEN 0

/*!
def CPPAD_HAS_IPOPT
Was a ipopt_prefix specified on the cmake command line.
*/
# define CPPAD_HAS_IPOPT 0

/*!
\def CPPAD_DEPRECATED
This symbol is not currently being used.
*/
# define CPPAD_DEPRECATED 

/*!
\def CPPAD_BOOSTVECTOR
If this symbol is one, and _MSC_VER is not defined,
we are using boost vector for CPPAD_TESTVECTOR.
It this symbol is zero,
we are not using boost vector for CPPAD_TESTVECTOR.
*/
# define CPPAD_BOOSTVECTOR 0

/*!
\def CPPAD_CPPADVECTOR
If this symbol is one,
we are using CppAD vector for CPPAD_TESTVECTOR.
It this symbol is zero,
we are not using CppAD vector for CPPAD_TESTVECTOR.
*/
# define CPPAD_CPPADVECTOR 1

/*!
\def CPPAD_STDVECTOR
If this symbol is one,
we are using standard vector for CPPAD_TESTVECTOR.
It this symbol is zero,
we are not using standard vector for CPPAD_TESTVECTOR.
*/
# define CPPAD_STDVECTOR 0

/*!
\def CPPAD_EIGENVECTOR
If this symbol is one,
we are using Eigen vector for CPPAD_TESTVECTOR.
If this symbol is zero,
we are not using Eigen vector for CPPAD_TESTVECTOR.
*/
# define CPPAD_EIGENVECTOR 0

/*!
\def CPPAD_HAS_GETTIMEOFDAY
If this symbol is one, and _MSC_VER is not defined,
this system supports the gettimeofday funcgtion.
Otherwise, this smybol should be zero.
*/
# define CPPAD_HAS_GETTIMEOFDAY 1

/*!
\def CPPAD_TAPE_ADDR_TYPE
Is the type used to store address on the tape. If not size_t, then
<code>sizeof(CPPAD_TAPE_ADDR_TYPE) <= sizeof( size_t )</code>
to conserve memory.
This type must support std::numeric_limits,
the <= operator,
and conversion to size_t.
Make sure that the type chosen returns true for is_pod<CPPAD_TAPE_ADDR_TYPE>
in pod_vector.hpp.
This type is later defined as addr_t in the CppAD namespace.
*/
# define CPPAD_TAPE_ADDR_TYPE unsigned int

/*!
\def CPPAD_TAPE_ID_TYPE
Is the type used to store tape identifiers. If not size_t, then
<code>sizeof(CPPAD_TAPE_ID_TYPE) <= sizeof( size_t )</code>
to conserve memory.
This type must support std::numeric_limits,
the <= operator,
and conversion to size_t.
Make sure that the type chosen returns true for is_pod<CPPAD_TAPE_ID_TYPE>
in pod_vector.hpp.
This type is later defined as tape_id_t in the CppAD namespace.
*/
# define CPPAD_TAPE_ID_TYPE unsigned int

/*!
\def CPPAD_MAX_NUM_THREADS
Specifies the maximum number of threads that CppAD can support
(must be greater than or equal four).

The user may define CPPAD_MAX_NUM_THREADS before including any of the CppAD
header files.  If it is not yet defined,
*/
# ifndef CPPAD_MAX_NUM_THREADS
# define CPPAD_MAX_NUM_THREADS 48
# endif

/*!
\def CPPAD_HAS_MKSTEMP
It true, mkstemp works in C++ on this system.
*/
# define CPPAD_HAS_MKSTEMP 1

/*!
\def CPPAD_HAS_TMPNAM_S
It true, tmpnam_s works in C++ on this system.
*/
# define CPPAD_HAS_TMPNAM_S 0

// ---------------------------------------------------------------------------
// defines that only depend on values above
// ---------------------------------------------------------------------------
/*!
\def CPPAD_NULL
This preprocessor symbol is used for a null pointer.

If it is not yet defined,
it is defined when cppad/local/define.hpp is included.
*/
# ifndef CPPAD_NULL
# if CPPAD_USE_CPLUSPLUS_2011
# define CPPAD_NULL     nullptr
# else
# define CPPAD_NULL     0
# endif
# endif

# endif
