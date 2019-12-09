# ifndef CPPAD_LOCAL_ATOMIC_INDEX_HPP
# define CPPAD_LOCAL_ATOMIC_INDEX_HPP
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
Store and retrieve atomic function information by and index.

\tparam Base
Is the base type for the tape
that will use index values to identify atomic functions.

\par special case
In the special case (not included in the documentation below)
set_null it is true and index is zero.
For this case, number of atomic functions stored in atomic_index,
is returned and not other changes are made.
In this case, the atomic functions correspond to indices from
1 to the return value inclusive.

\param set_null
This value should only be true during a call to an atomic function destructor.
If it is true, the pointer corresponding to index is set to null.

\param index
This value should only be zero during a call to an atomic function constructor.
If it is zero, a copy of the
type, *name, and ptr are stored and the corresponding index
is the value retured by atomic_index2object.
Otherwise,
the information corresponding to this index is returned.

\param type
If index is zero, type is an input.
Otherwise it is set to the type correponding to index.
This is intended to be 2 for atomic_two, and 3 for atomic_three.

\param name
If index is zero, name is an input (and must not be null).
Otherwise, if name is not null, *name is set to the name correponding to index.
Allowing for name to be null avoids a string copy when it is not necessary.

\param ptr
If index is zero, ptr is an input.
Otherwise it is set to the pointer correponding to index.
If set_null is true, the null value is returned for ptr
(and for all future calls with this index).

\return
If index is zero, the return value is the index
corresponding to type, *name, and ptr (and is not zero).
Otherwise, the return value is zero.
*/
namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file atomic_index.hpp
Map indices to atomic function information
*/

struct atomic_index_info {
    size_t      type;
    std::string name;
    void*       ptr;
};

template <class Base>
size_t atomic_index(
    bool               set_null      ,
    const size_t&      index         ,
    size_t&            type          ,
    std::string*       name          ,
    void*&             ptr           )
{   //
    // information for each index
    static std::vector<atomic_index_info> vec;
# ifndef NDEBUG
    if( index == 0 || set_null )
    {   CPPAD_ASSERT_KNOWN( ! thread_alloc::in_parallel(),
        "calling atomic function constructor or destructor in parallel mode"
        );
    }
# endif
    if( set_null & (index == 0) )
        return vec.size();
    //
    // case were we are retreving informaiton for an atomic function
    if( 0 < index )
    {   CPPAD_ASSERT_UNKNOWN( index <= vec.size() )
        //
        // case where we are setting the pointer to null
        if( set_null )
            vec[index-1].ptr = CPPAD_NULL;
        //
        atomic_index_info& entry = vec[index - 1];
        type = entry.type;
        ptr  = entry.ptr;
        if( name != CPPAD_NULL )
            *name  = entry.name;
        return 0;
    }
    //
    // case where we are storing information for an atomic function
    atomic_index_info entry;
    entry.type = type;
    entry.name = *name;
    entry.ptr  = ptr;
    vec.push_back(entry);
    //
    return vec.size();
}

} } // END_CPPAD_LOCAL_NAMESPACE

# endif
