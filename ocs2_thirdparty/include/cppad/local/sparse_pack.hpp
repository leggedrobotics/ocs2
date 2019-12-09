# ifndef CPPAD_LOCAL_SPARSE_PACK_HPP
# define CPPAD_LOCAL_SPARSE_PACK_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-18 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/core/cppad_assert.hpp>
# include <cppad/local/pod_vector.hpp>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file sparse_pack.hpp
Vector of sets of positive integers stored as a packed array of bools.
*/
class sparse_pack_const_iterator;

// ==========================================================================
/*!
Vector of sets of postivie integers, each set stored as a packed boolean array.

All the public members for this class are also in the
sparse_list and sparse_vecsize classes.
This defines the CppAD vector_of_sets concept.
*/

class sparse_pack {
    friend class sparse_pack_const_iterator;
private:
    /// Type used to pack elements (should be the same as corresponding
    /// typedef in multiple_n_bit() in test_more/sparse_hacobian.cpp)
    typedef size_t Pack;
    /// Number of bits per Pack value
    const size_t n_bit_;
    /// Number of sets that we are representing
    /// (set by constructor and resize).
    size_t n_set_;
    /// Possible elements in each set are 0, 1, ..., end_ - 1
    /// (set by constructor and resize).
    size_t end_;
    /// Number of Pack values necessary to represent end_ bits.
    /// (set by constructor and resize).
    size_t n_pack_;
    /// Data for all the sets.
    pod_vector<Pack>  data_;
// ============================================================================
    /*!
    Assign a set equal to the union of a set and a vector;

    \param target
    is the index in this sparse_list object of the set being assinged.

    \param left
    is the index in this sparse_list object of the
    left operand for the union operation.
    It is OK for target and left to be the same value.

    \param right
    is a vector of size_t, sorted in accending order.
    right operand for the union operation.
    Elements can be repeated in right, but are not be repeated in the
    resulting set.
    All of the elements must have value less than end();
    */
    void binary_union(
        size_t                    target ,
        size_t                    left   ,
        const pod_vector<size_t>& right  )
    {
        // initialize target = left
        size_t t = target * n_pack_;
        size_t l = left   * n_pack_;
        size_t j = n_pack_;
        while(j--)
            data_[t++] = data_[l++];

        // add the elements in right
        for(size_t i = 0; i < right.size(); ++i)
            add_element(target, right[i]);
    }
public:
    /// declare a const iterator
    typedef sparse_pack_const_iterator const_iterator;
    // -----------------------------------------------------------------
    /*!
    Default constructor (no sets)
    */
    sparse_pack(void) :
    n_bit_( std::numeric_limits<Pack>::digits ),
    n_set_(0)      ,
    end_(0)        ,
    n_pack_(0)
    { }
    // -----------------------------------------------------------------
    /*!
    Make use of copy constructor an error

    \param v
    vector that we are attempting to make a copy of.
    */
    sparse_pack(const sparse_pack& v) :
    n_bit_( std::numeric_limits<Pack>::digits )
    {   // Error:
        // Probably a sparse_pack argument has been passed by value
        CPPAD_ASSERT_UNKNOWN(0);
    }
    // -----------------------------------------------------------------
    /*!
    Assignment operator.

    \param other
    this sparse_pack will be set to a deep copyof other.

    */
    void operator=(const sparse_pack& other)
    {   CPPAD_ASSERT_UNKNOWN( n_bit_  == other.n_bit_);
        n_set_  = other.n_set_;
        end_    = other.end_;
        n_pack_ = other.n_pack_;
        data_   = other.data_;
    }
    // -----------------------------------------------------------------
    /*!
    swap (used by move semantics version of ADFun assignment operator)

    \param other
    this sparse_pack will be swapped with other.
    */
    void swap(sparse_pack& other)
    {   // size_t objects
        CPPAD_ASSERT_UNKNOWN( n_bit_  == other.n_bit_);
        std::swap(n_set_  , other.n_set_);
        std::swap(end_    , other.end_);
        std::swap(n_pack_ , other.n_pack_);
        //
        // pod_vectors
        data_.swap(other.data_);
    }
    // -----------------------------------------------------------------
    /*!
    Destructor
    */
    ~sparse_pack(void)
    { }
    // -----------------------------------------------------------------
    /*!
    Change number of sets, set end, and initialize all sets as empty

    If n_set is zero, any memory currently allocated for this object
    is freed. Otherwise, new memory may be allocated for the sets (if needed).

    \param n_set
    is the number of sets in this vector of sets.

    \param end
    is the maximum element plus one. The minimum element is 0 and
    end must be greater than zero (unless n_set is also zero).
    If n_set is zero, end must also be zero.
    */
    void resize(size_t n_set, size_t end)
    {
        n_set_          = n_set;
        end_            = end;
        if( n_set_ == 0 )
        {   CPPAD_ASSERT_UNKNOWN( end == 0 );
            data_.clear();
            return;
        }
        // now start a new vector with empty sets
        Pack zero(0);

        n_pack_         = ( 1 + (end_ - 1) / n_bit_ );
        size_t i        = n_set_ * n_pack_;

        data_.resize(i);
        while(i--)
            data_[i] = zero;
    }
    // -----------------------------------------------------------------
    /*!
    Count number of elements in a set.

    \param i
    is the index in of the set we are counting the elements of.
    */
    size_t number_elements(size_t i) const
    {   static Pack one(1);
        CPPAD_ASSERT_UNKNOWN( i < n_set_ );
        size_t count  = 0;
        for(size_t k = 0; k < n_pack_; k++)
        {   Pack   unit = data_[ i * n_pack_ + k ];
            Pack   mask = one;
            size_t n    = std::min(n_bit_, end_ - n_bit_ * k);
            for(size_t bit = 0; bit < n; bit++)
            {   CPPAD_ASSERT_UNKNOWN( mask > one || bit == 0);
                if( mask & unit )
                    ++count;
                mask = mask << 1;
            }
        }
        return count;
    }
    /*!
    Post an element for delayed addition to a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the value of the element that we are posting.
    The same element may be posted multiple times.

    \par
    It is faster to post multiple elements to set i and then call
    process_post(i) then to add each element individually.
    It is an error to call any member function,
    that depends on the value of set i,
    before processing the posts to set i.
    */
    void post_element(size_t i, size_t element)
    {   add_element(i, element); }
    // -----------------------------------------------------------------
    /*!
    process post entries for a specific set.

    \param i
    index of the set for which we are processing the post entries.

    \par post_
    Upon call, post_[i] is location in data_ of the elements that get
    added to the i-th set.  Upon return, post_[i] is zero.
    */
    void process_post(size_t i)
    {   return; }
    // -----------------------------------------------------------------
    /*!
    Add one element to a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the element we are adding to the set.
    */
    void add_element(size_t i, size_t element)
    {   static Pack one(1);
        CPPAD_ASSERT_UNKNOWN( i   < n_set_ );
        CPPAD_ASSERT_UNKNOWN( element < end_ );
        size_t j  = element / n_bit_;
        size_t k  = element - j * n_bit_;
        Pack mask = one << k;
        data_[ i * n_pack_ + j] |= mask;
    }
    // -----------------------------------------------------------------
    /*!
    Is an element of a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the element we are checking to see if it is in the set.
    */
    bool is_element(size_t i, size_t element) const
    {   static Pack one(1);
        static Pack zero(0);
        CPPAD_ASSERT_UNKNOWN( i   < n_set_ );
        CPPAD_ASSERT_UNKNOWN( element < end_ );
        size_t j  = element / n_bit_;
        size_t k  = element - j * n_bit_;
        Pack mask = one << k;
        return (data_[ i * n_pack_ + j] & mask) != zero;
    }
    // -----------------------------------------------------------------
    /*!
    Assign the empty set to one of the sets.

    \param target
    is the index of the set we are setting to the empty set.

    \par Checked Assertions
    \li target < n_set_
    */
    void clear(size_t target)
    {   // value with all its bits set to false
        static Pack zero(0);
        CPPAD_ASSERT_UNKNOWN( target < n_set_ );
        size_t t = target * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = zero;
    }
    // -----------------------------------------------------------------
    /*!
    Assign one set equal to another set.

    \param this_target
    is the index (in this sparse_pack object) of the set being assinged.

    \param other_value
    is the index (in the other sparse_pack object) of the
    that we are using as the value to assign to the target set.

    \param other
    is the other sparse_pack object (which may be the same as this
    sparse_pack object).

    \par Checked Assertions
    \li this_target  < n_set_
    \li other_value  < other.n_set_
    \li n_pack_     == other.n_pack_
    */
    void assignment(
        size_t               this_target  ,
        size_t               other_value  ,
        const sparse_pack&   other        )
    {   CPPAD_ASSERT_UNKNOWN( this_target  <   n_set_        );
        CPPAD_ASSERT_UNKNOWN( other_value  <   other.n_set_  );
        CPPAD_ASSERT_UNKNOWN( n_pack_      ==  other.n_pack_ );
        size_t t = this_target * n_pack_;
        size_t v = other_value * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = other.data_[v++];
    }
    // -----------------------------------------------------------------
    /*!
    Assing a set equal to the union of two other sets.

    \param this_target
    is the index (in this sparse_pack object) of the set being assinged.

    \param this_left
    is the index (in this sparse_pack object) of the
    left operand for the union operation.
    It is OK for this_target and this_left to be the same value.

    \param other_right
    is the index (in the other sparse_pack object) of the
    right operand for the union operation.
    It is OK for this_target and other_right to be the same value.

    \param other
    is the other sparse_pack object (which may be the same as this
    sparse_pack object).

    \par Checked Assertions
    \li this_target <  n_set_
    \li this_left   <  n_set_
    \li other_right <  other.n_set_
    \li n_pack_     == other.n_pack_
    */
    void binary_union(
        size_t                  this_target  ,
        size_t                  this_left    ,
        size_t                  other_right  ,
        const sparse_pack&      other        )
    {   CPPAD_ASSERT_UNKNOWN( this_target < n_set_         );
        CPPAD_ASSERT_UNKNOWN( this_left   < n_set_         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.n_set_   );
        CPPAD_ASSERT_UNKNOWN( n_pack_    ==  other.n_pack_ );

        size_t t = this_target * n_pack_;
        size_t l  = this_left  * n_pack_;
        size_t r  = other_right * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = ( data_[l++] | other.data_[r++] );
    }
    // -----------------------------------------------------------------
    /*!
    Assing a set equal to the intersection of two other sets.

    \param this_target
    is the index (in this sparse_pack object) of the set being assinged.

    \param this_left
    is the index (in this sparse_pack object) of the
    left operand for the intersection operation.
    It is OK for this_target and this_left to be the same value.

    \param other_right
    is the index (in the other sparse_pack object) of the
    right operand for the intersection operation.
    It is OK for this_target and other_right to be the same value.

    \param other
    is the other sparse_pack object (which may be the same as this
    sparse_pack object).

    \par Checked Assertions
    \li this_target <  n_set_
    \li this_left   <  n_set_
    \li other_right <  other.n_set_
    \li n_pack_     == other.n_pack_
    */
    void binary_intersection(
        size_t                  this_target  ,
        size_t                  this_left    ,
        size_t                  other_right  ,
        const sparse_pack&      other        )
    {   CPPAD_ASSERT_UNKNOWN( this_target < n_set_         );
        CPPAD_ASSERT_UNKNOWN( this_left   < n_set_         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.n_set_   );
        CPPAD_ASSERT_UNKNOWN( n_pack_    ==  other.n_pack_ );

        size_t t = this_target * n_pack_;
        size_t l  = this_left  * n_pack_;
        size_t r  = other_right * n_pack_;

        size_t j = n_pack_;
        while(j--)
            data_[t++] = ( data_[l++] & other.data_[r++] );
    }
    // -----------------------------------------------------------------
    /*!
    Fetch n_set for vector of sets object.

    \return
    Number of from sets for this vector of sets object
    */
    size_t n_set(void) const
    {   return n_set_; }
    // -----------------------------------------------------------------
    /*!
    Fetch end for this vector of sets object.

    \return
    is the maximum element value plus one (the minimum element value is 0).
    */
    size_t end(void) const
    {   return end_; }
    // -----------------------------------------------------------------
    /*!
    Amount of memory used by this vector of sets

    \return
    The amount of memory in units of type unsigned char memory.
    */
    size_t memory(void) const
    {   return data_.capacity() * sizeof(Pack);
    }
    /*!
    Print the vector of sets (used for debugging)
    */
    void print(void) const;
};
// ==========================================================================
/*!
cons_iterator for one set of positive integers in a sparse_pack object.

All the public members for this class are also in the
sparse_list_const_iterator and sparse_sizevec_const_iterator classes.
This defines the CppAD vector_of_sets iterator concept.
*/
class sparse_pack_const_iterator {
private:
    /// Type used to pack elements in sparse_pack
    typedef sparse_pack::Pack Pack;

    /// data for the entire vector of sets
    const pod_vector<Pack>&  data_;

    /// Number of bits per Pack value
    const size_t             n_bit_;

    /// Number of Pack values necessary to represent end_ bits.
    const size_t             n_pack_;

    /// Possible elements in each set are 0, 1, ..., end_ - 1;
    const size_t             end_;

    /// index of this set in the vector of sets;
    const size_t             set_index_;

    /// value of the next element in this set
    /// (use end_ for no such element exists; i.e., past end of the set).
    size_t                   next_element_;
public:
    /// construct a const_iterator for a set in a sparse_pack object
    sparse_pack_const_iterator (const sparse_pack& pack, size_t set_index)
    :
    data_          ( pack.data_ )         ,
    n_bit_         ( pack.n_bit_ )        ,
    n_pack_        ( pack.n_pack_ )       ,
    end_           ( pack.end_ )          ,
    set_index_     ( set_index )
    {   static Pack one(1);
        CPPAD_ASSERT_UNKNOWN( set_index_ < pack.n_set_ );
        //
        next_element_ = 0;
        if( next_element_ < end_ )
        {   Pack check = data_[ set_index_ * n_pack_ + 0 ];
            if( check & one )
                return;
        }
        // element with index zero is not in this set of integers,
        // advance to first element or end
        ++(*this);
    }

    /// advance to next element in this set
    sparse_pack_const_iterator& operator++(void)
    {   static Pack one(1);
        CPPAD_ASSERT_UNKNOWN( next_element_ <= end_ );
        if( next_element_ == end_ )
            return *this;
        //
        ++next_element_;
        if( next_element_ == end_ )
            return *this;
        //
        // initialize packed data index
        size_t j  = next_element_ / n_bit_;

        // initialize bit index
        size_t k  = next_element_ - j * n_bit_;

        // initialize mask
        size_t mask = one << k;

        // start search at this packed value
        Pack check = data_[ set_index_ * n_pack_ + j ];
        //
        while( true )
        {   // check if this element is in the set
            if( check & mask )
                return *this;

            // increment next element before checking this one
            next_element_++;
            if( next_element_ == end_ )
                return *this;

            // shift mask to left one bit so corresponds to next_element_
            // (use mask <<= 1. not one << k, so compiler knows value)
            k++;
            mask <<= 1;
            CPPAD_ASSERT_UNKNOWN( k <= n_bit_ );

            // check if we must go to next packed data index
            if( k == n_bit_ )
            {   // get next packed value
                k     = 0;
                mask  = one;
                j++;
                CPPAD_ASSERT_UNKNOWN( j < n_pack_ );
                check = data_[ set_index_ * n_pack_ + j ];
            }
        }
        // should never get here
        CPPAD_ASSERT_UNKNOWN(false);
        return *this;
    }

    /// obtain value of this element of the set of positive integers
    /// (end_ for no such element)
    size_t operator*(void) const
    {   return next_element_; }
};
// =========================================================================
/*!
Print the vector of sets (used for debugging)
*/
inline void sparse_pack::print(void) const
{   std::cout << "sparse_pack:\n";
    for(size_t i = 0; i < n_set(); i++)
    {   std::cout << "set[" << i << "] = {";
        const_iterator itr(*this, i);
        while( *itr != end() )
        {   std::cout << *itr;
            if( *(++itr) != end() )
                std::cout << ",";
        }
        std::cout << "}\n";
    }
    return;
}

// ==========================================================================

/*!
Copy a user vector of sets sparsity pattern to an internal sparse_pack object.

\tparam SetVector
is a simple vector with elements of type std::set<size_t>.

\param internal
The input value of sparisty does not matter.
Upon return it contains the same sparsity pattern as user
(or the transposed sparsity pattern).

\param user
sparsity pattern that we are placing internal.

\param n_set
number of sets (rows) in the internal sparsity pattern.

\param end
end of set value (number of columns) in the interanl sparsity pattern.

\param transpose
if true, the user sparsity patter is the transposed.

\param error_msg
is the error message to display if some values in the user sparstiy
pattern are not valid.
*/
template<class SetVector>
void sparsity_user2internal(
    sparse_pack&            internal  ,
    const SetVector&        user      ,
    size_t                  n_set     ,
    size_t                  end       ,
    bool                    transpose ,
    const char*             error_msg )
{   CPPAD_ASSERT_KNOWN(size_t( user.size() ) == n_set * end, error_msg );

    // size of internal sparsity pattern
    internal.resize(n_set, end);

    if( transpose )
    {   // transposed pattern case
        for(size_t j = 0; j < end; j++)
        {   for(size_t i = 0; i < n_set; i++)
            {   if( user[ j * n_set + i ] )
                    internal.add_element(i, j);
            }
        }
        return;
    }
    else
    {   for(size_t i = 0; i < n_set; i++)
        {   for(size_t j = 0; j < end; j++)
            {   if( user[ i * end + j ] )
                internal.add_element(i, j);
            }
        }
    }
    return;
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
