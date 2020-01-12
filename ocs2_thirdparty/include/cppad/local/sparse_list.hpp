# ifndef CPPAD_LOCAL_SPARSE_LIST_HPP
# define CPPAD_LOCAL_SPARSE_LIST_HPP
/* --------------------------------------------------------------------------
CppAD: C++ Algorithmic Differentiation: Copyright (C) 2003-19 Bradley M. Bell

CppAD is distributed under the terms of the
             Eclipse Public License Version 2.0.

This Source Code may also be made available under the following
Secondary License when the conditions for such availability set forth
in the Eclipse Public License, Version 2.0 are satisfied:
      GNU General Public License, Version 2.0 or later.
---------------------------------------------------------------------------- */
# include <cppad/local/define.hpp>
# include <cppad/local/is_pod.hpp>
# include <list>

namespace CppAD { namespace local { // BEGIN_CPPAD_LOCAL_NAMESPACE
/*!
\file sparse_list.hpp
Vector of sets of positive integers stored as singly linked lists
with the element values strictly increasing.
*/
class sparse_list_const_iterator;

// =========================================================================
/*!
Vector of sets of positive integers, each set stored as a singly
linked list.

All the public members for this class are also in the
sparse_pack and sparse_vecsize classes.
This defines the CppAD vector_of_sets concept.
*/
class sparse_list {
    friend class sparse_list_const_iterator;
private:
    // -----------------------------------------------------------------
    /// type used for each entry in a singly linked list.
    struct pair_size_t {
        /// For the first entry in each list, this is the reference count.
        /// For the other entries in the list this is an element of the set.
        size_t value;

        /// This is the data index of the next entry in the list.
        /// If there are no more entries in the list, this value is zero.
        /// (The first entry in data_ is not used.)
        size_t next;
    };
    friend bool is_pod<pair_size_t>(void);
    // -----------------------------------------------------------------
    /// Possible elements in each set are 0, 1, ..., end_ - 1;
    size_t end_;

    /// number of elements in data_ that are not being used.
    size_t number_not_used_;

    /// list of elements of data_ that are not being used.
    size_t data_not_used_;

    /// The data for all the singly linked lists.
    pod_vector<pair_size_t> data_;

    /*!
    Starting point for i-th set is start_[i].

    \li
    If the i-th set has no elements, start_[i] is zero.
    Otherwise the conditions below hold.

    \li
    data_[ start_[i] ].value is the reference count for this list.
    This element is not in the list.

    \li
    data_[ start_[i] ].next point the the first element in the list
    and is not zero because there is at least one entry in this list.

    \li
    For all lists, the last pair in the list has data_ index zero,
    data_[0].value == end_ and data_[0].next = 0, and is not in the set.
    */
    pod_vector<size_t> start_;

    /*!
    Vectors of elements that have not yet been added to corresponding sets.

    \li
    If all the post_element calls for the i-th set have been added,
    post_[i] is zero. Otherwise the conditions below hold.

    \li
    data_[ post_[i] ].value  is the first element that has been posted,
    but not yet added, to set i.

    \li
    For all lists, the last pair in the list has data_ index zero,
    data_[0].value == end_ and data_[0].next = 0, and is not in the posted
    elements.
    */
    pod_vector<size_t> post_;

    /*!
    A temporary vector used by member functions that keeps its capacity.
    */
    pod_vector<size_t> temporary_;

    // -----------------------------------------------------------------
    /*!
    Counts references to sets.

    \param i
    is the index of the set that we are counting the references to.

    \return
    if the set is empty, the return value is zero.
    Otherwise it is the number of sets that share the same linked list
    */
    size_t reference_count(size_t i) const
    {   // start data index
        size_t start = start_[i];
        if( start == 0 )
            return 0;
        //
        // reference count
        return data_[start].value;
    }
    // -----------------------------------------------------------------
    /*!
    drop a set and its postings (no longer being used).

    \param i
    is the index of the set that will be dropped.

    \par reference_count
    if the set is non-empty,
    the reference count data_[ start_[i] ] will be decremented.

    \par start_
    The value start_[i] is set to zero.

    \par post_
    the value post_[i] will be set to zero.

    \par data_not_used_
    the eleemmnts of data_ that are dropped are added to this list.

    \return
    is the additional number of elements of data_ that are not used.
    This is non-zero when the initial reference count is one.
    */
    size_t drop(size_t i)
    {   // inialize count of addition elements not being used.
        size_t number_drop = 0;

        // the elements in the post list will no longer be used
        size_t post = post_[i];
        if( post != 0 )
        {   // drop this posting
            post_[i]    = 0;
            //
            // count elements in this posting
            ++number_drop;
            size_t previous = post;
            size_t next     = data_[previous].next;
            while( next != 0 )
            {   previous = next;
                next     = data_[previous].next;
                ++number_drop;
            }
            //
            // add the posting elements to data_not_used_
            data_[previous].next = data_not_used_;
            data_not_used_       = post;
        }

        // check for empty set
        size_t start = start_[i];
        if( start == 0 )
            return number_drop;

        // decrement reference counter
        CPPAD_ASSERT_UNKNOWN( data_[start].value > 0 );
        data_[start].value--;

        // set this set to empty
        start_[i] = 0;

        // If new reference count is positive, the list corresponding to
        // start is still being used.
        if( data_[start].value > 0 )
            return number_drop;

        //
        // count elements representing this set
        ++number_drop;
        size_t previous = start;
        size_t next     = data_[previous].next;
        while( next != 0 )
        {   previous = next;
            next     = data_[previous].next;
            ++number_drop;
        }
        //
        // add representing this set to data_not_used_
        data_[previous].next = data_not_used_;
        data_not_used_       = start;
        //
        return number_drop;
    }
    // -----------------------------------------------------------------
    /*!
    get a new data_ element for use.

    \par number_not_used_
    if this is non-zero, it is decremented by one.

    \par data_not_used_
    if this list is non-empty, one element is removed from it.

    \return
    is the index in data_ of the new element.
    */
    size_t get_data_index(void)
    {   size_t index;
        if( data_not_used_ > 0 )
        {   CPPAD_ASSERT_UNKNOWN( number_not_used_ > 0 );
            --number_not_used_;
            index          = data_not_used_;
            data_not_used_ = data_[index].next;
        }
        else
        {   index = data_.extend(1);
        }
        return index;
    }
    // -----------------------------------------------------------------
    /*!
    Checks data structure
    (effectively const, but modifies and restores values)
    */
# ifdef NDEBUG
    void check_data_structure(void)
    {   return; }
# else
    void check_data_structure(void)
    {   // number of sets
        CPPAD_ASSERT_UNKNOWN( post_.size() == start_.size() );
        size_t n_set = start_.size();
        if( n_set == 0 )
        {   CPPAD_ASSERT_UNKNOWN( end_ == 0 );
            CPPAD_ASSERT_UNKNOWN( number_not_used_ == 0 );
            CPPAD_ASSERT_UNKNOWN( data_not_used_ == 0 );
            CPPAD_ASSERT_UNKNOWN( data_.size() == 0 );
            CPPAD_ASSERT_UNKNOWN( start_.size() == 0 );
            return;
        }
        // check data index zero
        CPPAD_ASSERT_UNKNOWN( data_[0].value == end_ );
        CPPAD_ASSERT_UNKNOWN( data_[0].next  == 0  );
        // -----------------------------------------------------------
        // save the reference counters
        pod_vector<size_t> ref_count(n_set);
        for(size_t i = 0; i < n_set; i++)
            ref_count[i] = reference_count(i);
        // -----------------------------------------------------------
        // number of entries in data used by sets and posts
        size_t number_used_by_sets = 1;
        // -----------------------------------------------------------
        // count the number of entries in data_ that are used by sets
        for(size_t i = 0; i < n_set; i++)
        {   size_t start = start_[i];
            if( start > 0 )
            {   // check structure for this non-empty set
                size_t reference_count = data_[start].value;
                size_t next            = data_[start].next;
                CPPAD_ASSERT_UNKNOWN( reference_count > 0 );
                CPPAD_ASSERT_UNKNOWN( next != 0 );
                CPPAD_ASSERT_UNKNOWN( data_[next].value < end_ );
                //
                // decrement the reference counter
                data_[start].value--;
                //
                // count the entries when find last reference
                if( data_[start].value == 0 )
                {
                    // restore reference count
                    data_[start].value = ref_count[i];

                    // number of data entries used for this set
                    number_used_by_sets += number_elements(i) + 1;
                    /*
                    number of elements checks that value < end_
                    each pair in the list except for the start pair
                    and the pair with index zero.
                    */
                }
            }
        }
        // ------------------------------------------------------------------
        // count the number of entries in data_ that are used by posts
        size_t number_used_by_posts = 0;
        for(size_t i = 0; i < n_set; i++)
        {   size_t post = post_[i];
            if( post > 0 )
            {   size_t value = data_[post].value;
                size_t next  = data_[post].next;
                CPPAD_ASSERT_UNKNOWN( value < end_ );
                //
                while( value < end_ )
                {   ++number_used_by_posts;
                    value = data_[next].value;
                    next  = data_[next].next;
                }
            }
        }
        // ------------------------------------------------------------------
        // count number of entries in data_not_used_
        size_t count = 0;
        size_t next = data_not_used_;
        while( next != 0 )
        {   ++count;
            next = data_[next].next;
        }
        CPPAD_ASSERT_UNKNOWN( number_not_used_ == count );
        // ------------------------------------------------------------------
        size_t number_used = number_used_by_sets + number_used_by_posts;
        CPPAD_ASSERT_UNKNOWN(
            number_used + number_not_used_ == data_.size()
        );
        return;
    }
# endif
    // -----------------------------------------------------------------
    /*!
    Check if one of two sets is a subset of the other set

    \param one_this
    is the index in this sparse_sizevec object of the first set.

    \param two_other
    is the index in other sparse_sizevec object of the second set.

    \param other
    is the other sparse_sizevec object which may be the same as this object.

    \return
    If zero, niether set is a subset of the other.
    If one, then one is a subset of two and they are not equal.
    If two, then two is a subset of one and they are not equal.
    If three, then the sets are equal.
    */
    size_t is_subset(
        size_t                  one_this    ,
        size_t                  two_other   ,
        const sparse_list&      other       ) const
    {
        CPPAD_ASSERT_UNKNOWN( one_this  < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( two_other < other.start_.size()   );
        CPPAD_ASSERT_UNKNOWN( end_  == other.end_               );
        //
        // start
        size_t start_one    = start_[one_this];
        size_t start_two    = other.start_[two_other];
        //
        if( start_one == 0 )
        {   // set one is empty
            if( start_two == 0 )
            {   // set two is empty
                return 3;
            }
            return 1;
        }
        if( start_two == 0 )
        {   // set two is empty and one is not empty
            return 2;
        }
        //
        // next
        size_t next_one     = data_[start_one].next;
        size_t next_two     = other.data_[start_two].next;
        //
        // value
        size_t value_one    = data_[next_one].value;
        size_t value_two    = other.data_[next_two].value;
        //
        bool one_subset     = true;
        bool two_subset     = true;
        //
        size_t value_union = std::min(value_one, value_two);
        while( (one_subset | two_subset) & (value_union < end_) )
        {   if( value_one > value_union )
                two_subset = false;
            else
            {   next_one = data_[next_one].next;
                value_one = data_[next_one].value;
            }
            if( value_two > value_union )
                one_subset = false;
            else
            {   next_two = other.data_[next_two].next;
                value_two = other.data_[next_two].value;
            }
            value_union = std::min(value_one, value_two);
        }
        if( one_subset )
        {   if( two_subset )
            {   // sets are equal
                return 3;
            }
            // one is a subset of two
            return 1;
        }
        if( two_subset )
        {   // two is a subset of one
            return 2;
        }
        //
        // neither is a subset
        return 0;
    }
    // -----------------------------------------------------------------
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
    {   CPPAD_ASSERT_UNKNOWN( post_[left] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( target < start_.size() );
        CPPAD_ASSERT_UNKNOWN( left   < start_.size() );

        // get start indices before drop modifies modify start_ in case target
        // and left are the same.
        size_t start_left   = start_[left];

        // -------------------------------------------------------------------
        // Check if right is a subset of left so that we used reference count
        // and not copies of identical sets.
        //
        // initialize index for left and right sets
        size_t current_left  = start_left;
        size_t current_right = 0;
        //
        // initialize value_left
        size_t value_left  = end_;
        if( current_left > 0 )
        {   // advance from reference counter to data
            current_left = data_[current_left].next;
            CPPAD_ASSERT_UNKNOWN( current_left != 0 )
            //
            value_left = data_[current_left].value;
            CPPAD_ASSERT_UNKNOWN( value_left < end_);
        }
        //
        // initialize value_right
        size_t value_right = end_;
        if( right.size() > 0 )
            value_right = right[current_right];
        //
        bool subset = true;
        while( subset & (value_right < end_) )
        {   while( value_left < value_right )
            {   // advance left
                current_left = data_[current_left].next;
                value_left = data_[current_left].value;
            }
            if( value_right < value_left )
                subset = false;
            else
            {   // advance right
                ++current_right;
                if( current_right == right.size() )
                    value_right = end_;
                else
                    value_right = right[current_right];
            }
        }
        //
        if( subset )
        {   // target = left will use reference count for identical sets
            assignment(target, left, *this);
            return;
        }

        // -------------------------------------------------------------------

        // start new version of target
        size_t start        = get_data_index();
        data_[start].value  = 1; // reference count
        //
        // previous index for new set
        size_t previous_target = start;
        //
        // initialize index for left and right sets
        current_left  = start_left;
        current_right = 0;
        //
        // initialize value_left
        value_left  = end_;
        if( current_left > 0 )
        {   // advance from reference counter to data
            current_left = data_[current_left].next;
            CPPAD_ASSERT_UNKNOWN( current_left != 0 )
            //
            value_left = data_[current_left].value;
            CPPAD_ASSERT_UNKNOWN( value_left < end_);
        }
        //
        // initialize value_right
        value_right = end_;
        if( right.size() > 0 )
            value_right = right[current_right];
        //
        // merge
        while( (value_left < end_) | (value_right < end_) )
        {   if( value_left == value_right)
            {   // advance left so left and right are no longer equal
                current_left = data_[current_left].next;
                value_left   = data_[current_left].value;
                CPPAD_ASSERT_UNKNOWN( value_right < value_left );
            }
            // place to put new element
            size_t current_target       = get_data_index();
            data_[previous_target].next = current_target;
            //
            if( value_left < value_right )
            {   // value_left case
                CPPAD_ASSERT_UNKNOWN( value_left < end_ );
                data_[current_target].value = value_left;
                //
                // advance left
                current_left = data_[current_left].next;
                value_left   = data_[current_left].value;
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( value_right < value_left )
                // value_right case
                CPPAD_ASSERT_UNKNOWN( value_right < end_);
                data_[current_target].value = value_right;
                //
                // advance right (skip values equal to this one)
                size_t previous_value = value_right;
                while( value_right == previous_value )
                {   ++current_right;
                    if( current_right == right.size() )
                        value_right = end_;
                    else
                    {   value_right = right[current_right];
                        CPPAD_ASSERT_UNKNOWN( value_right < end_ );
                    }
                }
            }
            // done setting current target value
            previous_target  = current_target;
        }
        // make end of target list
        data_[previous_target].next = 0;

        // adjust number_not_used_
        size_t number_drop = drop(target);
        number_not_used_  += number_drop;

        // set the new start value for target
        start_[target] = start;

        return;
    }
// ===========================================================================
public:
    /// declare a const iterator
    typedef sparse_list_const_iterator const_iterator;
    // -----------------------------------------------------------------
    /*!
    Default constructor (no sets)
    */
    sparse_list(void) :
    end_(0)              ,
    number_not_used_(0)  ,
    data_not_used_(0)    ,
    data_(0)             ,
    start_(0)            ,
    post_(0)
    { }
    // -----------------------------------------------------------------
    /// Destructor
    ~sparse_list(void)
    {   check_data_structure();
    }
    // -----------------------------------------------------------------
    /*!
    Using copy constructor is a programing (not user) error

    \param v
    vector of sets that we are attempting to make a copy of.
    */
    sparse_list(const sparse_list& v)
    {   // Error: Probably a sparse_list argument has been passed by value
        CPPAD_ASSERT_UNKNOWN(false);
    }
    // -----------------------------------------------------------------
    /*!
    Assignement operator.

    \param other
    this sparse_list with be set to a deep copy of other.

    \par vector_of_sets
    This public member function is not yet part of
    the vector_of_sets concept.
    */
    void operator=(const sparse_list& other)
    {   end_             = other.end_;
        number_not_used_ = other.number_not_used_;
        data_not_used_   = other.data_not_used_;
        data_            = other.data_;
        start_           = other.start_;
        post_            = other.post_;
    }
    // -----------------------------------------------------------------
    /*!
    swap (used by move semantics version of ADFun assignment operator)

    \param other
    this sparse_list with be swapped with other.

    \par vector_of_sets
    This public member function is not yet part of
    the vector_of_sets concept.
    */
    void swap(sparse_list& other)
    {   // size_t objects
        std::swap(end_             , other.end_);
        std::swap(number_not_used_ , other.number_not_used_);
        std::swap(data_not_used_   , other.data_not_used_);

        // pod_vectors
        data_.swap(       other.data_);
        start_.swap(      other.start_);
        post_.swap(       other.post_);
        temporary_.swap(  other.temporary_);
    }
    // -----------------------------------------------------------------
    /*!
    Start a new vector of sets.

    \param n_set
    is the number of sets in this vector of sets.
    \li
    If n_set is zero, any memory currently allocated for this object
    is freed.
    \li
    If n_set is non-zero, a vector of n_set sets is created and all
    the sets are initilaized as empty.

    \param end
    is the maximum element plus one (the minimum element is 0).
    If n_set is zero, end must also be zero.
    */
    void resize(size_t n_set, size_t end)
    {   check_data_structure();

        if( n_set == 0 )
        {   CPPAD_ASSERT_UNKNOWN( end == 0 );
            //
            // restore object to start after constructor
            // (no memory allocated for this object)
            data_.clear();
            start_.clear();
            post_.clear();
            number_not_used_  = 0;
            data_not_used_    = 0;
            end_              = 0;
            //
            return;
        }
        end_                   = end;
        //
        start_.resize(n_set);
        post_.resize(n_set);
        //
        for(size_t i = 0; i < n_set; i++)
        {   start_[i] = 0;
            post_[i]  = 0;
        }
        //
        // last element, marks the end for all lists
        data_.resize(1);
        data_[0].value    = end_;
        data_[0].next     = 0;
        //
        number_not_used_  = 0;
        data_not_used_    = 0;
    }
    // -----------------------------------------------------------------
    /*!
    Count number of elements in a set.

    \param i
    is the index of the set we are counting the elements of.

    \par
    number of elements checks that value < end_ for each element of the set.
    */
    size_t number_elements(size_t i) const
    {   CPPAD_ASSERT_UNKNOWN( post_[i] == 0 );

        // check if the set is empty
        size_t start   = start_[i];
        if( start == 0 )
            return 0;

        // initialize counter
        size_t count   = 0;

        // advance to the first element in the set
        size_t next    = data_[start].next;
        while( next != 0 )
        {   CPPAD_ASSERT_UNKNOWN( data_[next].value < end_ );
            count++;
            next  = data_[next].next;
        }
        CPPAD_ASSERT_UNKNOWN( count > 0 );
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
    {   CPPAD_ASSERT_UNKNOWN( i < start_.size() );
        CPPAD_ASSERT_UNKNOWN( element < end_ );

        // put element at the front of this list
        size_t next         = post_[i];
        size_t post         = get_data_index();
        post_[i]            = post;
        data_[post].value   = element;
        data_[post].next    = next;

        return;
    }
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
    {   // post
        size_t post = post_[i];
        //
        // check if there are no elements to process
        if( post == 0 )
            return;
        //
        // check if there is only one element to process
        size_t next  = data_[post].next;
        if( next == 0 )
        {   // done with this posting
            size_t value     = data_[post].value;
            post_[i]         = 0;
            data_[post].next = data_not_used_;
            data_not_used_   = post;
            ++number_not_used_;
            //
            add_element(i, value);
            //
            return;
        }
        //
        // copy the elements that need to be processed into temporary
        temporary_.resize(0);
        size_t previous  = post;
        size_t value     = data_[previous].value;
        CPPAD_ASSERT_UNKNOWN( value < end_ );
        temporary_.push_back(value);
        while( next != 0 )
        {   previous = next;
            value    = data_[previous].value;
            CPPAD_ASSERT_UNKNOWN( value < end_ );
            temporary_.push_back(value);
            next     = data_[previous].next;
        }
        size_t number_post = temporary_.size();
        //
        // done with this posting
        post_[i]              = 0;
        data_[previous].next  = data_not_used_;
        data_not_used_        = post;
        number_not_used_     += number_post;;
        //
        // sort temporary_
        CPPAD_ASSERT_UNKNOWN( number_post > 1 );
        std::sort( temporary_.data(), temporary_.data() + number_post);
        //
        // add the elements to the set
        binary_union(i, i, temporary_);
        //
        return;
    }
    // -----------------------------------------------------------------
    /*!
    Add one element to a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the element we are adding to the set.
    */
    void add_element(size_t i, size_t element)
    {   CPPAD_ASSERT_UNKNOWN( i   < start_.size() );
        CPPAD_ASSERT_UNKNOWN( element < end_ );

        // check for case where starting set is empty
        size_t start = start_[i];
        if( start == 0 )
        {   start              = get_data_index();
            start_[i]          = start;
            data_[start].value = 1; // reference count
            //
            size_t next        = get_data_index();
            data_[start].next  = next;
            //
            data_[next].value  = element;
            data_[next].next   = 0;
            return;
        }
        //
        // start of set with this index
        size_t previous = start_[i];
        //
        // first entry in this set
        size_t next     = data_[previous].next;
        size_t value    = data_[next].value;
        //
        // locate place to insert this element
        while( value < element )
        {   previous = next;
            next     = data_[next].next;
            value = data_[next].value;
        }
        //
        // check for case where element is in the set
        if( value == element )
            return;
        //
        //
        // check for case where this is the only reference to this set
        CPPAD_ASSERT_UNKNOWN( element < value );
        if( data_[start].value == 1 )
        {   size_t insert         = get_data_index();
            data_[insert].next    = next;
            data_[insert].value   = element;
            data_[previous].next  = insert;
            //
            return;
        }
        //
        // must make a separate copy with new element inserted
        CPPAD_ASSERT_UNKNOWN( data_[start].value > 1 );
        data_[start].value--;   // reverence counter for old list
        //
        size_t start_new       = get_data_index();
        data_[start_new].value = 1;         // reference counter for new list
        size_t previous_new    = start_new;
        //
        // start of old set with this index
        previous  = start_[i];
        //
        // first entry in old set
        next    = data_[previous].next;
        value   = data_[next].value;
        //
        // locate place to insert this element
        while( value < element )
        {   // copy to new list
            size_t next_new          = get_data_index();
            data_[previous_new].next = next_new;
            data_[next_new].value    = value;
            previous_new             = next_new;
            //
            // get next value
            previous = next;
            next     = data_[next].next;
            value = data_[next].value;
        }
        CPPAD_ASSERT_UNKNOWN( element < value );
        //
        // insert the element
        size_t next_new          = get_data_index();
        data_[previous_new].next = next_new;
        data_[next_new].value    = element;
        previous_new             = next_new;
        //
        // copy rest of the old set
        while( value < end_ )
        {   // copy to new list
            next_new                 = get_data_index();
            data_[previous_new].next = next_new;
            data_[next_new].value    = value;
            previous_new             = next_new;
            //
            // get next value
            previous = next;
            next     = data_[next].next;
            value = data_[next].value;
        }
        CPPAD_ASSERT_UNKNOWN( next == 0 );
        data_[previous_new].next = 0;
        //
        // hook up new list
        start_[i] = start_new;
        return;
    }
    // -----------------------------------------------------------------
    /*!
    check an element is in a set.

    \param i
    is the index for this set in the vector of sets.

    \param element
    is the element we are checking to see if it is in the set.
    */
    bool is_element(size_t i, size_t element) const
    {   CPPAD_ASSERT_UNKNOWN( post_[i] == 0 );
        CPPAD_ASSERT_UNKNOWN( element < end_ );
        //
        size_t start = start_[i];
        if( start == 0 )
            return false;
        //
        size_t next  = data_[start].next;
        size_t value = data_[next].value;
        while( value < element )
        {   next  = data_[next].next;
            value = data_[next].value;
        }
        return element == value;
    }
    // -----------------------------------------------------------------
    /*!
    Assign the empty set to one of the sets.

    \param target
    is the index of the set we are setting to the empty set.

    \par number_not_used_
    increments this value by additional number of data_ elements that are
    no longer being used.
    */
    void clear(size_t target)
    {   CPPAD_ASSERT_UNKNOWN( target < start_.size() );

        // adjust number_not_used_
        size_t number_drop = drop(target);
        number_not_used_  += number_drop;

        return;
    }
    // -----------------------------------------------------------------
    /*!
    Assign one set equal to another set.

    \param this_target
    is the index in this sparse_list object of the set being assinged.

    \param other_source
    is the index in the other sparse_list object of the
    set that we are using as the value to assign to the target set.

    \param other
    is the other sparse_list object (which may be the same as this
    sparse_list object). This must have the same value for end_.

    \par number_not_used_
    increments this value by additional number of elements not being used.
    */
    void assignment(
        size_t               this_target  ,
        size_t               other_source ,
        const sparse_list&   other        )
    {   CPPAD_ASSERT_UNKNOWN( other.post_[ other_source ] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( this_target  <   start_.size()        );
        CPPAD_ASSERT_UNKNOWN( other_source <   other.start_.size()  );
        CPPAD_ASSERT_UNKNOWN( end_        == other.end_   );

        // check if we are assigning a set to itself
        if( (this == &other) & (this_target == other_source) )
            return;

        // set depending on cases below
        size_t this_start;

        // If this and other are the same, use another reference to same list
        size_t other_start = other.start_[other_source];
        if( this == &other )
        {   this_start = other_start;
            if( other_start != 0 )
            {   data_[other_start].value++; // increment reference count
                CPPAD_ASSERT_UNKNOWN( data_[other_start].value > 1 );
            }
        }
        else if( other_start  == 0 )
        {   this_start = 0;
        }
        else
        {   // make a copy of the other list in this sparse_list
            this_start        = get_data_index();
            size_t this_next  = get_data_index();
            data_[this_start].value = 1; // reference count
            data_[this_start].next  = this_next;
            //
            size_t next  = other.data_[other_start].next;
            CPPAD_ASSERT_UNKNOWN( next != 0 );
            while( next != 0 )
            {   data_[this_next].value = other.data_[next].value;
                next                   = other.data_[next].next;
                if( next == 0 )
                    data_[this_next].next = 0;
                else
                {   size_t tmp = get_data_index();
                    data_[this_next].next = tmp;
                    this_next             = tmp;
                }
            }
        }

        // adjust number_not_used_
        size_t number_drop = drop(this_target);
        number_not_used_  += number_drop;

        // set the new start value for this_target
        start_[this_target] = this_start;

        return;
    }
    // -----------------------------------------------------------------
    /*!
    Assign a set equal to the union of two other sets.

    \param this_target
    is the index in this sparse_list object of the set being assinged.

    \param this_left
    is the index in this sparse_list object of the
    left operand for the union operation.
    It is OK for this_target and this_left to be the same value.

    \param other_right
    is the index in the other sparse_list object of the
    right operand for the union operation.
    It is OK for this_target and other_right to be the same value.

    \param other
    is the other sparse_list object (which may be the same as this
    sparse_list object).
    */
    void binary_union(
        size_t                  this_target  ,
        size_t                  this_left    ,
        size_t                  other_right  ,
        const sparse_list&      other        )
    {   CPPAD_ASSERT_UNKNOWN( post_[this_left] == 0 );
        CPPAD_ASSERT_UNKNOWN( other.post_[ other_right ] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( this_target < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( this_left   < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.start_.size()   );
        CPPAD_ASSERT_UNKNOWN( end_        == other.end_           );

        // check if one of the two operands is a subset of the the other
        size_t subset = is_subset(this_left, other_right, other);

        // case where right is a subset of left or right and left are equal
        if( subset == 2 || subset == 3 )
        {   assignment(this_target, this_left, *this);
            return;
        }
        // case where the left is a subset of right and they are not equal
        if( subset == 1 )
        {   assignment(this_target, other_right, other);
            return;
        }
        // if niether case holds, then both left and right are non-empty
        CPPAD_ASSERT_UNKNOWN( reference_count(this_left) > 0 );
        CPPAD_ASSERT_UNKNOWN( other.reference_count(other_right) > 0 );

        // must get all the start indices before modify start_this
        // (incase start_this is the same as start_left or start_right)
        size_t start_left    = start_[this_left];
        size_t start_right   = other.start_[other_right];

        // start the new list
        size_t start        = get_data_index();
        size_t next         = start;
        data_[start].value  = 1; // reference count

        // next for left and right lists
        size_t next_left   = data_[start_left].next;
        size_t next_right  = other.data_[start_right].next;

        // value for left and right sets
        size_t value_left  = data_[next_left].value;
        size_t value_right = other.data_[next_right].value;

        CPPAD_ASSERT_UNKNOWN( value_left < end_ && value_right < end_ );
        while( (value_left < end_) | (value_right < end_) )
        {   if( value_left == value_right )
            {   // advance right so left and right are no longer equal
                next_right  = other.data_[next_right].next;
                value_right = other.data_[next_right].value;
            }
            if( value_left < value_right )
            {   size_t tmp        = get_data_index();
                data_[next].next  = tmp;
                next              = tmp;
                data_[next].value = value_left;
                // advance left to its next element
                next_left  = data_[next_left].next;
                value_left = data_[next_left].value;
            }
            else
            {   CPPAD_ASSERT_UNKNOWN( value_right < value_left )
                size_t tmp        = get_data_index();
                data_[next].next  = tmp;
                next              = tmp;
                data_[next].value = value_right;
                // advance right to its next element
                next_right  = other.data_[next_right].next;
                value_right = other.data_[next_right].value;
            }
        }
        data_[next].next = 0;

        // adjust number_not_used_
        size_t number_drop = drop(this_target);
        number_not_used_  += number_drop;

        // set the new start value for this_target
        start_[this_target] = start;

        return;
    }
    // -----------------------------------------------------------------
    /*!
    Assign a set equal to the intersection of two other sets.

    \param this_target
    is the index in this sparse_list object of the set being assinged.

    \param this_left
    is the index in this sparse_list object of the
    left operand for the intersection operation.
    It is OK for this_target and this_left to be the same value.

    \param other_right
    is the index in the other sparse_list object of the
    right operand for the intersection operation.
    It is OK for this_target and other_right to be the same value.

    \param other
    is the other sparse_list object (which may be the same as this
    sparse_list object).
    */
    void binary_intersection(
        size_t                  this_target  ,
        size_t                  this_left    ,
        size_t                  other_right  ,
        const sparse_list&      other        )
    {   CPPAD_ASSERT_UNKNOWN( post_[this_left] == 0 );
        CPPAD_ASSERT_UNKNOWN( other.post_[ other_right ] == 0 );
        //
        CPPAD_ASSERT_UNKNOWN( this_target < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( this_left   < start_.size()         );
        CPPAD_ASSERT_UNKNOWN( other_right < other.start_.size()   );
        CPPAD_ASSERT_UNKNOWN( end_        == other.end_           );
        //
        // check if one of the two operands is a subset of the the other
        size_t subset = is_subset(this_left, other_right, other);

        // case where left is a subset of right or left and right are equal
        if( subset == 1 || subset == 3 )
        {   assignment(this_target, this_left, *this);
            return;
        }
        // case where the right is a subset of left and they are not equal
        if( subset == 2 )
        {   assignment(this_target, other_right, other);
            return;
        }
        // if niether case holds, then both left and right are non-empty
        CPPAD_ASSERT_UNKNOWN( reference_count(this_left) > 0 );
        CPPAD_ASSERT_UNKNOWN( other.reference_count(other_right) > 0 );

        // must get all the start indices before modify start_this
        // (incase start_this is the same as start_left or start_right)
        size_t start_left    = start_[this_left];
        size_t start_right   = other.start_[other_right];

        // start the new list as emptyh
        size_t start        = 0;
        size_t next         = start;

        // next for left and right lists
        size_t next_left   = data_[start_left].next;
        size_t next_right  = other.data_[start_right].next;

        // value for left and right sets
        size_t value_left  = data_[next_left].value;
        size_t value_right = other.data_[next_right].value;

        CPPAD_ASSERT_UNKNOWN( value_left < end_ && value_right < end_ );
        while( (value_left < end_) & (value_right < end_) )
        {   if( value_left == value_right )
            {   if( start == 0 )
                {   // this is the first element in the intersection
                    start               = get_data_index();
                    next                = start;
                    data_[start].value  = 1; // reference count
                    CPPAD_ASSERT_UNKNOWN( start > 0 );
                    // must delay this until after drop below
                    // start_[this_target] = start;
                }
                size_t tmp        = get_data_index();
                data_[next].next  = tmp;
                next              = tmp;
                data_[next].value = value_left;
                //
                // advance left
                next_left  = data_[next_left].next;
                value_left = data_[next_left].value;
                //
            }
            if( value_left > value_right )
            {   // advance right
                next_right  = other.data_[next_right].next;
                value_right = other.data_[next_right].value;
            }
            if( value_right > value_left )
            {   // advance left
                next_left  = data_[next_left].next;
                value_left = data_[next_left].value;
            }
        }
        if( start != 0 )
        {   CPPAD_ASSERT_UNKNOWN( next != 0 );
            data_[next].next = 0;
        }

        // adjust number_not_used_
        size_t number_drop = drop(this_target);
        number_not_used_  += number_drop;

        // set new start for this_target
        start_[this_target] = start;

        return;
    }
    // -----------------------------------------------------------------
    /*! Fetch n_set for vector of sets object.

    \return
    Number of from sets for this vector of sets object
    */
    size_t n_set(void) const
    {   return start_.size(); }
    // -----------------------------------------------------------------
    /*! Fetch end for this vector of sets object.

    \return
    is the maximum element value plus one (the minimum element value is 0).
    */
    size_t end(void) const
    {   return end_; }
    // -----------------------------------------------------------------
    /*! Amount of memory used by this vector of sets

    \return
    The amount of memory in units of type unsigned char memory.
    */
    size_t memory(void) const
    {   return data_.capacity() * sizeof(pair_size_t);
    }
    /*!
    Print the vector of sets (used for debugging)
    */
    void print(void) const;
};
// =========================================================================
/*!
cons_iterator for one set of positive integers in a sparse_list object.

All the public members for this class are also in the
sparse_pack_const_iterator and sparse_sizevec_const_iterator classes.
This defines the CppAD vector_of_sets iterator concept.
*/
class sparse_list_const_iterator {
private:
    /// type used by sparse_list to represent one element of the list
    typedef sparse_list::pair_size_t pair_size_t;

    /// data for the entire vector of sets
    const pod_vector<pair_size_t>& data_;

    /// Possible elements in a list are 0, 1, ..., end_ - 1;
    const size_t                   end_;

    /// next element in the singly linked list
    /// (next_pair_.value == end_ for past end of list)
    pair_size_t                    next_pair_;
public:
    /// construct a const_iterator for a list in a sparse_list object
    sparse_list_const_iterator (const sparse_list& list, size_t i)
    :
    data_( list.data_ )    ,
    end_ ( list.end_ )
    {   CPPAD_ASSERT_UNKNOWN( list.post_[i] == 0 );
        //
        size_t start = list.start_[i];
        if( start == 0 )
        {   next_pair_.next  = 0;
            next_pair_.value = end_;
        }
        else
        {   // value for this entry is reference count for list
            CPPAD_ASSERT_UNKNOWN( data_[start].value > 0 );

            // data index where list truely starts
            size_t next = data_[start].next;
            CPPAD_ASSERT_UNKNOWN( next != 0 );

            // true first entry in the list
            next_pair_ = data_[next];
            CPPAD_ASSERT_UNKNOWN( next_pair_.value < end_ );
        }
    }

    /// advance to next element in this list
    sparse_list_const_iterator& operator++(void)
    {   next_pair_  = data_[next_pair_.next];
        return *this;
    }

    /// obtain value of this element of the set of positive integers
    /// (end_ for no such element)
    size_t operator*(void)
    {   return next_pair_.value; }
};
// =========================================================================
/*!
Print the vector of sets (used for debugging)
*/
inline void sparse_list::print(void) const
{   std::cout << "sparse_list:\n";
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
// =========================================================================
// Tell pod_vector class that each pair_size_t is plain old data and hence
// the corresponding constructor need not be called.
template <> inline bool is_pod<sparse_list::pair_size_t>(void)
{   return true; }

/*!
Copy a user vector of sets sparsity pattern to an internal sparse_list object.

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
    sparse_list&            internal  ,
    const SetVector&        user      ,
    size_t                  n_set     ,
    size_t                  end       ,
    bool                    transpose ,
    const char*             error_msg )
{
# ifndef NDEBUG
    if( transpose )
        CPPAD_ASSERT_KNOWN( end == size_t( user.size() ), error_msg);
    if( ! transpose )
        CPPAD_ASSERT_KNOWN( n_set == size_t( user.size() ), error_msg);
# endif

    // iterator for user set
    std::set<size_t>::const_iterator itr;

    // size of internal sparsity pattern
    internal.resize(n_set, end);

    if( transpose )
    {   // transposed pattern case
        for(size_t j = 0; j < end; j++)
        {   itr = user[j].begin();
            while(itr != user[j].end())
            {   size_t i = *itr++;
                CPPAD_ASSERT_KNOWN(i < n_set, error_msg);
                internal.add_element(i, j);
            }
        }
    }
    else
    {   for(size_t i = 0; i < n_set; i++)
        {   itr = user[i].begin();
            while(itr != user[i].end())
            {   size_t j = *itr++;
                CPPAD_ASSERT_KNOWN( j < end, error_msg);
                internal.add_element(i, j);
            }
        }
    }
    return;
}

} } // END_CPPAD_LOCAL_NAMESPACE
# endif
