/*
 * FindActiveIntervalIndex.h
 *
 *  Created on: Dec 7, 2017
 *      Author: farbod
 */

#ifndef FINDACTIVEINTERVALINDEX_OCS2_H_
#define FINDACTIVEINTERVALINDEX_OCS2_H_

#include <stdexcept>
#include <vector>

#include "ocs2_core/OCS2NumericTraits.h"

namespace ocs2{

/**
 * This function finds the interval in the input timeIntervals vector to which the input time belongs
 * to it. For an input timeIntervals vector of size n, we have n-1 intervals indexed from 0 to n-2.
 * If enquiryTime is smaller than timeIntervals.front(), the function returns -1. If enquiryTime is
 * greater than timeIntervals.back() it returns n-1 which is a non-existing interval index. Otherwise
 * enquiryTime is in interval i if: timeIntervals[i] < t <= timeIntervals[i+1]. The equality is in the
 * sense that two values are in the epsilon vicinity. Moreover as an exceptional case, if the time is
 * equal to timeIntervals.begin(), the function returns index 0.
 *
 * User can utilize the third input argument to incorporate his/her guessed output. The function then uses
 * this guessed index to start searching from. This can potentially increase the speed of the function if
 * the guess is reasonable. Notice that the overloaded function with two input arguments implements such a
 * heuristic for the case that the function is called for an increasing or a decreasing sequence of enquiries.
 * However, unlike this function, the overloaded function is not atomic which is potentially problematic in
 * multi-thread settings.
 *
 * @param [in] timeIntervals: a non-decreasing time sequence representing the time intervals segmentation points.
 * @param [in] enquiryTime: Enquiry time.
 * @param [in] guessedIndex: User guessed index for increasing efficiency.
 * @param [in] epsilon: Defines epsilon-vicinity equality.
 * @return: The active interval index. The output is an integer from -1 to n-1 where n is the size of
 * the timeIntervals vector.
 */
template <typename scalar_t, typename alloc_t = std::allocator<scalar_t> >
int findActiveIntervalIndex(
		const std::vector<scalar_t, alloc_t>& timeIntervals,
		const scalar_t& enquiryTime,
		const int& guessedIndex,
		scalar_t epsilon = OCS2NumericTraits<scalar_t>::week_epsilon()) {

	const int numTimeIntervals = timeIntervals.size()-1;

	if (numTimeIntervals < 1)
		throw std::runtime_error("The time interval array should have at least 2 elements.");

	if (guessedIndex<0 || guessedIndex>numTimeIntervals-1)
		throw std::runtime_error("The guessed index is out of range.");

	int index = -1;

	scalar_t timeMinus = enquiryTime - epsilon;

	if (timeMinus < timeIntervals[guessedIndex]) {
		for (int i=guessedIndex; i>=0; i--)  {
			if (timeIntervals[i] <= timeMinus) {
				index = i;
				break;
			}
		}
	} else {
		for (int i=guessedIndex; i<=numTimeIntervals; i++) {
			index = i;
			if (timeMinus < timeIntervals[i]) {
				index--;
				break;
			}
		}
	}

	// initial time case for epsilon > 0
	if (index==-1 && epsilon > 0)
		if(enquiryTime >= timeIntervals.front()-epsilon)
			index = 0;

	// final time case for epsilon < 0
	if (index==numTimeIntervals && epsilon < 0)
		if(enquiryTime <= timeIntervals.back()-epsilon)
			index = numTimeIntervals-1;

	return index;
}


/**
 * This function finds the interval in the input timeIntervals vector to which the input time belongs
 * to it. For an input timeIntervals vector of size n, we have n-1 intervals indexed from 0 to n-2.
 * If enquiryTime is smaller than timeIntervals.front(), the function returns -1. If enquiryTime is
 * greater than timeIntervals.back() it returns n-1 which is a non-existing interval index. Otherwise
 * enquiryTime is in interval i if: timeIntervals[i] < t <= timeIntervals[i+1]. The equality is in the
 * sense that two values are in the epsilon vicinity. Moreover as an exceptional case, if the time is
 * equal to timeIntervals.begin(), the function returns index 0.
 *
 * Note: do not assign this call output directly to unsigned integer, since the function may also return -1.
 *
 * This method also has an internal memory which remembers the output from the last call. Thus, if the
 * enquiries have an increasing or a decreasing trend it finds the solution faster. If you wish to
 * disable this feature, use the overloaded function with three inputs and set the last argument to
 * zero. Moreover, due to this internal memory this function call is not an atomic operation. In this
 * case, it is advised to use the overloaded function.
 *
 * @param [in] timeIntervals: a non-decreasing time sequence representing the time intervals segmentation points.
 * @param [in] enquiryTime: Enquiry time.
 * @param [in] epsilon: Defines epsilon-vicinity equality.
 * @return: The active interval index. The output is an integer from -1 to n-1 where n is the size of
 * the timeIntervals vector.
 */
template <typename scalar_t, typename alloc_t = std::allocator<scalar_t> >
int findActiveIntervalIndex(
		const std::vector<scalar_t, alloc_t>& timeIntervals,
		const scalar_t& enquiryTime,
		scalar_t epsilon = OCS2NumericTraits<scalar_t>::week_epsilon()) {

	static int guessedIndex_ = 0;

	int index = findActiveIntervalIndex(timeIntervals, enquiryTime, guessedIndex_, epsilon);

	guessedIndex_ = std::max(index,0);

	return index;
}

} // namespace ocs2

#endif /* FINDACTIVEINTERVALINDEX_OCS2_H_ */
