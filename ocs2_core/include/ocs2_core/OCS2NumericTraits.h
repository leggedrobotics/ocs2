/*
 * OCS2NumericTraits.h
 *
 *  Created on: Dec 15, 2017
 *      Author: farbod
 */

#ifndef OCS2NUMERICTRAITS_OCS2_H_
#define OCS2NUMERICTRAITS_OCS2_H_

namespace ocs2{
/**
 * Holds information about the various numeric traits.
 */
template<typename T=double>
class OCS2NumericTraits
{
public:

	/**
	 * If a floating number v is in limit_epsilon vicinity of w, then we assume that v approaches to w.
	 * This limit_epsilon value should be greater week_epsilon value.
	 *
	 * @return limit epsilon value.
	 */
	static inline T limit_epsilon() { return 1e-4; }

	/**
	 * Defines the precision during comparison.
	 *
	 * @return week epsilon value
	 */
	static inline T week_epsilon() { return 1e-6; }
};

} // end of ocs2 namespace

#endif /* OCS2NUMERICTRAITS_OCS2_H_ */
