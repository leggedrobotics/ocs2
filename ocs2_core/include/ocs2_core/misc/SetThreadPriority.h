/*
 * SetThreadPriority.h
 *
 *  Created on: Jul 25, 2018
 *      Author: farbod
 */

#ifndef SETTHREADPRIORITY_OCS2_H_
#define SETTHREADPRIORITY_OCS2_H_

#include <thread>
#include <pthread.h>

namespace ocs2{

/**
 * Sets the priority of the input thread.
 *
 * @param priority: The priority of the thread from 0 (lowest) to 99 (highest)
 * @param thread: A reference to the tread.
 */
inline void SetThreadPriority(const int priority, std::thread& thread) {

	sched_param sched;
	sched.sched_priority = priority;

	if(priority != 0) {
		if (pthread_setschedparam(thread.native_handle(), SCHED_FIFO, &sched) != 0) {
			std::cerr << "WARNING: Failed to set threads priority (one possible reason could be "
					"that the user and the group permissions are not set properly.)" << std::endl;
		}
	}

}

} // namespace ocs2


#endif /* SETTHREADPRIORITY_OCS2_H_ */
