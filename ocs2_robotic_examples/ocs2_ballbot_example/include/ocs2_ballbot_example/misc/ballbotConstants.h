/*!
 * definitions.h
 *
 *  Created on: March 12, 2019
 *      Author: Maria Vittoria Minniti
 */

#ifndef BALLBOT_CONSTANTS_H_
#define BALLBOT_CONSTANTS_H_

namespace ocs2{
namespace ballbot{

	static constexpr double ballRadius_ = 0.125;
	static constexpr double wheelRadius_ = 0.064;
	static constexpr double heightBallCenterToBase_ = 0.317; // this value is just used in simulation, the real value is smaller, due to the compressed spring

} // namespace ballbot
} //namespace ocs2

#endif /* BALLBOT_CONSTANTS_H_ */
