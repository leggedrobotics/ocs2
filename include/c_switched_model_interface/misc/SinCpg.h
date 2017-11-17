/*
 * SinCpg.h
 *
 *  Created on: Jul 5, 2016
 *      Author: farbod
 */

#ifndef SINCPG_H_
#define SINCPG_H_

#include <cmath>
#include <memory>

#include "CpgBase.h"

namespace switched_model {

class SinCpg : public CpgBase
{
public:
	typedef std::shared_ptr<SinCpg> Ptr;

	SinCpg(const double& swingLegLiftOff = 0.15, const double& swingTimeScale = 1.0)
	: CpgBase(swingLegLiftOff, swingTimeScale)
	{}

	~SinCpg() {}

	double calculatePosition(const double& time) override {
		const double omega = M_PI/(finalTime_-startTime_);
		double position = maxHight_ * pow(sin( omega*(time-startTime_) ), 2);
		return position;
	}

	double calculateVelocity(const double& time) override {
		const double omega = M_PI/(finalTime_-startTime_);
		double velocity = 2*omega * maxHight_ * cos( omega*(time-startTime_) ) * sin( omega*(time-startTime_) );
		return velocity;
	}

private:

};


}  // end of switched_model namespace


#endif /* SINCPG_H_ */
