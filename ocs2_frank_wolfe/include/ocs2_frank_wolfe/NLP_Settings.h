/*
 * NLP_Settings.h
 *
 *  Created on: Jul 30, 2018
 *      Author: farbod
 */

#ifndef NLP_SETTINGS_OCS2_H_
#define NLP_SETTINGS_OCS2_H_

#include <iostream>

namespace ocs2 {
namespace nlp {

/**
 * This structure contains the settings for the gradient-descent algorithm.
 */
class NLP_Settings
{
public:
	NLP_Settings()
	: displayGradientDescent_(true)
	, maxIterations_(1000)
	, minRelCost_(1e-6)
	, maxLearningRate_(1.0)
	, minLearningRate_(0.05)
	, useAscendingLineSearchNLP_(true)
	, minDisToBoundary_(0.0)
	{}

	/** This value determines to display the log output.*/
	bool displayGradientDescent_;
	/** This value determines the maximum number of algorithm iterations.*/
	size_t maxIterations_;
	/** This value determines the termination condition based on the minimum relative changes of the cost.*/
	double minRelCost_;
	/** This value determines the maximum step size for the line search scheme.*/
	double maxLearningRate_;
	/** This value determines the minimum step size for the line search scheme.*/
	double minLearningRate_;
	/**
	 * This value determines the line search scheme to be used. \n
	 * - \b Ascending: The step size eventually increases from the minimum value to the maximum. \n
	 * - \b Descending: The step size eventually decreases from the minimum value to the maximum.
	 * */
	bool useAscendingLineSearchNLP_;
	/** This value determines the minimum allowable difference between to consecutive switching times.*/
	double minDisToBoundary_;
};

}  // end of nlp namespace
}  // end of ocs2 namespace

#endif /* NLP_SETTINGS_OCS2_H_ */
