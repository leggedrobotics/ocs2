/*
 * LinearInterpolation.h
 *
 *  Created on: Dec 27, 2015
 *      Author: farbod
 */

#ifndef LINEARINTERPOLATION_OCS2_H_
#define LINEARINTERPOLATION_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <memory>
#include <vector>


namespace ocs2{

/**
 * Linear Interpolation Class
 * @tparam Data_T
 * @tparam Alloc
 */
template <typename Data_T, class Alloc=std::allocator<Data_T> >
class LinearInterpolation
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	/**
	 * Constructor
	 */
	LinearInterpolation()

	: index_(0),
	  zeroFunction_(false),
	  timeStampSize_(0),
	  timeStampPtr_(NULL),
	  dataPtr_(NULL)
	{}

	/**
	 * Constructor
	 * @param [in] timeStampPtr
	 * @param [in] dataPtr
	 */
	LinearInterpolation(const std::vector<double>* timeStampPtr, const std::vector<Data_T,Alloc>* dataPtr)

	: index_(0),
	  zeroFunction_(false),
	  timeStampSize_(timeStampPtr->size()),
	  timeStampPtr_(timeStampPtr),
	  dataPtr_(dataPtr)
	{
		checkTimeStamp();
		if (dataPtr_==NULL)  throw std::runtime_error("dataPtr is not initialized.");
	}

	/**
	 * Constructor
	 * @param [in] arg
	 */
	LinearInterpolation(const LinearInterpolation& arg):
		index_(arg.index_),
		timeStampSize_(arg.timeStampSize_),
		zeroFunction_(arg.zeroFunction_),
		timeStampPtr_(arg.timeStampPtr_),
		dataPtr_(arg.dataPtr_)
	{}

	/**
	 * Reset function
	 */
	void reset()  {
		index_ = 0;
		zeroFunction_ = false;
	}

    /**
     * Sets the time stamp
     * @param [in] timeStampPtr
     */
	void setTimeStamp(const std::vector<double>* timeStampPtr)	{
		reset();
		timeStampSize_ = timeStampPtr->size();
		timeStampPtr_ = timeStampPtr;
		checkTimeStamp();
	}

    /**
     * Sets data
     * @param [in] dataPtr
     */
	void setData(const std::vector<Data_T,Alloc>* dataPtr)	{
		reset();
		dataPtr_ = dataPtr;
		if (dataPtr_==NULL)  throw std::runtime_error("dataPtr is not initialized.");
	}

    /**
     * Sets zero
     */
	void setZero()	{
		reset();
		zeroFunction_ = true;
	}

    /**
     * Interpolate function
     * @param [in] enquiryTime
     * @param [out] enquiryData
     * @param [in] greatestLessTimeStampIndex
     */
	void interpolate(const double& enquiryTime, Data_T& enquiryData, int greatestLessTimeStampIndex = -1) {

		if (zeroFunction_==true)  {
			enquiryData.setZero();
			return;
		}

		if (dataPtr_->size() != timeStampSize_)
			throw std::runtime_error("LinearInterpolation.h : The size of timeStamp vector is not equal to the size of data vector.");

		if (timeStampSize_==1)  {
			enquiryData = dataPtr_->front();
			return;
		}

		if (greatestLessTimeStampIndex == -1)
			index_ = find(enquiryTime);
		else
			index_ = greatestLessTimeStampIndex;

		if (enquiryTime<timeStampPtr_->front()) {
			enquiryData = dataPtr_->front();
			return;
		}

		if (index_==timeStampSize_-1) {
			enquiryData = dataPtr_->back();
			return;
		}

		double alpha = (enquiryTime-timeStampPtr_->at(index_+1)) / (timeStampPtr_->at(index_)-timeStampPtr_->at(index_+1));
		enquiryData = alpha*dataPtr_->at(index_) + (1-alpha)*dataPtr_->at(index_+1);
	}

	size_t getGreatestLessTimeStampIndex() { return index_; }

protected:
    /**
     * Find function
     * @param enquiryTime
     * @return
     */
	size_t find(const double& enquiryTime) {

		int index = -1;

		if (timeStampPtr_->at(index_) > enquiryTime) {
			for (int i=index_; i>=0; i--)  {
				index = i;
				if (timeStampPtr_->at(i) <= enquiryTime)
					break;
			}
		} else {
			for (int i=index_; i<timeStampSize_; i++) {
				index = i;
				if (timeStampPtr_->at(i) > enquiryTime) {
					index = i-1;
					break;
				}
			}
		}

		// throw error if index is wrong
		if(index < 0)
			throw std::runtime_error("LinearInterpolation.h : index in protected member find((const double& enquiryTime) not computed properly");

		return (size_t)index;
	}

    /**
     * Checks the time stamp
     */
	void checkTimeStamp() {
		if (timeStampPtr_==NULL)  	throw std::runtime_error("timeStampPtr is not initialized.");
		if (timeStampSize_==0)  	throw std::runtime_error("LinearInterpolation.h : LinearInterpolation is not initialized.");
	}

	void checkData() {

	}

private:
	size_t index_;
	bool zeroFunction_;

	size_t timeStampSize_;
	const std::vector<double>* timeStampPtr_;
	const std::vector<Data_T,Alloc>* dataPtr_;

};


} // namespace ocs2

#endif /* LINEARINTERPOLATION_H_ */
