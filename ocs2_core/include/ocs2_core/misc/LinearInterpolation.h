/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#ifndef LINEARINTERPOLATION_OCS2_H_
#define LINEARINTERPOLATION_OCS2_H_

#include <Eigen/Dense>
#include <Eigen/StdVector>

#include <memory>
#include <vector>


namespace ocs2{

/**
 * Linear Interpolation class.
 *
 * @tparam Data_T: Date type
 * @tparam Alloc: Specialized allocation class
 */
template <typename Data_T, class Alloc=std::allocator<Data_T> >
class LinearInterpolation
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	typedef double scalar_t;

	/**
	 * Default constructor.
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
	 *
	 * @param [in] timeStampPtr: A pointer to time stamp.
	 * @param [in] dataPtr: A pointer to the data.
	 */
	LinearInterpolation(
			const std::vector<scalar_t>* timeStampPtr,
			const std::vector<Data_T,Alloc>* dataPtr)

	: index_(0)
	, zeroFunction_(false)
	, timeStampSize_(timeStampPtr->size())
	, timeStampPtr_(timeStampPtr)
	, dataPtr_(dataPtr)
	{
		checkTimeStamp();
		if (dataPtr_==NULL)  throw std::runtime_error("dataPtr is not initialized.");
	}

	/**
	 * Copy constructor
	 *
	 * @param [in] arg: Instance of the other class.
	 */
	LinearInterpolation(const LinearInterpolation& arg)

	: index_(arg.index_)
	, timeStampSize_(arg.timeStampSize_)
	, zeroFunction_(arg.zeroFunction_)
	, timeStampPtr_(arg.timeStampPtr_)
	, dataPtr_(arg.dataPtr_)
	{}

	/**
	 * Reset function
	 */
	void reset()  {

		index_ = 0;
		zeroFunction_ = false;
	}

    /**
     * Sets the time stamp.
     *
     * @param [in] timeStampPtr: A pointer to time stamp.
     */
	void setTimeStamp(const std::vector<scalar_t>* timeStampPtr) {

		reset();
		timeStampSize_ = timeStampPtr->size();
		timeStampPtr_ = timeStampPtr;
		checkTimeStamp();
	}

    /**
     * Sets data
     *
     * @param [in] dataPtr: A pointer to the data.
     */
	void setData(const std::vector<Data_T,Alloc>* dataPtr) {

		reset();
		dataPtr_ = dataPtr;
		if (dataPtr_==NULL)  throw std::runtime_error("dataPtr is not initialized.");
	}

    /**
     * Sets zero
     */
	void setZero() {

		reset();
		zeroFunction_ = true;
	}

    /**
     * Interpolate function.
     *
     * @param [in]  enquiryTime: The enquiry time for interpolation.
     * @param [out] enquiryData: The value of the trajectory at the requested time.
     * @param [in]  greatestLessTimeStampIndex (optional): The greatest smaller time stamp index. If provided, the interpolation will skip
     * the search scheme and readily calculates the output.
     */
	void interpolate(
			const scalar_t& enquiryTime,
			Data_T& enquiryData,
			int greatestLessTimeStampIndex = -1) {

		if (zeroFunction_==true)  {
			enquiryData.setZero();
			return;
		}

		if (dataPtr_->size() != timeStampSize_)
			throw std::runtime_error("LinearInterpolation.h : The size of timeStamp vector is not equal to the size of data vector.");

		if (enquiryTime<=timeStampPtr_->front()) {
			enquiryData = dataPtr_->front();
			index_ = 0;
			return;
		}

		if (enquiryTime>=timeStampPtr_->back()) {
			enquiryData = dataPtr_->back();
			index_ = timeStampSize_-1;
			return;
		}

		if (greatestLessTimeStampIndex == -1)
			index_ = find(enquiryTime);
		else
			index_ = greatestLessTimeStampIndex;

		scalar_t alpha = (enquiryTime-timeStampPtr_->at(index_+1)) / (timeStampPtr_->at(index_)-timeStampPtr_->at(index_+1));
		enquiryData = alpha*dataPtr_->at(index_) + (1-alpha)*dataPtr_->at(index_+1);
	}

	/**
	 * Returns the greatest smaller time stamp index found in the last interpolation function call.
	 * @return The greatest smaller time stamp index.
	 */
	int getGreatestLessTimeStampIndex() {

		return index_;
	}

protected:
    /**
     * Finds the index of the greatest smaller time stamp index for the enquiry time.
     *
     * @param [in] enquiryTime: The enquiry time for interpolation.
     * @return The greatest smaller time stamp index.
     */
	int find(const scalar_t& enquiryTime) {

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
					index--;
					break;
				}
			}
		}

		return index;
	}

    /**
     * Checks the time stamp
     */
	void checkTimeStamp() {

		if (timeStampPtr_==NULL)
			throw std::runtime_error("timeStampPtr is not initialized.");
		if (timeStampSize_==0)
			throw std::runtime_error("LinearInterpolation is not initialized.");
	}

private:
	int index_;
	bool zeroFunction_;

	size_t timeStampSize_;
	const std::vector<scalar_t>* timeStampPtr_;
	const std::vector<Data_T,Alloc>* dataPtr_;

};

// Specialization for Eigen types
template <typename Data_T>
using EigenLinearInterpolation = LinearInterpolation<Data_T, Eigen::aligned_allocator<Data_T>>;

} // namespace ocs2

#endif /* LINEARINTERPOLATION_H_ */
