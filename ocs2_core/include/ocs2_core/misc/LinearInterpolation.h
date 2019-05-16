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
	  timeStampPtr_(nullptr),
	  dataPtr_(nullptr)
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
	, timeStampPtr_(timeStampPtr)
	, dataPtr_(dataPtr)
	{
		if (timeStampPtr_ == nullptr)
			throw std::runtime_error("timeStampPtr is nullptr.");
		if (dataPtr_ == nullptr)
			throw std::runtime_error("dataPtr is nullptr.");
	}

	/**
	 * Copy constructor
	 *
	 * @param [in] arg: Instance of the other class.
	 */
	LinearInterpolation(const LinearInterpolation& arg) = default;

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
    if (timeStampPtr == nullptr) throw std::runtime_error("timeStampPtr is nullptr.");
		reset();
		timeStampPtr_ = timeStampPtr;
	}

    /**
     * Sets data
     *
     * @param [in] dataPtr: A pointer to the data.
     */
	void setData(const std::vector<Data_T, Alloc>* dataPtr) {
    if (dataPtr == nullptr) throw std::runtime_error("dataPtr is nullptr.");
		reset();
		dataPtr_ = dataPtr;
	}

    /**
     * Sets zero
     */
	void setZero() {

		reset();
		zeroFunction_ = true;
	}

    /**
     * Linearly interpolates at the given time.
     *
     * @param [in]  enquiryTime: The enquiry time for interpolation.
     * @param [out] enquiryData: The value of the trajectory at the requested time.
     * @param [in]  greatestLessTimeStampIndex (optional): The greatest smaller time stamp index. If provided, the interpolation will skip
     * the search scheme and readily calculates the output.
     */
	void interpolate(
			const scalar_t& enquiryTime,
			Data_T& enquiryData,
			int greatestLessTimeStampIndex = -1) const {

		if (zeroFunction_==true) {
			enquiryData.setZero();
			return;
		}

		const std::vector<scalar_t>& timeStamp = *timeStampPtr_;
		const std::vector<Data_T, Alloc>& dataArray = *dataPtr_;

		const int timeStampSize = timeStamp.size();

		if (timeStampSize==0 || dataArray.size()!=timeStampSize)
			throw std::runtime_error("LinearInterpolation.h : Sizes not suitable for interpolation.");

		if (greatestLessTimeStampIndex == -1)
			index_ = find(enquiryTime);
		else
			index_ = greatestLessTimeStampIndex;

		if (index_==-1) {
			enquiryData = dataArray.front();
			index_ = 0;
			return;
		}

		if (index_ == timeStampSize-1) {
			enquiryData = dataArray.back();
			index_ = timeStampSize - 1;
			return;
		}

		scalar_t alpha = (enquiryTime - timeStamp[index_ + 1]) / (timeStamp[index_] - timeStamp[index_ + 1]);
		enquiryData = alpha * dataArray[index_] + (1 - alpha) * dataArray[index_ + 1];
	}

	/**
	 * Returns the greatest smaller time stamp index found in the last interpolation function call.
	 * @return The greatest smaller time stamp index.
	 */
	int getGreatestLessTimeStampIndex() const {

		return index_;
	}

protected:
    /**
     * Finds the index of the greatest smaller time stamp index for the enquiry time.
     *
     * @param [in] enquiryTime: The enquiry time for interpolation.
     * @return The greatest smaller time stamp index.
     */
	int find(const scalar_t& enquiryTime) const {

		const std::vector<scalar_t>& timeStamp = *timeStampPtr_;
		const int timeStampSize = timeStamp.size();

		int index = -1;

		if (timeStamp[index_] > enquiryTime) {
			for (int i = index_; i >= 0; i--) {
				index = i;
				if (timeStamp[i] <= enquiryTime)
					break;
			}
		} else {
			for (int i = index_; i < timeStampSize; i++) {
				index = i;
				if (timeStamp[i] > enquiryTime) {
					index--;
					break;
				}
			}
		}

		return index;
	}

private:
	mutable int index_;

	bool zeroFunction_;

	const std::vector<scalar_t>* timeStampPtr_;
	const std::vector<Data_T, Alloc>* dataPtr_;
};

// Specialization for Eigen types
template <typename Data_T>
using EigenLinearInterpolation = LinearInterpolation<Data_T, Eigen::aligned_allocator<Data_T>>;

} // namespace ocs2

#endif /* LINEARINTERPOLATION_H_ */
