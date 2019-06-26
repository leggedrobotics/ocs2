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

#ifndef OCS2_LOADEIGENMATRIX_H_
#define OCS2_LOADEIGENMATRIX_H_

#include <iostream>
#include <Eigen/Dense>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ocs2 {

/**
 * An auxiliary function which loads a scalar value from a file. The file uses property tree data structure with INFO format (refer to www.goo.gl/fV3yWA).
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] scalarName: The key name assigned to the scalar in the config file.
 * @param [out] value: The loaded value.
 */
template <typename scalar_t>
inline void loadScalar(const std::string& filename, const std::string& scalarName, scalar_t& value) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  value = pt.get<scalar_t>(scalarName);
}

/**
 * An auxiliary function which loads an Eigen matrix from a file. The file uses property tree data structure with INFO format (refer to www.goo.gl/fV3yWA).
 *
 * It has the following format:	<br>
 * matrixName	<br>
 * {	<br>
 *   scaling 1e+0				<br>
 *   (0,0) value    ; M(0,0)	<br>
 *   (1,0) value    ; M(1,0)	<br>
 *   (0,1) value    ; M(0,1)	<br>
 *   (1,1) value    ; M(1,1)	<br>
 * } 	<br>
 *
 * If a value for a specific element is not defined it will set by default to zero.
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] matrixName: The key name assigned to the matrix in the config file.
 * @param [out] matrix: The loaded matrix.
 */
template <typename Derived>
inline void loadEigenMatrix(
		const std::string& filename,
		const std::string& matrixName,
		Eigen::MatrixBase<Derived>& matrix)  {

	using scalar_t = typename Eigen::MatrixBase<Derived>::Scalar;

	size_t rows = matrix.rows();
	size_t cols = matrix.cols();

	boost::property_tree::ptree pt;
	boost::property_tree::read_info(filename, pt);

	double scaling = pt.get<double>(matrixName + ".scaling", 1);

	scalar_t aij;
	bool failed = false;
	for (size_t i=0; i<rows; i++)
		for (size_t j=0; j<cols; j++)  {
			try	{
				aij = pt.get<scalar_t>(matrixName + "." + "(" +std::to_string(i) + "," + std::to_string(j) + ")");
			}
			catch (const std::exception& e){
				aij = 0;
				failed = true;
			}
			matrix(i,j) = scaling * aij;
		}

	if (failed==true)  {
		std::cerr << "WARNING: Failed to load matrix type: " + matrixName + "!" << std::endl;
	}
}


}  // namespace ocs2


#endif /* OCS2_LOADEIGENMATRIX_H_ */
