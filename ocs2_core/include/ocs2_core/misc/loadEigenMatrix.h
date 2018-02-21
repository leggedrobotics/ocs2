/*
 * loadEigenMatrix.h
 *
 *  Created on: Feb 16, 2018
 *      Author: farbod
 */

#ifndef OCS2_LOADEIGENMATRIX_H_
#define OCS2_LOADEIGENMATRIX_H_

#include <iostream>
#include <Eigen/Dense>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/info_parser.hpp>

namespace ocs2 {

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
inline void loadEigenMatrix(const std::string& filename, const std::string& matrixName, Eigen::MatrixBase<Derived>& matrix)  {

	typedef typename Eigen::MatrixBase<Derived>::Scalar scalar_t;

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


}  // end of ocs2 namespace



#endif /* OCS2_LOADEIGENMATRIX_H_ */
