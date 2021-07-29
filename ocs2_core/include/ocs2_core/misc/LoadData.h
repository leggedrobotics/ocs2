/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

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

#pragma once

#include <Eigen/Dense>
#include <iostream>
#include <string>
#include <vector>

#include <boost/property_tree/info_parser.hpp>
#include <boost/property_tree/ptree.hpp>

namespace ocs2 {
namespace loadData {

/**
 * Print settings option
 *
 * @param[in] stream: Output stream
 * @param[in] value: Value to be printed
 * @param[in] name: Property field name
 * @param[in] updated: True if the value was updated
 * @param[in] printWidth: The aligned printout width
 */
template <typename T>
static inline void printValue(std::ostream& stream, const T& value, const std::string& name, bool updated = true, long printWidth = 80) {
  const std::string nameString = " #### '" + name + "'";
  stream << nameString;

  printWidth = std::max<long>(printWidth, nameString.size() + 15);
  stream.width(printWidth - nameString.size());
  const char fill = stream.fill('.');

  if (updated) {
    stream << value << '\n';
  } else {
    stream << value << " (default)\n";
  }

  stream.fill(fill);
}

/**
 * An auxiliary function to help loading OCS2 settings from a property tree
 *
 * @param[in] pt: Fully initialized tree object
 * @param[out] value: The value to be read (unchanged if tree does not contain a corresponding entry)
 * @param[in] name: Property field name
 * @param[in] verbose: Whether or not to print the extreacted value (or error)
 * @param[in] printWidth: Optional argument to change the width of the aligned printout
 */
template <typename T>
inline void loadPtreeValue(const boost::property_tree::ptree& pt, T& value, const std::string& name, bool verbose, long printWidth = 80) {
  bool updated = true;

  try {
    value = pt.get<T>(name);
  } catch (const boost::property_tree::ptree_bad_path&) {
    updated = false;
  }

  if (verbose) {
    const std::string nameString = name.substr(name.find_last_of('.') + 1);
    printValue(std::cerr, value, nameString, updated, printWidth);
  }
}

/**
 * An auxiliary function which loads value of the c++ data types from a file. The file uses property tree data structure with INFO format
 * (refer to https://www.boost.org/doc/libs/1_65_1/doc/html/property_tree.html).
 *
 * @param [in] filename: File name which contains the configuration data.
 * @param [in] dataName: The key name assigned to the data in the config file.
 * @param [out] value: The loaded value.
 */
template <typename cpp_data_t>
inline void loadCppDataType(const std::string& filename, const std::string& dataName, cpp_data_t& value) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  value = pt.get<cpp_data_t>(dataName);
}

/**
 * An auxiliary function which loads an Eigen matrix from a file. The file uses property tree data structure with INFO format (refer to
 * www.goo.gl/fV3yWA).
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
 * @param [out] matrix: The loaded matrix, must have desired size.
 */
template <typename Derived>
inline void loadEigenMatrix(const std::string& filename, const std::string& matrixName, Eigen::MatrixBase<Derived>& matrix) {
  using scalar_t = typename Eigen::MatrixBase<Derived>::Scalar;

  size_t rows = matrix.rows();
  size_t cols = matrix.cols();

  if (rows == 0 || cols == 0) {
    throw std::runtime_error("[loadEigenMatrix] Loading empty matrix \"" + matrixName + "\" is not allowed.");
  }

  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  const scalar_t scaling = pt.get<scalar_t>(matrixName + ".scaling", 1.0);
  const scalar_t defaultValue = pt.get<scalar_t>(matrixName + ".default", 0.0);

  size_t numFailed = 0;
  for (size_t i = 0; i < rows; i++) {
    for (size_t j = 0; j < cols; j++) {
      scalar_t aij;
      try {
        aij = pt.get<scalar_t>(matrixName + "." + "(" + std::to_string(i) + "," + std::to_string(j) + ")");
      } catch (const std::exception&) {
        aij = defaultValue;
        numFailed++;
      }
      matrix(i, j) = scaling * aij;
    }
  }

  if (numFailed == matrix.size()) {
    throw std::runtime_error("[loadEigenMatrix] Could not load matrix \"" + matrixName + "\" from file \"" + filename + "\".");
  } else if (numFailed > 0) {
    std::cerr << "WARNING: Loaded at least one default value in matrix: \"" + matrixName + "\"\n";
  }
}

template <typename T>
inline void loadStdVector(const std::string& filename, const std::string& topicName, std::vector<T>& loadVector, bool verbose = true) {
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  std::vector<T> backup;
  backup.swap(loadVector);
  loadVector.clear();

  size_t vectorSize = 0;
  while (true) {
    try {
      loadVector.push_back(pt.get<T>(topicName + ".[" + std::to_string(vectorSize) + "]"));
      vectorSize++;
    } catch (const std::exception&) {
      if (vectorSize == 0) {
        loadVector.swap(backup);  // if nothing could be loaded, keep the old data
      }
      break;
    }
  }  // end of while loop

  // display
  if (verbose) {
    std::cerr << " #### '" << topicName << "': {";
    if (vectorSize == 0) {
      std::cerr << " }\n";
    } else {
      for (size_t i = 0; i < vectorSize; i++) {
        std::cerr << loadVector[i] << ", ";
      }
      std::cerr << "\b\b}\n";
    }
  }
}

}  // namespace loadData
}  // namespace ocs2
