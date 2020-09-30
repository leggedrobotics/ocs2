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

/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <pinocchio/fwd.hpp>

#include <ocs2_core/Types.h>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace pinocchio {

using scalar_t = ocs2::scalar_t;

// types
extern template struct ModelTpl<scalar_t, 0, JointCollectionDefaultTpl>;
extern template struct DataTpl<scalar_t, 0, JointCollectionDefaultTpl>;

// kinematics
extern template void forwardKinematics<scalar_t, 0, JointCollectionDefaultTpl>(const Model& model, Data& data,
                                                                               const Eigen::MatrixBase<Eigen::VectorXd>& q);
// frames
extern template void updateFramePlacements<scalar_t, 0, JointCollectionDefaultTpl>(const Model& model, Data& data);

// jacobian
extern template const Data::Matrix6x& computeJointJacobians<scalar_t, 0, JointCollectionDefaultTpl>(
    const Model& model, Data& data, const Eigen::MatrixBase<Data::ConfigVectorType>& q);
extern template const Data::Matrix6x& computeJointJacobians<scalar_t, 0, JointCollectionDefaultTpl>(const Model& model, Data& data);

// urdf
namespace urdf {

extern template Model& buildModel<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename, const Model::JointModel& rootJoint,
                                                                          Model& model, const bool verbose);
extern template Model& buildModel<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename, Model& model, const bool verbose);

extern template Model& buildModelFromXML<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename,
                                                                                 const Model::JointModel& rootJoint, Model& model,
                                                                                 const bool verbose);
extern template Model& buildModelFromXML<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename, Model& model,
                                                                                 const bool verbose);

}  // namespace urdf

}  // namespace pinocchio
