/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <pinocchio/fwd.hpp>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

namespace pinocchio {

// types
extern template struct ModelTpl<double, 0, JointCollectionDefaultTpl>;
extern template struct DataTpl<double, 0, JointCollectionDefaultTpl>;

// kinematics
extern template void forwardKinematics<double, 0, JointCollectionDefaultTpl>(const Model& model, Data& data,
                                                                             const Eigen::MatrixBase<Eigen::VectorXd>& q);
// frames
extern template void updateFramePlacements<double, 0, JointCollectionDefaultTpl>(const Model& model, Data& data);

// jacobian
extern template const Data::Matrix6x& computeJointJacobians<double, 0, JointCollectionDefaultTpl>(
    const Model& model, Data& data, const Eigen::MatrixBase<Data::ConfigVectorType>& q);
extern template const Data::Matrix6x& computeJointJacobians<double, 0, JointCollectionDefaultTpl>(const Model& model, Data& data);

// urdf
namespace urdf {

extern template Model& buildModel<double, 0, JointCollectionDefaultTpl>(const std::string& filename, const Model::JointModel& rootJoint,
                                                                        Model& model, const bool verbose);
extern template Model& buildModel<double, 0, JointCollectionDefaultTpl>(const std::string& filename, Model& model, const bool verbose);

extern template Model& buildModelFromXML<double, 0, JointCollectionDefaultTpl>(const std::string& filename,
                                                                               const Model::JointModel& rootJoint, Model& model,
                                                                               const bool verbose);
extern template Model& buildModelFromXML<double, 0, JointCollectionDefaultTpl>(const std::string& filename, Model& model,
                                                                               const bool verbose);

}  // namespace urdf

}  // namespace pinocchio
