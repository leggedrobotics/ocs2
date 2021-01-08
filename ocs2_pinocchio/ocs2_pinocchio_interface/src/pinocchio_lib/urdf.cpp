/*
 * Author: Michael Spieler
 * Date:   2020-09-08
 */

#include <ocs2_pinocchio_interface/pinocchio_lib.h>

#include <pinocchio/parsers/urdf.hpp>

namespace pinocchio {

namespace urdf {

template Model& buildModel<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename, const Model::JointModel& rootJoint,
                                                                   Model& model, const bool verbose);
template Model& buildModel<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename, Model& model, const bool verbose);

template Model& buildModelFromXML<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename, const Model::JointModel& rootJoint,
                                                                          Model& model, const bool verbose);
template Model& buildModelFromXML<scalar_t, 0, JointCollectionDefaultTpl>(const std::string& filename, Model& model, const bool verbose);

}  // namespace urdf

}  // namespace pinocchio
