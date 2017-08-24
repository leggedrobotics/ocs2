/*!
* @file   robcogen_tests.cpp
* @author Jan Carius
* @date   Aug, 2017
*/

#include <gtest/gtest.h>
#include "TestQuadrupedModel.hpp"
#include "kindr/common/gtest_eigen.hpp"
#include <kindr/Core>
#include "gtest_quadruped_model.hpp"
#include <c_anymal_switched_model/kinematics/AnymalKinematics.h>
#include <Eigen/Core>

using namespace quadruped_model;

class RobCoGenTest : public TestQuadrupedModel {
public:
};


TEST_F(RobCoGenTest, JacobianBodyToFoot){
  const std::string urdfFile("anymal_beth");
  initModel(urdfFile);

  int legId = 0;

  std::cout << "LF_HAA"<< "," << "LF_HFE"<< "," << "LF_KFE"<< "," <<
               "RF_HAA"<< "," << "RF_HFE"<< "," << "RF_KFE"<< "," <<
               "LH_HAA"<< "," << "LH_HFE"<< "," << "LH_KFE"<< "," <<
               "RH_HAA"<< "," << "RH_HFE"<< "," << "RH_KFE"<< "," <<
               "LF_FOOT_x_in_BASE" << "," <<
               "LF_FOOT_y_in_BASE" << "," <<
               "LF_FOOT_z_in_BASE" << std::endl;


  quadruped_model::BranchEnum branch = getModelPtr()->getBranchEnumFromLimbUInt(legId);

  quadruped_model::QuadrupedState state;
  Eigen::Matrix<double,3,1> footPositionQuadrupedModel;

  for(int i=0; i<50; i++){
    //state.setRandom();
    state.setZero();

    if(i==0){
      state.getJointPositions().setZero();
    }else if(i==1){
      state.getJointPositions() << 1,0,0,1,0,0,1,0,0,1,0,0;
    }else if(i==2){
      state.getJointPositions() << -1,0,0,1,0,0,1,0,0,1,0,0;
    }else if(i==3){
      state.getJointPositions() << 1,0,0,-1,0,0,1,0,0,1,0,0;
    }else if(i==4){
      state.getJointPositions() << 1,0,0,1,0,0,-1,0,0,1,0,0;
    }else if(i==5){
      state.getJointPositions() << 1,0,0,1,0,0,1,0,0,-1,0,0;
    }else{
      state.getJointPositions().setRandom();
    }

    getModelPtr()->setState(state, true);

    /*
     * Foot Position
     */

    getModelPtr()->getPositionBodyToBody(footPositionQuadrupedModel,
        quadruped_model::BodyEnum::BASE, quadruped_model::BodyEnum::LF_FOOT,
        quadruped_model::CoordinateFrame::BASE);

    for(int j=0; j<12; j++){
      std::cout << state.getJointPositions()(j) << ",";
    }

    std::cout << footPositionQuadrupedModel(0) << ","
              << footPositionQuadrupedModel(1) << ","
              << footPositionQuadrupedModel(2) << std::endl;
  }


  Eigen::Matrix<double,12,1> jointCoords = state.getJointPositions().toImplementation();

  Eigen::Matrix<double,3,1> footPositionRobCoGen;
  anymal::AnymalKinematics::FootPositionBaseFrame(jointCoords, legId, footPositionRobCoGen);

  KINDR_ASSERT_DOUBLE_MX_EQ(footPositionQuadrupedModel, footPositionRobCoGen, 1e-6, "");

  /*
   * Foot Jacobian
   */
  Eigen::MatrixXd jacobianWorldToLFFootInBaseFrameFromQuadrupedModel(6,18);
  // jacobianWorldToLFFootInBaseFrameFromQuadrupedModel.setZero();
  // getModelPtr()->getJacobianSpatialWorldToBody(
  //     jacobianWorldToLFFootInBaseFrameFromQuadrupedModel,
  //     branch,
  //     quadruped_model::BodyNodeEnum::FOOT,
  //     quadruped_model::CoordinateFrame::BASE);

  Eigen::MatrixXd jacobianTranslationBaseToLFFootInBaseFrameFromQuadrupedModel(3,18);
  jacobianTranslationBaseToLFFootInBaseFrameFromQuadrupedModel.setZero();
  getModelPtr()->getJacobianTranslationFloatingBaseToBody(
      jacobianTranslationBaseToLFFootInBaseFrameFromQuadrupedModel,
      branch,
      quadruped_model::BodyNodeEnum::FOOT,
      quadruped_model::CoordinateFrame::BASE);

  Eigen::MatrixXd jacobianRotationBaseToLFFootInBaseFrameFromQuadrupedModel(3,18);
  jacobianRotationBaseToLFFootInBaseFrameFromQuadrupedModel.setZero();
  getModelPtr()->getJacobianRotationFloatingBaseToBody(
      jacobianRotationBaseToLFFootInBaseFrameFromQuadrupedModel,
      branch,
      quadruped_model::BodyNodeEnum::FOOT,
      quadruped_model::CoordinateFrame::BASE);
  jacobianWorldToLFFootInBaseFrameFromQuadrupedModel.topRows(3) =
      jacobianRotationBaseToLFFootInBaseFrameFromQuadrupedModel;
  jacobianWorldToLFFootInBaseFrameFromQuadrupedModel.bottomRows(3) =
      jacobianTranslationBaseToLFFootInBaseFrameFromQuadrupedModel;

  Eigen::Matrix<double,6,12> footJacobianRobCoGen;
  footJacobianRobCoGen.setZero();
  anymal::AnymalKinematics::FootJacobainBaseFrame(jointCoords, legId, footJacobianRobCoGen);

  KINDR_ASSERT_DOUBLE_MX_EQ(footJacobianRobCoGen,
      jacobianWorldToLFFootInBaseFrameFromQuadrupedModel.rightCols(12), 1e-6, "");

}
