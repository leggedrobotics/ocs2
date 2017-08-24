/*!
* @file    TestQuadrupedModel.hpp
* @author  Christian Gehring, Dario Bellicoso
* @date    Sep, 2015
*/

#pragma once

// gtest
#include <gtest/gtest.h>

// boost
#include <boost/filesystem.hpp>

// eigen
#include <Eigen/Core>

// alma model
#include "quadruped_model/QuadrupedModel.hpp"
#include "quadruped_model/common/rbdl_utils.hpp"

// romo tests
#include "romo_test/TestRobotModel.hpp"

class TestQuadrupedModel : virtual public romo_test::TestRobotModel<quadruped_model::QuadrupedModel> {
 private:
  using BaseTest = romo_test::TestRobotModel<quadruped_model::QuadrupedModel>;

 public:
  TestQuadrupedModel()
      : BaseTest()
  {
    model_.reset(new quadruped_model::QuadrupedModel());
    state_.reset(new quadruped_model::QuadrupedState());
  }

  virtual ~TestQuadrupedModel() {

  }

  void init() {
    initModel("starleth_unit_test");
  }

  void initModel(const std::string& urdfName, bool useQuat = true, bool verbose = false) {
    useQuaternion = useQuat;

    // Reset the state.
    state_->setZero();

    //-- Load model for testing
    boost::filesystem::path filePath(__FILE__);
    std::string path = filePath.parent_path().parent_path().generic_string() + std::string{"/resources/" + urdfName + ".urdf"};
    ASSERT_TRUE(getModelPtr()->initModelFromUrdfFile(path, useQuaternion, verbose));

    // Set the state to the model.
    model_->setState(*state_);
  }

  void getRandomGeneralizedPositionsRbdl(Eigen::VectorXd& q) {
    quadruped_model::QuadrupedState state;
    state.setRandom();
    quadruped_model::setRbdlQFromState(q, state, useQuaternion);
  }

  void setRbdlQFromState(Eigen::VectorXd& rbdlQ, const RobotStateType& state) {
    quadruped_model::setRbdlQFromState(rbdlQ, state);
  }

  void setStateFromRbdlQ(RobotStateType& state, const Eigen::VectorXd& rbdlQ) {
    quadruped_model::setStateFromRbdlQ(state, rbdlQ);
  }

 public:
  bool useQuaternion = true;
};

