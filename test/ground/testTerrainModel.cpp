//
// Created by rgrandia on 28.07.19.
//

#include <gtest/gtest.h>

#include <ocs2_switched_model_interface/ground/TerrainModel.h>

using namespace switched_model;

TEST(testTerrainModel, stanceOnly) {
  std::shared_ptr<GaitSequence> gaitSequence(new GaitSequence());
  gaitSequence->time = {};
  gaitSequence->contactFlags = { {true, true, true, true} };

  TerrainModel terrainModel(gaitSequence);
  terrainModel.loadTestTerrain();

  Eigen::VectorXd desState(6);
  desState.setZero();
  desState[5] = 0.45;
  ocs2::CostDesiredTrajectories<double> costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory() = {0.0, 1.0};
  costDesiredTrajectories.desiredStateTrajectory().push_back(desState);
  costDesiredTrajectories.desiredStateTrajectory().push_back(desState);

  terrainModel.update(0.0, 1.0, desState, costDesiredTrajectories, nullptr);
  // No switches --> not terrain assigned
  for (int leg = 0; leg<4; leg ++){
    Eigen::MatrixXd constraints = terrainModel.getTerrainConstraints(0.0, leg);
    ASSERT_EQ(constraints.rows(), 0);
  }
}

TEST(testTerrainModel, liftedLegs) {
  std::shared_ptr<GaitSequence> gaitSequence(new GaitSequence());
  gaitSequence->time = {0.5};
  gaitSequence->contactFlags = { {false, true, true, true}, {false, false, true, true} };

  TerrainModel terrainModel(gaitSequence);
  terrainModel.loadTestTerrain();

  Eigen::VectorXd desState(6);
  desState.setZero();
  desState[5] = 0.45;
  ocs2::CostDesiredTrajectories<double> costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory() = {0.0, 1.0};
  costDesiredTrajectories.desiredStateTrajectory().push_back(desState);
  costDesiredTrajectories.desiredStateTrajectory().push_back(desState);

  terrainModel.update(0.0, 1.0, desState, costDesiredTrajectories, nullptr);
  std::vector<double> testTimes { -1.0, 0.0, 0.25, 0.5, 0.75, 1.5};
  // no switched to standing --> no terrain assigned
  for (auto t : testTimes) {
    for (int leg = 0; leg < 4; leg++) {
      Eigen::MatrixXd constraints = terrainModel.getTerrainConstraints(t, leg);
      ASSERT_EQ(constraints.rows(), 0);
    }
  }
}

TEST(testTerrainModel, step) {
  std::shared_ptr<GaitSequence> gaitSequence(new GaitSequence());
  gaitSequence->time = {0.0, 0.5};
  gaitSequence->contactFlags = { {true, true, true, true}, {false, true, true, true}, {true, true, true, true} };

  TerrainModel terrainModel(gaitSequence);
  terrainModel.loadTestTerrain();

  Eigen::VectorXd desState(6);
  desState.setZero();
  desState[5] = 0.45;
  ocs2::CostDesiredTrajectories<double> costDesiredTrajectories;
  costDesiredTrajectories.desiredTimeTrajectory() = {0.0, 1.0};
  costDesiredTrajectories.desiredStateTrajectory().push_back(desState);
  costDesiredTrajectories.desiredStateTrajectory().push_back(desState);

  std::vector<double> testTimes { -1.0, 0.0, 0.25, 0.5, 0.75, 1.5};
  terrainModel.update(0.0, 1.0, desState, costDesiredTrajectories, nullptr);
  // 1 leg lifts and lands, should have constraints only after touchdown
  for (auto t : testTimes) {
    for (int leg = 0; leg < 4; leg++) {
      Eigen::MatrixXd constraints = terrainModel.getTerrainConstraints(t, leg);
      if (leg == 0 && t > 0.5) {
        ASSERT_GT(constraints.rows(), 0);
      } else {
        ASSERT_EQ(constraints.rows(), 0);
      }
    }
  }

  // Propagate mpc horizon
  terrainModel.update(0.25, 1.25, desState, costDesiredTrajectories, nullptr);
  // 1 leg lifts and lands, should have constraints only after touchdown
  for (auto t : testTimes) {
    for (int leg = 0; leg < 4; leg++) {
      Eigen::MatrixXd constraints = terrainModel.getTerrainConstraints(t, leg);
      if (leg == 0 && t > 0.5) {
        ASSERT_GT(constraints.rows(), 0);
      } else {
        ASSERT_EQ(constraints.rows(), 0);
      }
    }
  }

  // When mpc passes the last event time, terrain is still assigned
  terrainModel.update(0.75, 1.75, desState, costDesiredTrajectories, nullptr);
  // 1 leg has landed and should have constraint for all times
  for (auto t : testTimes) {
    for (int leg = 0; leg < 4; leg++) {
      Eigen::MatrixXd constraints = terrainModel.getTerrainConstraints(t, leg);
      if (leg == 0) {
        ASSERT_GT(constraints.rows(), 0);
      } else {
        ASSERT_EQ(constraints.rows(), 0);
      }
    }
  }
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}