#include <ocs2_anymal_interface/AnymalPyBindings.h>

class AnymalModel {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  AnymalModel() = default;

  Eigen::Vector4d getFootHeights(anymal::AnymalPyBindings::state_vector_t x) {
    Eigen::Vector4d footHeights;

    constexpr int JOINT_COORD_SIZE = 12;
    Eigen::Matrix<double, 6, 1> comPose = x.template head<6>();
    Eigen::Matrix<double, JOINT_COORD_SIZE, 1> qJoints = x.template segment<JOINT_COORD_SIZE>(12);

    Eigen::Matrix<double, 6, 1> basePose;
    comModel_.calculateBasePose(qJoints, comPose, basePose);

    kinematicModel_.update(basePose, qJoints);
    Eigen::Matrix3d o_R_b = kinematicModel_.rotationMatrixOrigintoBase().transpose();

    for (size_t i = 0; i < 4; i++) {
      Eigen::Vector3d b_footPosition;
      kinematicModel_.footPositionBaseFrame(i, b_footPosition);

      // calculates foot position in the origin frame
      footHeights(i) = (o_R_b * b_footPosition + basePose.template tail<3>())(2);
    }
    return footHeights;
  }

  anymal::AnymalKinematics kinematicModel_;
  anymal::AnymalCom comModel_;
};

#define ROBOT_EQUAL_STATE_INPUT_DIMS
#include <ocs2_comm_interfaces/ocs2_interfaces/Pybind_Macros.h>

CREATE_ROBOT_PYTHON_BINDINGS(anymal::AnymalPyBindings, AnymalPyBindings)
#undef ROBOT_EQUAL_STATE_INPUT_DIMS
