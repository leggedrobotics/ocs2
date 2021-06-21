#pragma once

#include <ocs2_legged_robot_example/logic/ModeSequenceTemplate.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>
#include <vector>

namespace ocs2 {
namespace legged_robot {

/** This class implements ModeSequence communication using ROS. */
class LeggedRobotModeSequenceKeyboard {
 public:
  LeggedRobotModeSequenceKeyboard(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName,
                                  bool verbose = false);

  /** Prints the command line interface and responds to user input. Function returns after one user input. */
  void getKeyboardCommand();

 private:
  /** Gets the gait from the command line. */
  static std::string getCommandLine();

  /** Prints the list of available gaits. */
  void printGaitList(const std::vector<std::string>& gaitList) const;

  /** Publishes the mode sequence template. */
  void publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule);

  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  ros::Publisher modeSequenceTemplatePublisher_;
};

}  // namespace legged_robot
}  // end of namespace ocs2
