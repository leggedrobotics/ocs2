/*
 * ModeSequence_Keyboard_Quadruped.h
 *
 *  Created on: Oct 11, 2018
 *      Author: farbod
 */

#ifndef MODESEQUENCE_KEYBOARD_QUADRUPED_H_
#define MODESEQUENCE_KEYBOARD_QUADRUPED_H_

#include <algorithm>
#include <iomanip>
#include <string>
#include <thread>
#include <vector>

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/command/ModeSequence_ROS_Interface.h>
#include <ocs2_switched_model_interface/core/MotionPhaseDefinition.h>

namespace switched_model {

/**
 * This class implements ModeSequence communication using ROS.
 *
 * @tparam SCALAR_T: scalar type.
 */
template <typename SCALAR_T>
class ModeSequence_Keyboard_Quadruped : public ocs2::ModeSequence_ROS_Interface<SCALAR_T> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef ocs2::ModeSequence_ROS_Interface<SCALAR_T> BASE;
  typedef typename BASE::scalar_t scalar_t;
  typedef typename BASE::mode_sequence_template_t mode_sequence_template_t;

  /**
   * Constructor.
   *
   * @param [in] robotName: The robot's name.
   */
  ModeSequence_Keyboard_Quadruped(const std::string& gaitFile, const std::string& robotName = "robot", bool verbose = false)
      : BASE(robotName) {
    loadStdVector(gaitFile, "list", gaitList_, verbose);

    gaitBanck_.clear();
    for (const auto& s : gaitList_) {
      mode_sequence_template_t modeSequenceTemplate;
      loadModeSequenceTemplate<scalar_t>(gaitFile, s, modeSequenceTemplate, verbose);
      gaitBanck_[s] = modeSequenceTemplate;
    }
  }

  /**
   * Default destructor
   */
  ~ModeSequence_Keyboard_Quadruped() = default;

  /**
   * Gets command line input. If the input command is list, the method will display
   * available set of gaits.
   *
   * @param [in] commadMsg: Message to be displayed on screen.
   */
  void getKeyboardCommand(const std::string& commadMsg = "Enter the desired gait, for the list of available gait enter list") {
    while (ros::ok() && ros::master::check()) {
      // get command line
      std::cout << commadMsg << ": ";
      gaitCommand_ = getCommandLine();

      if (gaitCommand_.empty()) continue;

      if (gaitCommand_.compare("list") == 0) {
        printGaitList(gaitList_);
        continue;
      }

      mode_sequence_template_t modeSequenceTemplate;
      bool foundGiat = false;
      for (const auto& s : gaitList_)
        if (gaitCommand_.compare(s) == 0) {
          modeSequenceTemplate = gaitBanck_[gaitCommand_];
          foundGiat = true;
        }

      // publish cost desired trajectories
      if (foundGiat == true) {
        BASE::publishModeSequenceTemplate(modeSequenceTemplate);
      } else {
        std::cerr << "WARNING: undefined gait!" << std::endl;
        printGaitList(gaitList_);
        continue;
      }

    }  // end of while loop
  }

 protected:
  /**
   * Gets the gait from the command line.
   *
   * @return gaitCommand: The commanded gait.
   */
  std::string getCommandLine() {
    std::vector<std::string> gaitCommand(0);

    // Set up a thread to read user inputs
    std::string line;
    bool lineRead;
    std::thread thr([&line, &lineRead]() {
      lineRead = false;
      getline(std::cin, line);
      lineRead = true;
    });

    // wait till line is read or terminate if ROS is gone.
    ros::WallRate rate(30);
    while (!lineRead) {
      if (!ros::ok() || !ros::master::check()) {
        std::terminate();  // Need to terminate thread that is still waiting for input
      }
      rate.sleep();
    }
    thr.join();

    std::istringstream stream(line);
    std::string in;
    while (stream >> in) gaitCommand.push_back(in);

    if (gaitCommand.size() != 1) {
      std::cout << "WARNING: The command should be a single word." << std::endl;
      gaitCommand.resize(1);
      gaitCommand.front().clear();
    }

    // lower case transform
    std::transform(gaitCommand.front().begin(), gaitCommand.front().end(), gaitCommand.front().begin(), ::tolower);

    return gaitCommand.front();
  }

  /**
   * Prints the list of available gaits.
   *
   * @param [in] gaitList: The list of available gaits.
   */
  void printGaitList(const std::vector<std::string>& gaitList) const {
    std::cerr << "List of available gaits:" << std::endl;
    size_t itr = 0;
    for (const auto& s : gaitList) std::cerr << "[" << itr++ << "]: " << s << std::endl;
  }

 private:
  std::vector<std::string> gaitList_;
  std::map<std::string, mode_sequence_template_t> gaitBanck_;

  std::string gaitCommand_;
};

}  // end of namespace switched_model

#endif /* MODESEQUENCE_KEYBOARD_QUADRUPED_H_ */
