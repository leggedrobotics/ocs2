//
// Created by Timon Kaufmann in June 2021
//

#include "ocs2_anymal_commands/MotionCommandInterface.h"

#include "rclcpp/rclcpp.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>

#include "ocs2_anymal_commands/LoadMotions.h"

namespace switched_model
{

  MotionCommandInterface::MotionCommandInterface(const std::string &configFile)
  {
    bool verbose = false;

    scalar_t dt = 0.0;
    ocs2::loadData::loadCppDataType(configFile, "dt", dt);

    std::vector<std::string> motionList;
    ocs2::loadData::loadStdVector(configFile, "motionList", motionList, verbose);

    const std::string motionFilesPath = ament_index_cpp::get_package_share_directory("ocs2_anymal_commands") + "/config/motions/";
    for (const auto &motionName : motionList)
    {
      const auto csvData = readCsv(motionFilesPath + motionName + ".txt");
      motionData_.insert({motionName, readMotion(csvData, dt)});
    }
    printAnimationList();
  }

  void MotionCommandInterface::getKeyboardCommand()
  {
    const std::string commandMsg = "Enter the desired motion, for the list of available motions enter \"list\"";
    std::cout << commandMsg << ": ";

    auto shouldTerminate = []()
    { return !rclcpp::ok(); };
    const std::string motionCommand = ocs2::getCommandLineString(shouldTerminate);

    if (motionCommand.empty())
    {
      return;
    }

    if (motionCommand == "list")
    {
      printAnimationList();
      return;
    }

    try
    {
      const auto &motion = motionData_.at(motionCommand);
      std::cout << "Executing \"" << motionCommand << "\" \n";
      publishMotion(motion);
    }
    catch (const std::out_of_range &e)
    {
      std::cout << "Motion \"" << motionCommand << "\" not found.\n";
      printAnimationList();
    }
  }

  void MotionCommandInterface::printAnimationList() const
  {
    std::cout << "\nList of available motions:\n";
    for (const auto &s : motionData_)
    {
      std::cout << "  * " << s.first << "\n";
    }
    std::cout << std::endl;
  }

} // namespace switched_model