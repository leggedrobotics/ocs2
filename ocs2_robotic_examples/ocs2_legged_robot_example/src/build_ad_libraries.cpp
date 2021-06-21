#include <urdf_parser/urdf_parser.h>

#include <ocs2_legged_robot_example/LeggedRobotInterface.h>

using namespace ocs2;
using namespace legged_robot;

static const double constructionDurationThreshold = 5.0;
static const std::string redColor = "\033[1;31m";
static const std::string defaultColor = "\033[0m";

int main(int argc, char** argv) {
  std::vector<std::string> programArgs{};
  ::ros::removeROSArgs(argc, argv, programArgs);

  std::string taskName;

  if (programArgs.size() <= 1) {
    std::cerr << redColor << "No task file specified. Using the default, \"mpc\"" << defaultColor << std::endl;
    taskName = "mpc";
  } else {
    taskName = programArgs[1];
  }

  // Initialize ros node
  ros::init(argc, argv, "build_ad_libraries");
  ros::NodeHandle nodeHandle;

  std::string urdfString;
  const bool gotUrdfParam = ros::param::get("/alma_c_description", urdfString);
  if (gotUrdfParam) {
    ::urdf::ModelInterfaceSharedPtr urdfModel = ::urdf::parseURDF(urdfString);

    // As a horrible hack, this checks whether leggedRobotInterface took more than 5 seconds to construct, which means it probably built the
    // libraries.
    const ros::Time constructionStart = ros::Time::now();
    LeggedRobotInterface leggedRobotInterface(taskName, urdfModel);
    const ros::Time constructionEnd = ros::Time::now();

    const bool initialRecompileLibrariesSetting = leggedRobotInterface.modelSettings().recompileLibraries_;

    if ((constructionEnd - constructionStart).toSec() < constructionDurationThreshold) {
      std::cerr << redColor << "Since the construction of LeggedRObotInterface took less than " << constructionDurationThreshold
                << " seconds, I assume the ad libraries didn't build. Manually building now;" << defaultColor << std::endl;
      leggedRobotInterface.modelSettings().recompileLibraries_ = true;
      std::string taskFolder = ros::package::getPath("ocs2_legged_robot_example") + "/config/" + taskName;
      std::string taskFile_ = taskFolder + "/task.info";
      std::cerr << "Loading task file: " << taskFile_ << std::endl;
      leggedRobotInterface.setupOptimizer(taskFile_);
    } else {
      std::cerr << redColor << "Since the construction of LeggedRobotInterface took more than " << constructionDurationThreshold
                << " seconds, I assume the ad libraries built!" << defaultColor << std::endl;
    }

    if (initialRecompileLibrariesSetting) {
      std::cout << redColor
                << "Just so you know, the recompileLibraries flag is set to true, so it's going to rebuild when you start the controller"
                << defaultColor << std::endl;
    }
    return 0;

  } else {
    std::cerr << "build_ad_libraries requires the robot description to be loaded into /alma_c_description" << std::endl;
    return 0;
  }
}
