#include <gtest/gtest.h>

#include <fstream>
#include <sstream>
#include <string>
#include <array>

#include <boost/algorithm/string/predicate.hpp>

#include <ocs2_core/misc/Log.h>

TEST(testLogging, canPrintMessage) {
  ocs2::log::Settings settings;

  settings.file_ = false;
  settings.console_ = true;
  settings.consoleSeverity_ = ocs2::log::DEBUG;

  ocs2::log::init(settings);

  OCS2_DEBUG << "A debug severity message";
  OCS2_INFO << "An informational severity message";
  OCS2_WARN << "A warning severity message";
  OCS2_ERROR << "An error severity message";
  OCS2_INFO << settings;

  ocs2::log::reset();
}

TEST(testLogging, canWriteLogFile) {
  ocs2::log::Settings settings;

  settings.console_ = false;
  settings.file_ = true;
  settings.fileSeverity_ = ocs2::log::DEBUG;
  settings.fileName_ = "log/ocs2_test.log";

  ocs2::log::init(settings);

  OCS2_LOG(DEBUG) << "A debug severity message";
  OCS2_LOG(INFO) << "An informational severity message";
  OCS2_LOG(WARNING) << "A warning severity message";
  OCS2_LOG(ERROR) << "An error severity message";

  ocs2::log::reset();

  std::ifstream file(settings.fileName_);
  std::array<std::string, 4> expected = {"[   DEBUG ] A debug severity message", "[    INFO ] An informational severity message",
                                       "[ WARNING ] A warning severity message", "[   ERROR ] An error severity message"};

  for (const auto& end : expected) {
    std::string line;
    std::getline(file, line);
    EXPECT_TRUE(boost::algorithm::ends_with(line, end));
  }
}

TEST(testLogging, writesCorrectMessageToConsole) {
  ocs2::log::Settings settings;

  settings.file_ = false;
  settings.console_ = true;
  settings.consoleSeverity_ = ocs2::log::DEBUG;

  std::ostringstream console_stream;
  ocs2::log::init(settings, &console_stream);

  OCS2_LOG(DEBUG) << "A debug severity message";
  OCS2_LOG(INFO) << "An informational severity message";
  OCS2_LOG(WARNING) << "A warning severity message";
  OCS2_LOG(ERROR) << "An error severity message";

  const std::string expect =
      "[   DEBUG ] A debug severity message\n"
      "[    INFO ] An informational severity message\n"
      "[ WARNING ] A warning severity message\n"
      "[   ERROR ] An error severity message\n";

  EXPECT_EQ(console_stream.str(), expect);

  ocs2::log::reset();
}

TEST(testLogging, canFilterSeverity) {
  ocs2::log::Settings settings;

  settings.file_ = false;
  settings.console_ = true;
  settings.consoleSeverity_ = ocs2::log::ERROR;

  std::ostringstream console_stream;
  ocs2::log::init(settings, &console_stream);

  OCS2_LOG(DEBUG) << "NOT logged";
  OCS2_LOG(INFO) << "NOT logged";
  OCS2_LOG(WARNING) << "NOT logged";
  OCS2_LOG(ERROR) << "Logged";

  EXPECT_EQ(console_stream.str(), "[   ERROR ] Logged\n");

  ocs2::log::reset();
}

TEST(testLogging, canLoadSettings) {
  const std::string settingsFileName = "ocs2_test_log_settings.info";

  std::ofstream file(settingsFileName);
  file << R"(
; logging settings
log
{
  console         1         ; enable console log
  consoleSeverity INFO      ; console severity level
  file            1         ; enable file log
  fileSeverity    WARNING   ; file severity level
  fileName        ocs2.log  ; log file name
}
)";
  file.close();

  ocs2::log::Settings settings = ocs2::log::loadSettings(settingsFileName, "log");

  EXPECT_EQ(settings.console_, true);
  EXPECT_EQ(settings.consoleSeverity_, ocs2::log::INFO);
  EXPECT_EQ(settings.file_, true);
  EXPECT_EQ(settings.fileSeverity_, ocs2::log::WARNING);
  EXPECT_EQ(settings.fileName_, "ocs2.log");
}
