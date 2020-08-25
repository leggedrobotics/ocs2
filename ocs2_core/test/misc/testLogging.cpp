/******************************************************************************
Copyright (c) 2020, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <gtest/gtest.h>

#include <array>
#include <fstream>
#include <sstream>
#include <string>

#include <boost/algorithm/string/predicate.hpp>

#include <ocs2_core/misc/Log.h>

TEST(testLogging, canPrintMessage) {
  ocs2::log::Settings settings;

  settings.useLogFile = false;
  settings.useConsole = true;
  settings.consoleSeverity = ocs2::log::SeverityLevel::DEBUG;

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

  settings.useConsole = false;
  settings.useLogFile = true;
  settings.logFileSeverity = ocs2::log::SeverityLevel::DEBUG;
  settings.logFileName = "log/testLogging.log";

  ocs2::log::init(settings);

  OCS2_LOG(DEBUG) << "A debug severity message";
  OCS2_LOG(INFO) << "An informational severity message";
  OCS2_LOG(WARNING) << "A warning severity message";
  OCS2_LOG(ERROR) << "An error severity message";

  ocs2::log::reset();

  std::ifstream file(settings.logFileName);
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

  settings.useLogFile = false;
  settings.useConsole = true;
  settings.consoleSeverity = ocs2::log::SeverityLevel::DEBUG;

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

  settings.useLogFile = false;
  settings.useConsole = true;
  settings.consoleSeverity = ocs2::log::SeverityLevel::ERROR;

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
  useConsole        1         ; enable console log
  consoleSeverity   INFO      ; console severity level
  useLogFile        1         ; enable file log
  logFileSeverity   WARNING   ; file severity level
  logFileName       ocs2.log  ; log file name
}
)";
  file.close();

  ocs2::log::Settings settings = ocs2::log::loadSettings(settingsFileName, "log");

  EXPECT_EQ(settings.useConsole, true);
  EXPECT_EQ(settings.consoleSeverity, ocs2::log::SeverityLevel::INFO);
  EXPECT_EQ(settings.useLogFile, true);
  EXPECT_EQ(settings.logFileSeverity, ocs2::log::SeverityLevel::WARNING);
  EXPECT_EQ(settings.logFileName, "ocs2.log");
}
