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

#include <iomanip>
#include <memory>
#include <unordered_map>

#include <boost/log/core.hpp>

#include <boost/core/null_deleter.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sinks/sync_frontend.hpp>
#include <boost/log/sinks/text_file_backend.hpp>
#include <boost/log/sinks/text_ostream_backend.hpp>
#include <boost/log/sources/logger.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/support/date_time.hpp>
#include <boost/log/utility/setup/common_attributes.hpp>

#include <boost/smart_ptr/make_shared.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>

#include <ocs2_core/misc/LoadData.h>
#include <ocs2_core/misc/Log.h>

namespace ocs2 {
namespace log {

using text_sink_t = boost::log::sinks::synchronous_sink<boost::log::sinks::text_ostream_backend>;
using file_sink_t = boost::log::sinks::synchronous_sink<boost::log::sinks::text_file_backend>;

static std::shared_ptr<text_sink_t> consoleSink_;
static std::shared_ptr<file_sink_t> fileSink_;

BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", SeverityLevel);

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
const std::string& toString(const SeverityLevel lvl) {
  static const std::unordered_map<SeverityLevel, std::string> severityMap = {
      {SeverityLevel::DEBUG, "DEBUG"}, {SeverityLevel::INFO, "INFO"}, {SeverityLevel::WARNING, "WARNING"}, {SeverityLevel::ERROR, "ERROR"}};
  return severityMap.at(lvl);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SeverityLevel fromString(const std::string& severity) {
  static const std::unordered_map<std::string, SeverityLevel> severityMap = {
      {"DEBUG", SeverityLevel::DEBUG}, {"INFO", SeverityLevel::INFO}, {"WARNING", SeverityLevel::WARNING}, {"ERROR", SeverityLevel::ERROR}};
  return severityMap.at(severity);
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
Settings loadSettings(const std::string& fileName, const std::string& fieldName) {
  Settings settings;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(fileName, pt);

  loadData::loadPtreeValue(pt, settings.useConsole, fieldName + ".useConsole", false);

  std::string consoleSeverity = toString(settings.consoleSeverity);  // keep default
  loadData::loadPtreeValue(pt, consoleSeverity, fieldName + ".consoleSeverity", false);
  settings.consoleSeverity = fromString(consoleSeverity);

  loadData::loadPtreeValue(pt, settings.useLogFile, fieldName + ".useLogFile", false);

  std::string logFileSeverity = toString(settings.logFileSeverity);  // keep default
  loadData::loadPtreeValue(pt, logFileSeverity, fieldName + ".logFileSeverity", false);
  settings.logFileSeverity = fromString(logFileSeverity);

  loadData::loadPtreeValue(pt, settings.logFileName, fieldName + ".logFileName", false);

  return settings;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
std::ostream& operator<<(std::ostream& stream, const Settings& settings) {
  stream << "\n #### Log Settings:\n";
  stream << " #### =============================================================================\n";

  loadData::printValue(stream, settings.useConsole, "useConsole");
  loadData::printValue(stream, settings.consoleSeverity, "consoleSeverity");
  loadData::printValue(stream, settings.useLogFile, "useLogFile");
  loadData::printValue(stream, settings.logFileSeverity, "logFileSeverity");
  loadData::printValue(stream, settings.logFileName, "logFileName");

  stream << " #### =============================================================================\n";
  return stream;
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void init(const Settings& settings, std::ostream* console_stream) {
  auto core = boost::log::core::get();

  if (settings.useConsole) {
    auto backend = boost::make_shared<boost::log::sinks::text_ostream_backend>();
    backend->add_stream(boost::shared_ptr<std::ostream>(console_stream, boost::null_deleter()));
    // disable auto flush for performance concerns
    backend->auto_flush(false);
    consoleSink_ = std::make_shared<text_sink_t>(backend);
    consoleSink_->set_formatter(boost::log::expressions::stream << "[ " << std::setw(7) << std::setfill(' ') << ocs2::log::severity << " ] "
                                                                << boost::log::expressions::smessage);
    consoleSink_->set_filter(ocs2::log::severity >= settings.consoleSeverity);

    // connect boost::shared_ptr to global consoleSink_
    core->add_sink(boost::shared_ptr<text_sink_t>(consoleSink_.get(), [=](text_sink_t*) { consoleSink_.reset(); }));
  }

  if (settings.useLogFile) {
    auto backend = boost::make_shared<boost::log::sinks::text_file_backend>(boost::log::keywords::file_name = settings.logFileName,
                                                                            boost::log::keywords::auto_flush = true,
                                                                            boost::log::keywords::open_mode = std::ios::out);
    fileSink_ = std::make_shared<file_sink_t>(backend);
    fileSink_->set_formatter(boost::log::expressions::stream
                             << boost::log::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f")
                             << " [ " << std::setw(7) << std::setfill(' ') << ocs2::log::severity << " ] "
                             << boost::log::expressions::smessage);
    fileSink_->set_filter(ocs2::log::severity >= settings.logFileSeverity);

    // connect boost::shared_ptr to global fileSink_
    core->add_sink(boost::shared_ptr<file_sink_t>(fileSink_.get(), [=](file_sink_t*) { fileSink_.reset(); }));
  }

  boost::log::add_common_attributes();
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void reset() {
  auto core = boost::log::core::get();

  if (consoleSink_ != nullptr) {
    core->remove_sink(boost::shared_ptr<text_sink_t>(consoleSink_.get(), boost::null_deleter()));
    consoleSink_.reset();
  }

  if (fileSink_ != nullptr) {
    core->remove_sink(boost::shared_ptr<file_sink_t>(fileSink_.get(), boost::null_deleter()));
    fileSink_.reset();
  }
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
logger_t& getLogger() {
  static logger_t logger;
  return logger;
}

}  // namespace log
}  // namespace ocs2
