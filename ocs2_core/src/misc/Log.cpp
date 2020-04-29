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

static boost::shared_ptr<text_sink_t> console_sink_;
static boost::shared_ptr<file_sink_t> file_sink_;

BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", SeverityLevel);

const std::string& toString(const SeverityLevel lvl) {
  static const std::unordered_map<SeverityLevel, std::string> severityMap = {
      {DEBUG, "DEBUG"}, {INFO, "INFO"}, {WARNING, "WARNING"}, {ERROR, "ERROR"}};
  return severityMap.at(lvl);
}

SeverityLevel fromString(const std::string& severity) {
  static const std::unordered_map<std::string, SeverityLevel> severityMap = {
      {"DEBUG", DEBUG}, {"INFO", INFO}, {"WARNING", WARNING}, {"ERROR", ERROR}};
  return severityMap.at(severity);
}

Settings loadSettings(const std::string& filename, const std::string& fieldName) {
  Settings settings;
  boost::property_tree::ptree pt;
  boost::property_tree::read_info(filename, pt);

  loadData::loadPtreeValue(pt, settings.console_, fieldName + ".console", false);

  std::string consoleSeverity = toString(settings.consoleSeverity_);  // keep default
  loadData::loadPtreeValue(pt, consoleSeverity, fieldName + ".consoleSeverity", false);
  settings.consoleSeverity_ = fromString(consoleSeverity);

  loadData::loadPtreeValue(pt, settings.file_, fieldName + ".file", false);

  std::string fileSeverity = toString(settings.fileSeverity_);  // keep default
  loadData::loadPtreeValue(pt, fileSeverity, fieldName + ".fileSeverity", false);
  settings.fileSeverity_ = fromString(fileSeverity);

  loadData::loadPtreeValue(pt, settings.fileName_, fieldName + ".fileName", false);

  return settings;
}

template <typename T>
static inline void printOption(std::ostream& stream, const T& value, const std::string& name, bool updated = true, long printWidth = 80) {
  const std::string nameString = " #### '" + name + "'";
  stream << nameString;

  printWidth = std::max<long>(printWidth, nameString.size() + 15);
  stream.width(printWidth - nameString.size());
  const char fill = stream.fill('.');

  if (updated) {
    stream << value << '\n';
  } else {
    stream << value << " (default)\n";
  }

  stream.fill(fill);
}

std::ostream& operator<<(std::ostream& stream, const Settings& settings) {
  stream << "\n #### Log Settings:\n";
  stream << " #### =============================================================================\n";

  printOption(stream, settings.console_, "console");
  printOption(stream, settings.consoleSeverity_, "consoleSeverity");
  printOption(stream, settings.file_, "file");
  printOption(stream, settings.fileSeverity_, "fileSeverity");
  printOption(stream, settings.fileName_, "fileName");

  stream << " #### =============================================================================\n";
}

void init(const Settings& settings, std::ostream* console_stream) {
  auto core = boost::log::core::get();

  if (settings.console_) {
    auto backend = boost::make_shared<boost::log::sinks::text_ostream_backend>();
    backend->add_stream(boost::shared_ptr<std::ostream>(console_stream, boost::null_deleter()));
    backend->auto_flush(false);
    auto sink = boost::make_shared<text_sink_t>(backend);
    sink->set_formatter(boost::log::expressions::stream << "[ " << std::setw(7) << std::setfill(' ') << ocs2::log::severity << " ] "
                                                        << boost::log::expressions::smessage);
    sink->set_filter(ocs2::log::severity >= settings.consoleSeverity_);
    core->add_sink(sink);
    console_sink_ = sink;
  }

  if (settings.file_) {
    auto backend = boost::make_shared<boost::log::sinks::text_file_backend>(boost::log::keywords::file_name = settings.fileName_,
                                                                            boost::log::keywords::auto_flush = true,
                                                                            boost::log::keywords::open_mode = std::ios::out);
    auto sink = boost::make_shared<file_sink_t>(backend);
    sink->set_formatter(boost::log::expressions::stream
                        << boost::log::expressions::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << " [ "
                        << std::setw(7) << std::setfill(' ') << ocs2::log::severity << " ] " << boost::log::expressions::smessage);
    sink->set_filter(ocs2::log::severity >= settings.fileSeverity_);
    core->add_sink(sink);
    file_sink_ = sink;
  }

  boost::log::add_common_attributes();
}

void reset() {
  auto core = boost::log::core::get();

  if (console_sink_) {
    core->remove_sink(console_sink_);
    console_sink_.reset();
  }

  if (file_sink_) {
    core->remove_sink(file_sink_);
    file_sink_.reset();
  }
}

logger_t& getLogger() {
  static logger_t logger;
  return logger;
}

}  // namespace log
}  // namespace ocs2
