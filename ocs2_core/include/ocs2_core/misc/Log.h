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

#pragma once

#include <iostream>
#include <string>

#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>

namespace ocs2 {
namespace log {

/** Log severity level enum */
enum class SeverityLevel : size_t { DEBUG, INFO, WARNING, ERROR };

/* OCS2 multithreaded severity logger type */
using logger_t = boost::log::sources::severity_logger_mt<SeverityLevel>;

/**
 * Get string name of severity level
 * @param [in] lvl: severity level enum
 * @return name
 */
const std::string& toString(const SeverityLevel lvl);

/**
 * Get severity level from string severity
 * @param [in] severity: severity name
 * @return severity level
 */
SeverityLevel fromString(const std::string& severity);

/** Write severity level to stream. */
template <typename CharT, typename TraitsT>
inline std::basic_ostream<CharT, TraitsT>& operator<<(std::basic_ostream<CharT, TraitsT>& strm, SeverityLevel lvl) {
  strm << ocs2::log::toString(lvl);
  return strm;
}

/** Log settings struct */
struct Settings {
  /** Enable pringing log to console stream */
  bool useConsole = true;
  /** Log console severity level */
  SeverityLevel consoleSeverity = SeverityLevel::DEBUG;
  /** Enable writing log to file */
  bool useLogFile = false;
  /** Log file severity level */
  SeverityLevel logFileSeverity = SeverityLevel::INFO;
  /** File name, supports boost log file name pattern including date and time. */
  std::string logFileName = "ocs2_%Y%m%d_%H%M%S.log";
};

/**
 * Load log settings from file
 * @param [in] fileName: settings file name
 * @param [in] fieldName: log settings field name
 * @return Log settings
 */
Settings loadSettings(const std::string& fileName, const std::string& fieldName);

/** Write log settings to stream. */
std::ostream& operator<<(std::ostream& stream, const Settings& settings);

/** Init OCS2 logger with settings */
void init(const Settings& settings, std::ostream* console_stream = &std::clog);

/** Reset OCS2 logger sinks */
void reset();

/**
 * Get global OCS2 logger
 * @return global logger reference
 */
logger_t& getLogger();

/**
 * Logging helper macro
 *
 * \code{.cpp}
 * OCS2_LOG(INFO) << "Hello, world!";
 * \endcode
 */
#define OCS2_LOG(LVL) \
  BOOST_LOG_STREAM_WITH_PARAMS(::ocs2::log::getLogger(), (::boost::log::keywords::severity = ::ocs2::log::SeverityLevel::LVL))

/* Compact helper macros */
#define OCS2_DEBUG OCS2_LOG(DEBUG)
#define OCS2_INFO OCS2_LOG(INFO)
#define OCS2_WARN OCS2_LOG(WARNING)
#define OCS2_ERROR OCS2_LOG(ERROR)

}  // namespace log
}  // namespace ocs2
