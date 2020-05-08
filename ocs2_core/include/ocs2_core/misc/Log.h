
#include <iostream>
#include <string>

#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_logger.hpp>

#pragma once

namespace ocs2 {
namespace log {

/** Log severity level enum */
enum SeverityLevel { DEBUG, INFO, WARNING, ERROR };

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
  bool console_ = true;
  SeverityLevel consoleSeverity_ = DEBUG;
  bool file_ = false;
  SeverityLevel fileSeverity_ = INFO;
  std::string fileName_ = "ocs2_%Y%m%d_%H%M%S.log";
};

/**
 * Load log settings from file
 * @param [in] filename: settings file name
 * @param [in] filename: log settings field name
 * @return Log settings
 */
Settings loadSettings(const std::string& filename, const std::string& fieldName);

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
#define OCS2_LOG(LVL) BOOST_LOG_STREAM_WITH_PARAMS(::ocs2::log::getLogger(), (::boost::log::keywords::severity = ::ocs2::log::LVL))

/* Compact helper macros */
#define OCS2_DEBUG OCS2_LOG(DEBUG)
#define OCS2_INFO OCS2_LOG(INFO)
#define OCS2_WARN OCS2_LOG(WARNING)
#define OCS2_ERROR OCS2_LOG(ERROR)

}  // namespace log
}  // namespace ocs2
