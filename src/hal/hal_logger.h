#pragma once

#include <cstdint>
#include <string>

namespace fc {
namespace hal {

/// Log severity levels
enum class LogLevel {
    DEBUG,
    INFO,
    WARN,
    ERROR
};

/// Abstract logger interface
class HalLogger {
public:
    virtual ~HalLogger() = default;

    /// Log a message with the given severity level
    /// @param level Log severity level
    /// @param message Log message string
    virtual void log(LogLevel level, const std::string& message) = 0;

    /// Log flight data record (CSV format for analysis)
    /// @param timestamp_ms Timestamp in milliseconds
    /// @param data Comma-separated data fields
    virtual void log_data(uint64_t timestamp_ms, const std::string& data) = 0;
};

}  // namespace hal
}  // namespace fc
