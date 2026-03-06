#pragma once

#include "hal_logger.h"
#include <iostream>
#include <fstream>
#include <string>

namespace fc {
namespace sitl {

/// SITL Logger implementation - outputs to stdout and optional CSV file
class SitlLogger : public hal::HalLogger {
public:
    /// @param data_file Optional CSV file path for flight data logging
    /// @param enable_console Enable console log output
    explicit SitlLogger(const std::string& data_file = "",
                        bool enable_console = true);
    ~SitlLogger();

    void log(hal::LogLevel level, const std::string& message) override;
    void log_data(uint64_t timestamp_ms, const std::string& data) override;

    /// Write CSV header for data file
    void write_data_header(const std::string& header);

private:
    bool console_enabled_;
    std::ofstream data_file_;
    bool data_file_open_;

    static const char* level_str(hal::LogLevel level);
};

}  // namespace sitl
}  // namespace fc
