#include "sitl_logger.h"
#include <iomanip>

namespace fc {
namespace sitl {

SitlLogger::SitlLogger(const std::string& data_file, bool enable_console)
    : console_enabled_(enable_console), data_file_open_(false) {
    if (!data_file.empty()) {
        data_file_.open(data_file);
        data_file_open_ = data_file_.is_open();
    }
}

SitlLogger::~SitlLogger() {
    if (data_file_open_) {
        data_file_.close();
    }
}

void SitlLogger::log(hal::LogLevel level, const std::string& message) {
    if (console_enabled_) {
        std::cerr << "[" << level_str(level) << "] " << message << std::endl;
    }
}

void SitlLogger::log_data(uint64_t timestamp_ms, const std::string& data) {
    // Output to stdout for Python script consumption
    std::cout << "DATA," << timestamp_ms << "," << data << std::endl;

    // Also write to CSV file if open
    if (data_file_open_) {
        data_file_ << timestamp_ms << "," << data << "\n";
    }
}

void SitlLogger::write_data_header(const std::string& header) {
    std::cout << "HEADER," << header << std::endl;
    if (data_file_open_) {
        data_file_ << header << "\n";
    }
}

const char* SitlLogger::level_str(hal::LogLevel level) {
    switch (level) {
        case hal::LogLevel::DEBUG: return "DEBUG";
        case hal::LogLevel::INFO:  return "INFO ";
        case hal::LogLevel::WARN:  return "WARN ";
        case hal::LogLevel::ERROR: return "ERROR";
        default: return "?????";
    }
}

}  // namespace sitl
}  // namespace fc
