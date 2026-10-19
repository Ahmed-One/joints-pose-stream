#ifndef JOINTS_STREAM_LOGGER_HPP
#define JOINTS_STREAM_LOGGER_HPP

#include <ctime>
#include <deque>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <string>

enum class Level { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, FATAL = 4 };

inline std::ostream& operator<<(std::ostream& os, Level level) {
    switch (level) {
        case Level::DEBUG: os << "DEBUG"; break;
        case Level::INFO:  os << "INFO"; break;
        case Level::WARN:  os << "WARN"; break;
        case Level::ERROR: os << "ERROR"; break;
        case Level::FATAL: os << "FATAL"; break;
        default:           os << "UNKNOWN"; break;
    }
    return os;
}

class Log {
private:
    Level level_;
    std::string message_;
    std::time_t timestamp_;
    mutable std::mutex mutex_;

public:
    explicit Log(Level level)
        : level_(level), timestamp_(std::time(nullptr)) {}

    Level GetLevel() const {
        return level_;
    }

    std::string GetMessage() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return message_;
    }

    std::time_t GetTimestamp() const {
        return timestamp_;
    }

    void Append(const std::string& text) {
        std::lock_guard<std::mutex> lock(mutex_);
        message_ += text;
    }

    Log& operator<<(const std::string& text) {
        Append(text);
        return *this;
    }
};

namespace LOG
{
inline std::deque<Log> logs;
inline std::mutex logs_mutex;

inline Log& Info() {
    std::lock_guard<std::mutex> lock(logs_mutex);
    logs.emplace_back(Level::INFO);
    return logs.back();
}

inline Log& Warn() {
    std::lock_guard<std::mutex> lock(logs_mutex);
    logs.emplace_back(Level::WARN);
    return logs.back();
}

inline Log& Error() {
    std::lock_guard<std::mutex> lock(logs_mutex);
    logs.emplace_back(Level::ERROR);
    return logs.back();
}

inline Log& Debug() {
    std::lock_guard<std::mutex> lock(logs_mutex);
    logs.emplace_back(Level::DEBUG);
    return logs.back();
}

inline Log& Fatal() {
    std::lock_guard<std::mutex> lock(logs_mutex);
    logs.emplace_back(Level::FATAL);
    return logs.back();
}

inline std::size_t GetLogCount() {
    std::lock_guard<std::mutex> lock(logs_mutex);
    return logs.size();
}

inline bool ContainsMessage(const std::string& message) {
    std::lock_guard<std::mutex> lock(logs_mutex);
    for (const Log& log : logs) {
        if (log.GetMessage().find(message) != std::string::npos) {
            return true;
        }
    }
    return false;
}

inline void Dump() {
    std::lock_guard<std::mutex> lock(logs_mutex);
    for (const Log& log : logs) {
        const std::time_t timestamp = log.GetTimestamp();
        std::tm timeinfo{};
        localtime_r(&timestamp, &timeinfo);
        std::cout << std::put_time(&timeinfo, "%Y-%m-%d %H:%M:%S")
                  << " | " << std::setw(5) << log.GetLevel()
                  << " | " << log.GetMessage() << std::endl;
    }
}
}  // namespace LOG

#endif  // JOINTS_STREAM_LOGGER_HPP
