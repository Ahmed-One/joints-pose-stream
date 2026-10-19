#include <cassert>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// Log level enumeration
enum class Level { DEBUG = 0, INFO = 1, WARN = 2, ERROR = 3, FATAL = 4 };

// Overload the << operator for the Level enum class
std::ostream& operator<<(std::ostream& os, Level level) {
    switch (level) {
        case Level::DEBUG: os << "DEBUG"; break;
        case Level::INFO:  os << "INFO";  break;
        case Level::WARN:  os << "WARN";  break;
        case Level::ERROR: os << "ERROR"; break;
        case Level::FATAL: os << "FATAL"; break;
        default:           os << "UNKNOWN"; break;
    }
    return os;
}

// Log entry structure
class Log {
private:
  Level level;
  std::string message;
  std::time_t timestamp;
public:
  Log(Level lvl) : level(lvl), timestamp(std::time(nullptr)) { /* ... */  }
  
  Level GetLevel() const
    {
        return level;
    }

  const std::string& GetMessage() const
  {
      return message;
  }

  std::time_t GetTimestamp() const
  {
      return timestamp;
  }

  void Append(const std::string& text)
  {
      message += text;
  }

  Log &operator<<(const std::string& text){
    this->Append(text);
    return *this;
  }
};

namespace LOG
{
  std::vector<Log> logs;

  Log& Info()
  {
      logs.emplace_back(Level::INFO);
      return logs.back();
  }

  Log& Warn()
    {
        logs.emplace_back(Level::WARN);
        return logs.back();
    }
  
  Log& Error()
  {
      logs.emplace_back(Level::ERROR);
      return logs.back();
  }

  Log& Debug()
  {
      logs.emplace_back(Level::DEBUG);
      return logs.back();
  }

  Log& Fatal()
  {
      logs.emplace_back(Level::FATAL);
      return logs.back();
  }

  std::size_t GetLogCount(){
    return logs.size();
  }

  bool ContainsMessage(const std::string &msg){
    for(const Log& log : logs){
      if(log.GetMessage().find(msg) != std::string::npos){
        return true;
      }
    }
    return false;
  }

  void Dump(){
    for(const Log& log : logs){
      std::time_t t = log.GetTimestamp();
      std::tm* timeinfo = std::localtime(&t);
      std::cout<< std::put_time(timeinfo, "%Y-%m-%d %H:%M:%S") << " | " <<
                  std::setw(5) << log.GetLevel() << " | " <<
                  log.GetMessage() << std::endl;
    }
  }
}

int main() {
  std::cout << "==============================================\n";
  std::cout << "           LOGGING SYSTEM VALIDATOR\n";
  std::cout << "==============================================\n\n";

  // Test : Basic logging functionality
  std::cout << "--- Test 1: Basic Logging ---\n";
  LOG::Info() << "System started";
  LOG::Warn() << "Configuration file missing";
  LOG::Error() << "Database connection failed";
  LOG::Debug() << "This debug message should not appear";
  LOG::Fatal() << "Application must terminate";

  // Verify logs were stored
  assert(LOG::GetLogCount() == 5);
  assert(LOG::ContainsMessage("System started"));
  assert(LOG::ContainsMessage("Configuration file missing"));

  LOG::Dump();

  std::cout << "\n==============================================\n";
  std::cout << "         ALL TESTS PASSED! ✓\n";
  std::cout << "==============================================\n";

  return 0;
}
