#include <iostream>
#include <fstream>
#include <string>
#include <ctime>

enum class LogLevel { INFO, WARNING, ERROR };

class Logger {
public:
    Logger(const std::string& filename) {
        file_.open(filename, std::ios::out | std::ios::app);
        if (!file_.is_open()) {
            std::cerr << "Error: Failed to open the log file." << std::endl;
        }
    }

    ~Logger() {
        if (file_.is_open()) {
            file_.close();
        }
    }

    void Log(const std::string& message, LogLevel level = LogLevel::INFO) {
        std::string level_str;
        switch (level) {
            case LogLevel::INFO:
                level_str = "[INFO] ";
                break;
            case LogLevel::WARNING:
                level_str = "[WARNING] ";
                break;
            case LogLevel::ERROR:
                level_str = "[ERROR] ";
                break;
        }

        std::time_t current_time = std::time(nullptr);
        std::string timestamp = std::asctime(std::localtime(&current_time));
        timestamp = timestamp.substr(0, timestamp.size() - 1);  // Remove the newline character

        // Output to the terminal
        std::cout << timestamp << " " << level_str << message << std::endl;

        // Output to the file if it's open
        if (file_.is_open()) {
            file_ << timestamp << " " << level_str << message << std::endl;
        }
    }

private:
    std::ofstream file_;
};

int main() {
    // Example usage
    Logger logger("mylog.txt");

    logger.Log("This is an info message.", LogLevel::INFO);
    logger.Log("This is a warning message.", LogLevel::WARNING);
    logger.Log("This is an error message.", LogLevel::ERROR);

    return 0;
}
