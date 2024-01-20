#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <string>
#include <ctime>


class Logger {
public:

    // enum class LogLevel { INFO, WARNING, ERROR };

    std::string file_prefix;
    std::string filename_gen;
    std::string filename_info;
    std::string filename_error;
    std::string filename_warning;
    bool quiet;

    Logger(const std::string& filename_prefix = "output_log", bool dontLog = false);
    ~Logger();

    void Log(std::string& message, int level = 0);
    void LogToFile(const std::string& message, std::ofstream file_name);

private:
    std::ofstream file_info_;
    std::ofstream file_error_;
    std::ofstream file_warning_;
    std::ofstream file_gen_;
};

#endif // LOGGER_H