#include <iostream>
#include <fstream>
#include <string>
#include <ctime>


class Logger {
public:

    bool quiet = false;
    std::string file_prefix = "";
    std::string filename_gen = "";
    std::string filename_info = "";
    std::string filename_error = "";
    std::string filename_warning = "";

    Logger(const std::string& filename_prefix = "output_log", bool dontLog = false) {
        quiet = dontLog;
        file_prefix = filename_prefix;
        filename_gen = file_prefix + ".txt";
        filename_info = file_prefix + "_info.txt";
        filename_error = file_prefix + "_error.txt";
        filename_warning = file_prefix + "_warning.txt";



        file_info_.open(filename_info, std::ios::out | std::ios::app);
        file_error_.open(filename_error, std::ios::out | std::ios::app);
        file_warning_.open(filename_warning, std::ios::out | std::ios::app);
        file_gen_.open(filename_gen, std::ios::out | std::ios::app);
        
        if (!file_info_.is_open()) {
            std::cerr << "Error: Failed to open the INFO log file." << std::endl;
        }
        if (!file_error_.is_open()) {
            std::cerr << "Error: Failed to open the ERROR log file." << std::endl;
        }
        if (!file_warning_.is_open()) {
            std::cerr << "Error: Failed to open the WARNING log file." << std::endl;
        }
        if (!file_gen_.is_open()) {
            std::cerr << "Error: Failed to open the GEN log file." << std::endl;
        }

    }

    ~Logger() {
        if (file_info_.is_open()) {
            file_info_.close();
        }
        if (file_error_.is_open()) {
            file_error_.close();
        }
        if (file_warning_.is_open()) {
            file_warning_.close();
        }
        if (file_gen_.is_open()) {
            file_gen_.close();
        }
    }

    void Log(const std::string& message, int level = 0, bool outputTerminal=true, bool outputLogFile=true) {
        if (quiet) {
            return;
        }
        std::string level_str;
        std::ofstream *temp_file;
        switch (level) {
            case 0:
                level_str = "[INFO] ";
                temp_file = &file_info_;
                break;
            case 1:
                level_str = "[WARNING] ";
                temp_file = &file_warning_;
                break;
            case 2:
                level_str = "[ERROR] ";
                temp_file = &file_error_;
                break;
        }

        std::time_t current_time = std::time(nullptr);
        std::string timestamp = std::asctime(std::localtime(&current_time));
        timestamp = timestamp.substr(0, timestamp.size() - 1);  // Remove the newline character

        // Output to the terminal
        if (outputTerminal)
        {
            std::cout << level_str << message << std::endl;
        }
        if (outputLogFile)
        {
            LogToFile(message, *temp_file, level_str);
            LogToFile(message, file_gen_, level_str);
        }

        
    }

    void LogToFile(const std::string& message, std::ofstream &file_name, std::string level = "[INFO] "){

        if (file_name.is_open()) {
            file_name << level << message << std::endl;
        }
    }

private:
    std::ofstream file_info_;
    std::ofstream file_error_;
    std::ofstream file_warning_;
    std::ofstream file_gen_;
};