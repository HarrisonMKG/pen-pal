#include <iostream>
#include <fstream>
#include <string>
#include <sys/stat.h>
#include <ctime>


class Logger {
public:

    bool quiet = false;
    std::string file_prefix = "";
    std::string filename_gen = "";
    std::string filename_info = "";
    std::string filename_error = "";
    std::string filename_warning = "";

    Logger() {
        quiet = false;
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

    bool create_folder_and_files(const std::string& output_folder = "log_folder", const std::string& filename_prefix = "log", bool dontLog = false) {
        struct stat st;
        bool success = true;
        std::cout << "OUTPUTING LOG FOLDER NAME: " << output_folder;

        // Checking if folder exits, if not, create one
        if (stat(output_folder.c_str(), &st) != 0) {
            int result = 0;
            #if defined(_WIN32)
            result = mkdir(output_folder.c_str());
            #else 
            result = mkdir(output_folder.c_str(), 775); // notice that 777 is different than0777
            #endif

            if (result == 0) {
                std::cout << "Created output folder: '" << output_folder << "'. All log files will be stored there" << std::endl;
            }else {
                std::cerr << "Failed to create folder '" << output_folder << "'." << std::endl;
                dontLog = true;
                success = false;
            }
        } else {
            std::cout << "Output Folder Already exists, files will be found at: " << output_folder << std::endl;
        }

        file_prefix = output_folder + "/" + filename_prefix;
        filename_gen = file_prefix + ".txt";
        filename_info = file_prefix + "_info.txt";
        filename_error = file_prefix + "_error.txt";
        filename_warning = file_prefix + "_warning.txt";

        file_info_.open(filename_info, std::ios::out | std::ios::app);
        file_error_.open(filename_error, std::ios::out | std::ios::app);
        file_warning_.open(filename_warning, std::ios::out | std::ios::app);
        file_gen_.open(filename_gen, std::ios::out | std::ios::app);

        std::time_t current_time = std::time(nullptr);
        std::string timestamp = std::asctime(std::localtime(&current_time));
        timestamp = timestamp.substr(0, timestamp.size() - 1);  // Remove the newline character

        
        if (!file_info_.is_open()) {
            std::cerr << "Error: Failed to open the INFO log file." << std::endl;
            success = false;
        }
        if (!file_error_.is_open()) {
            std::cerr << "Error: Failed to open the ERROR log file." << std::endl;
            success = false;
        }
        if (!file_warning_.is_open()) {
            success = false;
            std::cerr << "Error: Failed to open the WARNING log file." << std::endl;
        }
        if (!file_gen_.is_open()) {
            success = false;
            std::cerr << "Error: Failed to open the GEN log file." << std::endl;
        }
        std::string new_run_msg = "=========================\nBeginning of this Log attempt\n" + timestamp + "\n=========================";
        LogToFile(new_run_msg, file_info_);
        LogToFile(new_run_msg, file_error_);
        LogToFile(new_run_msg, file_warning_);
        LogToFile(new_run_msg, file_gen_);
        
        return success;

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
        std::string final_message = level_str + "[" + timestamp + "] " + message; 
        // Output to the terminal
        if (outputTerminal)
        {
            std::cout << final_message << std::endl;
        }
        if (outputLogFile)
        {
            LogToFile(final_message, *temp_file);
            LogToFile(final_message, file_gen_);
        }

        
    }

    void LogToFile(const std::string& message, std::ofstream &file_name){

        if (file_name.is_open()) {
            file_name << message << std::endl;
        }
    }

private:
    std::ofstream file_info_;
    std::ofstream file_error_;
    std::ofstream file_warning_;
    std::ofstream file_gen_;
};
