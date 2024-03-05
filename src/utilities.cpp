#include <cstdlib>
#include "utilities.h"

//Helper function to parse program arguments
ExampleArgs ParseExampleArguments(int argc, char *argv[])
{
    cxxopts::Options options(argv[0], "Kortex example");
    
    options.add_options()
        ("ip", "IP address of destination", cxxopts::value<std::string>()->default_value("10.0.0.160"))
        ("h,help", "Print help")
        ("u,username", "username to login", cxxopts::value<std::string>()->default_value("admin"))
        ("p,password", "password to login", cxxopts::value<std::string>()->default_value("admin"))
        ("o,output", "output directory", cxxopts::value<std::string>()->default_value("kortex_logs"))
        ("c,coordinates", "file path to coordinates csv data", cxxopts::value<std::string>()->default_value("coordinates/ir_sensor_data.csv"))
        ("g,gain", "file path to gain values used", cxxopts::value<std::string>()->default_value("../gain_values/gain_1.txt"))
        ("r,repeat", "flag to indicate whether to repeate trajectories", cxxopts::value<std::string>()->default_value("N"))
        ("d,demo", "Indicate whether running demo. Will remove extra logs and code during execution", cxxopts::value<std::string>()->default_value("N"))
    ;

    ExampleArgs resultArgs;

    try
    {
        auto parsed_options = options.parse(argc, argv);

        if (parsed_options.count("help"))
        {
            std::cout << options.help() << std::endl;
            exit(EXIT_SUCCESS);
        }
        resultArgs.repeat = false;
        resultArgs.demo = false;
        resultArgs.ip_address = parsed_options["ip"].as<std::string>();
        resultArgs.username = parsed_options["username"].as<std::string>();
        resultArgs.password = parsed_options["password"].as<std::string>();
        resultArgs.output = parsed_options["output"].as<std::string>();
        resultArgs.coordinates = parsed_options["coordinates"].as<std::string>();
        resultArgs.gain = parsed_options["gain"].as<std::string>();

        if (parsed_options["repeat"].as<std::string>() == "Y" || parsed_options["repeat"].as<std::string>() == "y"){
            resultArgs.repeat = true;
        }
        if (parsed_options["demo"].as<std::string>() == "Y" || parsed_options["demo"].as<std::string>() == "y"){
            resultArgs.demo = true;
        }
    }
    catch(const cxxopts::OptionException& exception)
    {
        std::cerr << exception.what() << std::endl;
        exit(EXIT_FAILURE);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        exit(EXIT_FAILURE);
    }
    
    return resultArgs;
}
