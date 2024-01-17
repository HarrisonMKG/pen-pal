#include "KortexRobot.cpp"
#include "Logger.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    Logger logger("mylog.txt");

	KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password);
	pen_pal.go_home();
    logger.Log("This is an info message.", LogLevel::INFO);
    logger.Log("This is a warning message.", LogLevel::WARNING);
    logger.Log("This is an error message.", LogLevel::ERROR);


	vector<vector<float>> matrix = pen_pal.read_csv("../coordinates/ir_sensor_data.csv");

	for (const auto& row : matrix) {
        for (float value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
	return 0;

}
