#include "KortexRobot.cpp"
#include "logger.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    Logger logger("mylog");

	KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password);
	pen_pal.go_home();
    logger.Log("This is an info message.", 0);
    logger.Log("This is a warning message.", 1);
    logger.Log("This is an error message.", 2);


	vector<vector<float>> matrix = pen_pal.read_csv("../coordinates/ir_sensor_data.csv");
	for (const auto& row : matrix) {
        std::string output_line = "";

        for (float value : row) {
            // logger.Log("This is an info message.", 0);

            output_line += std::to_string(value);
            output_line += " ";
        }
        logger.Log(output_line, 0);
        // std::cout << std::endl;
    }
	return 0;

}
