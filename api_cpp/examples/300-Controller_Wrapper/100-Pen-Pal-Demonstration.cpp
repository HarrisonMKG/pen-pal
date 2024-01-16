#include "KortexRobot.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);


	KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password);
	pen_pal.go_home();


	vector<vector<float>> matrix = pen_pal.read_csv("../coordinates/ir_sensor_data.csv");

	for (const auto& row : matrix) {
        for (float value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
	return 0;

}
