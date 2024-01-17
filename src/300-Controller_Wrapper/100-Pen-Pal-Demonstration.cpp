#include "KortexRobot.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);


	KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password);
	pen_pal.go_home();
	vector<vector<float>> matrix = pen_pal.read_csv("../coordinates/square_cartesian.csv");
	pen_pal.move_cartesian(matrix);

	return 0;
}
