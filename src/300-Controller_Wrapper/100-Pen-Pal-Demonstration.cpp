#include "KortexRobot.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
	string coordinates_file = "../coordinates/square_cartesian.csv";

    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password, "output_dir");
    pen_pal.go_home();

    vector<vector<float>> matrix = pen_pal.read_csv(coordinates_file);
    pen_pal.move_cartesian(matrix);

    return 0;
}
