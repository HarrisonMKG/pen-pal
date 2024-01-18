#include "KortexRobot.cpp"
#include "logger.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
	string coordinates_file = "../"+parsed_args.coordinates; // '../' to make path relative to the src directory

    Logger logger(parsed_args.output);

    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password);
    pen_pal.go_home();
	pen_pal.writing_mode();

    logger.Log("This is an info message1234.", 0);
    logger.Log("This is a warning message1234.", 1);
    logger.Log("This is an error message1234.", 2);

    vector<vector<float>> matrix = pen_pal.read_csv(coordinates_file);
    pen_pal.move_cartesian(matrix);

    return 0;
}
