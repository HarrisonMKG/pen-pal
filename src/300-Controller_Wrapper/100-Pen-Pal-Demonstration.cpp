#include "KortexRobot.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
  	string coordinates_file = "../"+parsed_args.coordinates; // '../' to make path relative to the src directory

    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password, "output_dir");
    pen_pal.mylogger.Log("Running Go home command", INFO);
    pen_pal.go_home();
    pen_pal.mylogger.Log("Go home completed. Executing Read Csv using filename: " + coordinates_file, INFO);
	  pen_pal.writing_mode();
  
    vector<vector<float>> matrix = pen_pal.read_csv(coordinates_file);
    pen_pal.mylogger.Log("Executing Move cartesian.", INFO);

    pen_pal.move_cartesian(matrix);
    pen_pal.mylogger.Log("Completed Move cartesian, done example.", INFO);
    return 0;
}
