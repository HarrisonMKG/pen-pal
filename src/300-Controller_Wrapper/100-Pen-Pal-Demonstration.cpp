#include "KortexRobot.cpp"
#include "logger.cpp"

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
  	string coordinates_file = parsed_args.coordinates;

    // Logger logger(parsed_args.output);

    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password);
    // pen_pal.go_home();

    // pen_pal.mylogger.Log("Go home completed");
    // pen_pal.mylogger.Log("Entering Writing Mode");
	  // pen_pal.writing_mode();
    // pen_pal.mylogger.Log("Writing Mode activated");
    // pen_pal.mylogger.Log("Executing Read Csv using filename: " + coordinates_file, INFO);
  
    vector<vector<float>> matrix = pen_pal.read_csv(coordinates_file);
    pen_pal.start_plot();

    vector<vector<float>> matrix_subset(matrix);
    for(auto &points : matrix_subset)
    {
      points = {points.begin() + 1, points.end() - 1};
    }

    pen_pal.plot(matrix_subset);
    // pen_pal.mylogger.Log("Read CSV complete");
    // pen_pal.mylogger.Log("Executing Move cartesian.", INFO);

    //pen_pal.move_cartesian(matrix);

    return 0;
}
