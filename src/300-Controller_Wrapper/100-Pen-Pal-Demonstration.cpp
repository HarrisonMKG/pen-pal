#include "KortexRobot.cpp"
#include "logger.cpp"



int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
  	string coordinates_file = parsed_args.coordinates;
  	string gain_file = parsed_args.gain;


    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password);
    
    // pen_pal.go_home();
    pen_pal.get_gain_values(gain_file);
    // pen_pal.output_arm_limits_and_mode();
    // pen_pal.mylogger.Log("Go home completed");
    // pen_pal.mylogger.Log("Entering Writing Mode");
	  // pen_pal.writing_mode();
    // pen_pal.mylogger.Log("Writing Mode activated");
    // pen_pal.mylogger.Log("Executing Read Csv using filename: " + coordinates_file, INFO);
  
    vector<vector<float>> matrix = pen_pal.read_csv(coordinates_file);

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
