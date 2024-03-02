#include "KortexRobot.cpp"
#include "logger.cpp"


int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
  	string input_coordinates_file = parsed_args.coordinates;
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
  
    vector<vector<float>> expected_waypoints = pen_pal.read_csv(input_coordinates_file);
    vector<vector<float>> measured_joint_angles = pen_pal.move_cartesian(expected_waypoints);

  cout<< "Generating Perfromance File..." << endl;
    vector<vector<float>> measured_waypoints = pen_pal.generate_performance_file("measured_waypoints.csv",measured_joint_angles);
  cout<< "Calculating Plot:" <<endl;
    pen_pal.plot(expected_waypoints,measured_waypoints);
    float rms = pen_pal.rms_error(expected_waypoints,measured_waypoints); 
    cout << "RMS Error:\t"<< rms <<endl;

    // pen_pal.mylogger.Log("Read CSV complete");
    // pen_pal.mylogger.Log("Executing Move cartesian.", INFO);

    return 0;
}
