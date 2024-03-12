#include "KortexRobot.cpp"
#include "logger.cpp"


int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);
  	string input_coordinates_file = parsed_args.coordinates;
  	string gain_file = parsed_args.gain;
  	bool repeat = parsed_args.repeat;
  	bool demo = parsed_args.demo;

    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password, demo);
    pen_pal.execute_demo();
    pen_pal.get_gain_values(gain_file);
  
    vector<vector<float>> expected_waypoints = pen_pal.read_csv(input_coordinates_file);
    vector<vector<float>> measured_joint_angles = pen_pal.move_cartesian(expected_waypoints, repeat);

    cout<< "Generating Log File..." << endl;
    vector<vector<float>> measured_waypoints = pen_pal.generate_log("measured_waypoints.csv",measured_joint_angles);
    cout<< "Calculating Plot:" <<endl;
    pen_pal.plot(expected_waypoints,measured_waypoints);
    vector<float> rms = pen_pal.rms_error(expected_waypoints,measured_waypoints); 
    cout << "Spatial RMS Error:\t"<< rms[0] <<endl;
    cout << "Velocity RMS Error:\t"<< rms[1] <<endl;

    return 0;
}
