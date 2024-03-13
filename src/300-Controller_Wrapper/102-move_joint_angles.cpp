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
    
    pen_pal.get_gain_values(gain_file);
    // pen_pal.execute_demo();
    int joint_string_idx = input_coordinates_file.find("__joints");
  	string original_cart_file = input_coordinates_file.substr(0,joint_string_idx);
    original_cart_file = original_cart_file + ".csv";

    cout<<original_cart_file<<endl;
    vector<vector<float>> expected_waypoints = pen_pal.read_csv(original_cart_file);
   cout<<input_coordinates_file<<endl;
    vector<vector<float>> expected_angles = pen_pal.read_csv(input_coordinates_file, 1);
    vector<vector<float>> measured_joint_angles = pen_pal.move_cartesian(expected_angles, repeat, 180.0, 0.0, 90.0, true);
    //cout<<expected_waypoints[0][1] <<endl;


    //cout << "FIRST :" << expected_waypoints[0][1] << ", " << expected_waypoints[0][2] << ", " << expected_waypoints[0][3] << endl;
    //vector<float> temp = {expected_waypoints[0][1], expected_waypoints[0][2], expected_waypoints[0][3]};
    
    //pen_pal.calculate_bias(temp);

    cout<< "Generating Log File..." << endl;
    vector<vector<float>> measured_waypoints = pen_pal.generate_log("measured_waypoints.csv",measured_joint_angles);
    cout<< "Calculating Plot:" <<endl;
    pen_pal.plot(expected_waypoints,measured_waypoints);
    vector<float> rms = pen_pal.rms_error(expected_waypoints,measured_waypoints); 
    cout << "Spatial RMS Error:\t"<< rms[0] <<endl;
    cout << "Velocity RMS Expected:\t" << rms[1] <<endl;
    cout << "Velocity RMS Measured:\t" << rms[2] <<endl;
    cout << "Velocity RMS Error:\t"<< rms[3] <<endl;
    return 0;
}
