#include "KortexRobot.cpp"
#include "logger.cpp"


int main(int argc, char **argv)
{
    // Move arm to a specific position that can be moved to repeatedly
    // Then start script
    auto parsed_args = ParseExampleArguments(argc, argv);
  	string input_coordinates_file = parsed_args.coordinates;
  	string gain_file = parsed_args.gain;
  	bool repeat = parsed_args.repeat;
  	bool demo = parsed_args.demo;
  	string joint_file_name = parsed_args.coordinates;

    

    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password, demo);

  // Confirm starting origin points are loaded by here. Include this combination in naming convention
    cout << "STARTING POINTS" << pen_pal.altered_origin[1] << ", " << pen_pal.altered_origin[0] << ", " << pen_pal.altered_origin[2] << ", " << endl;

    if (input_coordinates_file.length() > 4 && input_coordinates_file.find(".csv")!= std::string::npos) {
      // If the provided input file is longer than 4 characters and ends with .csv, take off the csv part to modify the joint file name
      joint_file_name = input_coordinates_file.substr(0, input_coordinates_file.length() - 4) + "__joints";
      for (int i = 0; i < 3; i++) {
        if (pen_pal.altered_origin[0] < 0) {
          joint_file_name = joint_file_name + "_N" + std::to_string(pen_pal.altered_origin[i]*(-1));
        }else {
          joint_file_name = joint_file_name + "_" + std::to_string(pen_pal.altered_origin[i]);
        }
      }
      joint_file_name = joint_file_name + ".csv";
    } else {
      // For now assume naming convention of coordinate_cartestion file is wrong stop program for now
      cout << "Please use ensure name of input coordinate file is longer than 4 characters and is of type .csv" << endl;
      return 0;
    }

    cout << "Joint_coordinates file will be created under name:  " << joint_file_name << endl;
    cout << "Placed in the build folder" ;


    float kTheta_x = 180;
    float kTheta_y = 0;
    float kTheta_z = 90;
    vector<vector<float>> expected_waypoints = pen_pal.read_csv(input_coordinates_file);
    std::vector<vector<float>> target_joint_angles_IK;
    std::vector<vector<float>> target_waypoints;
    target_waypoints = pen_pal.convert_csv_to_cart_wp(expected_waypoints, kTheta_x, kTheta_y, kTheta_z);
    target_joint_angles_IK = pen_pal.convert_points_to_angles(target_waypoints);

    pen_pal.output_joint_values_to_csv(target_joint_angles_IK, joint_file_name);

    return 0;
}
