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

    KortexRobot pen_pal(parsed_args.ip_address,parsed_args.username,parsed_args.password, demo);
    float kTheta_x = 180;
    float kTheta_y = 0;
    float kTheta_z = 90;
    vector<vector<float>> expected_waypoints = pen_pal.read_csv(input_coordinates_file);
    std::vector<vector<float>> target_joint_angles_IK;
    std::vector<vector<float>> target_waypoints;
    target_waypoints = pen_pal.convert_csv_to_cart_wp(expected_waypoints, kTheta_x, kTheta_y, kTheta_z);
    target_joint_angles_IK = pen_pal.convert_points_to_angles(target_waypoints);

    pen_pal.output_joint_values_to_csv(target_joint_angles_IK, "output_joint.csv");

    return 0;
}
