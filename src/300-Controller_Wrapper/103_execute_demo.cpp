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
    pen_pal.execute_demo();
    
    return 0;
}
