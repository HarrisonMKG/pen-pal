#include "KortexRobot.cpp"

int main()
{
	KortexRobot pen_pal("1.2.3.4");
	vector<vector<float>> matrix = pen_pal.read_csv("../coordinates/ir_sensor_data.csv");

	for (const auto& row : matrix) {
        for (float value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
	return 0;

}
