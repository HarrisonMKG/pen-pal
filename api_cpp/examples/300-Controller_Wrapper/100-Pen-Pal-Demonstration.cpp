
#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#include "utilities.h"

#define PORT 10000
#define PORT_RT 10001
using namespace std;

// TODO: Check if we need to modify/delete the license below since the structure
//       of our code will likely resemble the examples in their repo and video

/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2019 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/
namespace k_api = Kinova::Api;
class KortexRobot
{
	private:

	public:
	KortexRobot(const std::string& ip_address)
	{
		//Constructor
	}

	~KortexRobot()
	{
		//Destructor
	}

	std::vector<std::vector<float>> read_csv(const std::string& filename) {
		std::vector<std::vector<float>> result;

		std::ifstream file(filename);
		if (!file.is_open()) {
			std::cerr << "Error opening file: " << filename << std::endl;
			return result;
		}

		std::string line;
		while (std::getline(file, line)) {
			std::stringstream ss(line);
			std::vector<float> row;
			std::string cell;

			while (std::getline(ss, cell, ',')) {
				try {
					float value = std::stof(cell);
					row.push_back(value);
				} catch (const std::invalid_argument& e) {
					std::cerr << "Invalid number format in line: " << line << std::endl;
					// Handle the error or skip the invalid value
				}
			}

			result.push_back(row);
		}

		file.close();

		return result;
	}
};

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
