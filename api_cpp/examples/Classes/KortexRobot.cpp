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
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <KortexRobot.h>

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_RT 10001

class KortexRobot
{
private:
    /* data */
public:
    KortexRobot(const std::string& ip_address);
    ~KortexRobot();

    k_api::TransportClientTcp()
};

KortexRobot::KortexRobot(/* args */)
{
}

KortexRobot::~KortexRobot()
{
}

std::vector<std::vector<int>> read_csv(const std::string& filename) {
    std::vector<std::vector<int>> result;

    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return result;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<int> row;
        std::string cell;

        while (std::getline(ss, cell, ',')) {
            try {
                int value = std::stoi(cell);
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
