// KortexRobot.h
#ifndef KORTEX_ROBOT_HPP
#define KORTEX_ROBOT_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <sstream>

#define PORT 10000
#define PORT_RT 10001


class KortexRobot
{
private:
    /* data */

public:
    KortexRobot(const std::string& ip_address);
    ~KortexRobot();

    std::vector<std::vector<int>> read_csv(const std::string &filename);

protected:
    std::string ip_address;
};

#endif // KORTEX_ROBOT_HPP

