// KortexRobot.h
#ifdef KORTEX_ROBOT_H
#define KORTEX_ROBOT_H

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

#define PORT 10000
#define PORT_RT 10001


class KortexRobot
{
private:
    /* data */

public:
    KortexRobot(/* args */);
    ~KortexRobot();

    std::vector<std::vector<int>> read_csv(const std::string &filename);

protected:
    std::string ip_address;
};

#endif // KORTEX_ROBOT_H

