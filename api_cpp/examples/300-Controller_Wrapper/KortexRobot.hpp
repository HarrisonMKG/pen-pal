// KortexRobot.hpp

#ifndef KORTEXROBOT_HPP
#define KORTEXROBOT_HPP

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


class KortexRobot
{
private:
    /* data */

public:
    KortexRobot(const std::string& ip_address);
    ~KortexRobot();

    std::vector<std::vector<float>> read_csv(const std::string &filename);

protected:
    std::string ip_address;
};

#endif // KORTEXROBOT_HPP

