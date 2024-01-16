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
#define PORT_REAL_TIME 10001

namespace k_api = Kinova::Api;

const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

class KortexRobot
{
private:
    std::string ip_address;
    std::string username;
    std::string password;

    k_api::TransportClientTcp* transport;
    k_api::RouterClient* router;
    k_api::TransportClientUdp* transport_real_time;
    k_api::RouterClient* router_real_time;

    k_api::Session::CreateSessionInfo create_session_info;

    k_api::SessionManager* session_manager;
    k_api::SessionManager* session_manager_real_time;

    k_api::Base::BaseClient* base;
    k_api::BaseCyclic::BaseCyclicClient* base_cyclic;
	
	std::function<void(k_api::Base::ActionNotification)>
	check_for_end_or_abort(bool& finished);

public:
    KortexRobot(const std::string& ip_address, const std::string& username, const std::string& password);
	void go_home();
	void connect();
	void disconnect();
    ~KortexRobot();
	
    std::vector<std::vector<float>> read_csv(const std::string &filename);

protected:
	//data
};

#endif // KORTEXROBOT_HPP

