// KortexRobot.hpp

#ifndef KORTEXROBOT_HPP
#define KORTEXROBOT_HPP

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <fstream>
#include <numeric>
#include <sstream>

#include <KDetailedException.h>

#include <BaseClientRpc.h>
#include <BaseCyclicClientRpc.h>
#include <ControlConfigClientRpc.h>
#include <ActuatorConfigClientRpc.h>
#include <DeviceConfigClientRpc.h>
#include <SessionClientRpc.h>
#include <SessionManager.h>

#include <RouterClient.h>
#include <TransportClientTcp.h>
#include <TransportClientUdp.h>

#include <google/protobuf/util/json_util.h>

#include "utilities.h"
#include "pid.cpp"

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

#define PORT 10000
#define PORT_REAL_TIME 10001
#define DURATION 15             // Network timeout (seconds)

namespace k_api = Kinova::Api;

const auto ACTION_WAITING_TIME = std::chrono::seconds(1);
float time_duration = DURATION; // Duration of the example (seconds)

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

    
    k_api::ActuatorConfig::ActuatorConfigClient* actuator_config;
    k_api::DeviceConfig::DeviceConfigClient* device_config;
	k_api::ControlConfig::ControlConfigClient* control_config;


	std::function<void(k_api::Base::ActionNotification)>
	check_for_end_or_abort(bool& finished);
	void printException(k_api::KDetailedException& ex);

	int64_t GetTickUs();

public:
    KortexRobot(const std::string& ip_address, const std::string& username, const std::string& password);
	void go_home();
	void connect();
	void disconnect();
    ~KortexRobot();
    void set_actuator_control_mode(int mode_control);
	void writing_mode();
	bool move_cartesian(std::vector<std::vector<float>> waypointsDefinition,
					float kTheta_x = 0.0f, float kTheta_y = -180.0f, float kTheta_z = 90.0f);

	std::vector<std::vector<float>> convert_points_to_angles(std::vector<vector<float>> target_points);
	
    std::vector<std::vector<float>> read_csv(const std::string &filename);
    std::vector<std::vector<float>> convert_csv_to_cart_wp(std::vector<std::vector<float>> csv_points, 
                                                                        float kTheta_x, float kTheta_y, 
                                                                        float kTheta_z);
    // get_lambda_feedback_callback();

    void output_arm_limits_and_mode();


    // PID LOOPS
    std::vector<float> pid_small_motors(float target_pos, float current_pos, float base_velocity,int motor);
    std::vector<float> pid_motor_0(float target_pos, float current_pos, float base_velocity);
    std::vector<float> pid_motor_1_2(float target_pos, float current_pos, float base_velocity);
	
	void init_pids();
    int actuator_count;
    vector<Pid_Loop> pids;


protected:
	//data
    

};

#endif // KORTEXROBOT_HPP

