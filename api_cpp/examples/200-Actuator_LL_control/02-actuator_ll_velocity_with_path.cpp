/*
* KINOVA (R) KORTEX (TM)
*
* Copyright (c) 2018 Kinova inc. All rights reserved.
*
* This software may be modified and distributed
* under the terms of the BSD 3-Clause license.
*
* Refer to the LICENSE file for details.
*
*/

#include <iostream>
#include <string>
#include <vector>
#include <math.h>

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

#if defined(_MSC_VER)
#include <Windows.h>
#else
#include <unistd.h>
#endif
#include <time.h>

namespace k_api = Kinova::Api;

#define PORT 10000
#define PORT_REAL_TIME 10001

#define DURATION 15             // Network timeout (seconds)

float velocity_large = 20.0f;         // Default velocity of the actuator (degrees per seconds)
float velocity_small = 10.0f;         // Default velocity of the actuator (degrees per seconds)
float time_duration = DURATION; // Duration of the example (seconds)

// Waiting time during actions
const auto ACTION_WAITING_TIME = std::chrono::seconds(1);

// Create closure to set finished to true after an END or an ABORT
std::function<void(k_api::Base::ActionNotification)> 
check_for_end_or_abort(bool& finished)
{
    return [&finished](k_api::Base::ActionNotification notification)
    {
        std::cout << "EVENT : " << k_api::Base::ActionEvent_Name(notification.action_event()) << std::endl;

        // The action is finished when we receive a END or ABORT event
        switch(notification.action_event())
        {
        case k_api::Base::ActionEvent::ACTION_ABORT:
        case k_api::Base::ActionEvent::ACTION_END:
            finished = true;
            break;
        default:
            break;
        }
    };
}

/*****************************
 * Example related function *
 *****************************/
int64_t GetTickUs()
{
#if defined(_MSC_VER)
    LARGE_INTEGER start, frequency;

    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);

    return (start.QuadPart * 1000000)/frequency.QuadPart;
#else
    struct timespec start;
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    return (start.tv_sec * 1000000LLU) + (start.tv_nsec / 1000);
#endif
}

/**************************
 * Example core functions *
 **************************/
void example_move_to_home_position(k_api::Base::BaseClient* base)
{
    // Make sure the arm is in Single Level Servoing before executing an Action
    auto servoingMode = k_api::Base::ServoingModeInformation();
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Move arm to ready position
    std::cout << "Moving the arm to a safe position" << std::endl;
    auto action_type = k_api::Base::RequestedActionType();
    action_type.set_action_type(k_api::Base::REACH_JOINT_ANGLES);
    auto action_list = base->ReadAllActions(action_type);
    auto action_handle = k_api::Base::ActionHandle();
    action_handle.set_identifier(0);
    for (auto action : action_list.action_list()) 
    {
        if (action.name() == "Home") 
        {
            action_handle = action.handle();
        }
    }

    if (action_handle.identifier() == 0) 
    {
        std::cout << "Can't reach safe position, exiting" << std::endl;
    } 
    else 
    {
        bool action_finished = false; 
        // Notify of any action topic event
        auto options = k_api::Common::NotificationOptions();
        auto notification_handle = base->OnNotificationActionTopic(
            check_for_end_or_abort(action_finished),
            options
        );

        base->ExecuteActionFromReference(action_handle);

        while(!action_finished)
        { 
            std::this_thread::sleep_for(ACTION_WAITING_TIME);
        }

        base->Unsubscribe(notification_handle);
    }
}

bool example_actuator_low_level_velocity_control(k_api::Base::BaseClient* base, k_api::BaseCyclic::BaseCyclicClient* base_cyclic)
{
    bool return_status = true;

    // Move arm to ready position
    example_move_to_home_position(base);
    
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

    std::vector<float> commands;
    std::vector<vector<float>> target_joint_angles = {{325.551, 59.2881, 294.432, 178.533, 54.9385, 235.541},{305.869,42.0627,251.539,177.517,29.3456,216.948},{54.8453,42.3845,251.666,177.292,29.2051,326.074},{35.5099,59.507,294.507,178.444,54.8327,305.565}};
    std::vector<float> velocity_commands(6, 0.0f);

    float position_tolerance = 0.1;
    float gain = 0.1f;

    auto servoingMode = k_api::Base::ServoingModeInformation();

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;

    int timeout = 0;

    std::cout << "Initializing the arm for velocity low-level control example" << std::endl;
    try
    {
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        int actuator_count = base->GetActuatorCount().count();

        // Initialize each actuator to its current position
        for(int i = 0; i < actuator_count; i++)
        {
            commands.push_back(base_feedback.actuators(i).position());
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // Define the callback function used in Refresh_callback
        auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
        {
            // We are printing the data of the moving actuator just for the example purpose,
            // avoid this in a real-time loop
            std::string serialized_data;
            std::string output_data; 
            serialized_data = serialized_data.append("Joint[0]");
            google::protobuf::util::MessageToJsonString(data.actuators(0), &serialized_data);
            serialized_data = serialized_data.append("\nJoint[1]");
            google::protobuf::util::MessageToJsonString(data.actuators(1), &serialized_data);
            serialized_data = serialized_data.append("\nJoint[2]");
            google::protobuf::util::MessageToJsonString(data.actuators(2), &serialized_data);
            serialized_data = serialized_data.append("\nJoint[3]");
            google::protobuf::util::MessageToJsonString(data.actuators(3), &serialized_data);
            serialized_data = serialized_data.append("\nJoint[4]");
            google::protobuf::util::MessageToJsonString(data.actuators(4), &serialized_data);
            serialized_data = serialized_data.append("\nJoint[5]");
            google::protobuf::util::MessageToJsonString(data.actuators(5), &serialized_data);
            std::cout << serialized_data << std::endl << std::endl;
        };
        bool target_reached = false;
        int stage = 0;
        int atPosition = 0;
        // Real-time loop
        while(timer_count < (time_duration * 1000))
        {
            now = GetTickUs();
            if(now - last > 1000 )
            {
                base_feedback = base_cyclic->RefreshFeedback();
                atPosition = 0;
                    for(int i = 0; i < actuator_count-1; i++)
                        {   
                        float current_pos = base_feedback.actuators(i).position();
                        float target_pos = target_joint_angles[stage][i];
                        float position_error = target_joint_angles[stage][i] - base_feedback.actuators(i).position();

                        float new_position = 0;
                        // velocity_commands[i] = position_error * gain;
                            if (std::abs(position_error) > position_tolerance) {
                                 if (std::abs(position_error) > 10.0f ){
                                    if(i != 0 && i != 1 && i != 2){
                                        //if the motors cant go full 360
                                        if(position_error > 0.0f){
                                            new_position = current_pos + 0.01*60.0f;
                                        }else{
                                            new_position = current_pos - 0.01*60.0f;
                                        }
                                        
                                    }else{
                                        //if motors can go full 360 with roll over protection for base
                                         if(position_error > 0.0f || (i == 0 && (abs(position_error) > 360 +target_joint_angles[stage][i] - base_feedback.actuators(i).position()))){
                                        new_position = current_pos + 0.01*30.0f;
                                         }else{
                                        new_position = current_pos - 0.01*30.0f;
                                         }
                                    }
                                   
                                }else{
                                    //slows down because were in threshold area
                                    if(i != 0 && i != 1 && i != 2){
                                        //speed limit for smaller motors
                                        if(position_error > 0.0f){
                                        new_position = current_pos + 0.01*10.0f;
                                        }else{
                                        new_position = current_pos - 0.01*10.0f;
                                        }
                                    }else{
                                        //speed limits for bigger motor
                                        if(position_error > 0.0f){
                                        new_position = current_pos + 0.01*10.0f;
                                        }else{
                                        new_position = current_pos - 0.01*10.0f;    
                                        }
                                    }
                                }
                                base_command.mutable_actuators(i)->set_position(new_position);
                            }else{
                                atPosition++;
                            } 
                        }
                if(atPosition == 5){
                    stage++;
                    std::cout << "finished stage: " <<stage << std::endl << std::endl;
                    
                }
                if(stage == 4){
                    timeout++;
                }
                try
                {
                    base_cyclic->Refresh_callback(base_command, lambda_fct_callback, 0);
                }
                catch(...)
                {
                    timeout++;
                }
                
                timer_count++;
                last = GetTickUs();
            }
        }
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Kortex error: " << ex.what() << std::endl;
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        std::cout << "Runtime error: " << ex2.what() << std::endl;
        return_status = false;
    }
 
    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}

int main(int argc, char **argv)
{
    auto parsed_args = ParseExampleArguments(argc, argv);

    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    
    auto transport = new k_api::TransportClientTcp();
    auto router = new k_api::RouterClient(transport, error_callback);
    transport->connect(parsed_args.ip_address, PORT);

    auto transport_real_time = new k_api::TransportClientUdp();
    auto router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(parsed_args.ip_address, PORT_REAL_TIME);

    // Set session data connection information
    auto create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(parsed_args.username);
    create_session_info.set_password(parsed_args.password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    auto session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    auto session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    auto base = new k_api::Base::BaseClient(router);
    auto base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    // Example core
    auto isOk = example_actuator_low_level_velocity_control(base, base_cyclic);
    if (!isOk)
    {
        std::cout << "There has been an unexpected error in example_cyclic_armbase() function." << std::endl;
    }

    // Close API session
    session_manager->CloseSession();
    session_manager_real_time->CloseSession();

    // Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();

    // Destroy the API
    delete base;
    delete base_cyclic;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}
