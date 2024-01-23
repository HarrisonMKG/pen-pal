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
#include "KortexRobot.hpp"

namespace k_api = Kinova::Api;

KortexRobot::KortexRobot(const std::string& ip_address, const std::string& username, const std::string& password)
{
	KortexRobot::ip_address = ip_address;
	KortexRobot::username = username;
	KortexRobot::password = password;
	
	KortexRobot::connect();
}

void KortexRobot::connect()
{
    // Create API objects
    auto error_callback = [](k_api::KError err){ cout << "_________ callback error _________" << err.toString(); };
    transport = new k_api::TransportClientTcp();
    router = new k_api::RouterClient(transport, error_callback);
    transport->connect(ip_address, PORT);

    transport_real_time = new k_api::TransportClientUdp();
    router_real_time = new k_api::RouterClient(transport_real_time, error_callback);
    transport_real_time->connect(ip_address, PORT_REAL_TIME);
    // Set session data connection information
    create_session_info = k_api::Session::CreateSessionInfo();
    create_session_info.set_username(username);
    create_session_info.set_password(password);
    create_session_info.set_session_inactivity_timeout(60000);   // (milliseconds)
    create_session_info.set_connection_inactivity_timeout(2000); // (milliseconds)

    // Session manager service wrapper
    std::cout << "Creating sessions for communication" << std::endl;
    session_manager = new k_api::SessionManager(router);
    session_manager->CreateSession(create_session_info);
    session_manager_real_time = new k_api::SessionManager(router_real_time);
    session_manager_real_time->CreateSession(create_session_info);
    std::cout << "Sessions created" << std::endl;

    // Create services
    base = new k_api::Base::BaseClient(router);
	base_cyclic = new k_api::BaseCyclic::BaseCyclicClient(router_real_time);

    control_config = new k_api::ControlConfig::ControlConfigClient(router);
    actuator_config = new k_api::ActuatorConfig::ActuatorConfigClient(router);
    device_config = new k_api::DeviceConfig::DeviceConfigClient(router);
    actuator_count = base->GetActuatorCount().count();

    //resets all current errors
    device_config -> ClearAllSafetyStatus();
    std::cout << "Cleared all errors on Robot" << std::endl;
}

void KortexRobot::disconnect()
{
	//Close API session
	session_manager->CloseSession();
	session_manager_real_time->CloseSession();

	// Deactivate the router and cleanly disconnect from the transport object
    router->SetActivationStatus(false);
    transport->disconnect();
    router_real_time->SetActivationStatus(false);
    transport_real_time->disconnect();
}

KortexRobot::~KortexRobot()
{
	KortexRobot::disconnect();

    delete base;
    delete base_cyclic;
    delete control_config;
    delete actuator_config;
    delete device_config;
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
}

/*
std::vector<std::vector<float>> KortexRobot::csv_to_cartesian_waypoints(std::vector<std::vector<float>> csv_waypoints, 
                                                                        float kTheta_x, float kTheta_y, float kTheta_z) 
{
    for (auto& point : csv_waypoints)
	{
		std::cout<< "point size :" << point.size() << std::endl;
		if(point.size()>3){
			point.erase(point.begin());// remove seconds value for IR sensor
		}
		point.insert(point.end(),{0,kTheta_x,kTheta_y,kTheta_z});
	}
    return csv_waypoints;
}
*/

void KortexRobot::writing_mode()
{
    float kTheta_x = 0.0;
    float kTheta_y = -180.0;
    float kTheta_z = 90.0;

	auto current_pose = base->GetMeasuredCartesianPose();
	vector<vector<float>> current_cartiesian = {{current_pose.x(),current_pose.y(),current_pose.z()}}; 
	KortexRobot::move_cartesian(current_cartiesian,kTheta_x,kTheta_y,kTheta_z);
}


void KortexRobot::go_home()
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

std::function<void(k_api::Base::ActionNotification)> 
KortexRobot::check_for_end_or_abort(bool& finished)
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

int64_t KortexRobot::GetTickUs()
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

void KortexRobot::set_actuator_control_mode(int mode_control)
{
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();

    if (mode_control == 1) {
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);
    }else {
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);

    }
    // control_mode_message.set_control_mode(mode_control);
    actuator_config->SetControlMode(control_mode_message,1);
    actuator_config->SetControlMode(control_mode_message,2);
    actuator_config->SetControlMode(control_mode_message,3);
    actuator_config->SetControlMode(control_mode_message,4);
    actuator_config->SetControlMode(control_mode_message,5);
    actuator_config->SetControlMode(control_mode_message,6);
}


std::vector<std::vector<float>> KortexRobot::read_csv(const std::string& filename) {
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

std::vector<std::vector<float>> KortexRobot::convert_csv_to_cart_wp(std::vector<std::vector<float>> csv_points, float kTheta_x,
                                                                    float kTheta_y, float kTheta_z) {
    bool verbose = false; 
    // Assuming the format of the csv_file will be in (time, x, y, z) for each line
    for (auto& point : csv_points)
	{
		if(point.size()>3){
			point.erase(point.begin());// remove seconds value for IR sensor
		}
		point.insert(point.end(),{0,kTheta_x,kTheta_y,kTheta_z});
	}
    
    if (verbose) {
        cout << "Modified Vector" << endl;
        for(auto point : csv_points)
        {
            cout << endl;
            for(auto element : point)
            {
                cout<< element << " ";
            }
        }    
    }
    return csv_points;
}

bool KortexRobot::move_cartesian(std::vector<std::vector<float>> waypointsDefinition, float kTheta_x, 
		float kTheta_y, float kTheta_z)
{
    // Define the callback function used in Refresh_callback
    auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
    {
        // TODO: Remove this in a real-time loop
		/*
        std::string serialized_data;
        std::string output_data; 
        serialized_data.append("Joint[0]");
        google::protobuf::util::MessageToJsonString(data.actuators(0), &serialized_data);
        serialized_data.append("\nJoint[1]");
        google::protobuf::util::MessageToJsonString(data.actuators(1), &serialized_data);
        serialized_data.append("\nJoint[2]");
        google::protobuf::util::MessageToJsonString(data.actuators(2), &serialized_data);
        serialized_data.append("\nJoint[3]");
        google::protobuf::util::MessageToJsonString(data.actuators(3), &serialized_data);
        serialized_data.append("\nJoint[4]");
        google::protobuf::util::MessageToJsonString(data.actuators(4), &serialized_data);
        serialized_data.append("\nJoint[5]");
        google::protobuf::util::MessageToJsonString(data.actuators(5), &serialized_data);
        std::cout << serialized_data << std::endl << std::endl;
    */
		};

    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;
    
    auto servoingMode = k_api::Base::ServoingModeInformation();
   
    std::vector<vector<float>> target_joint_angles_IK;
    std::vector<vector<float>> target_waypoints;

    target_waypoints = convert_csv_to_cart_wp(waypointsDefinition, kTheta_x, kTheta_y, kTheta_z);
    target_joint_angles_IK = convert_points_to_angles(target_waypoints);

    // Delay to allow for us to confirm angles being given to robot
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    float position_tolerance = 0.1f;
    float closer_range_limit = 10.f;
    float larger_velocity = 30.0f;
    float smaller_velocity = 10.0f;
    bool return_status = true;

    int timer_count = 0;
    int64_t now = 0;
    int64_t last = 0;
    int timeout = 0;

    //mylogger.Log("Initializing the arm for velocity low-level control example", INFO);
    try
    {   
        // Set the base in low-level servoing mode
        servoingMode.set_servoing_mode(k_api::Base::ServoingMode::LOW_LEVEL_SERVOING);
        base->SetServoingMode(servoingMode);
        base_feedback = base_cyclic->RefreshFeedback();

        // Initialize each actuator to its current position
        for(int i = 0; i < actuator_count; i++)
        {
            base_command.add_actuators()->set_position(base_feedback.actuators(i).position());
        }

        // TODO:    Confirm how we are flagging whether the actuator reached the current angle 
        //          or if it reached the last waypoint (what variables are used)
        //      -   How to ensure just bc one angle reached its target but the others, what to do
        int stage = 0;
        int atPosition = 0;
        int num_of_targets = target_waypoints.size();
        // Real-time loop
        while(timer_count < (time_duration * 1000))
        {
            now = GetTickUs();
            if(now - last > 1000 )
            {
                base_feedback = base_cyclic->RefreshFeedback();
                atPosition = 0;
                // PID LOOPS WOULD GO WITHIN HERE
                for(int i = 0; i < actuator_count-1; i++)
                    {   
                    float current_pos = base_feedback.actuators(i).position();
                    float target_pos = target_joint_angles_IK[stage][i];
                    float position_error = target_pos - current_pos;
                    float update_velocity;
                    std::vector<float> new_position_velocity(2,0.0f);

                    // float inverse_current_position = current_pos - 180; //Ex: 20 degrees
                    // if (inverse_current_position < 0)
                    // {
                    //     inverse_current_position += 360.0; //Ex: 200 degrees
                    // }
                    
                        if (std::abs(position_error) > position_tolerance) {
                            if (std::abs(position_error) > closer_range_limit) {
                                update_velocity = larger_velocity;
                            } else {
                                update_velocity = smaller_velocity;
                            }

                            if (i > 2) {
                                new_position_velocity = pid_small_motors(target_pos, current_pos, update_velocity);
                            } else if (i == 0) {
                                new_position_velocity = pid_motor_0(target_pos, current_pos, update_velocity);
                            } else {
                                new_position_velocity = pid_motor_1_2(target_pos, current_pos, update_velocity);
                            }

                            // if (std::abs(position_error) > 10.0f){
                            //     // TODO: Check if these are thecorrect joints, or update the comment below
                            //     // Joint 0 CAN go 360
                            //     if(i != 0 && i != 1 && i != 2){
                            //         //if the motors cant go full 360
                            //         if(position_error > 0.0f){
                            //             if (target_joint_angles_IK[stage][i] > 120 && i == 4){
                            //                 new_position = current_pos - 0.01*60.0f;
                            //             }else{
                            //             new_position = current_pos + 0.01*60.0f;}
                            //         }else{
                            //             new_position = current_pos - 0.01*60.0f;
                            //         }
                                    
                            //     }else{
                            //         //if motors can go full 360 with roll over protection for base
                            //         if (i == 0){
                            //             // Find what side the arm is on
                            //             if (current_pos > 180){
                            //                 if ((target_joint_angles_IK[stage][i] > inverse_current_position && target_joint_angles_IK[stage][i] < current_pos)){
                            //                     // Turn left
                            //                     new_position = current_pos - 0.01*30.0f;
                            //                     std::cout << "(LEFT-N) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                                                
                            //                 } else {
                            //                     // Turn Right
                            //                     std::cout << "(RIGHT-N) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                            //                     new_position = current_pos + 0.01*30.0f;
                            //                 }
                            //             } else {
                            //                 if (target_joint_angles_IK[stage][i] < inverse_current_position && target_joint_angles_IK[stage][i] > current_pos){
                            //                     // Turn left
                            //                     std::cout << "(RIGHT-I) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                            //                     new_position = current_pos + 0.01*30.0f;
                            //                 } else {
                            //                     // Turn Right
                            //                     std::cout << "(LEFT-I) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                            //                     new_position = current_pos - 0.01*30.0f;
                            //                 }
                            //             }
                            //         }else {
                            //             if(position_error > 0.0f){
                                                                                    
                            //                 // if((i == 0 && (abs(position_error) > 360 +target_joint_angles[stage][i] - base_feedback.actuators(i).position()))){
                            //                     // new_position = current_pos - 0.01*30.0f;
                            //                     // std::cout << "roll over left " <<stage << std::endl << std::endl;
                            //                 // }else{
                            //                 new_position = current_pos + 0.01*30.0f;
                            //                 //     if(i == 0){
                            //                 //         std::cout << "turning right" <<stage << std::endl << std::endl;
                            //                 //     }
                            
                            //                 // }
                            //             }else{
                            //                 // if(i == 0 && (abs(position_error) > 360 + target_joint_angles_IK[stage][i] - base_feedback.actuators(i).position())){
                            //                 // new_position = current_pos + 0.01*30.0f;
                            //                 // std::cout << "roll over right: " <<stage << std::endl << std::endl;
                            //                 // }else{
                            //                 new_position = current_pos - 0.01*30.0f;
                            //                     // if(i == 0){
                            //                     //     std::cout << "turning left" <<stage << std::endl << std::endl;
                            //                     // }
                            //                 // }
                            //             }
                            //         }
                            //     }
                            // }else{
                            //     //slows down because were in threshold area
                            //     if(i != 0 && i != 1 && i != 2){
                            //         //speed limit for smaller motors
                            //         if(position_error > 0.0f){
                            //         new_position = current_pos + 0.01*10.0f;
                            //         }else{
                            //         new_position = current_pos - 0.01*10.0f;
                            //         }
                            //     }else{
                            //         //speed limits for bigger motor
                            //         if(position_error > 0.0f){
                            //         new_position = current_pos + 0.01*10.0f;
                            //         }else{
                            //         new_position = current_pos - 0.01*10.0f;    
                            //         }
                            //     }
                            // }
                            // base_command.mutable_actuators(i)->set_position(new_position);
                        

                        // TODO: Confirm what we are doing (Position or velocity or both?)
                            base_command.mutable_actuators(i)->set_position(new_position_velocity[0]);
                            base_command.mutable_actuators(i)->set_velocity(new_position_velocity[1]);
                        }else{
                            base_command.mutable_actuators(i)->set_position(current_pos);
                            base_command.mutable_actuators(i)->set_velocity(0);
                            atPosition++;
                        } 
                    }
                // PID LOOPS ABOVE
                

                if(atPosition == 5){
                    stage++;
                    std::cout << "finished stage: " <<stage << std::endl << std::endl;
                }
                if(stage == num_of_targets){
                    break;
                }

                // 
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
        //mylogger.Log("Kortex error: " + std::string(ex.what()), ERR);
        return_status = false;
    }
    catch (std::runtime_error& ex2)
    {
        //mylogger.Log("Runtime error: " + std::string(ex2.what()), ERR);
        return_status = false;
    }
 
    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    return return_status;
}


void KortexRobot::printException(k_api::KDetailedException& ex)
{
    // You can print the error informations and error codes
    auto error_info = ex.getErrorInfo().getError();
    std::cout << "KDetailedoption detected what:  " << ex.what() << std::endl;
    
    std::cout << "KError error_code: " << error_info.error_code() << std::endl;
    std::cout << "KError sub_code: " << error_info.error_sub_code() << std::endl;
    std::cout << "KError sub_string: " << error_info.error_sub_string() << std::endl;

    // Error codes by themselves are not very verbose if you don't see their corresponding enum value
    // You can use google::protobuf helpers to get the string enum element for every error code and sub-code 
    std::cout << "Error code string equivalent: " << k_api::ErrorCodes_Name(k_api::ErrorCodes(error_info.error_code())) << std::endl;
    std::cout << "Error sub-code string equivalent: " << k_api::SubErrorCodes_Name(k_api::SubErrorCodes(error_info.error_sub_code())) << std::endl;
}

std::vector<std::vector<float>> KortexRobot::convert_points_to_angles(std::vector<std::vector<float>> target_points)
{   
    // Function take an array of target points in format (x,y,z,theta_x,theta_y,theta_z)
    // Feeds it into the Kinova IK function that will spit out the target joint angles 
    // for each actuator at each waypoint. Will also ensure all joint angles are positive and within 360.0
    
    bool verbose = false;
    std::vector<std::vector<float>> final_joint_angles;
    k_api::Base::JointAngles input_joint_angles;

    try
    {
        input_joint_angles = base->GetMeasuredJointAngles();
    }
    catch (k_api::KDetailedException& ex)
    {
        std::cout << "Unable to get current robot pose" << std::endl;
        printException(ex);
        return final_joint_angles;
    }

    for (std::vector<float> current_target : target_points) 
    {
        // Object containing cartesian coordinates and Angle Guess
        k_api::Base::IKData input_IkData; 
        k_api::Base::JointAngle* jAngle; 
        k_api::Base::JointAngles computed_joint_angles;
        
        // Fill the IKData Object with the cartesian coordinates that need to be converted
		
        input_IkData.mutable_cartesian_pose()->set_x(current_target[0]);
        input_IkData.mutable_cartesian_pose()->set_y(current_target[1]);
        input_IkData.mutable_cartesian_pose()->set_z(current_target[2]);

        input_IkData.mutable_cartesian_pose()->set_theta_x(current_target[4]);
        input_IkData.mutable_cartesian_pose()->set_theta_y(current_target[5]);
        input_IkData.mutable_cartesian_pose()->set_theta_z(current_target[6]);

        // Fill the IKData Object with the guessed joint angles
        for(auto joint_angle : input_joint_angles.joint_angles())
        {
            jAngle = input_IkData.mutable_guess()->add_joint_angles();
            if (joint_angle.value()+1 < 360 && joint_angle.value() -1 > 0 ){
                jAngle->set_value(joint_angle.value());
            }else{
                 jAngle->set_value(joint_angle.value());
            }
        }
        // Computing Inverse Kinematics (cartesian -> Angle convert) from arm's current pose and joint angles guess
        try
        {
            computed_joint_angles = base->ComputeInverseKinematics(input_IkData); 
        }
        catch (k_api::KDetailedException& ex)
        {
            std::cout << "Unable to compute inverse kinematics" << std::endl;
            printException(ex);
            return final_joint_angles;
        }

        int joint_identifier = 0;
        std::vector<float> temp_joints(6, 0.0f);

        for (auto joint_angle : computed_joint_angles.joint_angles()) 
        {
            temp_joints[joint_identifier] = joint_angle.value();
            joint_identifier++;
        }
        final_joint_angles.push_back(temp_joints);
    }
    // Ensure all the angles provided are mod360.0 and positive
    int num_points = final_joint_angles.size();
    for (int section = 0; section != num_points ;section++)
    {
        for (int rotator = 0; rotator != 6; rotator++){
            final_joint_angles[section][rotator] = fmod(final_joint_angles[section][rotator], 360.0);
            if (final_joint_angles[section][rotator] < 0.0) {
                final_joint_angles[section][rotator] = final_joint_angles[section][rotator] + 360;
            }
        }
    }
    if (verbose){
        for (int i = 0; i < final_joint_angles.size(); i++)
        {
            std::cout << "IK vs Premade generated Angles:" << i << std::endl;
            for (auto currAngle: final_joint_angles[i]){
                std::cout << currAngle << ", ";
            }
            std::cout << std::endl;
        }
    }
    
    return final_joint_angles;
}

// KortexRobot::get_lambda_feedback_callback(){
//     auto lambda_fct_callback = [](const Kinova::Api::Error &err, const k_api::BaseCyclic::Feedback data)
//         {
//             // We are printing the data of the moving actuator just for the example purpose,
//             // avoid this in a real-time loop
//             std::string serialized_data;
//             std::string output_data; 
//             // serialized_data = serialized_data.append("Joint[0]");
//             // google::protobuf::util::MessageToJsonString(data.actuators(0), &serialized_data);
//             // serialized_data = serialized_data.append("\nJoint[1]");
//             // google::protobuf::util::MessageToJsonString(data.actuators(1), &serialized_data);
//             // serialized_data = serialized_data.append("\nJoint[2]");
//             // google::protobuf::util::MessageToJsonString(data.actuators(2), &serialized_data);
//             // serialized_data = serialized_data.append("\nJoint[3]");
//             // google::protobuf::util::MessageToJsonString(data.actuators(3), &serialized_data);
//             // serialized_data = serialized_data.append("\nJoint[4]");
//             // google::protobuf::util::MessageToJsonString(data.actuators(4), &serialized_data);
//             // serialized_data = serialized_data.append("\nJoint[5]");
//             // google::protobuf::util::MessageToJsonString(data.actuators(5), &serialized_data);
//             // std::cout << serialized_data << std::endl << std::endl;
//         };
//     return lambda_fct_callback;
// }


// TODO: Call this function to print all the stuff had in the last example
void KortexRobot::output_arm_limits_and_mode()
{
    // k_api::ControlConfig::ControlModeInformation control_mode;
    // k_api::ActuatorConfig::ControlModeInformation past_mode;
    // k_api::ControlConfig::KinematicLimits hard_limits;
    // k_api::ControlConfig::KinematicLimits soft_limits;
    // k_api::ControlConfig::KinematicLimitsList soft_angle_limits;
    // k_api::ControlConfig::TwistLinearSoftLimit set_angle_limits;

    auto act_mode = k_api::ActuatorConfig::CommandModeInformation();
    auto control_mode = control_config->GetControlMode();
    auto hard_limits = control_config->GetKinematicHardLimits();
    auto soft_limits = control_config->GetKinematicSoftLimits(control_mode);
    auto soft_angle_limits = control_config->GetAllKinematicSoftLimits();

    std::cout<< control_mode.control_mode()<< std::endl;
    
    std::cout<< "control mode is: "<< control_mode.control_mode() << std::endl;
    std::cout<< "actuator control mode is: "<< act_mode.command_mode() << std::endl;
    
    std::cout<< "the hard limits are: "<< hard_limits.twist_angular() << std::endl;
    std::cout<< "the soft limits are: "<< soft_limits.twist_angular() << std::endl;
    // soft_angle_limits = control_config->GetAllKinematicSoftLimits();

    for (int i = 0; i < 12; i++){
        std::cout<< "the soft limit angle is : "<< soft_angle_limits.kinematic_limits_list(i).twist_angular() << std::endl;
    }
}


// ===============================================================================================
// ------------------------------------- PID LOOP FUNCTIONS -------------------------------------
// ===============================================================================================

std::vector<float> KortexRobot::pid_small_motors(float target_pos, float current_pos, float base_velocity)
{
    const float position_adjust = 0.02f * base_velocity;
    float new_position;
    float velocity;

    if((target_pos - current_pos) > 0.0f){
        if (target_pos > 120){ //TODO: Why 120? If its a joint limit, will this condition be there for other actuators?
            new_position = current_pos - position_adjust;
            velocity = -base_velocity;
        }else{
            new_position = current_pos + position_adjust;
            velocity = base_velocity;
        }
    }else{
        new_position = current_pos - position_adjust;
        velocity = -base_velocity;
    }
    return {new_position, velocity};
}

std::vector<float> KortexRobot::pid_motor_0(float target_pos, float current_pos, float base_velocity)
{
    const float position_adjust = 0.01f * base_velocity;
    float inverse_current_position = current_pos - 180; 
    float new_position;
    float velocity;

    if (inverse_current_position < 0)
    {
        inverse_current_position += 360.0;
    }
    // Find what side the arm is on
    if (current_pos > 180){
        if ((target_pos > inverse_current_position && target_pos < current_pos)){
            // Turn left
            new_position = current_pos - position_adjust;
            velocity = -base_velocity;
            
        } else {
            // Turn Right
            new_position = current_pos + position_adjust;
            velocity = base_velocity;
        }
    } else {
        if (target_pos < inverse_current_position && target_pos > current_pos){
            // Turn left
            new_position = current_pos + position_adjust;
            velocity = base_velocity;
        } else {
            // Turn Right
            new_position = current_pos - position_adjust;
            velocity = -base_velocity;
        }
    }
    return {new_position, velocity};
}

std::vector<float> KortexRobot::pid_motor_1_2(float target_pos, float current_pos, float base_velocity)
{
    const float position_adjust = 0.01f * base_velocity;
    float new_position;
    float velocity;

     if((target_pos - current_pos) > 0.0f){
        new_position = current_pos + position_adjust;
        velocity = base_velocity;
    }else{
        new_position = current_pos - position_adjust;
        velocity = -base_velocity;
    }
    return {new_position, velocity};
}


