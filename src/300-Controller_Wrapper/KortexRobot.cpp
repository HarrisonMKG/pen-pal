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
    delete session_manager;
    delete session_manager_real_time;
    delete router;
    delete router_real_time;
    delete transport;
    delete transport_real_time;
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

bool KortexRobot::move_cartesian(std::vector<std::vector<float>> waypointsDefinition)
{
    bool return_status = true;
    std::vector<vector<float>> target_joint_angles_IK;

    const float kTheta_x = -180.0;
    const float kTheta_y = 0.0;
    const float kTheta_z = 90.0;

	int indx = 0;
   
    k_api::BaseCyclic::Feedback base_feedback;
    k_api::BaseCyclic::Command  base_command;

	for (auto& point : waypointsDefinition)
	{
		point.insert(point.end(),{0,kTheta_x,kTheta_y,kTheta_z});
		if(point.size()>3){
			point.erase(point.begin());// remove seconds value for IR sensor
		}
	}

	//Print Modified Vector
	cout << "Modified Vector" << endl;
	for(auto point : waypointsDefinition)
	{
		cout << endl;
		for(auto element : point)
		{
			cout<< element << " ";
		}
	}


    target_joint_angles_IK = convert_points_to_angles(waypointsDefinition);
    
    std::vector<float> commands;
    std::vector<vector<float>> target_joint_angles = {
                                                        {325.551, 59.2881, 294.432, 178.533, 54.9385, 235.541},
                                                        {305.869,42.0627,251.539,177.517,29.3456,216.948},
                                                        {54.8453,42.3845,251.666,177.292,29.2051,326.074},
                                                        {35.5099,59.507,294.507,178.444,54.8327,305.565},
                                                        {325.551, 59.2881, 294.432, 178.533, 54.9385, 235.541}
                                                    };


    for (int i = 0; i < target_joint_angles_IK.size(); i++)
    {
        std::cout << "IK vs Premade generated Angles:" << indx << std::endl;
        for (auto currAngle: target_joint_angles_IK[i]){
            std::cout << currAngle << ", ";
        }
        std::cout << std::endl;
        for (auto currAngle: target_joint_angles[i]){
            std::cout << currAngle << ", ";
        }
        std::cout << std::endl;
    }
    // for (auto waypointAngles: target_joint_angles)
    // {
    //     std::cout << "Premade Angles:" << indx << std::endl;
    //     for (auto currAngle: waypointAngles){
    //         std::cout << currAngle << ", ";
    //     }
    //     std::cout << std::endl;
    // }
    std::this_thread::sleep_for(std::chrono::milliseconds(5000));

    // return true;

     
    std::vector<float> velocity_commands(6, 0.0f);

    float position_tolerance = 0.1;
    // float gain = 0.1f;

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
            // serialized_data = serialized_data.append("Joint[0]");
            // google::protobuf::util::MessageToJsonString(data.actuators(0), &serialized_data);
            // serialized_data = serialized_data.append("\nJoint[1]");
            // google::protobuf::util::MessageToJsonString(data.actuators(1), &serialized_data);
            // serialized_data = serialized_data.append("\nJoint[2]");
            // google::protobuf::util::MessageToJsonString(data.actuators(2), &serialized_data);
            // serialized_data = serialized_data.append("\nJoint[3]");
            // google::protobuf::util::MessageToJsonString(data.actuators(3), &serialized_data);
            // serialized_data = serialized_data.append("\nJoint[4]");
            // google::protobuf::util::MessageToJsonString(data.actuators(4), &serialized_data);
            // serialized_data = serialized_data.append("\nJoint[5]");
            // google::protobuf::util::MessageToJsonString(data.actuators(5), &serialized_data);
            // std::cout << serialized_data << std::endl << std::endl;
        };
        // bool target_reached = false;
        int stage = 0;
        int atPosition = 0;
        int segments = waypointsDefinition.size();
        std::cout << segments << std::endl << std::endl;
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

                    // float position_error = target_joint_angles[stage][i] - base_feedback.actuators(i).position();
                    float inverse_current_position = current_pos - 180; //Ex: 20 degrees
                    if (inverse_current_position < 0)
                    {
                        inverse_current_position += 360.0; //Ex: 200 degrees
                    }
                    
                    float position_error = target_joint_angles_IK[stage][i] - base_feedback.actuators(i).position();

                    float new_position = 0;
                        if (std::abs(position_error) > position_tolerance) {
                            if (std::abs(position_error) > 10.0f){
                                // TODO: Check if these are thecorrect joints, or update the comment below
                                // Joint 0 CAN go 360
                                if(i != 0 && i != 1 && i != 2){
                                    //if the motors cant go full 360
                                    if(position_error > 0.0f){
                                        if (target_joint_angles_IK[stage][i] > 120 && i == 4){
                                            new_position = current_pos - 0.01*60.0f;
                                        }else{
                                        new_position = current_pos + 0.01*60.0f;}
                                    }else{
                                        new_position = current_pos - 0.01*60.0f;
                                    }
                                    
                                }else{
                                    //if motors can go full 360 with roll over protection for base
                                    if (i == 0){
                                        // Find what side the arm is on
                                        if (current_pos > 180){
                                            if ((target_joint_angles_IK[stage][i] > inverse_current_position && target_joint_angles_IK[stage][i] < current_pos)){
                                                // Turn left
                                                new_position = current_pos - 0.01*30.0f;
                                                std::cout << "(LEFT-N) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                                                
                                            } else {
                                                // Turn Right
                                                std::cout << "(RIGHT-N) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                                                new_position = current_pos + 0.01*30.0f;
                                            }
                                        } else {
                                            if (target_joint_angles_IK[stage][i] < inverse_current_position && target_joint_angles_IK[stage][i] > current_pos){
                                                // Turn left
                                                std::cout << "(RIGHT-I) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                                                new_position = current_pos + 0.01*30.0f;
                                            } else {
                                                // Turn Right
                                                std::cout << "(LEFT-I) Cur: " <<current_pos << " , Target: "<< target_joint_angles_IK[stage][i] << std::endl << std::endl;
                                                new_position = current_pos - 0.01*30.0f;
                                            }
                                        }
                                    }else {
                                        if(position_error > 0.0f){
                                                                                    
                                            // if((i == 0 && (abs(position_error) > 360 +target_joint_angles[stage][i] - base_feedback.actuators(i).position()))){
                                                // new_position = current_pos - 0.01*30.0f;
                                                // std::cout << "roll over left " <<stage << std::endl << std::endl;
                                            // }else{
                                            new_position = current_pos + 0.01*30.0f;
                                            //     if(i == 0){
                                            //         std::cout << "turning right" <<stage << std::endl << std::endl;
                                            //     }
                            
                                            // }
                                        }else{
                                            // if(i == 0 && (abs(position_error) > 360 + target_joint_angles_IK[stage][i] - base_feedback.actuators(i).position())){
                                            // new_position = current_pos + 0.01*30.0f;
                                            // std::cout << "roll over right: " <<stage << std::endl << std::endl;
                                            // }else{
                                            new_position = current_pos - 0.01*30.0f;
                                                // if(i == 0){
                                                //     std::cout << "turning left" <<stage << std::endl << std::endl;
                                                // }
                                            // }
                                        }
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
                if(stage == segments){
                    break;
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

std::vector<vector<float>>
KortexRobot::convert_points_to_angles(std::vector<vector<float>> target_points)
{   
    std::vector<vector<float>> final_joint_angles;
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
        
        // Fill the IKData Object with the cartesian coordinates that need to be converted
        input_IkData.mutable_cartesian_pose()->set_x(current_target[0]);
        input_IkData.mutable_cartesian_pose()->set_y(current_target[1]);
        input_IkData.mutable_cartesian_pose()->set_z(current_target[2]);
        input_IkData.mutable_cartesian_pose()->set_theta_x(-180.0);
        input_IkData.mutable_cartesian_pose()->set_theta_y(0.0);
        input_IkData.mutable_cartesian_pose()->set_theta_z(90.0);

        // Fill the IKData Object with the guessed joint angles
        Kinova::Api::Base::JointAngle* jAngle; 
        for(auto joint_angle : input_joint_angles.joint_angles())
        {
            jAngle = input_IkData.mutable_guess()->add_joint_angles();
            // '- 1' to generate an actual "guess" for current joint angles
            if (joint_angle.value()+1 < 360 && joint_angle.value() -1 > 0 ){
                jAngle->set_value(joint_angle.value());
            }else{
                 jAngle->set_value(joint_angle.value());
            }
            
        }


        // Computing Inverse Kinematics (cartesian -> Angle convert) from arm's current pose and joint angles guess
        k_api::Base::JointAngles computed_joint_angles;
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

        // std::cout << "Joint ID : Joint Angle" << std::endl;
        int joint_identifier = 0;
        std::vector<float> temp_joints(6, 0.0f);
        // Kinova::Api::Base::JointAngle* hAngle; 

        for (auto joint_angle : computed_joint_angles.joint_angles()) 
        {
            
            // float temp_value = joint_angle.value();
            temp_joints[joint_identifier] = joint_angle.value();
            // std::cout << joint_identifier << " : " << joint_angle.value() << std::endl;

            joint_identifier++;
        }
        final_joint_angles.push_back(temp_joints);
    }
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
    
    return final_joint_angles;
}
