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
	init_pids();
}

void KortexRobot::plot(vector<vector<float>>data)
{
	start_plot();
  for(auto points: data)
  {
    KortexRobot::plot_data << points[0] << " " << points[1] << endl;
    fprintf(gnu_plot, "plot 'realtime_data.txt' with lines\n");
    fflush(gnu_plot);
  }
  
}

int KortexRobot::start_plot()
{
  FILE *gnuplotPipe = popen("gnuplot -persist", "w");
  KortexRobot::gnu_plot = gnuplotPipe;
  if (!gnu_plot) {
    std::cerr << "Error: Gnuplot not found." << std::endl;
    return 1;
  }

  // Open a file for writing
  KortexRobot::plot_data.open("realtime_data.txt");
  if (!plot_data.is_open()) {
    std::cerr << "Error: Unable to open data file." << std::endl;
    return 1;
  }
}

void KortexRobot::init_pids()
{
    actuator_count = base->GetActuatorCount().count();
	  vector<vector<float>> pid_inputs = {{0.05, 0.00002, 0.0005},//tuned
    {0.25, 0.25, 0.025}, //tuned
    {0.085, 0.075, 0.015}, //tuned
    {0.2, 0.0, 0.0}, //tuned
    {0.15, 0.05, 0.005}, //tuned
    {1, 0.0, 0.0} // tuned
	}; // This will turn into reading from a json later
    // Probably should include angle limits, max & min control sig limits
    // possibly position thresholds for all angles?

	for(int i=0; i<actuator_count-1; i++)
	{
		float k_p = pid_inputs[i][0];
		float k_i = pid_inputs[i][1];
		float k_d = pid_inputs[i][2];

		Pid_Loop pid(k_p,k_i,k_d);
		pids.push_back(pid);
	}
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
	base->ClearFaults();
    std::cout << "Cleared all errors on Robot" << std::endl;

    altered_origin = {0.25, 0.0, 0.3};
    bais_vector = {0.0, 0.0, 0.0};  
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
    set_actuator_control_mode(0);
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

void KortexRobot::find_paper()
{
	//Should have an if statement for config mode where you can change this hardcoded value in real time using the sequence of:
	//1) Pause the system
	//2) Wait till user manually moves arm to writing position
	//3) User presses "enter" in cli to confirm they found the paper
	//4) Continue
	KortexRobot::move_cartesian({surface_cords});
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

void KortexRobot::set_actuator_control_mode(int mode_control, int actuator_indx)
{
    auto control_mode_message = k_api::ActuatorConfig::ControlModeInformation();
    std::cout << "changed the mode to: ";

    if (mode_control == 1) {
        std::cout << "VELOCITY" << std::endl;
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::VELOCITY);
    }else {
        std::cout << "POSITION" << std::endl;
        control_mode_message.set_control_mode(k_api::ActuatorConfig::ControlMode::POSITION);
    }
    if (actuator_indx == -1) {
        actuator_config->SetControlMode(control_mode_message,1); 
        actuator_config->SetControlMode(control_mode_message,2);
        actuator_config->SetControlMode(control_mode_message,3);
        actuator_config->SetControlMode(control_mode_message,4);
        actuator_config->SetControlMode(control_mode_message,5);
        actuator_config->SetControlMode(control_mode_message,6);
    } else {
        actuator_config->SetControlMode(control_mode_message, actuator_indx); 
    }
    cout << "Updated Acutators: " << actuator_indx << std::endl;
	
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
				float value = std::stof(cell)/1000;
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
    bool verbose = true; 
    // Assuming the format of the csv_file will be in (time, x, y, z) for each line
    vector<float> temp_first_points(3, 0.0);
    int indx = 0;
    for (auto& point : csv_points)
	{
		if(point.size()>3){
			point.erase(point.begin());// remove seconds value for IR sensor
		}
        if (indx == 0) {
            temp_first_points = {point[0], point[1], point[2]};
            calculate_bias(temp_first_points);
            indx = 1;
        }
        point[0] -= bais_vector[0];
        point[1] -= bais_vector[1];
        point[2] -= bais_vector[2];
        temp_first_points = {point[0], point[1], point[2]};

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
        cout << endl;
    }
    return csv_points;
}

void KortexRobot::calculate_bias(std::vector<float> first_waypoint) {
    bais_vector[0] = first_waypoint[0] - altered_origin[0];
    bais_vector[1] = first_waypoint[1] - altered_origin[1];
    bais_vector[2] = first_waypoint[2] - altered_origin[2];
    cout << "BIAS: X: " << bais_vector[0] << "\tY: " <<bais_vector[1] << "\tZ: " << bais_vector[2] << endl;
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
    set_actuator_control_mode(1); //Set actuators to velocity mode dont use index values

    float position_tolerance = 2.0f;
    float closer_range_limit = 10.f;
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

        int stage = 0;
        std::vector<int> reachPositions(5, 0);
        int num_of_targets = target_waypoints.size();
        // Real-time loop
		vector<float> motor_velocity(actuator_count,19.0f); 
		vector<float> velocity_limits = {30.0, 10.0, 10.0, 10.0, 25.0, 20.0}; 
		vector<int> motor_direction(actuator_count,0);

        while(timer_count < (time_duration * 1000))
        {
            now = GetTickUs();
            if(now - last > 1000)
            {   
                base_feedback = base_cyclic->RefreshFeedback();
                // PID LOOPS WOULD GO WITHIN HERE
                for(int i = 0; i < actuator_count - 1; i++)
                    { 
                        if (i == 5){
                            continue;
                        } 
					// int i = 0;

					float current_pos = base_feedback.actuators(i).position();
                    float target_pos = target_joint_angles_IK[stage][i];
					float theta = abs(current_pos-target_pos); 

					// if(motor_direction[i] == 0)
					// {
					// 	motor_direction[i] = (abs(theta)>abs(360-theta)) ? -1:1;
					// }

                    float position_error = target_pos - current_pos;
					float control_sig = pids[i].calculate_pid(current_pos, target_pos, 5);
                    cout << "Stage: " << stage+1 << "\t Acc: " << i << "\t Current Pos: " << current_pos << "\t Target Pos: " << target_pos;
                        if (std::abs(position_error) > position_tolerance) {
                            reachPositions[i] = 0;
							// motor_velocity[i] = control_sig*SPEED_THRESHOLD*motor_direction[i]; 
							motor_velocity[i] = control_sig*SPEED_THRESHOLD; 
                             
                            

							cout << "\t Control Sig: " << std::fixed << std::setprecision(5) << control_sig;
                            cout << "\t Velocity: " << motor_velocity[i];
							if (abs(motor_velocity[i]) > velocity_limits[i]) {
                                
                                motor_velocity[i] = (motor_velocity[i] / abs(motor_velocity[i])) * velocity_limits[i]; 
                                cout << "\tCAPPED: " << motor_velocity[i];
                            }
                            cout << endl;

                        // TODO: Confirm what we are doing (Position or velocity or both?)
                            base_command.mutable_actuators(i)->set_position(current_pos);
                            base_command.mutable_actuators(i)->set_velocity(motor_velocity[i]);
                        }else{
                            base_command.mutable_actuators(i)->set_position(target_pos);
                            // base_command.mutable_actuators(i)->set_velocity(0);

                            std::cout << "\t Reach destination" << std::endl;
                            // std::cout << "Target: " << current_pos << "    Current: " << target_pos << std::endl;
                            reachPositions[i] = 1;
                        } 
                    }
                // PID LOOPS ABOVE
                int ready_joints = std::accumulate(reachPositions.begin(), reachPositions.end(), 0);


                if(ready_joints == 3){
                    stage++;
                    std::cout << "finished stage: " <<stage << std::endl << std::endl;
                    reachPositions = {0,0,0,0,0};
                    // std::this_thread::sleep_for(std::chrono::milliseconds(5000));
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
 
    set_actuator_control_mode(0);
    
    // Set back the servoing mode to Single Level Servoing
    servoingMode.set_servoing_mode(k_api::Base::ServoingMode::SINGLE_LEVEL_SERVOING);
    base->SetServoingMode(servoingMode);

    // Wait for a bit
	//no ty

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
    
    bool verbose = true;
    std::vector<std::vector<float>> final_joint_angles;
    k_api::Base::JointAngles input_joint_angles;
    vector<float> current_guess(6,0.0f);
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
    int guess_indx = 0;
    for(auto joint_angle : input_joint_angles.joint_angles())
    {
        current_guess[guess_indx] = joint_angle.value();
        guess_indx ++;
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
        guess_indx = 0;
        for(auto joint_angle : input_joint_angles.joint_angles())
        {
            jAngle = input_IkData.mutable_guess()->add_joint_angles();
            jAngle->set_value(current_guess[guess_indx]);
            guess_indx++;
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
            temp_joints[joint_identifier] = fmod(temp_joints[joint_identifier], 360.0);
            if (temp_joints[joint_identifier] < 0.0) {
                temp_joints[joint_identifier] = temp_joints[joint_identifier] + 360;
            }
            joint_identifier++;
        }
        current_guess = temp_joints;
        final_joint_angles.push_back(temp_joints);
    }
    // Ensure all the angles provided are mod360.0 and positive
    // int num_points = final_joint_angles.size();
    // for (int section = 0; section != num_points ;section++)
    // {
    //     for (int rotator = 0; rotator != 6; rotator++){
    //         final_joint_angles[section][rotator] = fmod(final_joint_angles[section][rotator], 360.0);
    //         if (final_joint_angles[section][rotator] < 0.0) {
    //             final_joint_angles[section][rotator] = final_joint_angles[section][rotator] + 360;
    //         }
    //     }
    // }
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
