#include "pid.hpp"

// I want to pass the actuator index so that it can reference its individual kp, ki, kd gains and then compute

// The dt term is from an example where it is called the "loop interval time" you multiply it to the error for the intergral
// and you devide it by the difference of errors for the Derivative (the example value was 0.1). 

Pid_Loop::Pid_Loop()
{

}

void Pid_Loop::config_params(float k_p, float k_i, float k_d, float dt) {
	Pid_Loop::k_p = k_p;
	Pid_Loop::k_i = k_i;
	Pid_Loop::k_d = k_d;
	Pid_Loop::dt = dt;
}

float Pid_Loop::calculate_pid(float currentLocation, float setPoint, int actuator_index)
{
	//PID Controller calculations

	float error = setPoint - currentLocation;
	// Proportional term
	float P = k_p * error;
	// Integral term
	float integral;
	integral += error * dt;
	float I = k_i * integral;
	// Derivative term
	derivative = (error - prevErr) / dt;
	float D = k_d * derivative; 	
	// Control signal after PID controller interference
	float controlSignal = P + I + D;
	
	prevErr = error;

	return controlSignal;
}



Pid_Loop::~Pid_Loop()
{
	//deconstructor (would this involve setting the variables back to zero? or destroying the gain variables? look into it further)
}

void Pid_Loop::set_direction(int direction)
{
	// Should only matter if the direction is reversed??
	if(direction == -1) // || direction == 1)
	{
		Pid_Loop::direction = direction;
	}
	return;
}
