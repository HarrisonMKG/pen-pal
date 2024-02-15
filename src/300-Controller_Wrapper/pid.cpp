#include "pid.hpp"

// I want to pass the actuator index so that it can reference its individual kp, ki, kd gains and then compute

// The dt term is from an example where it is called the "loop interval time" you multiply it to the error for the intergral
// and you devide it by the difference of errors for the Derivative (the example value was 0.1). 

Pid_Loop::Pid_Loop(float k_p, float k_i, float k_d)
{
	Pid_Loop::k_p = k_p;
	Pid_Loop::k_i = k_i;
	Pid_Loop::k_d = k_d;
	integral = 0;
	prevErr = 0;
}

float Pid_Loop::calculate_pid(float currentLocation, float setPoint, int actuator_index)
{
	//PID Controller calculations
	int direction = 1; 
	float error = setPoint - currentLocation; 
	// float other_error = currentLocation - setPoint; 
	if (actuator_index == 4 && ((currentLocation>300&& setPoint >120))){
		error = currentLocation-setPoint-300; 
	}else if (abs(error) >= 180) {
		if (error > 0){
			error = error - 360;
		} else {
			error = error + 360;
		}
	}

	// Proportional term
	float P = k_p * error;
	// Integral term
	integral += error * d_t;
	float I = k_i * integral;
	// Derivative term
	derivative = (error - prevErr) / d_t;
	float D = k_d * derivative; 	
	// Control signal after PID controller interference
	float controlSignal = P + I + D;
	
	prevErr = error;

	return controlSignal*direction;
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

void Pid_Loop::clear_integral()
{
	integral = 0;
	return;
}