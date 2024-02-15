#ifndef PID_HPP
#define PID_HPP

#include <string>
#include <vector>
#include <cmath>


class Pid_Loop
{
private:
	int direction; //Should only be -1 and 1

	// these should not be touched by the user and be consistent per actuator
	float k_p;
	float k_i;
	float k_d;
	const float d_t = 0.001;

	// Not sure if these need to be private
	float integral;
	float derivative;
	float setPoint;
	float prevErr;

	// Later could introduce a Max and a Min value that the output calculation must stay within (the controlSignal)
	// float max; // example values of max was 100
	// float min; // example values of min was -100
	
public:
	
	void set_direction(int direction);
	float calculate_pid(float currentLocation, float setPoint, int actuator_index);
	const float INTEGRAL_CLAMP = 0.01;
	void clear_integral();

	Pid_Loop(float k_p, float k_i, float k_d);
	~Pid_Loop();

protected:
	//data


};

#endif // PID.HPP
