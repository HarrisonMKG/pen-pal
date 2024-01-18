#include "pid.hpp"


PID::calculate_pid(float val)
{
	//implementation
}

PID::PID(float k_p, float k_i, float k_d);
{
	PID::k_p = k_p;
	PID::k_i = k_i;
	PID::k_d = k_d;
}

PID::~PID()
{
	//deconstructor
}

void PID::set_direction(int direction)
{
	if(direction == -1 || direction == 1)
	{
		PID::direction = direction;
	}
	return;
}
