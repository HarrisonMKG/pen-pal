#include "pid.hpp"


PID::calculate_pid(float val)
{
	//implementation
}

PID::PID()
{
	//constructor
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
