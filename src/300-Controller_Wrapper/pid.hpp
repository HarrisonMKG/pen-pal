#include <string>
#include <vector>
#include <cmath>

namespace k_api = Kinova::Api;

class PID
{
	private:
		int direction;//Should only be -1 and 1
	public:
		float k_p;
		float k_i;
		float k_d;


		void set_direction(int direction);
		float calculate_pid(float val); // not sure how this would work, just example of what you could do

		PID();
		~PID();
	protected:
		//data
}
