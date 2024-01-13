#include <cstdlib>



class KortexRobot
{
private:
    /* data */
public:
    KortexRobot(/* args */);
    ~KortexRobot();

protected:
    std::string ip_address;


    

};

KortexRobot::KortexRobot(/* args */);

KortexRobot::~KortexRobot();

std::vector<std::vector<int>> read_csv(const std::string& filename);
