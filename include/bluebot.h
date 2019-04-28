#ifndef BLUEBOT_H
#define BLUEBOT_H
#include <robotcontrol.h>
#include <vector>
#include <map>

namespace blue
{

constexpr int left_m_channel = 2;
constexpr int right_m_channel = 3;


enum class RobotState
{
    IDLE,
    MOVING,
    ERROR
};

enum class ProgramState 
{ 

};
class BlueBot
{
private:
    /* data */
    bool is_active;
    static void onPausePress();
    static void onPauseRelease();
    static void onModePress();
    static void onModeRelease();

    void initButtons();
    void initLeds();


public:
    // constructores
    BlueBot(/* args */);
    // Destructor
    ~BlueBot();

    bool isAlive();
    void setRedLed(int val);
    void setGreenLed(int val);

    // motors
    void driveMotors(double left, double right);
    std::pair<int, int> readEncoders();
};

}

#endif // BLUEBOT_H