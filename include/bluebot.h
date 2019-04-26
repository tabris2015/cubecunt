#ifndef BLUEBOT_H
#define BLUEBOT_H
#include <robotcontrol.h>
#include <vector>

namespace blue
{
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

public:
    BlueBot(/* args */);
    int drive(double left_speed, double right_speed);
    int diffDrive(double angular, double linear);
    int getLinePos();
    int openArm();
    int closeArm();
    std::vector<float> getDistances();

    ~BlueBot();
};

}

#endif // BLUEBOT_H