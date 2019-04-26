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
    // constructores
    BlueBot(/* args */);
    // Destructor
    ~BlueBot();

    bool isAlive();
};

}

#endif // BLUEBOT_H