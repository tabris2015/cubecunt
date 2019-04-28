#ifndef BLUEBOT_H
#define BLUEBOT_H
#include <robotcontrol.h>
#include <vector>
#include <map>

namespace blue
{
// for motors
constexpr int left_m_channel = 2;
constexpr int right_m_channel = 3;
// for mpu
constexpr int mpu_i2c_bus = 2;
constexpr int mpu_int_chip = 3;
constexpr int mpu_int_pin = 2;




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
    bool is_active_;

    int sample_rate_;
    // imu
    rc_mpu_config_t imu_config_;

    // callbacks
    static void onPausePress();
    static void onPauseRelease();
    static void onModePress();
    static void onModeRelease();
    static void onImuInterrupt();

    void initButtons();
    void initLeds();
    void initImu();


public:
    static rc_mpu_data_t imu_data_;
    // constructores
    BlueBot(int ts = 50);
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