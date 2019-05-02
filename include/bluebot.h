#ifndef BLUEBOT_H
#define BLUEBOT_H
#include <robotcontrol.h>
#include <vector>
#include <map>
#include <chrono>
#include <thread>
#include <atomic>

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
    // robot params
    const float wheel_radius_;
    const float base_length_;
    const float ticks_per_rev_;
    // robot state
    
    // imu data
    rc_mpu_data_t imu_data_;
    float mag_heading_;

    // timing data
    std::chrono::steady_clock::time_point last_time_;
    std::pair<int, int> last_ticks_;
    int sample_rate_;
    // threads for control loop
    const double microsPerClkTick_;
    std::chrono::milliseconds intervalMillis_;
    std::chrono::steady_clock::time_point currentStartTime_;
    std::chrono::steady_clock::time_point nextStartTime_;
    std::thread innerLoopThread;

    float vel_l_;
    float vel_r_;
    float v_;
    float w_;
    float last_x_;
    float last_y_;
    float last_phi_;
    
    bool is_active_;

    // imu
    rc_mpu_config_t imu_config_;


    // callbacks
    static void onPausePress();
    static void onPauseRelease();
    static void onModePress();
    static void onModeRelease();
    
    void initButtons();
    void initLeds();
    void initImu();

    // robot model
    std::pair<float, float> uniToDiff(float v, float w);

    void updateOdometry();
    void updateImu();


public:
    void updateStatePeriodic();
    // constructores
    // BlueBot(int ts=50);
    // constructor con parametros
    BlueBot(float R, float L, float N, int rate=50);
    // Destructor
    ~BlueBot();

    bool isAlive();
    void setRedLed(int val);
    void setGreenLed(int val);

    // motors
    void driveMotors(double left, double right);
    std::pair<int, int> readEncoders();
    // odometry
};

}
// 4899972882855154

#endif // BLUEBOT_H