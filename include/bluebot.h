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


// TEMPORAL 
enum class RobotState
{
    IDLE,
    MOVING,
    ERROR
};

class PidController
{
private:
    // sample time
    std::chrono::microseconds sample_time_;

    std::chrono::steady_clock::time_point lastTime_;
    float kp_;
    float ki_;
    float kd_;
    // pointers to variables
    float * my_input_;
    float * my_output_;
    float * my_setpoint_;

    float out_min_;
    float out_max_;

    float out_sum_;
    float err_sum_;
    float last_err_;

    float i_term_;
    float last_input_;

    bool angle_error_;


public:
    PidController(float * input, float * output, float * setpoint, 
                    float kp, float ki, float kd, 
                    std::chrono::microseconds sample_time, bool is_angle=false);
    // PidController(float kp, float ki, float kd, std::chrono::microseconds sample_time);
    
    void compute(void);
    void setOutputLimits(float min, float max);
    void setGains(float kp, float ki, float kd);
    void setSampleTime(std::chrono::microseconds new_sample_time);
    
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
    std::chrono::microseconds interval_us_;
    std::chrono::steady_clock::time_point current_start_time_;
    std::chrono::steady_clock::time_point next_start_time_;
    std::thread innerLoopThread;

    float vel_l_;
    float vel_r_;
    float v_;
    float w_;
    float last_x_;
    float last_y_;
    float last_phi_;
    
    float x_goal_;
    float y_goal_;
    float theta_goal_;
    float v_goal_;
    bool is_active_;
    
    // controllers
    // 
    PidController angle_pid_;

    float Kp_gtg_;
    float Ki_gtg_;
    float Kd_gtg_;
    float e_gtg_;
    float last_e_gtg_;
    float e_sum_gtg_;
    float delta_e_gtg_;

    std::pair<float, float> goal_;

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

    float distance(float x1, float y1, float x2, float y2);


public:
    void updateStatePeriodic();
    // constructores
    // BlueBot(int ts=50);
    // constructor con parametros
    BlueBot(float R, float L, float N, int rate=50, bool loop_thread=false);
    // Destructor
    ~BlueBot();

    bool isAlive();
    void setRedLed(int val);
    void setGreenLed(int val);

    // motors
    void driveMotors(double left, double right);
    void driveUnicycle(double v, double w);
    std::pair<int, int> readEncoders();
    // odometry

    // controllers 
    void setGoal(float x_goal, float y_goal);
    void setGoToGoalGains(float kp, float ki, float kd);
    void setAngle(float theta_goal);
    void setLinearVel(float v);
};

}
// 4899972882855154

#endif // BLUEBOT_H