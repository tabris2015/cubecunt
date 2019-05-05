#include "bluebot.h"
#include <iostream>
#include <iomanip>
#include <thread>

using blue::BlueBot;
using blue::PidController;

// constructor
BlueBot::BlueBot(float R, float L, float N, int rate, int motor_rate, bool loop_thread, bool motor_thread)   
:sample_rate_(rate), motor_sample_rate_(motor_rate),        // init sample rate [Hz]
wheel_radius_(R),           // init robot wheel radius [m]                                    
base_length_(L),            // init robot base length [m]
ticks_per_rev_(N),          // init encoder ticks per revolution
microsPerClkTick_(1.0E6 * std::chrono::steady_clock::period::num / std::chrono::steady_clock::period::den), // compute micros per clock tick
interval_us_(std::chrono::microseconds(1000000 / sample_rate_)), motor_interval_us_(std::chrono::microseconds(1000000 / motor_sample_rate_)), // compute interval in microseconds
last_x_(0), 
last_y_(0), 
last_phi_(0), 
x_goal_(last_x_), y_goal_(last_y_),
Kp_gtg_(1), Ki_gtg_(0), Kd_gtg_(0), 
angle_pid_(&last_phi_, &w_, &theta_goal_, Kp_gtg_, Ki_gtg_, Kd_gtg_, interval_us_, true),
loop_thread_enabled_(loop_thread), motor_thread_enabled_(motor_thread)
{

    // asegurarse que otra instancia no esta corriendo
    if(rc_kill_existing_process(2.0) < -2) throw "Instancia corriendo\n";

    // iniciar signal handler
    if(rc_enable_signal_handler() < 0) throw "Error al iniciar signal handler/n";

    // iniciar botones
    std::cout << "iniciando botones...\n";
    initButtons();

    // iniciar leds
    std::cout << "iniciando leds...\n";
    initLeds();

    // iniciar encoders
    std::cout << "iniciando encoders...\n";
    if(rc_encoder_eqep_init() != 0) throw "error al iniciar encoders\n";

    // iniciar IMU
    std::cout << "iniciando IMU...\n";
    initImu();

    // crear archivo pid 
    std::cout << "registrando proceso...\n";
    rc_make_pid_file();

    // preparar para iniciar
    std::cout << "iniciando...\n";


    // iniciar motores
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "iniciando motores...\n";
    if(rc_motor_init() != 0) throw "error al iniciar motores\n";

    std::cout << "system clock precision: "
                << microsPerClkTick_
                << " us/tick \nPeriodo main thread: "
                << interval_us_.count()
                << " us\t Periodo motor thread: "
                << motor_interval_us_.count();

    // iniciar hilos periodicos
    if(motor_thread_enabled_)
        initMotorThread();

    if(loop_thread_enabled_)
        initMainThread();

    init_time_ = std::chrono::steady_clock::now();
    rc_set_state(RUNNING);

}

void BlueBot::initMainThread()
{
    current_start_time_ = std::chrono::steady_clock::now();
    next_start_time_ = current_start_time_;
    innerLoopThread = std::thread(&BlueBot::updateStatePeriodic, this);
    loop_thread_enabled_ = true;
}

void BlueBot::initMotorThread()
{
    motor_current_start_time_ = std::chrono::steady_clock::now();
    motor_next_start_time_ = motor_current_start_time_;
    motorLoopThread = std::thread(&BlueBot::updateMotorPeriodic, this);
    motor_thread_enabled_ = true;
}


bool BlueBot::isAlive()
{
    return rc_get_state() != EXITING;
}

void BlueBot::initButtons()
{
    if(rc_button_init(RC_BTN_PIN_PAUSE, 
                        RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US) || 
        rc_button_init(RC_BTN_PIN_MODE, 
                        RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US))
    {
		throw "Error al iniciar botones\n";
	}
	
	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE, this->onPausePress, this->onPauseRelease);
	rc_button_set_callbacks(RC_BTN_PIN_MODE, this->onModePress, this->onModeRelease);
}

void BlueBot::initLeds()
{
    if((rc_led_set(RC_LED_GREEN, 0)==-1) || (rc_led_set(RC_LED_RED, 0)==-1))
    {
		throw "Error al iniciar leds\n";
	}
}


void BlueBot::initImu()
{

    imu_config_ = rc_mpu_default_config();
    imu_config_.enable_magnetometer = 1;
    imu_config_.show_warnings = 1;
    
    if(rc_mpu_initialize(&imu_data_, imu_config_)) throw "error al iniciar IMU\n";

    std::cout << "esperando que sensores se estabilicen...\n";
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // rc_mpu_set_dmp_callback(onImuInterrupt);
}

// update imu
void BlueBot::updateImu()
{
    rc_mpu_read_gyro(&imu_data_);
    rc_mpu_read_mag(&imu_data_);
    // compute raw heading
    mag_heading_ = -atan2(imu_data_.mag[1], imu_data_.mag[0]);
    std::cout << "raw heading: " << mag_heading_ << std::endl;
    // compute gyro heading

}
// odometry
void BlueBot::updateOdometry()
{
    // use encoders for compute odometry
    auto ticks = readEncoders();
    auto delta_ticks = std::make_pair(ticks.first - last_ticks_.first, ticks.second - last_ticks_.second);
    last_ticks_ = ticks;

    float dist_l = 2* M_PI * wheel_radius_ * (delta_ticks.first / ticks_per_rev_);
    float dist_r = 2* M_PI * wheel_radius_ * (delta_ticks.second / ticks_per_rev_);

    float dist_c = (dist_r + dist_l) / 2.0;

    last_phi_ += (dist_r - dist_l) / base_length_;

    last_x_ += dist_c * cosf(last_phi_);
    last_y_ += dist_c * sinf(last_phi_);

    // std::cout << "raw ticks: [" << ticks.first << ", " << ticks.second <<"]\n";
    // std::cout << "[x, y, phi]: [" << last_x_ << ", " << last_y_ << ", " << last_phi_ << "]\n";
}

// optimize
float BlueBot::distance(float x1, float y1, float x2, float y2)
{
    return sqrtf(powf(x2 - x1, 2) + powf(y2 - y1, 2));
}

void BlueBot::updateStatePeriodic()
{
    // hilo para actualizar periodicamente 
    while(rc_get_state() != EXITING)
    {
        // get current wakeup time
        current_start_time_ = std::chrono::steady_clock::now();
        // do task
        // updateImu();
        updateOdometry();
        // update goal
        theta_goal_ = atan2f((y_goal_ - last_y_) , (x_goal_ - last_x_));
        // controller
        std::cout << std::setprecision(5) << "pos: (" << last_x_ << ", " << last_y_ << ") \t";
        angle_pid_.compute();

        auto dist_to_goal = distance(last_x_, last_y_, x_goal_, y_goal_);
        std::cout << "dist: " << dist_to_goal << "\t";
        if(dist_to_goal < 0.04) v_ = 0.0;
        // actuation
        driveUnicycle(v_, w_);
        // print state
        // std::cout << theta_goal_ << ","
        //             << last_phi_ << ","
        //             << theta_goal_ - last_phi_ << ","
        //             << w_ << ","
        //             << distance(last_x_, last_y_, x_goal_, y_goal_) << ","
        //             << v << "\n";

        // determine next point 
        next_start_time_ = current_start_time_ + interval_us_;

        // sleep until next period
        std::this_thread::sleep_until(next_start_time_);
    }
}


void BlueBot::updateMotorPeriodic()
{
    std::pair<int, int> last_ticks{0,0};
    // hilo para actualizar periodicamente 
    while(rc_get_state() != EXITING)
    {
        // get current wakeup time
        motor_current_start_time_ = std::chrono::steady_clock::now();
        // do task
        double v_l = (2 * v_ - w_ * base_length_) / (2 * wheel_radius_);
        double v_r = (2 * v_ + w_ * base_length_) / (2 * wheel_radius_);
        
        v_l *= 0.1;
        v_r *= 0.1;
        
        auto ticks = readEncoders();
        auto delta_ticks = std::make_pair(ticks.first - last_ticks.first, ticks.second - last_ticks.second);

        last_ticks_ = readEncoders();
        
        float phi_l = 2* M_PI * (delta_ticks.first / ticks_per_rev_);        // en radianes
        float vel_l = phi_l / (motor_interval_us_.count() / 1000000.0);
        // move left motor
        rc_motor_set(left_m_channel, v_l);
        
        // for now, only print time, v, output, delta ticks, angular velocity
        auto micros = std::chrono::duration_cast<std::chrono::microseconds> (std::chrono::steady_clock::now() - init_time_);
        std::cout << micros.count() << ", "
                    << v_ << ", "
                    << v_l << ","
                    << delta_ticks.first << ", "
                    << vel_l << "\n";
        
        // determine next point 
        motor_next_start_time_ = motor_current_start_time_ + motor_interval_us_;

        // sleep until next period
        std::this_thread::sleep_until(motor_next_start_time_);
    }
}



void BlueBot::setRedLed(int val)
{
    rc_led_set(RC_LED_RED, val);
}

void BlueBot::setGreenLed(int val)
{
    rc_led_set(RC_LED_GREEN, val);
}


void BlueBot::setGoal(float x_goal, float y_goal)
{
    x_goal_ = x_goal;
    y_goal_ = y_goal;
    theta_goal_ = atan2f((y_goal_ - last_y_) , (x_goal_ - last_x_));

    std::cout << "new goal: [" << x_goal_ << ", " << y_goal_ << "] theta: " << theta_goal_ << std::endl;
}

void BlueBot::setGoToGoalGains(float kp, float ki, float kd)
{
    Kp_gtg_ = kp;
    Ki_gtg_ = ki;
    Kd_gtg_ = kd;
    angle_pid_.setGains(Kp_gtg_, Ki_gtg_, Kd_gtg_);
}

void BlueBot::setLinearVel(float v)
{
    v_ = v;
}

void BlueBot::onPausePress()
{
    std::cout << "Pause presionado\n";
}
void BlueBot::onPauseRelease()
{
    std::cout << "Pause soltado\n";
}
void BlueBot::onModePress()
{
    std::cout << "Mode presionado\n";
}
void BlueBot::onModeRelease()
{
    std::cout << "Mode soltado\n";
}

// motors

void BlueBot::driveMotors(double left, double right)
{
    constexpr double scaler = 0.1;
    rc_motor_set(left_m_channel, scaler * left);
    rc_motor_set(right_m_channel, scaler * right);
}

void BlueBot::driveUnicycle(double v, double w)
{
    double v_r = (2 * v + w * base_length_) / (2 * wheel_radius_);
    double v_l = (2 * v - w * base_length_) / (2 * wheel_radius_);
    // std::cout << "velocities: [" << v_l << ", " << v_r << "]\n";
    driveMotors(v_l, v_r);
}

//encoders
std::pair<int, int> BlueBot::readEncoders()
{
    return std::make_pair(rc_encoder_eqep_read(left_m_channel), rc_encoder_eqep_read(right_m_channel));
}


BlueBot::~BlueBot()
{
    std::cout << "limpiando...\n";
    rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
    rc_button_cleanup();
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
    rc_mpu_power_off();
	rc_remove_pid_file();
};