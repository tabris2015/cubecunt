#include "bluebot.h"
#include <iostream>
using blue::PidController;



PidController::PidController(float * input, float * output, float * setpoint, 
                    float kp, float ki, float kd, 
                    std::chrono::microseconds sample_time, bool is_angle)
:my_input_(input), 
my_output_(output), 
my_setpoint_(setpoint), 
sample_time_(sample_time), 
angle_error_(is_angle)
{
    setOutputLimits(-1.0, 1.0);
    setGains(kp, ki, kd);
}


void PidController::setSampleTime(std::chrono::microseconds new_sample_time)
{
    if(new_sample_time > std::chrono::microseconds::zero())
    {
        float ratio = (float)new_sample_time.count() / sample_time_.count();

        ki_ *= ratio;
        kd_ /= ratio;
        sample_time_ = new_sample_time;
    }
}

void PidController::setGains(float kp, float ki, float kd)
{
    if(kp < 0 || ki < 0 || kd < 0) return;

    float sample_time_sec_ = sample_time_.count() / 1000000.0;
    std::cout << "sample time s pid: " << sample_time_sec_ << std::endl;
    
    kp_ = kp;
    ki_ = ki * sample_time_sec_;
    kd_ = kd / sample_time_sec_;

    std::cout << "new pid gains: [" << kp_ << ", " << ki_ << ", " << kd_ << "]\n";
}

void PidController::setOutputLimits(float min, float max)
{
    if(min >= max) return;
    out_min_ = min;
    out_max_ = max;

    if(*my_output_ > out_max_) *my_output_ = out_max_;
    else if(*my_output_ < out_min_) *my_output_ = out_min_;

    if(out_sum_ > out_max_) out_sum_ = out_max_;
    else if(out_sum_ < out_min_) out_sum_ = out_min_;
}

void PidController::compute()
{
    float input = *my_input_;

    float error = *my_setpoint_ - input;

    if(angle_error_) error = atan2f(sinf(error), cosf(error));

    float delta_input = input - last_input_;
    out_sum_ += ki_ * error;
    
    if(out_sum_ > out_max_) out_sum_ = out_max_;
    else if(out_sum_ < out_min_) out_sum_ = out_min_;
    
    float output = kp_ * error + out_sum_ - kd_ * delta_input;

    if(output > out_max_) output = out_max_;
    else if(output < out_min_) output = out_min_;

    *my_output_ = output;
    #ifdef DEBUG
    std::cout << "set: " << *my_setpoint_
                << " \tin: " << input
                << " \terr: " << error 
                << " \tout: " << output << std::endl;    
    #endif // DEBUG

    last_input_ = input;
}
