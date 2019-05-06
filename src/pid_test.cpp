#include <iostream>
#include <chrono>
#include <thread>
#include "bluebot.h"

#define DEBUG 1
using namespace std::chrono;

float input, output, setpoint;

int last_ticks;
// R L N
float R = 0.065 /2;
float L = 0.17;
float N = 1496.0;
//(0.065/2, 0.17, 1496.0



int main(int argc, char *argv[])
{

    float kp = atof(argv[1]);
    float ki = atof(argv[2]);
    float kd = atof(argv[3]);
    
    setpoint = atof(argv[4]);

    int motor_rate = atoi(argv[5]);

    int motor_id = atoi(argv[6]);

    
    auto sample_time = microseconds(1000000/motor_rate);

    //
    std::cout << "---------------------------------------> iniciando" << std::endl;
    // asegurarse que otra instancia no esta corriendo
    if(rc_kill_existing_process(2.0) < -2) return -1;

    // iniciar signal handler
    if(rc_enable_signal_handler() < 0) return -1;
    if(rc_encoder_eqep_init() != 0) return -1;
    rc_make_pid_file();
    if(rc_motor_init() != 0) return -1;

    blue::PidController motor_pid(&input, &output, &setpoint, kp, ki, kd, sample_time);
    
    last_ticks = rc_encoder_eqep_read(motor_id);

    auto current_time = steady_clock::now();
    auto next_time = steady_clock::time_point{current_time};
    rc_set_state(RUNNING);

    while(rc_get_state() == RUNNING)
    {
        //
        current_time = steady_clock::now();
        auto wakeup_error = current_time - next_time;
        auto error_micros = duration_cast<microseconds>(wakeup_error);
        std::cout << error_micros.count() << " \t";
        //

        auto delta_ticks = rc_encoder_eqep_read(motor_id) - last_ticks;                              // [ticks] left
        float phi_l = 2* M_PI * (delta_ticks / N);        // [rad]
        input = phi_l / (sample_time.count() / 1000000.0);     // [rad/s]
        motor_pid.compute();
        rc_motor_set(motor_id, output);
        last_ticks = rc_encoder_eqep_read(motor_id);

        next_time = current_time + sample_time - wakeup_error;
        std::this_thread::sleep_until(next_time);

    }
    
    std::cout << "---------------------------------------> FIN <-------------------------------" << std::endl;

    return 0;
}
