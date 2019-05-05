#include <iostream>
#include <chrono>
#include <thread>
#include "bluebot.h"

using namespace std::chrono;

int main(int argc, char *argv[])
{

    float kp = atoi(argv[1]);
    float ki = atoi(argv[2]);
    float kd = atoi(argv[3]);
    
    double vel = atof(argv[4]);
    
    std::cout << "iniciando" << std::endl;
    blue::BlueBot bot(0.065/2, 0.17, 1496.0, 40, motor_rate);
    // init pids
    bot.setMotorGains(kp, ki, kd);
    // set velocity to zero
    bot.initMotorThread();

    bot.setUnicycle(0, 0);
    std::this_thread::sleep_for(milliseconds(1000));

    bot.setUnicycle(vel, 0);
    std::this_thread::sleep_for(milliseconds(2000));

    bot.setUnicycle(0, 0);
    std::this_thread::sleep_for(milliseconds(1500));


    return 0;
}
