#include <iostream>
#include <chrono>
#include <thread>
#include "bluebot.h"

using namespace std::chrono;

// 0.11, 0.15, 0.0001
int main(int argc, char *argv[])
{

    float kp = atof(argv[1]);
    float ki = atof(argv[2]);
    float kd = atof(argv[3]);
    
    double vel = atof(argv[4]);

    int motor_rate = atoi(argv[5]);

    
    std::cout << "---------------------------------------> iniciando" << std::endl;
    blue::BlueBot bot(0.065/2, 0.17, 1496.0, 40, motor_rate);
    // init pids
    std::cout << "---------------------------------------> iniciando pid motores" << std::endl;
    bot.setMotorGains(kp, ki, kd);
    // set velocity to zero
    std::cout << "---------------------------------------> iniciando hilo motores" << std::endl;
    bot.initMotorThread();
    bot.driveMotors(0,0);
    bot.setUnicycle(0, 0);
    std::this_thread::sleep_for(milliseconds(1000));

    bot.setUnicycle(vel, 0);
    std::this_thread::sleep_for(milliseconds(2000));

    bot.setUnicycle(0, 0);
    std::this_thread::sleep_for(milliseconds(1000));
    
    std::cout << "---------------------------------------> FIN <-------------------------------" << std::endl;

    return 0;
}
