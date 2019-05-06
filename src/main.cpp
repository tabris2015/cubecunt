#include <iostream>
#include <chrono>
#include <thread>
#include "bluebot.h"

void testMotors(blue::BlueBot* bot)
{
    double delta = 0.01;
    for(double i = 0.0; i < 1.0; i += delta)
    {
        bot->driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot->readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }
    for(double i = 1.0; i > 0.0; i -= delta)
    {
        bot->driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot->readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }

    for(double i = 0.0; i > -1.0; i -= delta)
    {
        bot->driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot->readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }
    for(double i = -1.0; i < 0.0; i += delta)
    {
        bot->driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot->readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }

}
// 0.11, 0.15, 0.0001
int main(int argc, char *argv[])
{

    std::vector<std::vector<float>> goals{{0.4, 0},{0.4, -0.4}, {0.75, 0},{1.1, -0.5}, {0.5, -0.5},{0,0}};

    std::cout << "iniciando" << std::endl;
    blue::BlueBot bot(0.065/2, 0.17, 1496.0);
    // pid motores
    bot.setMotorGains(0.12,0.16,0.0001);
    bot.driveMotors(0,0);
    bot.setUnicycle(0,0);
    bot.setGoToGoalGains(atof(argv[1]), atof(argv[2]),atof(argv[3]));
    bot.setVMax(atof(argv[4]));
    bot.initMotorThread();
    bot.initMainThread();
   
    bot.setRedLed(1);
    bot.setGreenLed(0);
    
    for(auto goal: goals)
    {
        bot.setGoal(goal[0], goal[1]);
        while(!bot.isOnGoal()){};
        std::cout << "------> objetivo alcanzado!!!!\n";
    }

    bot.setRedLed(0);
    bot.setGreenLed(1);
    std::cout << "COMPLETADO!!!!\n";
    return 0;
}
