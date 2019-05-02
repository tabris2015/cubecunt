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

int main(int argc, char *argv[])
{


    std::cout << "iniciando" << std::endl;
    blue::BlueBot bot(0.065/2, 0.17, 1496.0, 50, true);
    bot.setGoToGoalGains(atof(argv[1]), atof(argv[2]),atof(argv[3]));
    bot.setGoal(atof(argv[4]), atof(argv[5]));
    // testMotors(&bot);
    // std::cout << "both motors 0.15" << std::endl;
    // bot.driveMotors(0.15, 0.15);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // bot.driveMotors(0, 0);
    // std::cout << "unicycle 0.1, 0.0" << std::endl;
    // bot.driveUnicycle(0.1, 0.0);
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    // bot.driveUnicycle(0, 0);
    // std::this_thread::sleep_for(std::chrono::seconds(1));
    // std::cout << "stop" << std::endl;
    // bot.driveMotors(0, 0);
    
    while(bot.isAlive())
    {
        bot.setGreenLed(1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bot.setGreenLed(0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
