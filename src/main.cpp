#include <iostream>
#include <chrono>
#include <thread>
#include "bluebot.h"

int main(void)
{
    std::cout << "hola bola" << std::endl;
    blue::BlueBot bot;

    for(double i = 0.0; i < 1.0; i += 0.001)
    {
        bot.driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot.readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }
    for(double i = 1.0; i > 0.0; i -= 0.001)
    {
        bot.driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot.readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }

    for(double i = 0.0; i > -1.0; i -= 0.001)
    {
        bot.driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot.readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }
    for(double i = -1.0; i < 0.0; i += 0.001)
    {
        bot.driveMotors(i,i);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        auto enc = bot.readEncoders();
        std::cout << "motors: " << enc.first << ", " << enc.second << "\n";
    }

    while(bot.isAlive())
    {
        bot.setGreenLed(1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bot.setGreenLed(0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
