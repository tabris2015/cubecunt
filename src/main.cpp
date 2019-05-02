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

int main(void)
{

    std::cout << "hola bola" << std::endl;
    blue::BlueBot bot(0.065/2, 0.17, 1364.8, 50);

    // testMotors(&bot);

    while(bot.isAlive())
    {
        bot.setGreenLed(1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bot.setGreenLed(0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
