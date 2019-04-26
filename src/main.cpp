#include <iostream>
#include <chrono>
#include <thread>
#include "bluebot.h"

int main(void)
{
    std::cout << "hola bola" << std::endl;
    blue::BlueBot bot;

    while(bot.isAlive())
    {
        bot.setGreenLed(1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        bot.setGreenLed(0);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    return 0;
}
