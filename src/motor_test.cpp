#include <iostream>
#include <chrono>
#include <thread>
#include "bluebot.h"

using namespace std::chrono;

int main(int argc, char *argv[])
{


    std::cout << "iniciando" << std::endl;
    blue::BlueBot bot(0.065/2, 0.17, 1496.0);
    // set velocity to zero
    bot.initMotorThread();
    bot.setUnicycle(0, 0);
    bot.setRedLed(1);
    bot.setGreenLed(0);
    std::this_thread::sleep_for(seconds(1));
    bot.setUnicycle(0.5, 0);
    bot.setRedLed(0);
    bot.setGreenLed(1);
    std::this_thread::sleep_for(seconds(2));
    bot.setUnicycle(0, 0);
    bot.setRedLed(1);
    bot.setGreenLed(0);
    
    return 0;
}
