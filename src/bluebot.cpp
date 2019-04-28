#include <bluebot.h>
#include <iostream>

using blue::BlueBot;
// constructor
BlueBot::BlueBot()
{
    // asegurarse que otra instancia no esta corriendo
    if(rc_kill_existing_process(2.0) < -2) throw "Instancia corriendo\n";

    // iniciar signal handler
    if(rc_enable_signal_handler() < 0) throw "Error al iniciar signal handler/n";

    // iniciar botones
    std::cout << "iniciando botones...\n";
    initButtons();

    // iniciar leds
    std::cout << "iniciando leds...\n";
    initLeds();

    // iniciar motores
    std::cout << "iniciando motores...\n";
    if(rc_motor_init() != 0) throw "error al iniciar motores\n";

    // iniciar encoders
    std::cout << "iniciando encoders...\n";
    if(rc_encoder_eqep_init() != 0) throw "error al iniciar encoders\n";
    // crear archivo pid 
    std::cout << "registrando proceso...\n";
    rc_make_pid_file();

    // preparar para iniciar
    std::cout << "iniciando...\n";
    rc_set_state(RUNNING);

}

bool BlueBot::isAlive()
{
    return rc_get_state() != EXITING;
}

void BlueBot::initButtons()
{
    if(rc_button_init(RC_BTN_PIN_PAUSE, 
                        RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US) || 
        rc_button_init(RC_BTN_PIN_MODE, 
                        RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US))
    {
		throw "Error al iniciar botones\n";
	}
	
	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE, this->onPausePress, this->onPauseRelease);
	rc_button_set_callbacks(RC_BTN_PIN_MODE, this->onModePress, this->onModeRelease);
}

void BlueBot::initLeds()
{
    if((rc_led_set(RC_LED_GREEN, 0)==-1) || (rc_led_set(RC_LED_RED, 0)==-1))
    {
		throw "Error al iniciar leds\n";
	}
}


void BlueBot::setRedLed(int val)
{
    rc_led_set(RC_LED_RED, val);
}

void BlueBot::setGreenLed(int val)
{
    rc_led_set(RC_LED_GREEN, val);
}



void BlueBot::onPausePress()
{
    std::cout << "Pause presionado\n";
}
void BlueBot::onPauseRelease()
{
    std::cout << "Pause soltado\n";
}
void BlueBot::onModePress()
{
    std::cout << "Mode presionado\n";
}
void BlueBot::onModeRelease()
{
    std::cout << "Mode soltado\n";
}


// motors

void BlueBot::driveMotors(double left, double right)
{
    rc_motor_set(left_m_channel, left);
    rc_motor_set(right_m_channel, right);
}

//encoders
std::pair<int, int> BlueBot::readEncoders()
{
    return std::make_pair(rc_encoder_eqep_read(left_m_channel), rc_encoder_eqep_read(right_m_channel));
}


BlueBot::~BlueBot()
{
    std::cout << "limpiando...\n";
    rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
    rc_button_cleanup();
    rc_motor_cleanup();
    rc_encoder_eqep_cleanup();
	rc_remove_pid_file();
};