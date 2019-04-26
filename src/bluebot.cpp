#include <bluebot.h>
#include <iostream>

using blue::BlueBot;

BlueBot::BlueBot()
{
    // asegurarse que otra instancia no esta corriendo
    if(rc_kill_existing_process(2.0) < -2) throw "Instancia corriendo\n";

    // iniciar signal handler
    if(rc_enable_signal_handler() < 0) throw "Error al iniciar signal handler/n";

    // iniciar botones
    std::cout << "iniciando botones...\n";
    initButtons();

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

BlueBot::~BlueBot()
{
    rc_button_cleanup();
	rc_remove_pid_file();
};