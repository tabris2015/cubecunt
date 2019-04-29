#include <bluebot.h>
#include <iostream>
#include <thread>
#include <chrono>
using blue::BlueBot;

rc_mpu_data_t BlueBot::imu_data_;
// constructor
BlueBot::BlueBot(int ts):sample_rate_(ts)               // iniciar tiempo de muestreo
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

    // iniciar IMU
    std::cout << "iniciando IMU...\n";
    initImu();

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


void BlueBot::initImu()
{
    imu_config_ = rc_mpu_default_config();
    // imu_config_.i2c_bus = mpu_i2c_bus;
    // imu_config_.gpio_interrupt_pin_chip = mpu_int_chip;
    // imu_config_.gpio_interrupt_pin = mpu_int_pin;
    // ignore priorities and scheduling for now
    imu_config_.enable_magnetometer = 1;
    imu_config_.show_warnings = 1;
    imu_config_.compass_time_constant = 0.5;
    imu_config_.dmp_auto_calibrate_gyro = 1;
    if(rc_mpu_initialize_dmp(&imu_data_, imu_config_)) throw "error al iniciar IMU\n";
    std::cout << "esperando que sensores se estabilicen...\n";
    std::this_thread::sleep_for(std::chrono::seconds(3));
    rc_mpu_set_dmp_callback(onImuInterrupt);
}

void BlueBot::onImuInterrupt()
{
    std::cout << imu_data_.compass_heading*RAD_TO_DEG << std::endl;
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
    rc_mpu_power_off();
	rc_remove_pid_file();
};