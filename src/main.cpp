// #define SPI_SR_TXP

#include "can/can.h"
#include "error/error.h"
#include "hw/hw.h"
#include "id/id.h"
#include "led/led.h"

uint8_t motor_id = 0;

void setup()
{
    led::init();
    led::set_blink_led(led::CYAN, led::OFF, 100);

    led::delay(200);

    id::init();
    hw::init();

    led::set_blink_led(led::BLUE, led::OFF, 50);
    led::delay(200);

    motor_id = id::get_id();
    hw::motor.target = 0;

    can::init();

    led::set_blink_led(led::GREEN, led::BLUE, 50, 0.5);
    for (int i = 0; i < 10; i++)
    {
        led::tick();
        delay(20);
    }

    led::set_blink_led(led::YELLOW);
}

uint8_t current_errors = error::NONE;
uint8_t current_warnings = error::NONE;

uint32_t last_time = 0;
uint32_t dt = 0;

float motor_temp = 0;
float pcb_temp = 0;
float vbus = 0;

void loop()
{
    dt = micros() - last_time;
    last_time = micros();

    motor_temp = hw::get_motor_temp();
    pcb_temp = hw::get_pcb_temp();
    vbus = hw::get_vbus();

    if (motor_id != 0)
    {
        current_errors |= hw::motor.motor_status == FOCMotorStatus::motor_init_failed
                              ? error::MOTOR_INIT_FAILED
                              : error::NONE;
        current_errors |= hw::motor.motor_status == FOCMotorStatus::motor_calib_failed
                              ? error::MOTOR_CALIB_FAILED
                              : error::NONE;
        current_errors |= hw::motor.motor_status == FOCMotorStatus::motor_error
                              ? error::MOTOR_ERROR
                              : error::NONE;
        current_errors |= !hw::is_pcb_temp_ok(pcb_temp) ? error::PCB_TEMP : error::NONE;
        current_errors |= !hw::is_motor_temp_ok(motor_temp) ? error::MOTOR_TEMP : error::NONE;

        current_warnings |= hw::is_pcb_temp_warning(pcb_temp) ? error::PCB_TEMP : error::NONE;
        current_warnings |=
            hw::is_motor_temp_warning(motor_temp) ? error::MOTOR_TEMP : error::NONE;

        if (current_errors == error::NONE && current_warnings == error::NONE)
        {
            hw::foc_loop();
            led::set_blink_led(led::GREEN, led::OFF, 2500, 1 - 0.025);
        }
        else if (current_warnings != error::NONE && current_errors == error::NONE)
        {
            static bool was_warning = false;

            hw::reinit();

            hw::foc_loop();
            led::set_blink_led(led::GREEN, led::YELLOW, 1000 * current_warnings, 0.5);
        }
        else
        {
            hw::foc_abort();
            led::set_blink_led(led::RED, led::YELLOW, 1000 * current_errors, 0.5);
        }
    }
    else
    {
        static uint32_t timer1 = 0;
        static int8_t flag = 0;

        hw::motor.loopFOC();
        if (millis() - timer1 >= 2000)
        {
            flag += 1;
            if (flag >= 2)
                flag = 0;
            timer1 = millis();
        }
        if (flag == 0)
        {
            hw::motor.move(10);
        }
        else if (flag == 1)
        {
            hw::motor.move(-5);
        }
    }

    led::tick();
}