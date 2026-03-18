#pragma once

#include <SimpleFOC.h>

#define RED_LED_PIN PB_11
#define GREEN_LED_PIN PB_13
#define BLUE_LED_PIN PC_6

namespace led
{
enum COLORS
{
    OFF = 0x0,
    RED = 0b100,
    GREEN = 0b010,
    BLUE = 0b001,
    YELLOW = 0b110,
    PURPLE = 0b101,
    CYAN = 0b011,
    WHITE = 0b111,
};

COLORS on_color = OFF;
COLORS off_color = OFF;
uint32_t period = 1000;  // in milliseconds
float duty_cycle = 0.5;  // in fraction

void init()
{
    pinMode(pinNametoDigitalPin(RED_LED_PIN), OUTPUT);
    pinMode(pinNametoDigitalPin(GREEN_LED_PIN), OUTPUT);
    pinMode(pinNametoDigitalPin(BLUE_LED_PIN), OUTPUT);
}

void set_on_color(uint8_t color)
{
    on_color = static_cast<COLORS>(color);
}

void set_off_color(uint8_t color)
{
    off_color = static_cast<COLORS>(color);
}

void set_period(uint32_t _period)
{
    period = _period;
}

void set_duty_cycle(float _duty_cycle)
{
    duty_cycle = _duty_cycle;
}

void tick()
{
    if (millis() % period < period * duty_cycle)
    {
        digitalWrite(pinNametoDigitalPin(RED_LED_PIN), (on_color & RED) ? LOW : HIGH);
        digitalWrite(pinNametoDigitalPin(GREEN_LED_PIN), (on_color & GREEN) ? LOW : HIGH);
        digitalWrite(pinNametoDigitalPin(BLUE_LED_PIN), (on_color & BLUE) ? LOW : HIGH);
    }
    else
    {
        digitalWrite(pinNametoDigitalPin(RED_LED_PIN), (off_color & RED) ? LOW : HIGH);
        digitalWrite(pinNametoDigitalPin(GREEN_LED_PIN), (off_color & GREEN) ? LOW : HIGH);
        digitalWrite(pinNametoDigitalPin(BLUE_LED_PIN), (off_color & BLUE) ? LOW : HIGH);
    }
}

void set_blink_led(uint8_t on_color, uint8_t off_color = OFF, uint32_t period = 1000,
                   float duty_cycle = 0.5)
{
    set_on_color(on_color);
    set_off_color(off_color);
    set_period(period);
    set_duty_cycle(duty_cycle);
    tick();
}

void set_led(uint8_t color)
{
    set_on_color(color);
    set_off_color(color);
    tick();
}

void delay(uint32_t ms)
{
    for (uint32_t i = 0; i < ms; i++)
    {
        tick();
        delayMicroseconds(1000);
    }
}

};  // namespace led
