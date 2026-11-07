#include "hw/hw.h"

// Status values: 0 = not initialized, 1 = success, -1 = failure.
volatile int driver_init_status = 0;
volatile int current_sense_init_status = 0;

volatile float phase_current_a = 0.0f;
volatile float phase_current_b = 0.0f;
volatile float phase_current_c = 0.0f;

volatile float phase_voltage_a = 0.0f;
volatile float phase_voltage_b = 0.0f;
volatile float phase_voltage_c = 0.0f;

// Debugger-selectable state: 0=A, 1=AB, 2=B, 3=BC, 4=C, 5=CA.
volatile uint8_t test_commutation_state = 0;

namespace
{
constexpr float ACTIVE_PHASE_VOLTAGE = POWER_SUPPLY;
}  // namespace

void setup()
{
    const bool initialized = hw::init_current_sense_test();
    driver_init_status = hw::driver.initialized ? 1 : -1;
    current_sense_init_status = hw::current_sense.initialized ? 1 : -1;

    if (!initialized)
    {
        hw::driver.disable();
    }
}

void loop()
{
    if (driver_init_status != 1 || current_sense_init_status != 1)
    {
        return;
    }

    const PhaseCurrent_s currents = hw::current_sense.getPhaseCurrents();
    phase_current_a = currents.a;
    phase_current_b = currents.b;
    phase_current_c = currents.c;

    bool phase_a_active = false;
    bool phase_b_active = false;
    bool phase_c_active = false;

    switch (test_commutation_state)
    {
        case 0:
            phase_a_active = true;
            break;
        case 1:
            phase_a_active = true;
            phase_b_active = true;
            break;
        case 2:
            phase_b_active = true;
            break;
        case 3:
            phase_b_active = true;
            phase_c_active = true;
            break;
        case 4:
            phase_c_active = true;
            break;
        case 5:
            phase_c_active = true;
            phase_a_active = true;
            break;
        default:
            break;
    }

    phase_voltage_a = phase_a_active ? ACTIVE_PHASE_VOLTAGE : 0.0f;
    phase_voltage_b = phase_b_active ? ACTIVE_PHASE_VOLTAGE : 0.0f;
    phase_voltage_c = phase_c_active ? ACTIVE_PHASE_VOLTAGE : 0.0f;

    hw::driver.setPwm(phase_voltage_a, phase_voltage_b, phase_voltage_c);
}
