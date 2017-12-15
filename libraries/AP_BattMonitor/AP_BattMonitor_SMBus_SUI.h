#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_Math/AP_Math.h>
#include "AP_BattMonitor_SMBus.h"
#include <AP_HAL/I2CDevice.h>

// Base SUI class
class AP_BattMonitor_SMBus_SUI : public AP_BattMonitor_SMBus
{
public:

    // Constructor
    AP_BattMonitor_SMBus_SUI(AP_BattMonitor &mon,
                    AP_BattMonitor::BattMonitor_State &mon_state,
                    AP_BattMonitor_Params &params,
                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                    uint8_t full_cap_register,
                    uint8_t rem_cap_register,
                    uint8_t temp_register,
                    uint8_t serial_register,
                    uint8_t current_register,
                    uint8_t voltage_register,
                    uint8_t cell_count
                    );

protected:

    void timer(void);

    // read_block - returns number of characters read if successful, zero if unsuccessful
    uint8_t read_block(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const;
    uint8_t read_block_bare(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const;

    uint8_t _button_press_count;

    uint8_t _current_register;
    uint8_t _voltage_register;
    uint8_t _cell_count;
    int32_t _capacity;
};

// Xray battery
class AP_BattMonitor_SMBus_Xray: public AP_BattMonitor_SMBus_SUI {
public:

    // Constructor
    AP_BattMonitor_SMBus_Xray(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

};

// Endurance battery
class AP_BattMonitor_SMBus_Endurance: public AP_BattMonitor_SMBus_SUI {
public:

    // Constructor
    AP_BattMonitor_SMBus_Endurance(AP_BattMonitor &mon,
                             AP_BattMonitor::BattMonitor_State &mon_state,
                             AP_BattMonitor_Params &params,
                             AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

};
