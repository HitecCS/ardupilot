#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Notify/AP_Notify.h>
#include "AP_BattMonitor.h"
#include "AP_BattMonitor_SMBus_SUI.h"
#include <utility>

#include <GCS_MAVLink/GCS.h>

//
// Battery registers
//
#define REG_VOLTAGE             0x09
#define REG_REMCAP              0x0f
#define REG_FULLCAP             0x10
#define REG_TEMP                0x08
#define REG_SERIAL              0x1C
#define REG_MANUF_NAME          0x20
#define REG_DEVICE_NAME         0x21
#define REG_DEVICE_CHEM         0x22
#define REG_MANUF_INFO          0x25
#define REG_MANUF_DATA          0x23    /// manufacturer data
#define REG_CELL_VOLTAGE        0x28    // cell voltage register
#define REG_CURRENT             0x2a

#define BATTMONITOR_XRAY_NUM_CELLS      3
#define BATTMONITOR_ENDURANCE_NUM_CELLS 6

#define BATTMONITOR_SMBUS_SUI_BUTTON_DEBOUNCE      3       // button held down for 3 intervals will cause a power off event

const int AVG_SIZE = 10;

/*
 * Other potentially useful registers, listed here for future use
 * #define BATTMONITOR_SMBUS_SUI_VOLTAGE           0x09    // voltage register
 * #define BATTMONITOR_SMBUS_SUI_BATTERY_STATUS    0x16    // battery status register including alarms
 * #define BATTMONITOR_SMBUS_SUI_DESIGN_CAPACITY   0x18    // design capacity register
 * #define BATTMONITOR_SMBUS_SUI_DESIGN_VOLTAGE    0x19    // design voltage register
 * #define BATTMONITOR_SMBUS_SUI_SERIALNUM         0x1c    // serial number register
 * #define BATTMONITOR_SMBUS_SUI_MANUFACTURE_NAME  0x20    // manufacturer name
 * #define BATTMONITOR_SMBUS_SUI_DEVICE_NAME       0x21    // device name
 * #define BATTMONITOR_SMBUS_SUI_DEVICE_CHEMISTRY  0x22    // device chemistry
 * #define BATTMONITOR_SMBUS_SUI_MANUFACTURE_INFO  0x25    // manufacturer info including cell voltage
 */

// Constructor
AP_BattMonitor_SMBus_SUI::AP_BattMonitor_SMBus_SUI(AP_BattMonitor &mon,
                    AP_BattMonitor::BattMonitor_State &mon_state,
                    AP_BattMonitor_Params &params,
                    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                    uint8_t full_cap_register,
                    uint8_t rem_cap_register,
                    uint8_t temp_register,
                    uint8_t serial_register,
                    uint8_t current_register,
                    uint8_t voltage_register,
                    uint8_t cell_count)
    : AP_BattMonitor_SMBus(mon, mon_state, params, std::move(dev), full_cap_register, rem_cap_register, temp_register, serial_register),
    _current_register(current_register),
    _cell_count(MIN(4, cell_count)),
    _capacity(0)
{
    _pec_supported = false;
    _dev->set_split_transfers(true);
}

static float sum(std::vector<float>& v) {
    float val = 0.0f;

    for(std::vector<float>::iterator e = v.begin(); e != v.end(); e++) {
        val += *e;
    }

    return val;
}

static float avg(std::vector<float>& values) {
    return sum(values) / values.size();
}

void AP_BattMonitor_SMBus_SUI::timer()
{
    uint8_t buff[8];
    uint32_t tnow = AP_HAL::micros();

    read_cell_voltages();

    // read current
    if (read_block_bare(_current_register, buff, 4, false) == 4) {
        float current_amps = -(float)((int32_t)((uint32_t)buff[3]<<24 | (uint32_t)buff[2]<<16 | (uint32_t)buff[1]<<8 | (uint32_t)buff[0])) / 1000.0f;

        // float average = avg(currents);
        bool add = (fabs(current_amps - _state.current_amps) < 50 || currents.size() == 0);

        if(add) {
            currents.push_back(current_amps);
            if(currents.size() > AVG_SIZE) {
                currents.erase(currents.begin());
            }
        }

        _state.current_amps = avg(currents);
        _state.last_time_micros = tnow;
    } else {
        _state.current_amps = avg(currents);
    }

    // read voltage
    if(_cell_count <= 0) {
        if (read_block(_voltage_register, buff, 2, false) == 2) {
            _state.voltage = (float)((uint32_t)buff[1]<<8 | (uint32_t)buff[0]) / 1000.0f;
            _state.last_time_micros = tnow;
            _state.healthy = true;
        }
    }

    if(_full_charge_capacity == 0) {
        if(read_block_bare(_full_cap_register, buff, 4, false) == 4) {
            _full_charge_capacity = ((uint16_t)buff[3]<<24 | (uint16_t)buff[2]<<16 | (uint16_t)buff[1]<<8 | (uint16_t)buff[0]);
        }
    }

    if(_capacity == 0) {
        _capacity = read_full_charge_capacity();
    }

    read_remaining_capacity();

    // timeout after 5 seconds
    if ((tnow - _state.last_time_micros) > AP_BATTMONITOR_SMBUS_TIMEOUT_MICROS) {
        _state.healthy = false;
        // do not attempt to ready any more data from battery
        return;
    } else {
        _state.healthy = true;
    }

    // read the button press indicator
    if (read_block(REG_MANUF_DATA, buff, 6, false) == 6) {
        bool pressed = (buff[1] >> 3) & 0x01;

        if (_button_press_count >= BATTMONITOR_SMBUS_SUI_BUTTON_DEBOUNCE) {
            // battery will power off
            AP_Notify::flags.powering_off = true;
        } else if (pressed) {
            // battery will power off if the button is held
            _button_press_count++;

        } else {
            // button released, reset counters
            _button_press_count = 0;
            AP_Notify::flags.powering_off = false;
        }
    }

    read_temp();

    if(_serial_register) {
        read_serial_number();
    }
}

void AP_BattMonitor_SMBus_SUI::read_cell_voltages() {
    uint32_t tnow = AP_HAL::micros();

    // read cell voltages
    if(_cell_count > 0) {
        uint8_t voltbuff[8];

        // accumulate the pack voltage out of the total of the cells
        if (read_block_bare(REG_CELL_VOLTAGE, voltbuff, 8, false)) {
            float pack_voltage_mv = 0.0f;

            for (uint8_t i = 0; i < _cell_count; i++) {
                uint16_t cell = voltbuff[(i * 2) + 1] << 8 | voltbuff[i * 2];
                _state.cell_voltages.cells[i] = cell;
                pack_voltage_mv += (float)cell;
            }

            _has_cell_voltages = true;

            // accumulate the pack voltage out of the total of the cells
            _state.voltage = pack_voltage_mv * 1e-3;
            _state.last_time_micros = tnow;
            _state.healthy = true;
        }
    } else {
        _has_cell_voltages = false;
    }
}

bool AP_BattMonitor_SMBus_SUI::read_temp(void)
{
    uint16_t data;
    if (read_word(_temp_register, data)) {
        float temp = ((float)(data - 2731)) * 0.1f;

        if((fabs(temp - avg(temperatures)) < 100) || temperatures.size() == 0) {
            temperatures.push_back(temp);
            if(temperatures.size() > 10) {
                temperatures.erase(temperatures.begin());
            }
        }
    }

    _state.temperature = avg(temperatures);
    _state.temperature_time = AP_HAL::millis();

    return true;
}

// read_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_SUI::read_block(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const
{
    uint8_t buff[max_len+2];    // buffer to hold results (2 extra byte returned holding length and PEC)

    // read bytes
    if (!_dev->read_registers(reg, buff, sizeof(buff))) {
        return 0;
    }

    // get length
    uint8_t bufflen = buff[0];

    // sanity check length returned by smbus
    if (bufflen == 0 || bufflen > max_len) {
        return 0;
    }

    // check PEC
    uint8_t pec = get_PEC(AP_BATTMONITOR_SMBUS_I2C_ADDR, reg, true, buff, bufflen+1);
    if (pec != buff[bufflen+1]) {
        return 0;
    }

    // copy data (excluding PEC)
    memcpy(data, &buff[1], bufflen);

    // optionally add zero to end
    if (append_zero) {
        data[bufflen] = '\0';
    }

    // return success
    return bufflen;
}

bool AP_BattMonitor_SMBus_SUI::read_remaining_capacity() {
    int32_t capacity = _params._pack_capacity;

    if (capacity > 0) {
        uint16_t data;
        if (read_word(_rem_cap_register, data)) {
            float consumed = (capacity - data);
            if((consumed > 0) && (consumed < capacity)) {
                _state.consumed_mah = consumed;
                return true;
            }
        }
    }

    return false;
}

// read_bare_block - returns number of characters read if successful, zero if unsuccessful
uint8_t AP_BattMonitor_SMBus_SUI::read_block_bare(uint8_t reg, uint8_t* data, uint8_t max_len, bool append_zero) const
{
    // read bytes
    if (!_dev->read_registers(reg, data, max_len)) {
        return 0;
    }

    // return success
    return max_len;
}

//
// Xeno battery
//
AP_BattMonitor_SMBus_Xray::AP_BattMonitor_SMBus_Xray(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus_SUI(mon, mon_state, params, std::move(dev),
        REG_FULLCAP, REG_REMCAP, REG_TEMP, REG_SERIAL, REG_CURRENT, REG_VOLTAGE, BATTMONITOR_XRAY_NUM_CELLS)
{}

//
// Takes a cell count in the constructor
//
AP_BattMonitor_WithCellCount::AP_BattMonitor_WithCellCount(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : AP_BattMonitor_SMBus_SUI(mon, mon_state, params, std::move(dev), 
        REG_FULLCAP, REG_REMCAP, REG_TEMP, REG_SERIAL, REG_CURRENT, REG_VOLTAGE, BATTMONITOR_ENDURANCE_NUM_CELLS)
{}

AP_BattMonitor_WithCellCount::AP_BattMonitor_WithCellCount(AP_BattMonitor &mon,
                                                   AP_BattMonitor::BattMonitor_State &mon_state,
                                                   AP_BattMonitor_Params &params,
                                                   AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                   uint8_t num_cells)
    : AP_BattMonitor_SMBus_SUI(mon, mon_state, params, std::move(dev), 
        REG_FULLCAP, REG_REMCAP, REG_TEMP, REG_SERIAL, REG_CURRENT, REG_VOLTAGE, num_cells)
{}

void AP_BattMonitor_WithCellCount::timer() {
    AP_BattMonitor_SMBus_SUI::timer();
    read_remaining_capacity();
}

void AP_BattMonitor_WithCellCount::read_cell_voltages() {
    uint32_t tnow = AP_HAL::micros();

    // read cell voltages
    if(_cell_count > 0) {
        // uint8_t voltbuff[_cell_count * 2];
        uint8_t voltbuff[8];

        if (read_block(REG_CELL_VOLTAGE, voltbuff, 8, false)) {
            float pack_voltage_mv = 0.0f;

            for (uint8_t i = 0; i < MIN(4, _cell_count); i++) {
                uint16_t cell = voltbuff[(i * 2) + 1] << 8 | voltbuff[i * 2];
                _state.cell_voltages.cells[i] = cell;
                pack_voltage_mv += (float)cell;
            }

            if(_cell_count >= 4) {
                float avg = (pack_voltage_mv / 4);

                for(uint8_t i = 4; i < 6; ++i) {
                    _state.cell_voltages.cells[i] = avg;
                    pack_voltage_mv += _state.cell_voltages.cells[i];
                }
            }

            _has_cell_voltages = true;

            // accumulate the pack voltage out of the total of the cells
            _state.voltage = pack_voltage_mv * 1e-3;
            _state.last_time_micros = tnow;
            _state.healthy = true;
        }
    } else {
        // voltage will be read below
        _has_cell_voltages = false;
    }
}


