/* warp-charger
 * Copyright (C) 2020-2021 Erik Fleckstein <erik@tinkerforge.com>
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#pragma once

#include "ArduinoJson.h"

#include "bindings/bricklet_evse.h"

#include "config.h"

#include "lwip/sockets.h"

class ENplus {
public:
    ENplus();
    void setup();
    void register_urls();
    void loop();

    bool evse_found = false;
    bool initialized = false;
    bool charging = false;

private:
    void setup_evse();
    void update_evse_state();
    void update_evse_low_level_state();
    void update_evse_max_charging_current();
    void update_evse_auto_start_charging();
    void update_evse_managed();
    void update_evse_user_calibration();
    void update_evse_charge_stats();
    bool is_in_bootloader(int rc);
    bool flash_firmware();
    bool flash_plugin(int regular_plugin_upto);
    bool wait_for_bootloader_mode(int mode);
    String get_evse_debug_header();
    String get_evse_debug_line();
    void set_managed_current(uint16_t current);
    String get_hex_privcomm_line(byte *data);
    void Serial2write(byte *data, int size);
    void PrivCommSend(byte cmd, uint16_t datasize, byte *data);
    void PrivCommAck(byte cmd, byte *data);
    void sendCommand(byte *data, int datasize);
    void sendTimeLong (void);
    void update_evseStatus(uint8_t evseStatus);

    #define PRIV_COMM_BUFFER_MAX_SIZE 1024
    byte PrivCommRxBuffer[PRIV_COMM_BUFFER_MAX_SIZE] = {'0'};
    byte PrivCommTxBuffer[PRIV_COMM_BUFFER_MAX_SIZE] = {0xFA, 0x03, 0x00, 0x00}; // Magic byte, Version, 16 bit Address - always the same
    char PrivCommHexBuffer[PRIV_COMM_BUFFER_MAX_SIZE*3] = {'0'};

    bool debug = false;

    int bs_evse_start_charging(TF_EVSE *evse);
    int bs_evse_stop_charging(TF_EVSE *evse);
    int bs_evse_set_charging_autostart(TF_EVSE *evse, bool autostart);
    int bs_evse_set_max_charging_current(TF_EVSE *evse, uint16_t max_current);
    int bs_evse_get_state(TF_EVSE *evse, uint8_t *ret_iec61851_state, uint8_t *ret_vehicle_state, uint8_t *ret_contactor_state, uint8_t *ret_contactor_error, uint8_t *ret_charge_release, uint16_t *ret_allowed_charging_current, uint8_t *ret_error_state, uint8_t *ret_lock_state, uint32_t *ret_time_since_state_change, uint32_t *ret_uptime);

    Config evse_config;
    Config evse_state;
    Config evse_hardware_configuration;
    Config evse_low_level_state;
    Config evse_max_charging_current;
    Config evse_auto_start_charging;
    Config evse_auto_start_charging_update;
    Config evse_current_limit;
    Config evse_stop_charging;
    Config evse_start_charging;
    Config evse_managed;
    Config evse_managed_update;
    Config evse_managed_current;
    Config evse_user_calibration;
    Config evse_privcomm;

    TF_EVSE evse;

    void start_managed_tasks();

    bool source_addr_valid = false;
    struct sockaddr_storage source_addr;
    int sock;
    uint32_t last_current_update = 0;
    bool shutdown_logged = false;
};
