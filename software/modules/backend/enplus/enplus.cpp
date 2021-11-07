/* warp-charger
 * Copyright (C) 2020-2021 Erik Fleckstein <erik@tinkerforge.com>
 * Copyright (C)      2021 Birger Schmidt <bs-warp@netgaroo.com>
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

#include "enplus.h"
#include "enplus_firmware.h"

#include "bindings/errors.h"

#include "api.h"
#include "event_log.h"
#include "task_scheduler.h"
#include "tools.h"
#include "web_server.h"
#include "modules.h"
#include "HardwareSerial.h"
#include "Time/TimeLib.h"

extern EventLog logger;

extern TaskScheduler task_scheduler;
extern TF_HalContext hal;
extern WebServer server;

extern API api;
extern bool firmware_update_allowed;

// Charging profile:
// 10A ESP> W (2021-06-06 11:05:10) [PRIV_COMM, 1859]: Tx(cmd_AD len:122) :  FA 03 00 00 AD 1D 70 00 00 44 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 06 06 0B 05 0A 00 00 00 00 0A 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 03 01 00 00 00 00 00 00 00 00 00 00 CE 75
// 12A ESP> W (2021-06-03 18:37:19) [PRIV_COMM, 1859]: Tx(cmd_AD len:122) :  FA 03 00 00 AD 19 70 00 00 44 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 06 03 12 25 14 00 00 00 00 0C 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 03 01 00 00 00 00 00 00 00 00 00 00 BF 11
// 11A ESP> W (2021-06-04 08:07:58) [PRIV_COMM, 1859]: Tx(cmd_AD len:122) :  FA 03 00 00 AD 39 70 00 00 44 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 02 FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 15 06 04 08 07 32 00 00 00 00 0B 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 03 01 00 00 00 00 00 00 00 00 00 00 3A 0C
//

// Commands
// First byte = command code, then payload bytes, no crc bytes
byte Init1[] = {0xAC, 0x11, 0x0B, 0x01, 0x00, 0x00};
byte Init2[] = {0xAC, 0x11, 0x09, 0x01, 0x00, 0x01};
byte Init3[] = {0xAC, 0x11, 0x0A, 0x01, 0x00, 0x00};
byte Init4[] = {0xAC, 0x11, 0x0C, 0x01, 0x00, 0x00};
byte Init5[] = {0xAA, 0x18, 0x3E, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00};
byte Init6[] = {0xAC, 0x11, 0x0D, 0x04, 0x00, 0xB8, 0x0B, 0x00, 0x00};
byte Init7[] = {0xAA, 0x18, 0x3F, 0x04, 0x00, 0x1E, 0x00, 0x00, 0x00};
byte Init8[] = {0xAA, 0x18, 0x25, 0x0E, 0x00, 0x05, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x02};
byte Init9[] = {0xAA, 0x18, 0x12, 0x01, 0x00, 0x03, 0x7B, 0x89};

byte Init10[] = {0xAA, 0x18, 0x12, 0x01, 0x00, 0x03, 0x3B, 0x9C}; // is Init10 the same as Init12?

// ctrl_cmd set ack done, type:0
//[2019-01-01 03:36:46] cmd_AA [privCommCmdAACfgCtrl]!
//[2019-01-01 03:36:46] cfg ctrl  addr:18 size:1 set:1 gun_id:0 len:1
//[2019-01-01 03:36:46] cfg ctrl_ack start_addr:18 end_addr:19 now_addr:18 set:1 gun_id:0 len:1
//[2019-01-01 03:36:46] ctrl_cmd:18 setType:1 [cmdAACtrlSetReset]!
byte Init12[] = {0xAA, 0x18, 0x12, 0x01, 0x00, 0x03}; // this triggers 0x02 SN, Hardware, Version
//W (1970-01-01 00:08:47) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 03 05 00 18 12 01 00 03 FB F6
//W (1970-01-01 00:08:48) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 03 05 00 14 12 01 00 00 53 F1
//I (1970-01-01 00:08:48) [PRIV_COMM, 51]: ctrl_cmd set ack done, type:0
//W (1970-01-01 00:08:48) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 04 05 00 18 12 01 00 03 BA 10
//W (1970-01-01 00:08:48) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 04 05 00 14 12 01 00 00 12 17
//I (1970-01-01 00:08:48) [PRIV_COMM, 51]: ctrl_cmd set ack done, type:0
//W (1970-01-01 00:08:48) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 05 05 00 18 12 01 00 03 7B DC
//W (1970-01-01 00:08:49) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 05 05 00 14 12 01 00 00 D3 DB
//I (1970-01-01 00:08:49) [PRIV_COMM, 51]: ctrl_cmd set ack done, type:0

// cmdAACtrlcantestsetAck test cancom...111
byte Init11[] = {0xAA, 0x18, 0x2A, 0x00, 0x00};

byte Init13[] = {0xA2, 0x00}; // is this just an ack for 0x02?
//ack for 03  //byte Init14[] = {0xA3, 0x18, 0x02, 0x06, 0x00, 0x15, 0x06, 0x0A, 0x07, 0x08, 0x26};

// ctrl_cmd set start power mode done
byte Init15[] = {0xAA, 0x18, 0x09, 0x01, 0x00, 0x00};
//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 40 05 00 18 09 01 00 00 F9 36
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 40 05 00 14 09 01 00 00 11 30
//I (2021-04-11 18:36:27) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower: 3150080

//privCommCmdA7StartTransAck
byte StartCharging[] = {0xA7, 0x57, 0x41, 0x52, 0x50, 0x20, 0x43, 0x68, 0x61, 0x72, 0x67, 0x65, 0x72, 0x20, 0x66, 0x6F, 0x72, 0x20, 0x45, 0x4E, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte StopCharging[] = {0xA7, 0x57, 0x41, 0x52, 0x50, 0x20, 0x43, 0x68, 0x61, 0x72, 0x67, 0x65, 0x72, 0x20, 0x66, 0x6F, 0x72, 0x20, 0x45, 0x4E, 0x2B, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00};

//privCommCmdAFSmartCurrCtl
byte ChargingSettings[] = {0xAF, 0, 0x15, 0x06, 0x04, 0x0D, 0x0A, 0x21, 0x80, 0x51, 0x01, 0, 0x01, 0, 0, 0, 0, 0x09, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Commands found in Autoaid protocol document, chapter 6.3:
// Enter boot mode: Tx(cmd_AB len: 20): FA 03 00 00 AB 14 0A 00 00 00 00 00 00 00 05 00 00 00 62 B2
// Enter boot mode acknowledge: Rx(cmd_0B len: 16): FA 03 00 00 0B 14 06 00 00 00 05 00 00 00 F1 3A
byte EnterBootMode[] = {0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00};
// Exit boot mode, enter application mode: Tx (cmd_AB len: 24): FA 03 00 00 AB 5A 0E 00 00 00 03 08 FC FF 00 00 02 00 4F 4B 00 00 E1 98
byte EnterAppMode[] = {0xAB, 0x00, 0x00, 0x03, 0x08, 0xFC, 0xFF, 0x00, 0x00, 0x02, 0x00, 0x4F, 0x4B, 0x00, 0x00};

uint16_t crc16_modbus(uint8_t *buffer, uint32_t length) {
        //uint16_t crc = 0xFFFF;
        uint16_t crc = 0x0;

        while (length--) {
                uint8_t tmp = *buffer++ ^ crc;
                crc >>= 8;
                crc ^= crc16_modbus_table[tmp];
        }

        return crc;
}

#define PRIVCOMM_MAGIC      0
#define PRIVCOMM_VERSION    1
#define PRIVCOMM_ADDR       2
#define PRIVCOMM_CMD        3
#define PRIVCOMM_SEQ        4
#define PRIVCOMM_LEN        5
#define PRIVCOMM_PAYLOAD    6
#define PRIVCOMM_CRC        7

#define PayloadStart        8

String ENplus::get_hex_privcomm_line(byte *data) {
    #define LOG_LEN 4048 //TODO work without such a big buffer by writing line by line
    char log[LOG_LEN] = {0};
    char *local_log = log;
    uint16_t len = (uint16_t)(data[7] << 8 | data[6]) + 10; // payload length + 10 bytes header and crc
    if (len > 1000) { // mqtt buffer is the limiting factor
        logger.printfln("ERROR: buffer (%d bytes) too big for mqtt buffer (max 1000).", len);
        len = 1000;
    }

    int offset = 0;
    for(uint16_t i = 0; i < len; i++) {
        #define BUFFER_CHUNKS 20
        if(i % BUFFER_CHUNKS == 0) {
            if(i>0) { local_log += snprintf(local_log, LOG_LEN - (local_log - log), "\r\n"); }
            if(i>=BUFFER_CHUNKS) { local_log += snprintf(local_log, LOG_LEN - (local_log - log), "            "); }
            local_log += snprintf(local_log, LOG_LEN - (local_log - log), "          %.3d: ", offset);
            offset = offset + BUFFER_CHUNKS;
        }
        local_log += snprintf(local_log, LOG_LEN - (local_log - log), "%.2X ", data[i]);
    }
    local_log += snprintf(local_log, LOG_LEN - (local_log - log), "\r\n");
    logger.write(log, local_log - log);
    return String(log);
}

void ENplus::Serial2write(byte *data, int size) {
    int bytes_to_send = size;
    int offset = 0;
    uint32_t start_time = millis();
    while(bytes_to_send > 0 || millis() - start_time >= 1000) {
        int afw = Serial2.availableForWrite();
        if(afw < bytes_to_send) { // send chunk
            bytes_to_send = bytes_to_send - Serial2.write(PrivCommTxBuffer + offset, afw);
            //logger.printfln("    send: Tx afw:%d bytes_to_send:%d offset:%d", afw, bytes_to_send, offset);
            offset = offset + afw;
            //logger.printfln("    could not send in one shot... delay(1250)");
            //delay(1250);
        } else { // send reminder
            bytes_to_send = bytes_to_send - Serial2.write(PrivCommTxBuffer + offset, bytes_to_send);
            //logger.printfln("    sEND: Tx afw:%d bytes_to_send:%d offset:%d", afw, bytes_to_send, offset);
        }
    }
    if(bytes_to_send > 0) {
        logger.printfln("ERR Tx time out, but still %d bytes_to_send, which where discarded now", bytes_to_send);
    }
}

void ENplus::sendCommand(byte *data, int datasize) {
    PrivCommTxBuffer[4] = data[0]; // command code
    PrivCommTxBuffer[5]++; // increment sequence number
    PrivCommTxBuffer[6] = (datasize-1) & 0xFF;
    PrivCommTxBuffer[7] = (datasize-1) >> 8;
    memcpy(PrivCommTxBuffer+8, data+1, datasize-1);

    uint16_t crc = crc16_modbus(PrivCommTxBuffer, datasize + 7);

    PrivCommTxBuffer[datasize+7] = crc & 0xFF;
    PrivCommTxBuffer[datasize+8] = crc >> 8;

    get_hex_privcomm_line(PrivCommTxBuffer); // PrivCommHexBuffer now holds the hex representation of the buffer
    logger.printfln("Tx cmd_%.2X seq:%.2X, len:%d, crc:%.4X", PrivCommTxBuffer[4], PrivCommTxBuffer[5], datasize+9, crc);

    Serial2write(data, datasize + 9);
    evse_privcomm.get("TX")->updateString(PrivCommHexBuffer);
}

void ENplus::PrivCommSend(byte cmd, uint16_t datasize, byte *data) {
    // the first 4 bytes never change and should be set already
    data[4] = cmd;
    //data[5] = sendSequence-1;
    data[6] = datasize & 0xFF;
    data[7] = datasize >> 8;
    //for (int i=1; i<=datasize; i++) {
    //    PrivCommTxBuffer[i+7] = payload[i];
    //}

    uint16_t crc = crc16_modbus(data, datasize + 8);

    data[datasize+8] = crc & 0xFF;
    data[datasize+9] = crc >> 8;

    get_hex_privcomm_line(data); // PrivCommHexBuffer now holds the hex representation of the buffer
    logger.printfln("Tx cmd_%.2X seq:%.2X, len:%d, crc:%.4X", cmd, data[5], datasize, crc);

    Serial2write(data, datasize + 10);
    evse_privcomm.get("TX")->updateString(PrivCommHexBuffer);
}

void ENplus::PrivCommAck(byte cmd, byte *data) {
    // the first 4 bytes never change and should be set already
    data[4] = cmd ^ 0xA0;  // complement command for ack
    data[6] = 1; //len
    data[7] = 0; //len
    data[8] = 0; //payload
    uint16_t crc = crc16_modbus(data, 9);
    data[9] = crc & 0xFF;
    data[10] = crc >> 8;

    get_hex_privcomm_line(data); // PrivCommHexBuffer now holds the hex representation of the buffer
    logger.printfln("Tx cmd_%.2X seq:%.2X, crc:%.4X", data[4], data[5], crc);

    Serial2write(data, 11);
    evse_privcomm.get("TX")->updateString(PrivCommHexBuffer);
}


void ENplus::sendTimeLong() {
    PrivCommTxBuffer[5]++; // increment sequence number
    time_t t = now(); // get current time
    PrivCommTxBuffer[PayloadStart + 0] = 0x18;
    PrivCommTxBuffer[PayloadStart + 1] = 0x02;
    PrivCommTxBuffer[PayloadStart + 2] = 0x06;
    PrivCommTxBuffer[PayloadStart + 3] = 0x00;
    PrivCommTxBuffer[PayloadStart + 4] = year(t) -2000;
    PrivCommTxBuffer[PayloadStart + 5] = month(t);
    PrivCommTxBuffer[PayloadStart + 6] = day(t);
    PrivCommTxBuffer[PayloadStart + 7] = hour(t);
    PrivCommTxBuffer[PayloadStart + 8] = minute(t);
    PrivCommTxBuffer[PayloadStart + 9] = second(t);
    PrivCommSend(0xAA, 10, PrivCommTxBuffer);
}

ENplus::ENplus()
{
    evse_config = Config::Object({
        {"auto_start_charging", Config::Bool(true)},
        {"managed", Config::Bool(true)},
        {"max_current_configured", Config::Uint16(0)}
    });

    evse_state = Config::Object({
        {"iec61851_state", Config::Uint8(0)},
        {"vehicle_state", Config::Uint8(0)},
        {"GD_state", Config::Uint8(0)},
        {"contactor_state", Config::Uint8(0)},
        {"contactor_error", Config::Uint8(0)},
        {"charge_release", Config::Uint8(0)},
        {"allowed_charging_current", Config::Uint16(0)},
        {"error_state", Config::Uint8(0)},
        {"lock_state", Config::Uint8(0)},
        {"time_since_state_change", Config::Uint32(0)},
        {"last_state_change", Config::Uint32(0)},
        {"uptime", Config::Uint32(0)}
    });

    evse_privcomm = Config::Object({
        {"RX", Config::Str("",1000)},
        {"TX", Config::Str("",1000)}
    });

    evse_hardware_configuration = Config::Object({
        {"Hardware", Config::Str("",20)},
        {"FirmwareVersion", Config::Str("",20)},
        {"SerialNumber", Config::Str("",20)},
        {"evse_found", Config::Bool(false)},
        {"initialized", Config::Bool(false)},
        {"jumper_configuration", Config::Uint8(3)}, // 3 = 16 Ampere = 11KW for the EN+ wallbox
        {"has_lock_switch", Config::Bool(false)}    // no key lock switch
    });

    evse_low_level_state = Config::Object ({
        {"low_level_mode_enabled", Config::Bool(false)},
        {"led_state", Config::Uint8(0)},
        {"cp_pwm_duty_cycle", Config::Uint16(0)},
        {"adc_values", Config::Array({
                Config::Uint16(0),
                Config::Uint16(0),
            }, Config::Uint16(0), 2, 2, Config::type_id<Config::ConfUint>())
        },
        {"voltages", Config::Array({
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
            }, Config::Int16(0), 3, 3, Config::type_id<Config::ConfInt>())
        },
        {"resistances", Config::Array({
                Config::Uint32(0),
                Config::Uint32(0),
            }, Config::Uint32(0), 2, 2, Config::type_id<Config::ConfUint>())
        },
        {"gpio", Config::Array({Config::Bool(false),Config::Bool(false),Config::Bool(false),Config::Bool(false), Config::Bool(false)}, Config::Bool(false), 5, 5, Config::type_id<Config::ConfBool>())}
    });

    evse_max_charging_current = Config::Object ({
        {"max_current_configured", Config::Uint16(0)},
        {"max_current_incoming_cable", Config::Uint16(16000)},
        {"max_current_outgoing_cable", Config::Uint16(16000)},
        {"max_current_managed", Config::Uint16(0)},
    });

    evse_auto_start_charging = Config::Object({
        {"auto_start_charging", Config::Bool(true)}
    });

    evse_auto_start_charging_update = Config::Object({
        {"auto_start_charging", Config::Bool(true)}
    });
    evse_current_limit = Config::Object({
        {"current", Config::Uint(32000, 6000, 32000)}
    });

    evse_stop_charging = Config::Null();
    evse_start_charging = Config::Null();

    evse_managed_current = Config::Object ({
        {"current", Config::Uint16(0)}
    });

    evse_managed = Config::Object({
        {"managed", Config::Bool(false)}
    });

    evse_managed_update = Config::Object({
        {"managed", Config::Bool(false)},
        {"password", Config::Uint32(0)}
    });

    evse_user_calibration = Config::Object({
        {"user_calibration_active", Config::Bool(false)},
        {"voltage_diff", Config::Int16(0)},
        {"voltage_mul", Config::Int16(0)},
        {"voltage_div", Config::Int16(0)},
        {"resistance_2700", Config::Int16(0)},
        {"resistance_880", Config::Array({
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
            }, Config::Int16(0), 14, 14, Config::type_id<Config::ConfInt>())},
    });
}

int ENplus::bs_evse_start_charging(TF_EVSE *evse) {
    uint8_t allowed_charging_current = uint8_t(evse_state.get("allowed_charging_current")->asUint()/1000);
    logger.printfln("EVSE start charging with max %d Ampere", allowed_charging_current);

    time_t t=now();     // get current time
    ChargingSettings[2] = year(t) -2000;
    ChargingSettings[3] = month(t);
    ChargingSettings[4] = day(t);
    ChargingSettings[5] = hour(t);
    ChargingSettings[6] = minute(t);
    ChargingSettings[7] = second(t);
    ChargingSettings[17] = allowed_charging_current;
    sendCommand(ChargingSettings, sizeof(ChargingSettings));

    sendCommand(StartCharging, sizeof(StartCharging));
    return 0;
}

int ENplus::bs_evse_stop_charging(TF_EVSE *evse) {
    logger.printfln("EVSE stop charging");
    sendCommand(StopCharging, sizeof(StopCharging));
    return 0;
}

int ENplus::bs_evse_persist_config() {
    String error = api.callCommand("evse/config_update", Config::ConfUpdateObject{{
        {"auto_start_charging", evse_auto_start_charging.get("auto_start_charging")->asBool()},
        {"max_current_configured", evse_max_charging_current.get("max_current_configured")->asUint()},
        {"managed", evse_managed.get("managed")->asBool()}
    }});
    if (error != "") {
        logger.printfln("Failed to save config: %s", error.c_str());
        return 500;
    } else {
	logger.printfln("saved config - auto_start_charging: %s, managed: %s, max_current_configured: %d",
            evse_config.get("auto_start_charging")->asBool() ?"true":"false",
            evse_config.get("managed")->asBool() ?"true":"false",
            evse_config.get("max_current_configured")->asUint());
        return 0;
    }
}

int ENplus::bs_evse_set_charging_autostart(TF_EVSE *evse, bool autostart) {
    logger.printfln("EVSE set auto start charging to %s", autostart ? "true" :"false");
    evse_auto_start_charging.get("auto_start_charging")->updateBool(autostart);
    bs_evse_persist_config();
    return 0;
}

int ENplus::bs_evse_set_max_charging_current(TF_EVSE *evse, uint16_t max_current) {
    evse_max_charging_current.get("max_current_configured")->updateUint(max_current);
    bs_evse_persist_config();
    update_evse_state();
    uint8_t allowed_charging_current = evse_state.get("allowed_charging_current")->asUint()/1000;
    logger.printfln("EVSE set configured charging limit to %d Ampere", uint8_t(max_current/1000));
    logger.printfln("EVSE calculated allowed charging limit is %d Ampere", allowed_charging_current);

    time_t t=now();     // get current time
    ChargingSettings[2] = year(t) -2000;
    ChargingSettings[3] = month(t);
    ChargingSettings[4] = day(t);
    ChargingSettings[5] = hour(t);
    ChargingSettings[6] = minute(t);
    ChargingSettings[7] = second(t);
    ChargingSettings[17] = allowed_charging_current;
    sendCommand(ChargingSettings, sizeof(ChargingSettings));

    return 0;
}

int ENplus::bs_evse_get_state(TF_EVSE *evse, uint8_t *ret_iec61851_state, uint8_t *ret_vehicle_state, uint8_t *ret_contactor_state, uint8_t *ret_contactor_error, uint8_t *ret_charge_release, uint16_t *ret_allowed_charging_current, uint8_t *ret_error_state, uint8_t *ret_lock_state, uint32_t *ret_time_since_state_change, uint32_t *ret_uptime) {
//    bool response_expected = true;
//    tf_tfp_prepare_send(evse->tfp, TF_EVSE_FUNCTION_GET_STATE, 0, 17, response_expected);
    uint32_t allowed_charging_current;

    *ret_iec61851_state = evse_state.get("iec61851_state")->asUint();
    *ret_vehicle_state = evse_state.get("iec61851_state")->asUint(); // == 1 ? // charging ? 2 : 1; // 1 verbunden 2 leadt
    *ret_contactor_state = 2;
    *ret_contactor_error = 0;
    *ret_charge_release = 1; // manuell 0 automatisch
    // find the charging current maximum
    allowed_charging_current = min(
        evse_max_charging_current.get("max_current_incoming_cable")->asUint(),
        evse_max_charging_current.get("max_current_outgoing_cable")->asUint());
    if(evse_managed.get("managed")->asBool()) {
        allowed_charging_current = min(
            allowed_charging_current,
            evse_max_charging_current.get("max_current_managed")->asUint());
    }
    *ret_allowed_charging_current = min(
        allowed_charging_current,
        evse_max_charging_current.get("max_current_configured")->asUint());
    *ret_error_state = 0;
    *ret_lock_state = 0;
    *ret_time_since_state_change = evse_state.get("time_since_state_change")->asUint();
    *ret_uptime = millis();

    return TF_E_OK;
}

void ENplus::setup()
{
    setup_evse();
    if(!api.restorePersistentConfig("evse/config", &evse_config)) {
        logger.printfln("EVSE error, could not restore persistent storage config");
    } else {
        evse_auto_start_charging.get("auto_start_charging")     -> updateBool(evse_config.get("auto_start_charging")->asBool());
        evse_max_charging_current.get("max_current_configured") -> updateUint(evse_config.get("max_current_configured")->asUint());
        evse_managed.get("managed")                             -> updateBool(evse_config.get("managed")->asBool());
	logger.printfln("restored config - auto_start_charging: %s, managed: %s, max_current_configured: %d",
            evse_config.get("auto_start_charging")->asBool() ?"true":"false",
            evse_config.get("managed")->asBool() ?"true":"false",
            evse_config.get("max_current_configured")->asUint());
    }

    task_scheduler.scheduleWithFixedDelay("update_evse_state", [this](){
        update_evse_state();
    }, 0, 1000);

    task_scheduler.scheduleWithFixedDelay("update_evse_low_level_state", [this](){
        update_evse_low_level_state();
    }, 0, 1000);

    task_scheduler.scheduleWithFixedDelay("update_evse_user_calibration", [this](){
        update_evse_user_calibration();
    }, 0, 10000);

    task_scheduler.scheduleWithFixedDelay("update_evse_charge_stats", [this](){
        update_evse_charge_stats();
    }, 0, 10000);

#ifdef MODULE_CM_NETWORKING_AVAILABLE
    cm_networking.register_client([this](uint16_t current){
        set_managed_current(current);
        //evse_managed.get("managed")->updateBool(true);
	logger.printfln("evse_managed: %s, current: %d", evse_managed.get("managed")->asBool() ?"true":"false", current);
    });

    task_scheduler.scheduleWithFixedDelay("evse_send_cm_networking_client", [this](){
        cm_networking.send_client_update(
            evse_state.get("iec61851_state")->asUint(),
            evse_state.get("vehicle_state")->asUint(),
            evse_state.get("error_state")->asUint(),
            evse_state.get("charge_release")->asUint(),
            evse_state.get("uptime")->asUint(),
            evse_state.get("allowed_charging_current")->asUint()
        );
    }, 1000, 1000);

    task_scheduler.scheduleWithFixedDelay("evse_managed_current_watchdog", [this]() {
        if (!deadline_elapsed(this->last_current_update + 30000))
            return;
        if(!this->shutdown_logged)
            logger.printfln("Got no managed current update for more than 30 seconds. Setting managed current to 0");
        this->shutdown_logged = true;
        evse_managed_current.get("current")->updateUint(0);
    }, 1000, 1000);
#endif
}

String ENplus::get_evse_debug_header() {
    return "millis,iec,vehicle,contactor,_error,charge_release,allowed_current,error,lock,t_state_change,uptime,low_level_mode_enabled,led,cp_pwm,adc_pe_cp,adc_pe_pp,voltage_pe_cp,voltage_pe_pp,voltage_pe_cp_max,resistance_pe_cp,resistance_pe_pp,gpio_in,gpio_out,gpio_motor_in,gpio_relay,gpio_motor_error\n";
}

String ENplus::get_evse_debug_line() {
    if(!initialized)
        return "EVSE is not initialized!";

    uint8_t iec61851_state, vehicle_state, contactor_state, contactor_error, charge_release, error_state, lock_state;
    uint16_t allowed_charging_current;
    uint32_t time_since_state_change, uptime;

    int rc = bs_evse_get_state(&evse,
        &iec61851_state,
        &vehicle_state,
        &contactor_state,
        &contactor_error,
        &charge_release,
        &allowed_charging_current,
        &error_state,
        &lock_state,
        &time_since_state_change,
        &uptime);

    if(rc != TF_E_OK) {
        return String("evse_get_state failed: rc: ") + String(rc);
    }

    bool low_level_mode_enabled;
    uint8_t led_state;
    uint16_t cp_pwm_duty_cycle;

    uint16_t adc_values[2];
    int16_t voltages[3];
    uint32_t resistances[2];
    bool gpio[5];

//    rc = tf_evse_get_low_level_state(&evse,
//        &low_level_mode_enabled,
//        &led_state,
//        &cp_pwm_duty_cycle,
//        adc_values,
//        voltages,
//        resistances,
//        gpio);

    if(rc != TF_E_OK) {
        return String("evse_get_low_level_state failed: rc: ") + String(rc);
    }

    char line[150] = {0};
    snprintf(line, sizeof(line)/sizeof(line[0]), "%lu,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%c,%u,%u,%u,%u,%d,%d,%d,%u,%u,%c,%c,%c,%c,%c\n",
        millis(),
        iec61851_state,
        vehicle_state,
        contactor_state,
        contactor_error,
        charge_release,
        allowed_charging_current,
        error_state,
        lock_state,
        time_since_state_change,
        uptime,
        low_level_mode_enabled ? '1' : '0',
        led_state,
        cp_pwm_duty_cycle,
        adc_values[0],adc_values[1],
        voltages[0],voltages[1],voltages[2],
        resistances[0],resistances[1],
        gpio[0] ? '1' : '0',gpio[1] ? '1' : '0',gpio[2] ? '1' : '0',gpio[3] ? '1' : '0',gpio[4] ? '1' : '0');

    return String(line);
}

void ENplus::set_managed_current(uint16_t current) {
    //is_in_bootloader(tf_evse_set_managed_current(&evse, current));
    evse_managed_current.get("current")->updateUint(current);
    evse_max_charging_current.get("max_current_managed")->updateUint(current);
    this->last_current_update = millis();
    this->shutdown_logged = false;
}

void ENplus::register_urls()
{
    if (!evse_found)
        return;

    api.addPersistentConfig("evse/config", &evse_config, {}, 1000);

    api.addState("evse/state", &evse_state, {}, 1000);
    api.addState("evse/hardware_configuration", &evse_hardware_configuration, {}, 1000);
    api.addState("evse/low_level_state", &evse_low_level_state, {}, 1000);
    api.addState("evse/max_charging_current", &evse_max_charging_current, {}, 1000);
    api.addState("evse/auto_start_charging", &evse_auto_start_charging, {}, 1000);
    api.addState("evse/privcomm", &evse_privcomm, {}, 1000);

    api.addCommand("evse/auto_start_charging_update", &evse_auto_start_charging_update, {}, [this](){
        bs_evse_set_charging_autostart(&evse, evse_auto_start_charging_update.get("auto_start_charging")->asBool());
    }, false);

    api.addCommand("evse/current_limit", &evse_current_limit, {}, [this](){
        bs_evse_set_max_charging_current(&evse, evse_current_limit.get("current")->asUint());
    }, false);

    api.addCommand("evse/stop_charging", &evse_stop_charging, {}, [this](){bs_evse_stop_charging(&evse);}, true);
    api.addCommand("evse/start_charging", &evse_start_charging, {}, [this](){bs_evse_start_charging(&evse);}, true);

    api.addCommand("evse/managed_current_update", &evse_managed_current, {}, [this](){
        this->set_managed_current(evse_managed_current.get("current")->asUint());
    }, true);

    api.addState("evse/managed", &evse_managed, {}, 1000);
    api.addCommand("evse/managed_update", &evse_managed_update, {"password"}, [this](){
        evse_managed.get("managed")->updateBool(evse_managed_update.get("managed")->asBool());
        bs_evse_persist_config();
    }, true);

    api.addState("evse/user_calibration", &evse_user_calibration, {}, 1000);
    api.addCommand("evse/user_calibration_update", &evse_user_calibration, {}, [this](){

    }, true);

#ifdef MODULE_WS_AVAILABLE
    server.on("/evse/start_debug", HTTP_GET, [this](WebServerRequest request) {
        task_scheduler.scheduleOnce("enable evse debug", [this](){
            ws.pushStateUpdate(this->get_evse_debug_header(), "evse/debug_header");
            debug = true;
        }, 0);
        request.send(200);
    });

    server.on("/evse/stop_debug", HTTP_GET, [this](WebServerRequest request){
        task_scheduler.scheduleOnce("enable evse debug", [this](){
            debug = false;
        }, 0);
        request.send(200);
    });
#endif
}

void ENplus::loop()
{
    static uint32_t last_check = 0;
    static uint32_t nextMillis = 2000;
    static uint8_t evseStatus = 0;
    static uint8_t cmd;
    static uint8_t seq;
    static uint16_t len;
    uint16_t crc;
    static bool cmd_to_process = false;
    static byte PrivCommRxState = PRIVCOMM_MAGIC;
    static int PrivCommRxBufferPointer = 0;
    byte rxByte;

    if(evse_found && !initialized && deadline_elapsed(last_check + 10000)) {
        last_check = millis();
        setup_evse();
    }

#ifdef MODULE_WS_AVAILABLE
    static uint32_t last_debug = 0;
    if(debug && deadline_elapsed(last_debug + 50)) {
        last_debug = millis();
        ws.pushStateUpdate(this->get_evse_debug_line(), "evse/debug");
    }
#endif

    if( Serial2.available() > 0 && !cmd_to_process) {
        do {
            rxByte = Serial2.read();
            PrivCommRxBuffer[PrivCommRxBufferPointer++] = rxByte;
            //Serial.print(rxByte, HEX);
            //Serial.print(" ");
            switch( PrivCommRxState ) {
                // Magic Header (0xFA) Version (0x03) Address (0x0000) CMD (0x??) Seq No. (0x??) Length (0x????) Payload (0-1015) Checksum (crc16)
                case PRIVCOMM_MAGIC:
                    PrivCommRxBufferPointer=1;
                    if(rxByte == 0xFA) {
                        PrivCommRxState = PRIVCOMM_VERSION;
                    } else {
                        logger.printfln("PRIVCOMM ERR: out of sync byte: %.2X", rxByte);
                    }
                    break;
                case PRIVCOMM_VERSION:
                    if(rxByte == 0x03) {
                        PrivCommRxState = PRIVCOMM_ADDR;
                    } else {
                        logger.printfln("PRIVCOMM ERR: got Rx Packet with wrong Version %.2X.", rxByte);
                        PrivCommRxState = PRIVCOMM_MAGIC;
                    }
                    break;
                case PRIVCOMM_ADDR:
                    if(rxByte == 0x00) {
                        if(PrivCommRxBufferPointer == 4) { // this was the second byte of the address, move on
                            PrivCommRxState = PRIVCOMM_CMD;
                        }
                    } else {
                        logger.printfln("PRIVCOMM ERR: got Rx Packet with wrong Address %.2X%.2X. PrivCommRxBufferPointer: %d", PrivCommRxBuffer[2], PrivCommRxBuffer[3], PrivCommRxBufferPointer);
                        PrivCommRxState = PRIVCOMM_MAGIC;
                    }
                    break;
                case PRIVCOMM_CMD:
                    PrivCommRxState = PRIVCOMM_SEQ;
                    cmd = rxByte;
                    break;
                case PRIVCOMM_SEQ:
                    PrivCommRxState = PRIVCOMM_LEN;
                    seq = rxByte;
                    PrivCommTxBuffer[5] = seq;
                    break;
                case PRIVCOMM_LEN:
                    if(PrivCommRxBufferPointer == 8) { // this was the second byte of the length, move on
                        PrivCommRxState = PRIVCOMM_PAYLOAD;
                        len = (uint16_t)(PrivCommRxBuffer[7] << 8 | PrivCommRxBuffer[6]);
                        //logger.printfln("PRIVCOMM INFO: len: %d cmd:%.2X", len, cmd);
                    }
                    break;
                case PRIVCOMM_PAYLOAD:
                    if(PrivCommRxBufferPointer == len + 8) {
                        //final byte of Payload received.
                        PrivCommRxState = PRIVCOMM_CRC;
                    } else {//if(cmd == 0x09) {
                        // this is an ugly hack to deal with non conforming 0x09 "upload log" packets
                        // cmd09 messeges are always too short and with no crc??? wtf?
                        // hopefully there are no more packets like this, or enough to recognize a pattern
                        if(PrivCommRxBuffer[PrivCommRxBufferPointer-4] == 0xFA &&
                           PrivCommRxBuffer[PrivCommRxBufferPointer-3] == 0x03 &&
                           PrivCommRxBuffer[PrivCommRxBufferPointer-2] == 0x00 &&
                           PrivCommRxBuffer[PrivCommRxBufferPointer-1] == 0x00) {
                            logger.printfln("PRIVCOMM BUG: process the next command albeit the last one was not finished. Buggy! cmd:%.2X len:%d cut off:%d", cmd, len, PrivCommRxBufferPointer-4);
                            PrivCommRxState = PRIVCOMM_CMD;
                            PrivCommRxBufferPointer = 4;
                            get_hex_privcomm_line(PrivCommRxBuffer); // PrivCommHexBuffer now holds the hex representation of the buffer
                            evse_privcomm.get("RX")->updateString(PrivCommHexBuffer);
                            cmd_to_process = true;
                        }
                    }
                    break;
                case PRIVCOMM_CRC:
                    if(PrivCommRxBufferPointer == len + 10) {
            //Serial.println();
                        PrivCommRxState = PRIVCOMM_MAGIC;
                        get_hex_privcomm_line(PrivCommRxBuffer); // PrivCommHexBuffer now holds the hex representation of the buffer
                        crc = (uint16_t)(PrivCommRxBuffer[len + 9] << 8 | PrivCommRxBuffer[len + 8]);
                        uint16_t checksum = crc16_modbus(PrivCommRxBuffer, len+8);
                        if(crc == checksum) {
                            if(!evse_found) {
                                logger.printfln("EN+ GD EVSE found. Enabling EVSE support.");
                                evse_found = true;
                                evse_hardware_configuration.get("evse_found")->updateBool(evse_found);
                            }
                            logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X", cmd, seq, len, crc);
                        } else {
                            logger.printfln("CRC ERROR Rx cmd_%.2X seq:%.2X len:%d crc:%.4X checksum:%.4X", cmd, seq, len, crc, checksum);
                            break;
                        }
                        // log the whole packet?, but logger.printfln only writes 128 bytes / for now Serial.print on Serial2.read it is.
                        evse_privcomm.get("RX")->updateString(PrivCommHexBuffer);
                        cmd_to_process = true;
                    }
                    break;
            }//switch read packet
        } while((Serial2.available() > 0) && !cmd_to_process && PrivCommRxBufferPointer<sizeof(PrivCommRxBuffer)/sizeof(PrivCommRxBuffer[0])); // one command at a time
    }

    char str[20];

    if(cmd_to_process) {
        switch( cmd ) {
            case 0x02: // Info: Serial number, Version
//W (1970-01-01 00:08:52) [PRIV_COMM, 1919]: Rx(cmd_02 len:135) :  FA 03 00 00 02 26 7D 00 53 4E 31 30 30 35 32 31 30 31 31 39 33 35 37 30 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 24 D1 00 41 43 30 31 31 4B 2D 41 55 2D 32 35 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 31 2E 31 2E 32 37 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 09 00 00 00 00 00 5A 00 1E 00 00 00 00 00 00 00 00 00 D9 25
//W (1970-01-01 00:08:52) [PRIV_COMM, 1764]: Tx(cmd_A2 len:11) :  FA 03 00 00 A2 26 01 00 00 99 E0
                logger.printfln("   cmd_%.2X seq:%.2X Ack Serial number and Version.", cmd, seq);
                PrivCommAck(cmd, PrivCommTxBuffer); // privCommCmdA2InfoSynAck
                sprintf(str, "%s", PrivCommRxBuffer+8);
                evse_hardware_configuration.get("SerialNumber")->updateString(str);
                sprintf(str, "%s",PrivCommRxBuffer+43);
                evse_hardware_configuration.get("Hardware")->updateString(str);
                sprintf(str, "%s",PrivCommRxBuffer+91);
                evse_hardware_configuration.get("FirmwareVersion")->updateString(str);
                logger.printfln("EVSE serial: %s hw: %s fw: %s", 
                    evse_hardware_configuration.get("SerialNumber")->asString().c_str(),
                    evse_hardware_configuration.get("Hardware")->asString().c_str(),
                    evse_hardware_configuration.get("FirmwareVersion")->asString().c_str());
                if(!evse_hardware_configuration.get("initialized")->asBool()) {
                    initialized =
                        evse_hardware_configuration.get("Hardware")->asString().compareTo("AC011K-AU-25") == 0      &&
                        evse_hardware_configuration.get("FirmwareVersion")->asString().startsWith("1.1.", 0)        && // known working: 1.1.27, 1.1.212, 1.1.258
                        evse_hardware_configuration.get("FirmwareVersion")->asString().substring(4).toInt() <= 258;    // higest known working version (we assume earlier versions work as well)
                    evse_hardware_configuration.get("initialized")->updateBool(initialized);
                    if(initialized) {
                         logger.printfln("EN+ GD EVSE initialized.");
                         register_urls();
                    } else {
                         logger.printfln("EN+ GD EVSE Firmware Version or Hardware is not supported.");
                    }
                }
                break;
            case 0x03:
//W (1970-01-01 00:08:52) [PRIV_COMM, 1919]: Rx(cmd_03 len:24) :  FA 03 00 00 03 27 0E 00 00 09 09 0D 00 00 02 00 00 00 00 00 04 00 80 BC
//W (1970-01-01 00:08:52) [PRIV_COMM, 1764]: Tx(cmd_A3 len:17) :  FA 03 00 00 A3 27 07 00 00 E2 01 01 00 08 34 CF 2F
                evseStatus = PrivCommRxBuffer[9];
                update_evseStatus(evseStatus);
                logger.printfln("   cmd_%.2X seq:%.2X status:%d (%s).", cmd, seq, evseStatus, evse_status_text[evseStatus]);
                PrivCommAck(cmd, PrivCommTxBuffer); // privCommCmdA3StatusAck
                break;
            case 0x04: // time request / ESP32-GD32 communication heartbeat
//W (1970-01-01 00:08:52) [PRIV_COMM, 1919]: Rx(cmd_04 len:16) :  FA 03 00 00 04 28 06 00 09 00 00 00 00 00 C0 5E
//W (1970-01-01 00:08:52) [PRIV_COMM, 1764]: Tx(cmd_A4 len:17) :  FA 03 00 00 A4 28 07 00 00 E2 01 01 00 08 34 E5 6B
                evseStatus = PrivCommRxBuffer[8];
                update_evseStatus(evseStatus);
                logger.printfln("   cmd_%.2X seq:%.2X status:%d %s value:%d  time request / privCommCmdA4HBAck", cmd, seq, evseStatus, evse_status_text[evseStatus], PrivCommRxBuffer[12]);
                {
                    time_t t = now(); // get current time
                    PrivCommTxBuffer[PayloadStart + 0] = 1; // type: 1 = answer ; 0x10 = init
                    PrivCommTxBuffer[PayloadStart + 1] = year(t) -2000;
                    PrivCommTxBuffer[PayloadStart + 2] = month(t);
                    PrivCommTxBuffer[PayloadStart + 3] = day(t);
                    PrivCommTxBuffer[PayloadStart + 4] = hour(t);
                    PrivCommTxBuffer[PayloadStart + 5] = minute(t);
                    PrivCommTxBuffer[PayloadStart + 6] = second(t);
                    PrivCommSend(0xA4, 7, PrivCommTxBuffer); // privCommCmdA4HBAck
                }
                break;
            case 0x05:
//                                                                                        2  0  9  d  e  e  e  1                                                                          10             zeit
//W (2021-08-07 07:43:39) [PRIV_COMM, 1919]: Rx(cmd_05 len:57) :  FA 03 00 00 05 E3 2F 00 32 30 39 64 65 65 65 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10 00 00 00 00 15 08 07 07 2B 26 01 00 00 00 F7 F7
//W (2021-08-07 07:43:39) [EN_WSS, 677]: send[0:41] [2,"87","Authorize",{"idTag":"209deee1"}]
//W (2021-08-07 07:43:39) [EN_WSS, 712]: recv[0:44] [3,"87",{"idTagInfo":{"status":"Accepted"}}]
//                                                                                                               // 40 = accept RFID charging
//                                                                                                                                                                                        40
//W (2021-08-07 07:43:39) [PRIV_COMM, 1764]: Tx(cmd_A5 len:47) :  FA 03 00 00 A5 19 25 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 40 00 00 00 00 D5 41
//
//"idTag":"50a674e1"
//"status":"Invalid"
//                                                                                        5  0  A  6  7  4  E  1                                                                          10             zeit
//W (2021-08-07 07:47:56) [PRIV_COMM, 1919]: Rx(cmd_05 len:57) :  FA 03 00 00 05 FA 2F 00 35 30 61 36 37 34 65 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10 00 00 00 00 15 08 07 07 2F 37 01 00 00 00 2E C2
// D0 = reject RFID charging ?
//                                                                                                                                                                                        D0
//W (2021-08-07 07:47:56) [PRIV_COMM, 1764]: Tx(cmd_A5 len:47) :  FA 03 00 00 A5 1D 25 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D0 00 00 00 00 A4 86
//
		// der GD sendet nut ein cmd_05 wenn er in einem bestimmten mode ist.
		// vor dem cmd_05 sagt er dann: online_net_ok_start (das "create window fail" kommt aber immer)
//[2021-08-07 07:52:33]
//[m1] get_sn->5: 59 50 A6 74 E1
//[2021-08-07 07:52:33]
//[m1] auth_keyA->7: 06 59 FF FF FF FF FF
//[2021-08-07 07:52:33] [win] create window !!!
//[2021-08-07 07:52:33] [win] window had not init, create window fail
//[2021-08-07 07:52:33] online_net_ok_start
//                                                                  5  0  A  6  7  4  E  1                                                                          10             zeit
//[2021-08-07 07:52:33] Tx(cmd_05 len:57) : FA 03 00 00 05 06 2F 00 35 30 61 36 37 34 65 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10 00 00 00 00 15 08 07 07 34 21 01 00 00 00 13 DC
// D0 = reject ?
//[2021-08-07 07:52:34] Rx(cmd_A5 len:47) : FA 03 00 00 A5 20 25 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D0 00 00 00 00 99 D0
//[2021-08-07 07:52:34] cmd_A5 [privCommCmdA5CardAuthAck]!
//[2021-08-07 07:52:34] cmdA5 countsta=1,leakmoneysta=3
//[2021-08-07 07:52:34] cmdA5 order_id=
//[2021-08-07 07:52:34] illegality_card
//

                // Command 05, payload 37 30 38 36 36 31 65 31 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 10 00 00 00 00 00 00 00 00 00 00 00 00 00 00
                sprintf(str, "%s", PrivCommRxBuffer+8);
                logger.printfln("   cmd_%.2X seq:%.2X RFID card detected. ID: %s", cmd, seq, str);
                //Tx(cmd_A5 len:47) :  FA 03 00 00 A5 1D 25 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D0 00 00 00 00 A4 86
		// privCommCmdA5CardAuthAck PrivCommTxBuffer+40 = 0x40; // allow charging
		// privCommCmdA5CardAuthAck PrivCommTxBuffer+40 = 0xD0; // decline charging
		break;
            case 0x06:
		// ack for cmd_A6 srvOcppRemoteStartTransactionReq (triggers cmd_07 ?)
                logger.printfln("   cmd_%.2X seq:%.2X   cp call srv_ocpp ack srv remote ctrl ack", cmd, seq);
//ESP> W (2021-08-22 16:04:37) [EN_WSS, 712]: recv[0:111] [2,"50d4459e-1d1e-42d9-b932-ee2f4784bd38","RemoteStartTransaction",{"connectorId":1,"idTag":"19800020490_APP"}]^M
//------ JSON RemoteStartTransaction --> idTag: 19800020490_APP
//ESP> D (2021-08-22 16:04:37) [MIDDLE_OPT, 285]: esp_mesh_is_root:  hw_opt_read,len:111^M
//ESP> D (2021-08-22 16:04:37) [OCPP_SRV, 3435]: chan[0] uniqId:50d4459e-1d1e-42d9-b932-ee2f4784bd38 actType:20^M
//ESP> D (2021-08-22 16:04:37) [OCPP_SRV, 3442]: srvCallfuns:20 [srvOcppRemoteStartTransactionReq]!^M
//ESP> D (2021-08-22 16:04:37) [OCPP_SRV, 2058]: chan[0] recv srv remote start transaction req info^M
//ESP> D (2021-08-22 16:04:37) [OCPP_SRV, 145]: srvUniqIdListPush add map:  srv:50d4459e-1d1e-42d9-b932-ee2f4784bd38 <==> cp:0x4C^M
//ESP> W (2021-08-22 16:04:37) [PRIV_COMM, 1779]: Tx(cmd_A6 len:83) :  FA 03 00 00 A6 4C 49 00 31 39 38 30 30 30 32 30 34 39 30 5F 41 50 50 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 30 00 00 00 00 00 00 00 00 D0 2A  ^M
//ESP> W (2021-08-22 16:04:38) [PRIV_COMM, 1934]: Rx(cmd_06 len:76) :  FA 03 00 00 06 4C 42 00 31 39 38 30 30 30 32 30 34 39 30 5F 41 50 50 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 30 00 78 37  ^M
//ESP> E (2021-08-22 16:04:38) [PRIV_COMM, 931]: gun_id:0 recv rmt ctrl start charging ack^M
//ESP> D (2021-08-22 16:04:38) [OCPP_SRV, 3648]: ocpp_sevice_ntc_evt: 6, chan:0,sts:8^M
//ESP> D (2021-08-22 16:04:38) [OCPP_SRV, 3121]: chan[0] cp call srv_ocpp ack srv remote ctrl ack^M
//ESP> D (2021-08-22 16:04:38) [OCPP_SRV, 3133]: uid:50d4459e-1d1e-42d9-b932-ee2f4784bd38 actType:20  seqNo:76^M
//ESP> ^M
//ESP> I (2021-08-22 16:04:38) [OCPP_SRV, 3140]: cp[SN10052106061308:1] rmt ctrl(3) succeed^M
//ESP> D (2021-08-22 16:04:38) [MIDDLE_OPT, 306]: esp_mesh_is_root:  hw_opt_send, chan:0^M
//ESP> W (2021-08-22 16:04:38) [EN_WSS, 677]: send[0:64] [3,"50d4459e-1d1e-42d9-b932-ee2f4784bd38",{"status":"Accepted"}]^M
                break;
            case 0x07:
                logger.printfln("   cmd_%.2X seq:%.2X Charging started", cmd, seq);
                  //+"at "+String(message[74]+2000)+"/"+String(message[75])+"/"+String(message[76])+" "+String(message[77])+":"+String(message[78])+":"+String(message[79])
                  //+", startMode(?): "+String(message[73])  // 0:app, 1:card, 2:vin
                  //+", meter start: "+String(message[80]+256*message[81])+"Wh"
                  //);
                break;
            case 0x08:
		//TODO ACK
		//
// 
		//
// W (2021-08-07 07:55:19) [PRIV_COMM, 1764]: Tx(cmd_A8 len:21) :  FA 03 00 00 A8 25 0B 00 40 15 08 07 07 37 13 00 00|[2021-08-07 07:55:18] Rx(cmd_A8 len:21) : FA 03 00 00 A8 25 0B 00 40 15 08 07 07 37 13 00 00 00 00 1B BE 00 00 1B BE                                                                                                      |[2021-08-07 07:55:18] cmd_A8 [privCommCmdA8RTDataAck]!
//D (2021-08-07 07:55:19) [OCPP_SRV, 3550]: ocpp_sevice_ntc_evt: 9, chan:0,sts:8                                    |[2021-08-07 07:55:18] charger A8 settime:21-8-7 7:55:19
//D (2021-08-07 07:55:19) [OCPP_SRV, 3031]: startMode(0:app 1:card 2:vin):1, stopreson:Remote timestamp:2021-08-07T0|[2021-08-07 07:55:19] [comm] cmd03 cpNowSts=0, gunNowSts=1,gunPreSts=0,chargerreson=6
//7:55:17Z idTag:50a674e1                                                                                           |[2021-08-07 07:55:19] Tx(cmd_03 len:24) : FA 03 00 00 03 18 0E 00 80 01 01 06 00 00 00 00 [2021-08-07 07:55:19] [
                if (PrivCommRxBuffer[77] < 10) {  // statistics
                    // TODO is it true that PrivCommRxBuffer[77] is the evseStatus?
                    evseStatus = PrivCommRxBuffer[77];
                    update_evseStatus(evseStatus);
                    logger.printfln("   cmd_%.2X seq:%.2X status:%d (%s)", cmd, seq, evseStatus, evse_status_text[evseStatus]);
                    logger.printfln("\t%dWh\t%d\t%dWh\t%d\t%d\t%d\t%dW\t%d\t%fV\t%fV\t%fV\t%fA\t%d\t%d\t%d\t",
                              PrivCommRxBuffer[84]+256*PrivCommRxBuffer[85],  // charged energy Wh
                              PrivCommRxBuffer[86]+256*PrivCommRxBuffer[87],
                              PrivCommRxBuffer[88]+256*PrivCommRxBuffer[89],  // charged energy Wh
                              PrivCommRxBuffer[90]+256*PrivCommRxBuffer[91],
                              PrivCommRxBuffer[92]+256*PrivCommRxBuffer[93],
                              PrivCommRxBuffer[94]+256*PrivCommRxBuffer[95],
                              PrivCommRxBuffer[96]+256*PrivCommRxBuffer[97],  // charging power
                              PrivCommRxBuffer[98]+256*PrivCommRxBuffer[99],
                              float(PrivCommRxBuffer[100]+256*PrivCommRxBuffer[101]/10),  // L1 plug voltage * 10
                              float(PrivCommRxBuffer[102]+256*PrivCommRxBuffer[103]/10),  // L2 plug voltage * 10 ?
                              float(PrivCommRxBuffer[104]+256*PrivCommRxBuffer[105]/10),  // L3 plug voltage * 10 ?
                              float(PrivCommRxBuffer[106]+256*PrivCommRxBuffer[107]/10),  // charging current * 10
                              PrivCommRxBuffer[108]+256*PrivCommRxBuffer[109],
                              PrivCommRxBuffer[110]+256*PrivCommRxBuffer[111],
                              PrivCommRxBuffer[113]+256*PrivCommRxBuffer[114]
                              );
                } else {
                    logger.printfln("   cmd_%.2X seq:%.2X type:%.2X", cmd, seq, PrivCommRxBuffer[77]);
                    if (PrivCommRxBuffer[77] == 0x10) {  // RFID card
                        String rfid = "";
                        for (int i=0; i<8; i++) {rfid += PrivCommRxBuffer[40 + i];}  // Card number in bytes 40..47
                        logger.printfln("RFID Card %s", rfid);
                    }
                }
                break;
            case 0x09:
                logger.printfln("   cmd_%.2X seq:%.2X Charging stop reason: %d start: %04d/%02d/%02d %02d:%02d:%02d stopp: %04d/%02d/%02d %02d:%02d:%02d meter: %dWh v1: %d v2: %d v3: %d",
                    cmd, seq,
                    PrivCommRxBuffer[77],  // "stopreson": 1 = Remote, 3 = EVDisconnected
                    PrivCommRxBuffer[80]+2000, PrivCommRxBuffer[81], PrivCommRxBuffer[82], PrivCommRxBuffer[83], PrivCommRxBuffer[84], PrivCommRxBuffer[85],
                    PrivCommRxBuffer[86]+2000, PrivCommRxBuffer[87], PrivCommRxBuffer[88], PrivCommRxBuffer[89], PrivCommRxBuffer[90], PrivCommRxBuffer[91],
                    PrivCommRxBuffer[96]+256*PrivCommRxBuffer[97],
                    PrivCommRxBuffer[78]+256*PrivCommRxBuffer[79],
                    PrivCommRxBuffer[92]+256*PrivCommRxBuffer[93],
                    PrivCommRxBuffer[94]+256*PrivCommRxBuffer[95]);
                PrivCommAck(cmd, PrivCommTxBuffer); // privCommCmdA9RecordAck
                break;
            case 0x0A:
                switch( PrivCommRxBuffer[9] ) { // 8: always 14, 9: answertype?
                    case 0x02: // answer to set time
//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:20) :  FA 03 00 00 AA 09 0A 00 18 02 06 00 15 04 0B 12 24 1B 5C 78
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:20) :  FA 03 00 00 0A 09 0A 00 14 02 06 00 15 04 0B 12 24 1B 3C E7
//I (2021-04-11 18:36:27) [PRIV_COMM, 94]: ctrl_cmd set time done -> time: 2021-04-11 18:36:27
    // ctrl_cmd set start power mode done
                        logger.printfln("   cmd_%.2X seq:%.2X Set Time done", cmd, seq);
                        break;
                    case 0x08: // answer to set hb timeout
//W (1970-01-01 00:08:53) [PRIV_COMM, 1764]: Tx(cmd_AA len:16) :  FA 03 00 00 AA 07 06 00 18 08 02 00 1E 00 95 80
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:16) :  FA 03 00 00 0A 07 06 00 14 08 02 00 1E 00 93 CE
//I (2021-04-11 18:36:27) [PRIV_COMM, 249]: ctrl_cmd set heart beat time out done -> 30      (=1E)
//
// [2021-08-07 07:55:18] Rx(cmd_A8 len:21) : FA 03 00 00 A8 25 0B 00 40 15 08 07 07 37 13 00 00 00 00 1B BE
// [2021-08-07 07:55:18] cmd_A8 [privCommCmdA8RTDataAck]!
// [2021-08-07 07:55:18] charger A8 settime:21-8-7 7:55:19
//
                        logger.printfln("   cmd_%.2X seq:%.2X Heartbeat Timeout:%ds", cmd, seq, PrivCommRxBuffer[12]);
                        break;
                    case 0x09: // answer to ctrl_cmd set start power mode
//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 40 05 00 18 09 01 00 00 F9 36
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 40 05 00 14 09 01 00 00 11 30
//I (2021-04-11 18:36:27) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower:  3.150.080

//W (2021-04-11 18:36:30) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 42 05 00 18 09 01 00 00 78 EF
//W (2021-04-11 18:36:31) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 42 05 00 14 09 01 00 00 90 E9
//I (2021-04-11 18:36:31) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower: 15.306.752
                        logger.printfln("   cmd_%.2X seq:%.2X ctrl_cmd set start power mode done", cmd, seq);
                        break;
                    case 0x12: // ctrl_cmd set ack done, type:0
//W (1970-01-01 00:08:53) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 08 05 00 18 12 01 00 03 BA 45
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 08 05 00 14 12 01 00 00 12 42
//I (2021-04-11 18:36:27) [PRIV_COMM, 51]: ctrl_cmd set ack done, type:0
                        logger.printfln("   cmd_%.2X seq:%.2X ctrl_cmd set ack done, type:0", cmd, seq);
                        break;
                    case 0x2A: // answer to cmdAACtrlcantestsetAck test cancom...111
//W (1970-01-01 00:00:03) [PRIV_COMM, 1764]: Tx(cmd_AA len:14) :  FA 03 00 00 AA 02 04 00 18 2A 00 00 DB 76
//W (1970-01-01 00:00:03) [PRIV_COMM, 1919]: Rx(cmd_0A len:14) :  FA 03 00 00 0A 02 04 00 14 2A 00 00 D2 5E
//E (1970-01-01 00:00:03) [PRIV_COMM, 78]: cmdAACtrlcantestsetAck test cancom...111
    // cmdAACtrlcantestsetAck test cancom...111
                        logger.printfln("   cmd_%.2X seq:%.2X cmdAACtrlcantestsetAck test cancom...111 done", cmd, seq);
                        break;
                    default:
                        logger.printfln("   cmd_%.2X seq:%.2X I don't know what %.2X means.", cmd, seq, PrivCommRxBuffer[9]);
                        break;
                }//switch cmdAA answer processing
                break;
            case 0x0E:
// [2021-08-07 07:55:05] Tx(cmd_0E len:76) : FA 03 00 00 0E 11 42 00 00 00 00 00 00 00 00 00 00 0A 01 77 02 37 35 32 30 33 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 66 08 97 08 14 00 7A 01 01 00 00 00 00 00 00 00 00 00 00 DE 91
// [2021-08-07 07:55:05] cmd0E_DutyData pwmMax:266
                logger.printfln("   cmd_%.2X seq:%.2X duty:%d cpVolt:%d power factors:%d/%d/%d/%d offset0:%d offset1:%d leakcurr:%d AMBTemp:%d lockstatus:%d",
                    cmd, seq,
                    PrivCommRxBuffer[17]+256*PrivCommRxBuffer[18],
                    PrivCommRxBuffer[19]+256*PrivCommRxBuffer[20],
                    PrivCommRxBuffer[9]+256*PrivCommRxBuffer[10],PrivCommRxBuffer[11]+256*PrivCommRxBuffer[12],PrivCommRxBuffer[13]+256*PrivCommRxBuffer[14],PrivCommRxBuffer[15]+256*PrivCommRxBuffer[16],
                    PrivCommRxBuffer[55]+256*PrivCommRxBuffer[56],
                    PrivCommRxBuffer[57]+256*PrivCommRxBuffer[58],
                    PrivCommRxBuffer[59]+256*PrivCommRxBuffer[60],
                    PrivCommRxBuffer[61]+256*PrivCommRxBuffer[62],
                    PrivCommRxBuffer[63]);
                //PrivCommAck(cmd, PrivCommTxBuffer); // Ack?
                break;
            default:
                logger.printfln("   cmd_%.2X seq:%.2X I don't know what to do about it.", cmd, seq);
                break;
        }//switch process cmd
        cmd_to_process = false;
    }

    evse_state.get("time_since_state_change")->updateUint(millis() - evse_state.get("last_state_change")->asUint());
}

void ENplus::setup_evse()
{
    Serial2.begin(115200, SERIAL_8N1, 26, 27); // PrivComm to EVSE GD32 Chip
    Serial2.setRxBufferSize(1024);
    Serial2.setTimeout(90);
    logger.printfln("Set up PrivComm: 115200, SERIAL_8N1, RX 26, TX 27, timeout 90ms");


    // TODO start: look out for this on a unconfigured box ( no wifi ) - if it still works, delete the code
    setTime(23,59,00,31,12,2018);
    switch (timeStatus()){
        case timeNotSet:
            logger.printfln("the time has never been set, the clock started on Jan 1, 1970");
            break;
        case timeNeedsSync:
            logger.printfln("the time had been set but a sync attempt did not succeed");
            break;
        case timeSet:
            logger.printfln("the time is set and is synced");
            break;
    }
    logger.printfln("the time is %d", now());
    logger.printfln("the now() call was not blocking");
    // TODO end: look out for this on a unconfigured box ( no wifi ) - if it still works, delete the code


//W (1970-01-01 00:08:53) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 08 05 00 18 12 01 00 03 BA 45
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 08 05 00 14 12 01 00 00 12 42
//I (2021-04-11 18:36:27) [PRIV_COMM, 51]: ctrl_cmd set ack done, type:0

    // ctrl_cmd set ack done, type:0 // this triggers 0x02 SN, Hardware, Version
    sendCommand(Init12, sizeof(Init12));


//W (1970-01-01 00:08:53) [PRIV_COMM, 1764]: Tx(cmd_AA len:16) :  FA 03 00 00 AA 07 06 00 18 08 02 00 1E 00 95 80
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:16) :  FA 03 00 00 0A 07 06 00 14 08 02 00 1E 00 93 CE
//I (2021-04-11 18:36:27) [PRIV_COMM, 249]: ctrl_cmd set heart beat time out done -> 30      (=1E)

    // ctrl_cmd set heart beat time out
    PrivCommTxBuffer[PayloadStart + 0] = 0x18;
    PrivCommTxBuffer[PayloadStart + 1] = 0x08;
    PrivCommTxBuffer[PayloadStart + 2] = 0x02;
    PrivCommTxBuffer[PayloadStart + 3] = 0x00;
    PrivCommTxBuffer[PayloadStart + 4] =  120; // 120 sec hb timeout
    PrivCommTxBuffer[PayloadStart + 5] = 0x00; // hb timeout 16bit?
    PrivCommSend(0xAA, 6, PrivCommTxBuffer);


//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:20) :  FA 03 00 00 AA 09 0A 00 18 02 06 00 15 04 0B 12 24 1B 5C 78
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:20) :  FA 03 00 00 0A 09 0A 00 14 02 06 00 15 04 0B 12 24 1B 3C E7
//I (2021-04-11 18:36:27) [PRIV_COMM, 94]: ctrl_cmd set time done -> time: 2021-04-11 18:36:27

    sendTimeLong();


//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 40 05 00 18 09 01 00 00 F9 36
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 40 05 00 14 09 01 00 00 11 30
//I (2021-04-11 18:36:27) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower:  3.150.080

//W (2021-04-11 18:36:30) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 42 05 00 18 09 01 00 00 78 EF
//W (2021-04-11 18:36:31) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 42 05 00 14 09 01 00 00 90 E9
//I (2021-04-11 18:36:31) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower: 15.306.752

    // ctrl_cmd set start power mode done
    sendCommand(Init15, sizeof(Init15));


//W (1970-01-01 00:00:03) [PRIV_COMM, 1764]: Tx(cmd_AA len:14) :  FA 03 00 00 AA 02 04 00 18 2A 00 00 DB 76
//W (1970-01-01 00:00:03) [PRIV_COMM, 1919]: Rx(cmd_0A len:14) :  FA 03 00 00 0A 02 04 00 14 2A 00 00 D2 5E
//E (1970-01-01 00:00:03) [PRIV_COMM, 78]: cmdAACtrlcantestsetAck test cancom...111

    // cmdAACtrlcantestsetAck test cancom...111
    sendCommand(Init11, sizeof(Init11));

/*
    do { // wait for the first PRIVCOMM signal to decide if we have a GD chip to talk to
        logger.printfln("wait for PrivComm");
        if (Serial2.available() == 0) { delay(250); }
    } while(Serial2.available() == 0 && millis()<10000); // TODO disable EVSE in case of no show
*/


    //int result = tf_evse_create(&evse, uid, &hal);
//    if(result != TF_E_OK) {
//        logger.printfln("Failed to initialize EVSE bricklet. Disabling EVSE support.");
//        return;
//    }

    initialized = true;
}

void ENplus::update_evse_low_level_state() {
    if(!initialized)
        return;

    bool low_level_mode_enabled;
    uint8_t led_state;
    uint16_t cp_pwm_duty_cycle;

    uint16_t adc_values[2];
    int16_t voltages[3];
    uint32_t resistances[2];
    bool gpio[5];

//    int rc = tf_evse_get_low_level_state(&evse,
//        &low_level_mode_enabled,
//        &led_state,
//        &cp_pwm_duty_cycle,
//        adc_values,
//        voltages,
//        resistances,
//        gpio);

        low_level_mode_enabled = true;
        led_state = 1;
        cp_pwm_duty_cycle = 100;
        adc_values[0] = 200;
        adc_values[1] = 201;
        voltages[0] = 300;
        voltages[1] = 301;
        voltages[2] = 302;
        resistances[0] = 400;
        resistances[1] = 401;
        gpio[0] = false;
        gpio[1] = false;
        gpio[2] = false;
        gpio[3] = false;
        gpio[4] = false;

    evse_low_level_state.get("low_level_mode_enabled")->updateBool(low_level_mode_enabled);
    evse_low_level_state.get("led_state")->updateUint(led_state);
    evse_low_level_state.get("cp_pwm_duty_cycle")->updateUint(cp_pwm_duty_cycle);

    for(int i = 0; i < sizeof(adc_values)/sizeof(adc_values[0]); ++i)
        evse_low_level_state.get("adc_values")->get(i)->updateUint(adc_values[i]);

    for(int i = 0; i < sizeof(voltages)/sizeof(voltages[0]); ++i)
        evse_low_level_state.get("voltages")->get(i)->updateInt(voltages[i]);

    for(int i = 0; i < sizeof(resistances)/sizeof(resistances[0]); ++i)
        evse_low_level_state.get("resistances")->get(i)->updateUint(resistances[i]);

    for(int i = 0; i < sizeof(gpio)/sizeof(gpio[0]); ++i)
        evse_low_level_state.get("gpio")->get(i)->updateBool(gpio[i]);
}

void ENplus::update_evse_state() {
    if(!initialized)
        return;
    uint8_t iec61851_state, vehicle_state, contactor_state, contactor_error, charge_release, error_state, lock_state;
    uint16_t last_allowed_charging_current = evse_state.get("allowed_charging_current")->asUint();
    uint16_t allowed_charging_current;
    uint32_t time_since_state_change, uptime;

    int rc = bs_evse_get_state(&evse,
        &iec61851_state,
        &vehicle_state,
        &contactor_state,
        &contactor_error,
        &charge_release,
        &allowed_charging_current,
        &error_state,
        &lock_state,
        &time_since_state_change,
        &uptime);

    firmware_update_allowed = vehicle_state == 0;

    evse_state.get("iec61851_state")->updateUint(iec61851_state);
    evse_state.get("vehicle_state")->updateUint(vehicle_state);
//logger.printfln("EVSE: vehicle_state %d", vehicle_state);
    evse_state.get("contactor_state")->updateUint(contactor_state);
    bool contactor_error_changed = evse_state.get("contactor_error")->updateUint(contactor_error);
    evse_state.get("charge_release")->updateUint(charge_release);
    if(last_allowed_charging_current != allowed_charging_current) {
        evse_state.get("allowed_charging_current")->updateUint(allowed_charging_current);
        logger.printfln("EVSE: allowed_charging_current %d", allowed_charging_current);
        time_t t=now();
        ChargingSettings[2] = year(t) -2000;
        ChargingSettings[3] = month(t);
        ChargingSettings[4] = day(t);
        ChargingSettings[5] = hour(t);
        ChargingSettings[6] = minute(t);
        ChargingSettings[7] = second(t);
        ChargingSettings[17] = uint8_t(allowed_charging_current/1000);
        sendCommand(ChargingSettings, sizeof(ChargingSettings));
    }
    bool error_state_changed = evse_state.get("error_state")->updateUint(error_state);
    evse_state.get("lock_state")->updateUint(lock_state);
    //evse_state.get("time_since_state_change")->updateUint(time_since_state_change);
    evse_state.get("uptime")->updateUint(uptime);
}

void ENplus::update_evse_charge_stats() {
    if(!initialized)
        return;

    // trigger status updates from the GD, process them in the regular loop
    if(evse_state.get("iec61851_state")->asUint() == 2) { // if charging
        // TODO this do not seem to work :-( and would be strange if it would
        //PrivCommAck(0x02, PrivCommTxBuffer); // privCommCmdA2InfoSynAck  A2 request status, triggers 03 and 08 answers
        //sendCommand(Init13, sizeof(Init13)); // as well A2
        // TODO try this
        //sendRequest0E(sendSequence++);  // send A8 40 time 00 00 00 00: trigger 0E answer if charging
    }
}

void ENplus::update_evseStatus(uint8_t evseStatus) {
    uint8_t last_iec61851_state = evse_state.get("iec61851_state")->asUint();
    uint8_t last_evseStatus = evse_state.get("GD_state")->asUint();
    evse_state.get("GD_state")->updateUint(evseStatus);
    switch (evseStatus) {
        case 1:                                              // Available (not engaged)
            evse_state.get("iec61851_state")->updateUint(0); // Nicht verbunden (Sicht des Fahrzeugs)
            break;
        case 2:                                              // Preparing (engaged, not started)
            evse_state.get("iec61851_state")->updateUint(1); // Verbunden
            break;
        case 3:                                              // Charging (charging ongoing, power output)
            evse_state.get("iec61851_state")->updateUint(2); // Lädt
            break;
        case 4:                                              // Suspended by charger (started but no power available)
            evse_state.get("iec61851_state")->updateUint(1); // Verbunden
            break;
        case 5:                                              // Suspended by EV (power available but waiting for the EV response)
            evse_state.get("iec61851_state")->updateUint(1); // Verbunden
            break;
        case 6:                                              // Finishing, charging acomplished (RFID stop or EMS control stop)
            evse_state.get("iec61851_state")->updateUint(1); // Verbunden
            break;
        case 7:                                              // (Reserved)
            evse_state.get("iec61851_state")->updateUint(4);
            break;
        case 8:                                              // (Unavailable)
            evse_state.get("iec61851_state")->updateUint(4);
            break;
        case 9:                                              // Fault (charger in fault condition)
            evse_state.get("iec61851_state")->updateUint(4);
            break;
        default:
            logger.printfln("err: can not determine EVSE status %d", evseStatus);
            break;
    }
    if(last_iec61851_state != evse_state.get("iec61851_state")->asUint()) {
        evse_state.get("last_state_change")->updateUint(millis());
        evse_state.get("time_since_state_change")->updateUint(millis() - evse_state.get("last_state_change")->asUint());
	if(evse_auto_start_charging.get("auto_start_charging")->asBool()
           && evseStatus == 2 || (evseStatus == 6 && last_evseStatus == 0)) { // just plugged in or already plugged in at startup
            logger.printfln("Start charging automatically");
            bs_evse_start_charging(&evse);
        }
    }
    evse_state.get("vehicle_state")->updateUint(evse_state.get("iec61851_state")->asUint());
}

void ENplus::update_evse_user_calibration() {
    if(!initialized)
        return;

    bool user_calibration_active;
    int16_t voltage_diff, voltage_mul, voltage_div, resistance_2700, resistance_880[14];

//    int rc = tf_evse_get_user_calibration(&evse,
//        &user_calibration_active,
//        &voltage_diff,
//        &voltage_mul,
//        &voltage_div,
//        &resistance_2700,
//        resistance_880);
//
//    evse_user_calibration.get("user_calibration_active")->updateBool(user_calibration_active);
//    evse_user_calibration.get("voltage_diff")->updateInt(voltage_diff);
//    evse_user_calibration.get("voltage_mul")->updateInt(voltage_mul);
//    evse_user_calibration.get("voltage_div")->updateInt(voltage_div);
//    evse_user_calibration.get("resistance_2700")->updateInt(resistance_2700);
//
//    for(int i = 0; i < sizeof(resistance_880)/sizeof(resistance_880[0]); ++i)
//        evse_user_calibration.get("resistance_880")->get(i)->updateInt(resistance_880[i]);
}

bool ENplus::is_in_bootloader(int rc) {
    return false;
}
