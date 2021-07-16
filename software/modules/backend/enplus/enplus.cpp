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

#include "enplus.h"
#include "enplus_firmware.h"

#include "bindings/errors.h"

#include "api.h"
#include "event_log.h"
#include "task_scheduler.h"
#include "tools.h"
#include "modules/sse/sse.h"
#include "HardwareSerial.h"
#include "Time/TimeLib.h"

extern EventLog logger;

extern TaskScheduler task_scheduler;
extern TF_HalContext hal;
extern AsyncWebServer server;
extern Sse sse;

extern API api;
extern bool firmware_update_allowed;

static const uint16_t crc16_modbus_table[] = {
       0X0000, 0XC0C1, 0XC181, 0X0140, 0XC301, 0X03C0, 0X0280, 0XC241,
       0XC601, 0X06C0, 0X0780, 0XC741, 0X0500, 0XC5C1, 0XC481, 0X0440,
       0XCC01, 0X0CC0, 0X0D80, 0XCD41, 0X0F00, 0XCFC1, 0XCE81, 0X0E40,
       0X0A00, 0XCAC1, 0XCB81, 0X0B40, 0XC901, 0X09C0, 0X0880, 0XC841,
       0XD801, 0X18C0, 0X1980, 0XD941, 0X1B00, 0XDBC1, 0XDA81, 0X1A40,
       0X1E00, 0XDEC1, 0XDF81, 0X1F40, 0XDD01, 0X1DC0, 0X1C80, 0XDC41,
       0X1400, 0XD4C1, 0XD581, 0X1540, 0XD701, 0X17C0, 0X1680, 0XD641,
       0XD201, 0X12C0, 0X1380, 0XD341, 0X1100, 0XD1C1, 0XD081, 0X1040,
       0XF001, 0X30C0, 0X3180, 0XF141, 0X3300, 0XF3C1, 0XF281, 0X3240,
       0X3600, 0XF6C1, 0XF781, 0X3740, 0XF501, 0X35C0, 0X3480, 0XF441,
       0X3C00, 0XFCC1, 0XFD81, 0X3D40, 0XFF01, 0X3FC0, 0X3E80, 0XFE41,
       0XFA01, 0X3AC0, 0X3B80, 0XFB41, 0X3900, 0XF9C1, 0XF881, 0X3840,
       0X2800, 0XE8C1, 0XE981, 0X2940, 0XEB01, 0X2BC0, 0X2A80, 0XEA41,
       0XEE01, 0X2EC0, 0X2F80, 0XEF41, 0X2D00, 0XEDC1, 0XEC81, 0X2C40,
       0XE401, 0X24C0, 0X2580, 0XE541, 0X2700, 0XE7C1, 0XE681, 0X2640,
       0X2200, 0XE2C1, 0XE381, 0X2340, 0XE101, 0X21C0, 0X2080, 0XE041,
       0XA001, 0X60C0, 0X6180, 0XA141, 0X6300, 0XA3C1, 0XA281, 0X6240,
       0X6600, 0XA6C1, 0XA781, 0X6740, 0XA501, 0X65C0, 0X6480, 0XA441,
       0X6C00, 0XACC1, 0XAD81, 0X6D40, 0XAF01, 0X6FC0, 0X6E80, 0XAE41,
       0XAA01, 0X6AC0, 0X6B80, 0XAB41, 0X6900, 0XA9C1, 0XA881, 0X6840,
       0X7800, 0XB8C1, 0XB981, 0X7940, 0XBB01, 0X7BC0, 0X7A80, 0XBA41,
       0XBE01, 0X7EC0, 0X7F80, 0XBF41, 0X7D00, 0XBDC1, 0XBC81, 0X7C40,
       0XB401, 0X74C0, 0X7580, 0XB541, 0X7700, 0XB7C1, 0XB681, 0X7640,
       0X7200, 0XB2C1, 0XB381, 0X7340, 0XB101, 0X71C0, 0X7080, 0XB041,
       0X5000, 0X90C1, 0X9181, 0X5140, 0X9301, 0X53C0, 0X5280, 0X9241,
       0X9601, 0X56C0, 0X5780, 0X9741, 0X5500, 0X95C1, 0X9481, 0X5440,
       0X9C01, 0X5CC0, 0X5D80, 0X9D41, 0X5F00, 0X9FC1, 0X9E81, 0X5E40,
       0X5A00, 0X9AC1, 0X9B81, 0X5B40, 0X9901, 0X59C0, 0X5880, 0X9841,
       0X8801, 0X48C0, 0X4980, 0X8941, 0X4B00, 0X8BC1, 0X8A81, 0X4A40,
       0X4E00, 0X8EC1, 0X8F81, 0X4F40, 0X8D01, 0X4DC0, 0X4C80, 0X8C41,
       0X4400, 0X84C1, 0X8581, 0X4540, 0X8701, 0X47C0, 0X4680, 0X8641,
       0X8201, 0X42C0, 0X4380, 0X8341, 0X4100, 0X81C1, 0X8081, 0X4040
};

// evse GD status strings
    const char *cmd_03_status[] = {
       "undefined",
       "Available (not engaged)",
       "Preparing (engaged, not started)",
       "Charging (charging ongoing, power output)",
       "Suspended by charger (started but no power available)",
       "Suspended by EV (power available but waiting for the EV response)",
       "Finishing, charging acomplished (RFID stop or EMS control stop",
       "(Reserved)",
       "(Unavailable)",
       "Fault (charger in fault condition)",
    };

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
byte Init10[] = {0xAA, 0x18, 0x12, 0x01, 0x00, 0x03, 0x3B, 0x9C};
byte Init11[] = {0xAA, 0x18, 0x2A, 0x00, 0x00};
byte Init12[] = {0xAA, 0x18, 0x12, 0x01, 0x00, 0x03}; // ?   x mal
//ack //byte Init13[] = {0xA2, 0x00};
//ack TODO //byte Init14[] = {0xA3, 0x18, 0x02, 0x06, 0x00, 0x15, 0x06, 0x0A, 0x07, 0x08, 0x26};
byte Init15[] = {0xAA, 0x18, 0x09, 0x01, 0x00, 0x00};

//byte StartCharging[] = {0xA7, 0x36, 0x32, 0x33, 0x33, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
byte StartCharging[] = {0xA7, 0x53, 0x6e, 0x69, 0x66, 0x66, 0x65, 0x72, 0x20, 0x63, 0x68, 0x61, 0x72, 0x67, 0x69, 0x6E, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
//byte StopCharging[] = {0xA7, 0x36, 0x32, 0x33, 0x33, 0x33, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00};
byte StopCharging[] = {0xA7, 0x53, 0x6e, 0x69, 0x66, 0x66, 0x65, 0x72, 0x20, 0x63, 0x68, 0x61, 0x72, 0x67, 0x69, 0x6E, 0x67, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x00};
//byte ChargingSettings[] = {0xAD, 0x00, 0x44, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x15, 0x06, 0x06, 0x0B, 0x05, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
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
    uint16_t len = (uint16_t)(data[7] << 8 | data[6]) + 10; // payload length + 10 bytes header and crc
    if (len > 1000) { // mqtt buffer is the limiting factor
        logger.printfln("ERROR: buffer (%d bytes) too big for mqtt buffer (max 1000).", len);
        len = 1000;
    }

    int offset = 0;
    for(uint32_t i = 0; i < len; i++) {
        #define BUFFER_CHUNKS 20
        if(i > 0 && i % BUFFER_CHUNKS == 0) {
            logger.printfln("privcomm: %s", PrivCommHexBuffer + offset);
            offset = offset + 3*BUFFER_CHUNKS;
        }
        snprintf(PrivCommHexBuffer + 3*i, sizeof(PrivCommHexBuffer)/sizeof(PrivCommHexBuffer[0]), "%.2X ", data[i]);
    }
    logger.printfln("privcomm: %s", PrivCommHexBuffer + offset);
    return String(PrivCommHexBuffer);
}

void ENplus::Serial2write(byte *data, int size) {
    int bytes_to_send = size;
    int offset = 0;
    while(bytes_to_send > 0) {
        int afw = Serial2.availableForWrite();
        if(afw < bytes_to_send) { // send chunk
            bytes_to_send = bytes_to_send - Serial2.write(PrivCommTxBuffer + offset, afw);
            //logger.printfln("    send: Tx afw:%d bytes_to_send:%d offset:%d", afw, bytes_to_send, offset);
            offset = offset + afw;
        } else { // send reminder
            bytes_to_send = bytes_to_send - Serial2.write(PrivCommTxBuffer + offset, bytes_to_send);
            //logger.printfln("    sEND: Tx afw:%d bytes_to_send:%d offset:%d", afw, bytes_to_send, offset);
        }
    }
}

void ENplus::sendCommand(byte *data, int datasize) {
    PrivCommTxBuffer[4] = data[0]; // command code
    PrivCommTxBuffer[5]++; // increment sequence number
    PrivCommTxBuffer[6] = (datasize-1) & 0xFF;
    PrivCommTxBuffer[7] = (datasize-1) >> 8;
    for (int i=1; i<=datasize; i++) {
        PrivCommTxBuffer[i+7] = data[i];
    }

    uint16_t crc = crc16_modbus(PrivCommTxBuffer, datasize + 7);

    PrivCommTxBuffer[datasize+7] = crc & 0xFF;
    PrivCommTxBuffer[datasize+8] = crc >> 8;

    get_hex_privcomm_line(PrivCommTxBuffer); // PrivCommHexBuffer now holds the hex representation of the buffer
    logger.printfln("Tx cmd_%.2X seq:%.2X, len:%d, crc:%.4X", PrivCommTxBuffer[4], PrivCommTxBuffer[5], datasize+9, crc);

    Serial2write(data, datasize + 9);
    evse_privcomm.get("TX")->updateString(PrivCommHexBuffer);

    PrivCommTxBuffer[5]++; // increment sequence number
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


void ENplus::sendTimeLong (void) {
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
    evse_state = Config::Object({
        {"iec61851_state", Config::Uint8(0)},
        {"vehicle_state", Config::Uint8(0)},
        {"contactor_state", Config::Uint8(0)},
        {"contactor_error", Config::Uint8(0)},
        {"charge_release", Config::Uint8(0)},
        {"allowed_charging_current", Config::Uint16(0)},
        {"error_state", Config::Uint8(0)},
        {"lock_state", Config::Uint8(0)},
        {"time_since_state_change", Config::Uint32(0)},
        {"uptime", Config::Uint32(0)}
    });

    evse_privcomm = Config::Object({
        {"RX", Config::Str("",1000)},
        {"TX", Config::Str("",1000)}
    });

    evse_hardware_configuration = Config::Object({
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
        //{"max_current_managed", Config::Uint16(0)},
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
/*
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
*/
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
    charging = true;
    logger.printfln("EVSE start charging");
    sendCommand(StartCharging, sizeof(StartCharging));
    return 0;
}

int ENplus::bs_evse_stop_charging(TF_EVSE *evse) {
    charging = false;
    logger.printfln("EVSE stop charging");
    sendCommand(StopCharging, sizeof(StopCharging));
    return 0;
}

int ENplus::bs_evse_set_max_charging_current(TF_EVSE *evse, uint16_t max_current) {
    logger.printfln("EVSE set charging limit to %d Ampere.", uint8_t(max_current/1000));
    evse_max_charging_current.get("max_current_configured")->updateUint(max_current);

    time_t t=now();     // get current time
    ChargingSettings[55] = year(t) -2000;
    ChargingSettings[56] = month(t);
    ChargingSettings[57] = day(t);
    ChargingSettings[58] = hour(t);
    ChargingSettings[59] = minute(t);
    ChargingSettings[60] = second(t);
    ChargingSettings[65] = uint8_t(max_current/1000);
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
//    allowed_charging_current = min(
//        allowed_charging_current,
//        evse_max_charging_current.get("max_current_managed")->asUint());
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

    task_scheduler.scheduleWithFixedDelay("update_evse_state", [this](){
        update_evse_state();
    }, 0, 1000);

    task_scheduler.scheduleWithFixedDelay("update_evse_low_level_state", [this](){
        update_evse_low_level_state();
    }, 0, 1000);

    task_scheduler.scheduleWithFixedDelay("update_evse_max_charging_current", [this](){
        update_evse_max_charging_current();
    }, 0, 1000);

    task_scheduler.scheduleWithFixedDelay("update_evse_auto_start_charging", [this](){
        update_evse_auto_start_charging();
    }, 0, 1000);

    /*task_scheduler.scheduleWithFixedDelay("update_evse_managed", [this](){
        update_evse_managed();
    }, 0, 1000);*/

    /*task_scheduler.scheduleWithFixedDelay("update_evse_user_calibration", [this](){
        update_evse_user_calibration();
    }, 0, 10000);*/
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

void ENplus::register_urls()
{
    if (!evse_found)
        return;

    api.addState("evse/state", &evse_state, {}, 1000);
    api.addState("evse/hardware_configuration", &evse_hardware_configuration, {}, 1000);
    api.addState("evse/low_level_state", &evse_low_level_state, {}, 1000);
    api.addState("evse/max_charging_current", &evse_max_charging_current, {}, 1000);
    api.addState("evse/auto_start_charging", &evse_auto_start_charging, {}, 1000);
    api.addState("evse/privcomm", &evse_privcomm, {}, 1000);

    api.addCommand("evse/auto_start_charging_update", &evse_auto_start_charging_update, {}, [this](){
        is_in_bootloader(tf_evse_set_charging_autostart(&evse, evse_auto_start_charging_update.get("auto_start_charging")->asBool()));
    }, false);

    api.addCommand("evse/current_limit", &evse_current_limit, {}, [this](){
        is_in_bootloader(bs_evse_set_max_charging_current(&evse, evse_current_limit.get("current")->asUint()));
    }, false);

    api.addCommand("evse/stop_charging", &evse_stop_charging, {}, [this](){bs_evse_stop_charging(&evse);}, true);
    api.addCommand("evse/start_charging", &evse_start_charging, {}, [this](){bs_evse_start_charging(&evse);}, true);
/*
    api.addCommand("evse/managed_current_update", &evse_managed_current, {}, [this](){
        is_in_bootloader(tf_evse_set_managed_current(&evse, evse_managed_current.get("current")->asUint()));
    }, true);

    api.addState("evse/managed", &evse_managed, {}, 1000);
    api.addCommand("evse/managed_update", &evse_managed_update, {"password"}, [this](){
        is_in_bootloader(tf_evse_set_managed(&evse, evse_managed_update.get("managed")->asBool(), evse_managed_update.get("password")->asUint()));
    }, true);
*/

    api.addState("evse/user_calibration", &evse_user_calibration, {}, 1000);
    api.addCommand("evse/user_calibration_update", &evse_user_calibration, {}, [this](){
        int16_t resistance_880[14];
        evse_user_calibration.get("resistance_880")->fillArray<int16_t, Config::ConfInt>(resistance_880, sizeof(resistance_880)/sizeof(resistance_880[0]));

        is_in_bootloader(tf_evse_set_user_calibration(&evse,
            0xCA11B4A0,
            evse_user_calibration.get("user_calibration_active")->asBool(),
            evse_user_calibration.get("voltage_diff")->asInt(),
            evse_user_calibration.get("voltage_mul")->asInt(),
            evse_user_calibration.get("voltage_div")->asInt(),
            evse_user_calibration.get("resistance_2700")->asInt(),
            resistance_880
            ));
    }, true);

    server.on("/evse/start_debug", HTTP_GET, [this](AsyncWebServerRequest *request) {
        task_scheduler.scheduleOnce("enable evse debug", [this](){
            sse.pushStateUpdate(this->get_evse_debug_header(), "evse/debug_header");
            debug = true;
        }, 0);
        request->send(200);
    });

    server.on("/evse/stop_debug", HTTP_GET, [this](AsyncWebServerRequest *request){
        task_scheduler.scheduleOnce("enable evse debug", [this](){
            debug = false;
        }, 0);
        request->send(200);
    });
}

void ENplus::loop()
{
    static uint32_t last_check = 0;
    static uint32_t last_debug = 0;
    static uint32_t nextMillis = 2000;
    static uint32_t last_state_change = millis();
    static uint8_t last_iec61851_state = 0;
    static uint32_t nextCommand = 12; // Start initialization with Init12 command
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
        if(!is_in_bootloader(TF_E_TIMEOUT))
            setup_evse();
    }

    if(debug && deadline_elapsed(last_debug + 50)) {
        last_debug = millis();
        sse.pushStateUpdate(this->get_evse_debug_line(), "evse/debug");
    }

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
                                register_urls();
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

    if(cmd_to_process) {
        switch( cmd ) {
            case 0x02: // Info: Serial number, Version
                logger.printfln("   cmd_%.2X seq:%.2X  Ack Serial number and Version.", cmd, seq);
                PrivCommAck(cmd, PrivCommTxBuffer); // privCommCmdA2InfoSynAck
                // TODO extract relevant data
                logger.printfln("EVSE SN: %s hw: %s fw: %s", PrivCommRxBuffer+8, PrivCommRxBuffer+43, PrivCommRxBuffer+91);
//                int result = ensure_matching_firmware(&hal, uid, "EVSE", "EVSE", evse_firmware_version, / *evse_bricklet_firmware_bin, evse_bricklet_firmware_bin_len,* / &logger);
//                if(result != 0) {
//                    return;
//                }
    //ctrl_cmd set heart beat time out
    PrivCommTxBuffer[PayloadStart + 0] = 0x18;
    PrivCommTxBuffer[PayloadStart + 1] = 0x08;
    PrivCommTxBuffer[PayloadStart + 2] = 0x02;
    PrivCommTxBuffer[PayloadStart + 3] = 0x00;
    PrivCommTxBuffer[PayloadStart + 4] = 0x1E; // 1E = 30 sec hb timeout, C8 = 200 sec
    PrivCommTxBuffer[PayloadStart + 5] = 0x00; // hb timeout 16bit?
    PrivCommSend(0xAA, 6, PrivCommTxBuffer);
                break;
            case 0x03:
                if(last_iec61851_state != PrivCommRxBuffer[9]) {
                    last_iec61851_state = PrivCommRxBuffer[9];
                    last_state_change = millis();
                }
                switch (PrivCommRxBuffer[9]) { // status
                    // TODO adapt to EN+ states in web interface
                    case 1:
                        evse_state.get("iec61851_state")->updateUint(0);
                        break;
                    case 2:
                        evse_state.get("iec61851_state")->updateUint(1);
                        break;
                    case 3:
                        evse_state.get("iec61851_state")->updateUint(2);
                        break;
                    case 4:
                        evse_state.get("iec61851_state")->updateUint(4);
                        break;
                    case 5:
                        evse_state.get("iec61851_state")->updateUint(4);
                        break;
                    case 6:
                        evse_state.get("iec61851_state")->updateUint(4);
                        break;
                    case 7:
                        evse_state.get("iec61851_state")->updateUint(3);
                        break;
                    case 8:
                        evse_state.get("iec61851_state")->updateUint(3);
                        break;
                    case 9:
                        evse_state.get("iec61851_state")->updateUint(4);
                        break;
                }
                logger.printfln("   cmd_%.2X seq:%.2X  status:%.2X (%s).", cmd, seq, PrivCommRxBuffer[9], cmd_03_status[PrivCommRxBuffer[9]]);
//6948        Buffer: FA 03 00 00 03 02 0E 00 00 01 01 00 00 00 00 00 00 00 00 00 04 00 CE C5
//6949        Rx cmd_03 seq:02 len:14 crc:C5CE
                PrivCommAck(cmd, PrivCommTxBuffer); // privCommCmdA3StatusAck
//[PRIV_COMM, 1859]: Tx(cmd_A3 len:17) :  FA 03 00 00 A3 08 07 00 10 15 06 03 10 2C 1B 15 92
//[PRIV_COMM, 1859]: Tx(cmd_A3 len:17) :  FA 03 00 00 A3 0A 07 00 10 15 06 03 10 2C 1C F5 9A
                break;
            case 0x04: // time request / ESP32-GD32 communication heartbeat
                logger.printfln("   cmd_%.2X seq:%.2X status:%d value:%d  Answer time request / ESP32-GD32 communication heartbeat.", cmd, seq, PrivCommRxBuffer[8], PrivCommRxBuffer[12]);
                // && PrivCommRxBuffer[9] == 0x01
                // && PrivCommRxBuffer[10] == 0x00
                // && PrivCommRxBuffer[11] == 0x00
                // && PrivCommRxBuffer[12] == 0x00
                // && PrivCommRxBuffer[13] == 0x00
                // && PrivCommRxBuffer[14] == 0x00
                //PrivCommTxBuffer[5] = seq;
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
            case 0x08:
                logger.printfln("   cmd_%.2X seq:%.2X  type:%.2X", cmd, seq, PrivCommRxBuffer[77]);
                if (PrivCommRxBuffer[77] < 10) {  // statistics
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
                }
                else if (PrivCommRxBuffer[77] == 0x10) {  // RFID card
                  String rfid = "";
                  for (int i=0; i<8; i++) {rfid += PrivCommRxBuffer[40 + i];}  // Card number in bytes 40..47
                  logger.printfln("RFID Card %s", rfid);
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
                logger.printfln("   cmd_%.2X seq:%.2X Heartbeat Timeout:%ds", cmd, seq, PrivCommRxBuffer[12]);
                break;
            case 0x0E:
                logger.printfln("   cmd_%.2X seq:%.2X  type:%.2X", cmd, seq, PrivCommRxBuffer[77]);
                //logger.printfln("0E: "+printHex8(PrivCommRxBuffer,61)+", cpVolt: "+String(PrivCommRxBuffer[20]+256*PrivCommRxBuffer[21])+", "+String(PrivCommRxBuffer[17]+256*PrivCommRxBuffer[18]));
                break;
            default:
                logger.printfln("   cmd_%.2X seq:%.2X  I don't know what to do about it.", cmd, seq);
                break;
        }//switch process cmd
        cmd_to_process = false;
    }

    evse_state.get("time_since_state_change")->updateUint(millis() - last_state_change);
}

void ENplus::setup_evse()
{
    Serial2.begin(115200, SERIAL_8N1, 26, 27); // PrivComm to EVSE GD32 Chip
    Serial2.setRxBufferSize(1024);
    Serial2.setTimeout(180);
    logger.printfln("Set up PrivComm: 115200, SERIAL_8N1, RX 26, TX 27, timeout 90ms");

/*
    //ctrl_cmd set heart beat time out
    PrivCommTxBuffer[PayloadStart + 0] = 0x18;
    PrivCommTxBuffer[PayloadStart + 1] = 0x08;
    PrivCommTxBuffer[PayloadStart + 2] = 0x02;
    PrivCommTxBuffer[PayloadStart + 3] = 0x00;
    PrivCommTxBuffer[PayloadStart + 4] = 0x1E; // 1E = 30 sec hb timeout, C8 = 200 sec
    PrivCommTxBuffer[PayloadStart + 5] = 0x00; // hb timeout 16bit?
    PrivCommSend(0xAA, 6, PrivCommTxBuffer);

    do { // wait for the first PRIVCOMM signal to decide if we have a GD chip to talk to
        logger.printfln("wait for PrivComm");
        if (Serial2.available() == 0) { delay(250); }
    } while(Serial2.available() == 0 && millis()<10000); // TODO disable EVSE in case of no show
*/

    // TODO start: look out for this on a unconfigured box ( no wifi ) - if it still works, delete the code
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

//W (1970-01-01 00:23:18) [PRIV_COMM, 1764]: Tx(cmd_AA len:16) :  FA 03 00 00 AA 07 06 00 18 08 02 00 1E 00 95 80
//W (1970-01-01 00:23:18) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 08 05 00 18 12 01 00 03 BA 45
//W (2021-06-12 18:06:03) [PRIV_COMM, 1764]: Tx(cmd_AA len:20) :  FA 03 00 00 AA 09 0A 00 18 02 06 00 15 06 0C 12 06 03 3C 66
//W (2021-06-12 18:06:03) [EN_WSS, 677]: send[0:148] [2,"2","DataTransfer",{"vendorId":"EN+","messageId":"gatewayInfo","data":"{\"SN\":\"ESP32GATEWAY001\",\"fwVer\":\"V3.2.418\",\"gateCode\":\"91\"}"}]
//W (2021-06-12 18:06:03) [EN_WSS, 712]: recv[0:144] [2,"1320fa60-3986-4124-97fe-ab101fa0a7e9","DataTransfer",{"data":"{\"mode\":\"normal\"}","messageId":"cpStartTransactionMode","vendorId":"EN+"}]
//W (2021-06-12 18:06:03) [EN_WSS, 712]: recv[0:144] [2,"1320fa60-3986-4124-97fe-ab101fa0a7e9","DataTransfer",{"data
//I (2021-06-12 18:06:03) [PRIV_COMM, 249]: ctrl_cmd set heart beat time out done -> 30  (=1E)

    char uid[7] = {0}; // put SN here?
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
//
//    if(rc != TF_E_OK) {
//        is_in_bootloader(rc);
//        return;
//    }

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
        is_in_bootloader(rc);
        return;
    }

    firmware_update_allowed = vehicle_state == 0;

    evse_state.get("iec61851_state")->updateUint(iec61851_state);
    evse_state.get("vehicle_state")->updateUint(vehicle_state);
    evse_state.get("contactor_state")->updateUint(contactor_state);
    bool contactor_error_changed = evse_state.get("contactor_error")->updateUint(contactor_error);
    evse_state.get("charge_release")->updateUint(charge_release);
    evse_state.get("allowed_charging_current")->updateUint(allowed_charging_current);
    //logger.printfln("EVSE: allowed_charging_current %d", allowed_charging_current);
    bool error_state_changed = evse_state.get("error_state")->updateUint(error_state);
    evse_state.get("lock_state")->updateUint(lock_state);
    //evse_state.get("time_since_state_change")->updateUint(time_since_state_change);
    evse_state.get("uptime")->updateUint(uptime);

//    if (contactor_error_changed) {
//        if (contactor_error != 0) {
//            logger.printfln("EVSE: Contactor error %d", contactor_error);
//        } else {
//            logger.printfln("EVSE: Contactor error cleared");
//        }
//    }
//
//    if (error_state_changed) {
//        if (error_state != 0) {
//            logger.printfln("EVSE: Error state %d", error_state);
//        } else {
//            logger.printfln("EVSE: Error state cleared");
//        }
//    }
}

void ENplus::update_evse_max_charging_current() {
    if(!initialized)
        return;
    uint16_t configured, incoming, outgoing, managed;

    //configured = 7000;
    //incoming = 16000;
    //outgoing = 16000;

    //evse_max_charging_current.get("max_current_configured")->updateUint(configured);
    //evse_max_charging_current.get("max_current_incoming_cable")->updateUint(incoming);
    //evse_max_charging_current.get("max_current_outgoing_cable")->updateUint(outgoing);
//    evse_max_charging_current.get("max_current_managed")->updateUint(managed);
}

void ENplus::update_evse_auto_start_charging() {
    if(!initialized)
        return;
    bool auto_start_charging;

//    int rc = tf_evse_get_charging_autostart(&evse,
//        &auto_start_charging);
//
//    if(rc != TF_E_OK) {
//        is_in_bootloader(rc);
//        return;
//    }

    //auto_start_charging = false;

    //evse_auto_start_charging.get("auto_start_charging")->updateBool(auto_start_charging);
}

void ENplus::update_evse_managed() {
    if(!initialized)
        return;
    bool managed;

    int rc = tf_evse_get_managed(&evse,
        &managed);

    if(rc != TF_E_OK) {
        is_in_bootloader(rc);
        return;
    }

    evse_managed.get("managed")->updateBool(managed);
}

void ENplus::update_evse_user_calibration() {
    if(!initialized)
        return;

    bool user_calibration_active;
    int16_t voltage_diff, voltage_mul, voltage_div, resistance_2700, resistance_880[14];

    int rc = tf_evse_get_user_calibration(&evse,
        &user_calibration_active,
        &voltage_diff,
        &voltage_mul,
        &voltage_div,
        &resistance_2700,
        resistance_880);

    if(rc != TF_E_OK) {
        is_in_bootloader(rc);
        return;
    }

    evse_user_calibration.get("user_calibration_active")->updateBool(user_calibration_active);
    evse_user_calibration.get("voltage_diff")->updateInt(voltage_diff);
    evse_user_calibration.get("voltage_mul")->updateInt(voltage_mul);
    evse_user_calibration.get("voltage_div")->updateInt(voltage_div);
    evse_user_calibration.get("resistance_2700")->updateInt(resistance_2700);

    for(int i = 0; i < sizeof(resistance_880)/sizeof(resistance_880[0]); ++i)
        evse_user_calibration.get("resistance_880")->get(i)->updateInt(resistance_880[i]);
}

bool ENplus::is_in_bootloader(int rc) {
    return false;

    if(rc != TF_E_TIMEOUT && rc != TF_E_NOT_SUPPORTED)
        return false;

    uint8_t mode;
    int bootloader_rc = tf_evse_get_bootloader_mode(&evse, &mode);
    if(bootloader_rc != TF_E_OK) {
        return false;
    }

    if(mode != TF_EVSE_BOOTLOADER_MODE_FIRMWARE) {
        initialized = false;
    }

    return mode != TF_EVSE_BOOTLOADER_MODE_FIRMWARE;
}
