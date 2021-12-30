/* warp-charger
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

#include "bindings/errors.h"

#include "api.h"
#include "event_log.h"
#include "task_scheduler.h"
#include "tools.h"
#include "web_server.h"
#include "modules.h"

#include <AsyncTCP.h>

#include <WiFiUdp.h>
#include "yeelight.h"
#include "event_log.h"
extern EventLog logger;

WiFiUDP _udp;
IPAddress _ipMulti(239, 255, 255, 250);
char _packetBuffer[550];

String _location, _support;
String _params = "";
String _method = "set_scene";
bool initialized = false;
bool _powered = false;
int _intensity = 10; //percent
IPAddress _ip;

uint16_t _cmdid = 0, _port = 0;

unsigned long target_time = 0L;
const unsigned long PERIOD = 15*1000UL; //miliseconds

AsyncClient *client_tcp = new AsyncClient;

extern EventLog logger;

extern TaskScheduler task_scheduler;

extern API api;

void replyToServer(void *arg)
{
	AsyncClient *client = reinterpret_cast<AsyncClient *>(arg);
    // add the turn off timeout first as a watchdog that ensures there is no stale YeeLight status
    String payload = String("{\"id\":0,\"method\":\"cron_add\",\"params\":[0,1\"]}\r\n") + 
        // then set the light to whatever status shall be displayed
        "{\"id\":" + (++_cmdid) + ",\"method\":\"" + _method + "\",\"params\":" + _params + "}\r\n";
	// send reply
	if (client->space() > strlen(payload.c_str()) && client->canSend())
	{
		client->add(payload.c_str(), strlen(payload.c_str()));
        //logger.printfln("%s:%d%s", _ip.toString().c_str(), _port, payload.c_str());
		client->send();
	}
}

void handleData(void *arg, AsyncClient *client, void *data, size_t len)
{
    /* char reply[len + 1]; */
    /* memcpy(reply, data, len); */
    /* reply[len] = 0; // Null termination. */

    /* logger.printfln("<%s: %s", client->remoteIP().toString().c_str(), reply); */

	//Serial.write((uint8_t *)data, len);
}

void onConnect(void *arg, AsyncClient *client)
{
	//Serial.printf("\n client has been connected to %s on port %d \n", SERVER_HOST_NAME, TCP_SERVER_PORT);
	replyToServer(client);
}

yeelight::yeelight()
{
    yeelight_update = Config::Null();
    yeelight_prepare_command = Config::Object({
        {"method", Config::Str("",9)},
        {"params", Config::Str("",500)}
    });
    yeelight_color = Config::Object({
        {"color", Config::Uint(0xFFFFFF)}
    });
}

void yeelight::setup()
{
	client_tcp->onData(handleData, client_tcp);
	client_tcp->onConnect(onConnect, client_tcp);

    this->prepareCommand("set_scene", "[\"cf\",0,0,\"500,2,2700,10,500,2,2700,1\"]"); // pulse 500ms warm white 10% 1%
    this->sendCommand();

    task_scheduler.scheduleWithFixedDelay("update_yeelight", [this](){
        this->update();
    }, 0, 5000);

    /* task_scheduler.scheduleWithFixedDelay("yeelight_watchdog", [this]() { */
    /*     if (!deadline_elapsed(this->last_yeelight_update + 30000)) */
    /*         return; */
    /*     if(!this->shutdown_logged) */
    /*         logger.printfln("Got no managed current update for more than 30 seconds. Setting managed current to 0"); */
    /*     this->shutdown_logged = true; */
    /*     evse_managed_current.get("current")->updateUint(0); */
    /* }, 1000, 1000); */
}

/* void yeelight::set_managed_current(uint16_t current) { */
/*     this->last_yeelight_update = millis(); */
/*     this->shutdown_logged = false; */
/* } */

void yeelight::register_urls()
{
    api.addCommand("yeelight/prepare_command", &yeelight_prepare_command, {}, [this](){
        this->prepareCommand(yeelight_prepare_command.get("method")->asString(), yeelight_prepare_command.get("method")->asString());
    }, true);
    api.addCommand("yeelight/color", &yeelight_color, {}, [this](){
        this->color(yeelight_color.get("color")->asUint());
    }, true);
    api.addCommand("yeelight/update", &yeelight_update, {}, [this](){
        this->update();
    }, true);
}

void yeelight::loop()
{
}

/* yeelight::~yeelight() */
/* { */
/*     _udp.stop(); */
/* } */

void yeelight::lookup()
{
    if (millis () - target_time >= PERIOD) {
        logger.printfln("YeeLight lookup");
        target_time += PERIOD ;   // not more often than every PERIOD
        _udp.beginMulticast(_ipMulti, 1982);
        _udp.beginMulticastPacket();
        _udp.print("M-SEARCH * HTTP/1.1\r\nHOST: 239.255.255.250:1982\r\nMAN: \"ssdp:discover\"\r\nST: wifi_bulb");
        _udp.endPacket();
        _udp.begin(1982);
    }
}

int yeelight::feedback()
{
  //logger.printfln("YeeLight feedback");
  int packetSize = _udp.parsePacket();
  if (packetSize) {
    int len = _udp.read(_packetBuffer, 550);
    if (len > 0) {
      _packetBuffer[len] = 0;
    }
    parseFeedback(_packetBuffer, len);
  }
  return packetSize;
}

void yeelight::parseFeedback(char* buffer, size_t len)
{
  //logger.printfln("YeeLight parse feedback");
  int i=0, _i=0;
  char _b[255];
  while (i<len) {
    if (buffer[i] == '\r' &&
        i+1 <= len &&
        buffer[i+1] == '\n') {
        _b[_i]=0;

        // ----
        String _str = String(_b);
        //logger.printfln(_str.c_str());
        if (_str.startsWith("Location: yeelight://")) {
          int colon = _str.indexOf(':', 21);
          _location = _str.substring(10);
          _ip.fromString(_str.substring(21, colon));
          _port = _str.substring(colon + 1).toInt();
          initialized = true;
          //IPAddress tatschi(192,168,188,82);
          //_ip = tatschi;
          logger.printfln("YeeLight found: %s:%d", _ip.toString().c_str(), _port);
        }
        if (_str.startsWith("support: ")) {
          _support = _str.substring(9);
          //logger.printfln("YeeLight support: %s", _support.c_str());
        }
        if (_str.startsWith("power: ")) {
          _powered = _str.substring(7) == "on";
          //logger.printfln("YeeLight power: %s", _powered ? "on" : "off");
        }
        // ----

        i=i+2;
        _i=0;
    } else {
      _b[_i]=buffer[i];
      i++;
      _i++;
    }
  }
}

void yeelight::sendCommand() {
// https://www.yeelight.com/download/Yeelight_Inter-Operation_Spec.pdf
    if (initialized) {
       	client_tcp->connect(_ip.toString().c_str(), _port);
        client_tcp->close();
    } else {
        this->lookup();
        this->feedback();
    }
}

void yeelight::prepareCommand(String method, String params)
{
    _method = method;
    _params = params;
    //    logger.printfln("m:%s p:%s", _method.c_str(), _params.c_str());
}

void yeelight::color(int color)
{
    this->prepareCommand("set_scene", "[\"cf\",1,1,\"3000,1," + String(color) + "," + String(_intensity) + "\"]");
}

void yeelight::update()
{
    // TODO do not hammer the YeeLight with commands, because it will ignore us if the threshold is reached

    /* NOTE: Currently WiFi smart device support up to 4 simultaneous TCP connections, any further */
    /* connect attempt will be rejected. For each connection, there is a command message quota, */
    /* that is 60 commands per minute. There is also a total quota for all the LAN commands: 144 */
    /* commands per minute 4 × 60 × 60%. */

    this->sendCommand();
}
