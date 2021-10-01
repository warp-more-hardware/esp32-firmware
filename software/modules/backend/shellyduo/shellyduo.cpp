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

#include "shellyduo.h"
#include "event_log.h"
extern EventLog logger;

String _uri = "";
bool initialized = false;
int _intensity = 10; //percent
String _ip = "10.10.10.9";

unsigned long target_time = 0L;
const unsigned long PERIOD = 15*1000UL; //miliseconds

AsyncClient *client_tcp = new AsyncClient;

extern EventLog logger;

extern TaskScheduler task_scheduler;

extern API api;

void replyToServer(void *arg)
{
	AsyncClient *client = reinterpret_cast<AsyncClient *>(arg);
    // prepare the http get
    String payload = 
        String("GET ") + 
        // set the light to whatever status shall be displayed
        _uri + 
        // add the turn off timeout as a watchdog that ensures there is no stale status
        "&timer=25 HTTP/1.1\n" +
        // add the host header
        "Host: " + _ip + "\n\n";
    //logger.printfln("%s", payload.c_str());
	// send reply
	if (client->space() > strlen(payload.c_str()) && client->canSend())
	{
		client->add(payload.c_str(), strlen(payload.c_str()));
		client->send();
	}
}

void handleData(void *arg, AsyncClient *client, void *data, size_t len)
{
    char reply[len + 1];
    memcpy(reply, data, len);
    reply[len] = 0; // Null termination.

    //logger.printfln("<%s: %s", client->remoteIP().toString().c_str(), reply);
	//Serial.write((uint8_t *)data, len);
}

void onConnect(void *arg, AsyncClient *client)
{
	replyToServer(client);
}

ShellyDUO::ShellyDUO()
{
    shellyduo_update = Config::Null();
    shellyduo_prepare_command = Config::Object({
        {"uri", Config::Str("",500)}
    });
    shellyduo_color = Config::Object({
        {"color", Config::Uint(0xFFFFFF)}
    });
}

void ShellyDUO::setup()
{
	client_tcp->onData(handleData, client_tcp);
	client_tcp->onConnect(onConnect, client_tcp);

    initialized = true;

    logger.printfln("Statuslight initialized.");

    this->prepareCommand(String("/color/0?turn=on&effect=3&red=100&green=100&blue=100&gain=10")); // pulse very light blue
    this->sendCommand();

    task_scheduler.scheduleWithFixedDelay("update_shellyduo", [this](){
        this->update();
    }, 0, 5000);

    /* task_scheduler.scheduleWithFixedDelay("shellyduo_watchdog", [this]() { */
    /*     if (!deadline_elapsed(this->last_shellyduo_update + 30000)) */
    /*         return; */
    /*     if(!this->shutdown_logged) */
    /*         logger.printfln("Got no managed current update for more than 30 seconds. Setting managed current to 0"); */
    /*     this->shutdown_logged = true; */
    /*     evse_managed_current.get("current")->updateUint(0); */
    /* }, 1000, 1000); */
}

/* void ShellyDUO::set_managed_current(uint16_t current) { */
/*     this->last_shellyduo_update = millis(); */
/*     this->shutdown_logged = false; */
/* } */

void ShellyDUO::register_urls()
{
    api.addCommand("statuslight/prepare_command", &shellyduo_prepare_command, {}, [this](){
        this->prepareCommand(shellyduo_prepare_command.get("uri")->asString());
    }, true);
    api.addCommand("statuslight/color", &shellyduo_color, {}, [this](){
        this->color(shellyduo_color.get("color")->asUint());
    }, true);
    api.addCommand("statuslight/update", &shellyduo_update, {}, [this](){
        this->update();
    }, true);
}

void ShellyDUO::loop()
{
}

/* void ShellyDUO::parseFeedback(char* buffer, size_t len) */
/* { */
/*   //logger.printfln("shellyduo parse feedback"); */
/*         if (_str.startsWith("Location: yeelight://")) { */
/*           int colon = _str.indexOf(':', 21); */
/*           _ip.fromString(_str.substring(21, colon)); */
/*           initialized = true; */
/*           //IPAddress tatschi(192,168,188,82); */
/*           //_ip = tatschi; */
/*           logger.printfln("shellyduo found: %s", _ip.c_str()); */
/*         } */
/* } */

void ShellyDUO::sendCommand() {
    if (initialized) {
        //logger.printfln("Statuslight update.");
       	client_tcp->connect(_ip.c_str(), 80);
        client_tcp->close();
    }
}

void ShellyDUO::prepareCommand(String uri)
{
    if (_uri != uri) {
       _uri = uri;
       logger.printfln("%s", _uri.c_str());
    }
}

void ShellyDUO::color(int color)
{
    uint8_t red   = (color>>16)&0x0ff;
    uint8_t green = (color>>8) &0x0ff;
    uint8_t blue  = (color)    &0x0ff;
    this->prepareCommand(String("/color/0?turn=on&red=" + String(red) + "&green=" + String(green) + "&blue=" + String(blue) + "&effect=0&transition=3000&gain=10"));
}

void ShellyDUO::update()
{
    // TODO do not hammer the shellyduo with commands, because it will ignore us if the threshold is reached

    /* NOTE: Currently WiFi smart device support up to 4 simultaneous TCP connections, any further */
    /* connect attempt will be rejected. For each connection, there is a command message quota, */
    /* that is 60 commands per minute. There is also a total quota for all the LAN commands: 144 */
    /* commands per minute 4 × 60 × 60%. */

    this->sendCommand();
}
