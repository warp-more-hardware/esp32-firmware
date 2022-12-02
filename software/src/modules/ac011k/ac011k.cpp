/* esp32-firmware
 * Copyright (C) 2020-2021 Erik Fleckstein <erik@tinkerforge.com>
 * Copyright (C) 2021-2022 Birger Schmidt <bs-warp@netgaroo.com>
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

#include "ac011k.h"

#include "bindings/errors.h"

#include "api.h"
#include "event_log.h"
#include "task_scheduler.h"
#include "tools.h"
#include "web_server.h"
#include "modules.h"

extern EventLog logger;

extern TaskScheduler task_scheduler;
extern WebServer server;

extern API api;
extern bool firmware_update_allowed;

/* experimental: Test command receiver */
#include <WiFiUdp.h>
#define UDP_RX_PACKET_MAX_SIZE 1024
WiFiUDP UdpListener;
unsigned int commandPort = 43211;  // local port to listen on
char receiveCommandBuffer[UDP_RX_PACKET_MAX_SIZE];  // buffer to hold incoming UDP data
/* end experimental */

#define SLOT_ACTIVE(x) ((bool)(x & 0x01))
#define SLOT_CLEAR_ON_DISCONNECT(x) ((bool)(x & 0x02))

void AC011K::pre_setup()
{
    // States
    evse_state = Config::Object({
        {"iec61851_state", Config::Uint8(0)},
        {"charger_state", Config::Uint8(0)},
        {"GD_state", Config::Uint8(0)},
        {"contactor_state", Config::Uint8(0)},
        {"contactor_error", Config::Uint8(0)},
        {"allowed_charging_current", Config::Uint16(0)},
        {"error_state", Config::Uint8(0)},
        {"lock_state", Config::Uint8(0)},
        {"time_since_state_change", Config::Uint32(0)},
        {"last_state_change", Config::Uint32(0)},
    });

    evse_hardware_configuration = Config::Object({
        {"Hardware", Config::Str("", 0, 20)},
        {"FirmwareVersion", Config::Str("", 0, 20)},
        {"SerialNumber", Config::Str("", 0, 20)},
        {"evse_found", Config::Bool(false)},
        {"initialized", Config::Bool(false)},
        {"GDFirmwareVersion", Config::Uint16(0)},
        {"jumper_configuration", Config::Uint8(3)}, // 3 = 16 Ampere = 11KW for the EN+ wallbox
        {"has_lock_switch", Config::Bool(false)},    // no key lock switch
        {"evse_version", Config::Uint8(0)},
        {"energy_meter_type", Config::Uint8(0)}
    });

    evse_low_level_state = Config::Object ({
        {"led_state", Config::Uint8(0)},
        {"cp_pwm_duty_cycle", Config::Uint16(0)},
        {"adc_values", Config::Array({
                Config::Uint16(0),
                Config::Uint16(0),
                Config::Uint16(0),
                Config::Uint16(0),
                Config::Uint16(0),
                Config::Uint16(0),
                Config::Uint16(0),
            }, new Config{Config::Uint16(0)}, 7, 7, Config::type_id<Config::ConfUint>())
        },
        {"voltages", Config::Array({
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
                Config::Int16(0),
            }, new Config{Config::Int16(0)}, 7, 7, Config::type_id<Config::ConfInt>())
        },
        {"resistances", Config::Array({
                Config::Uint32(0),
                Config::Uint32(0),
            }, new Config{Config::Uint32(0)}, 2, 2, Config::type_id<Config::ConfUint>())
        },
        {"gpio", Config::Array({
            Config::Bool(false), Config::Bool(false),  Config::Bool(false),Config::Bool(false),
            Config::Bool(false), Config::Bool(false),  Config::Bool(false),Config::Bool(false),
            Config::Bool(false), Config::Bool(false),  Config::Bool(false),Config::Bool(false),
            Config::Bool(false), Config::Bool(false),  Config::Bool(false),Config::Bool(false),
            Config::Bool(false), Config::Bool(false),  Config::Bool(false),Config::Bool(false),
            Config::Bool(false), Config::Bool(false),  Config::Bool(false),Config::Bool(false),
            }, new Config{Config::Bool(false)}, 24, 24, Config::type_id<Config::ConfBool>())},
        {"charging_time", Config::Uint32(0)},
        {"time_since_state_change", Config::Uint32(0)},
        {"uptime", Config::Uint32(0)}
    });

    evse_energy_meter_values = Config::Object({
        {"power", Config::Float(0)},
        {"energy_rel", Config::Float(0)},
        {"energy_abs", Config::Float(0)},
        {"phases_active", Config::Array({Config::Bool(false),Config::Bool(false),Config::Bool(false)},
            new Config{Config::Bool(false)},
            3, 3, Config::type_id<Config::ConfBool>())},
        {"phases_connected", Config::Array({Config::Bool(false),Config::Bool(false),Config::Bool(false)},
            new Config{Config::Bool(false)},
            3, 3, Config::type_id<Config::ConfBool>())}
    });

    evse_energy_meter_errors = Config::Object({
        {"local_timeout", Config::Uint32(0)},
        {"global_timeout", Config::Uint32(0)},
        {"illegal_function", Config::Uint32(0)},
        {"illegal_data_access", Config::Uint32(0)},
        {"illegal_data_value", Config::Uint32(0)},
        {"slave_device_failure", Config::Uint32(0)},
    });

    evse_button_state = Config::Object({
        {"button_press_time", Config::Uint32(0)},
        {"button_release_time", Config::Uint32(0)},
        {"button_pressed", Config::Bool(false)},
    });

    Config *evse_charging_slot = new Config{Config::Object({
        {"max_current", Config::Uint32(0)},
        {"active", Config::Bool(false)},
        {"clear_on_disconnect", Config::Bool(false)}
    })};

    evse_slots = Config::Array({},
        evse_charging_slot,
        CHARGING_SLOT_COUNT, CHARGING_SLOT_COUNT,
        Config::type_id<Config::ConfObject>());

    for (int i = 0; i < CHARGING_SLOT_COUNT; ++i)
        evse_slots.add();

    evse_indicator_led = Config::Object({
        {"indication", Config::Int16(0)},
        {"duration", Config::Uint16(0)},
    });

    // Actions

    evse_reset_dc_fault_current_state = Config::Object({
        {"password", Config::Uint32(0)} // 0xDC42FA23
    });

    // TODO indicator LED


    // EVSE configs
    evse_gpio_configuration = Config::Object({
        {"shutdown_input", Config::Uint8(0)},
        {"input", Config::Uint8(0)},
        {"output", Config::Uint8(0)}
    });

    evse_gpio_configuration_update = evse_gpio_configuration;

    evse_button_configuration = Config::Object({
        {"button", Config::Uint8(2)}
    });

    evse_button_configuration_update = Config::Object({
        {"button", Config::Uint8(2)}
    });

    evse_control_pilot_configuration = Config::Object({
        {"control_pilot", Config::Uint8(0)}
    });

    evse_control_pilot_configuration_update = Config::Object({
        {"control_pilot", Config::Uint8(0)}
    });

    evse_control_pilot_connected = Config::Object({
        {"connected", Config::Bool(true)}
    });

    evse_auto_start_charging = Config::Object({
        {"auto_start_charging", Config::Bool(true)}
    });

    evse_auto_start_charging_update = Config::Object({
        {"auto_start_charging", Config::Bool(true)}
    });

    evse_global_current = Config::Object({
        {"current", Config::Uint16(32000)}
    });

    evse_global_current_update = evse_global_current;

    evse_management_enabled = Config::Object({
        {"enabled", Config::Bool(false)}
    });
    evse_management_enabled_update = evse_management_enabled;

    evse_user_current = Config::Object({
        {"current", Config::Uint16(32000)}
    });

    evse_user_enabled = Config::Object({
        {"enabled", Config::Bool(false)}
    });
    evse_user_enabled_update = evse_user_enabled;

    evse_external_enabled = Config::Object({
        {"enabled", Config::Bool(false)}
    });
    evse_external_enabled_update = evse_external_enabled;

    evse_external_defaults = Config::Object({
        {"current", Config::Uint16(0)},
        {"clear_on_disconnect", Config::Bool(false)},
    });
    evse_external_defaults_update = evse_external_defaults;

    evse_management_current = Config::Object({
        {"current", Config::Uint16(32000)}
    });

    evse_management_current_update = evse_management_current;

    evse_external_current = Config::Object({
        {"current", Config::Uint16(32000)}
    });

    evse_external_current_update = evse_external_current;

    evse_external_clear_on_disconnect = Config::Object({
        {"clear_on_disconnect", Config::Bool(false)}
    });

    evse_external_clear_on_disconnect_update = evse_external_clear_on_disconnect;

    evse_modbus_enabled = Config::Object({
        {"enabled", Config::Bool(false)}
    });
    evse_modbus_enabled_update = evse_modbus_enabled;

    evse_ocpp_enabled = Config::Object({
        {"enabled", Config::Bool(false)}
    });
    evse_ocpp_enabled_update = evse_ocpp_enabled;
}

bool AC011K::apply_slot_default(uint8_t slot, uint16_t current, bool enabled, bool clear)
{
    uint16_t old_current = evse_slots.get(slot)->get("max_current")->asUint();
    bool old_enabled     = evse_slots.get(slot)->get("active")->asBool();
    bool old_clear       = evse_slots.get(slot)->get("clear_on_disconnect")->asBool();

    if ((old_current == current) && (old_enabled == enabled) && (old_clear == clear))
        return false;

    //rc = tf_evse_v2_set_charging_slot_default(&device, slot, current, enabled, clear);
	logger.printfln("set - slot %d: max_current: %d, active: %s, clear_on_disconnect: %s",
            slot,
            current,
            enabled ?"true":"false",
            clear ?"true":"false");
    evse_slots.get(slot)->get("max_current")->updateUint(current);
    evse_slots.get(slot)->get("active")->updateBool(SLOT_ACTIVE(enabled));
    evse_slots.get(slot)->get("clear_on_disconnect")->updateBool(SLOT_CLEAR_ON_DISCONNECT(clear));
    return true;
}

void AC011K::apply_defaults()
{
    // Maybe this is the first start-up after updating the EVSE to firmware 2.1.0 (or higher)
    // (Or the first start-up at all)
    // Make sure, that the charging slot defaults are the expected ones.

    // Slot 0 to 3 are set according to the hardware
    apply_slot_default(CHARGING_SLOT_INCOMING_CABLE, 16000, true, false);
    apply_slot_default(CHARGING_SLOT_OUTGOING_CABLE, 16000, true, false);
    apply_slot_default(CHARGING_SLOT_SHUTDOWN_INPUT, 0, false, false);
    apply_slot_default(CHARGING_SLOT_GP_INPUT, 0, false, false);

    // Slot 4 (auto start): Enabled is read only, current and clear depend on the auto start setting
    // Doing anything with this slot should be unnecessary, as we would only change the current to 32000 or 0
    // depending on clear_on_disconnect. Any other value could be considered a bug in the EVSE firmware.
    apply_slot_default(CHARGING_SLOT_AUTOSTART_BUTTON, 0, false, false);

    // Slot 5 (global) has to be always enabled and never cleared. current is set per API
    // The charger API ensures that the slot's current and its default are always the same,
    // but if for any reason, they are not equal when booting up, just set the current value as
    // default. In the common case this has no effect. Also set enabled and clear in case this is
    // the first start-up.

    // If this is the first start-up, this slot will not be active.
    // In the old firmwares, the global current was not persistant
    // so setting it to 32000 is expected after start-up.
    if (!evse_slots.get(CHARGING_SLOT_GLOBAL)->get("active")->asBool())
        apply_slot_default(CHARGING_SLOT_GLOBAL, 32000, true, false);

    // Slot 6 (user) depends on user config.
    // It can be enabled per API
    // Set clear to true and current to 0 in any case: If disabled those are ignored anyway.
    apply_slot_default(CHARGING_SLOT_USER, 0, evse_slots.get(CHARGING_SLOT_USER)->get("active")->asBool(), true);

    // Slot 7 (charge manager) can be enabled per API
    // Set clear to true and current to 0 in any case: If disabled those are ignored anyway.
    apply_slot_default(CHARGING_SLOT_CHARGE_MANAGER, 0, evse_slots.get(CHARGING_SLOT_CHARGE_MANAGER)->get("active")->asBool(), true);

    // Slot 8 (external) is controlled via API, no need to change anything here
}

void AC011K::factory_reset()
{
    logger.printfln("EVSE factory reset is not implemented yet.");
}

void AC011K::setup()
{
    UdpListener.begin(commandPort); // experimental
    setup_evse();
    if(!api.restorePersistentConfig("evse/slots", &evse_slots)) {
        logger.printfln("EVSE error, could not restore persistent storage slots config");
    } 
    if(!api.restorePersistentConfig("evse/config", &evse_config)) {
        logger.printfln("EVSE error, could not restore persistent storage evse config");
    }
    /* } else { */
    /*     evse_auto_start_charging.get("auto_start_charging")     -> updateBool(evse_config.get("auto_start_charging")->asBool()); */
    /*     evse_max_charging_current.get("max_current_configured") -> updateUint(evse_config.get("max_current_configured")->asUint()); */
    /*     evse_managed.get("managed")                             -> updateBool(evse_config.get("managed")->asBool()); */
	/* logger.printfln("restored config - auto_start_charging: %s, managed: %s, max_current_configured: %d", */
    /*         evse_config.get("auto_start_charging")->asBool() ?"true":"false", */
    /*         evse_config.get("managed")->asBool() ?"true":"false", */
    /*         evse_config.get("max_current_configured")->asUint()); */
    /* } */

    // Get all data once before announcing the EVSE feature.
    update_all_data();
    api.addFeature("evse");
    api.addFeature("cp_disconnect");
    api.addFeature("button_configuration");

    task_scheduler.scheduleWithFixedDelay([this](){
        update_all_data();
    }, 0, 250);
}

String AC011K::get_evse_debug_header()
{
    return "\"millis,"
           "STATE,"
           "iec61851_state,"
           "charger_state,"
           "contactor_state,"
           "contactor_error,"
           "allowed_charging_current,"
           "error_state,"
           "lock_state,"
           "dc_fault_current_state,"
           "HARDWARE CONFIG,"
           "jumper_configuration,"
           "has_lock_switch,"
           "evse_version,"
           "energy_meter_type,"
           "ENERGY METER,"
           "power,"
           "energy_relative,"
           "energy_absolute,"
           "phase_0_active,"
           "phase_1_active,"
           "phase_2_active,"
           "phase_0_connected,"
           "phase_1_connected,"
           "phase_2_connected,"
           "ENERGY METER ERRORS,"
           "local_timeout,"
           "global_timeout,"
           "illegal_function,"
           "illegal_data_access,"
           "illegal_data_value,"
           "slave_device_failure,"
           "LL-State,"
           "led_state,"
           "cp_pwm_duty_cycle,"
           "charging_time,"
           "time_since_state_change,"
           "uptime,"
           "ADC VALUES,"
           "CP/PE before (PWM high),"
           "CP/PE after (PWM high),"
           "CP/PE before (PWM low),"
           "CP/PE after (PWM low),"
           "PP/PE,"
           "+12V,"
           "-12V,"
           "VOLTAGES,"
           "CP/PE before (PWM high),"
           "CP/PE after (PWM high),"
           "CP/PE before (PWM low),"
           "CP/PE after (PWM low),"
           "PP/PE,"
           "+12V,"
           "-12V,"
           "RESISTANCES,"
           "CP/PE,"
           "PP/PE,"
           "GPIOs,"
           "Config Jumper 0 (0),"
           "Motor Fault (1),"
           "DC Error (2),"
           "Config Jumper 1 (3),"
           "DC Test (4),"
           "Enable (5),"
           "Switch (6),"
           "CP PWM (7),"
           "Input Motor Switch (8),"
           "Relay (Contactor) (9),"
           "GP Output (10),"
           "CP Disconnect (11),"
           "Motor Enable (12),"
           "Motor Phase (13),"
           "AC 1 (14),"
           "AC 2 (15),"
           "GP Input (16),"
           "DC X6 (17),"
           "DC X30 (18),"
           "LED (19),"
           "unused (20),"
           "unused (21),"
           "unused (22),"
           "unused (23)"
           "\"";
}

String AC011K::get_evse_debug_line() {
    if(!initialized)
        return "EVSE is not initialized!";

    uint8_t iec61851_state, charger_state, contactor_state, contactor_error, error_state, lock_state;
    uint16_t allowed_charging_current;
    uint32_t time_since_state_change, uptime;

    int rc = bs_evse_get_state(
        &iec61851_state,
        &charger_state,
        &contactor_state,
        &contactor_error,
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

//    rc = tf_evse_get_low_level_state(
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
        charger_state,
        contactor_state,
        contactor_error,
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

void AC011K::set_managed_current(uint16_t current)
{
    //tf_evse_v2_set_charging_slot_max_current(CHARGING_SLOT_CHARGE_MANAGER, current));
    evse_slots.get(CHARGING_SLOT_CHARGE_MANAGER)->get("max_current")->updateUint(current);
    this->last_current_update = millis();
    this->shutdown_logged = false;
}

void AC011K::set_user_current(uint16_t current)
{
    //is_in_bootloader(tf_evse_v2_set_charging_slot_max_current(&device, CHARGING_SLOT_USER, current));
    evse_slots.get(CHARGING_SLOT_USER)->get("max_current")->updateUint(current);
}

void AC011K::set_modbus_current(uint16_t current)
{
    //is_in_bootloader(tf_evse_v2_set_charging_slot_max_current(&device, CHARGING_SLOT_MODBUS_TCP, current));
    evse_slots.get(CHARGING_SLOT_MODBUS_TCP)->get("max_current")->updateUint(current);
}

void AC011K::set_modbus_enabled(bool enabled)
{
    //is_in_bootloader(tf_evse_v2_set_charging_slot_max_current(&device, CHARGING_SLOT_MODBUS_TCP_ENABLE, enabled ? 32000 : 0));
    evse_slots.get(CHARGING_SLOT_MODBUS_TCP_ENABLE)->get("max_current")->updateUint(enabled ? 32000 : 0);
}

void AC011K::set_ocpp_current(uint16_t current)
{
    //is_in_bootloader(tf_evse_v2_set_charging_slot_max_current(&device, CHARGING_SLOT_OCPP, current));
    evse_slots.get(CHARGING_SLOT_OCPP)->get("max_current")->updateUint(current);
}

uint16_t AC011K::get_ocpp_current()
{
    return evse_slots.get(CHARGING_SLOT_OCPP)->get("max_current")->asUint();
}

void AC011K::register_urls()
{

#if MODULE_CM_NETWORKING_AVAILABLE()
    cm_networking.register_client([this](uint16_t current) {
        set_managed_current(current);
    });

    task_scheduler.scheduleWithFixedDelay([this](){
        uint16_t supported_current = 32000;
        for (int i = 0; i < CHARGING_SLOT_COUNT; ++i) {
            if (i == CHARGING_SLOT_CHARGE_MANAGER)
                continue;
            if (!evse_slots.get(i)->get("active")->asBool())
                continue;
            supported_current = min(supported_current, (uint16_t)evse_slots.get(i)->get("max_current")->asUint());
        }

        cm_networking.send_client_update(
            evse_state.get("iec61851_state")->asUint(),
            evse_state.get("charger_state")->asUint(),
            evse_state.get("error_state")->asUint(),
            evse_low_level_state.get("uptime")->asUint(),
            evse_low_level_state.get("charging_time")->asUint(),
            evse_slots.get(CHARGING_SLOT_CHARGE_MANAGER)->get("max_current")->asUint(),
            supported_current,
            evse_management_enabled.get("enabled")->asBool()
        );
    }, 1000, 1000);

    task_scheduler.scheduleWithFixedDelay([this]() {
        if (!deadline_elapsed(this->last_current_update + 30000))
            return;
        if (!evse_management_enabled.get("enabled")->asBool()) {
            // Push back the next check for 30 seconds: If managed gets enabled,
            // we want to wait 30 seconds before setting the current for the first time.
            this->last_current_update = millis();
            return;
        }
        if(!this->shutdown_logged)
            logger.printfln("Got no managed current update for more than 30 seconds. Setting managed current to 0");
        this->shutdown_logged = true;
        evse_slots.get(CHARGING_SLOT_CHARGE_MANAGER)->get("max_current")->updateUint(0);
    }, 1000, 1000);
#endif

    api.addPersistentConfig("evse/config", &evse_config, {}, 1000);

    // States
    api.addState("evse/state", &evse_state, {}, 1000);
    api.addState("evse/hardware_configuration", &evse_hardware_configuration, {}, 1000);
    api.addState("evse/low_level_state", &evse_low_level_state, {}, 1000);
    api.addState("evse/button_state", &evse_button_state, {}, 250);
    api.addPersistentConfig("evse/slots", &evse_slots, {}, 1000);
    api.addState("evse/indicator_led", &evse_indicator_led, {}, 1000);
    api.addState("evse/control_pilot_connected", &evse_control_pilot_connected, {}, 1000); //TODO

    // Actions
    api.addCommand("evse/reset_dc_fault_current_state", &evse_reset_dc_fault_current_state, {}, [this](){
        //is_in_bootloader(tf_evse_v2_reset_dc_fault_current_state(&device, evse_reset_dc_fault_current_state.get("password")->asUint()));
    }, true);

    api.addCommand("evse/stop_charging", Config::Null(), {}, [this](){
        if (evse_state.get("iec61851_state")->asUint() != IEC_STATE_A) {
            evse_slots.get(CHARGING_SLOT_AUTOSTART_BUTTON)->get("max_current")->updateUint(0);
            bs_evse_stop_charging(); // TODO scheduled function to start stop depending on slots?!
        }
    }, true);

    api.addCommand("evse/start_charging", Config::Null(), {}, [this](){
        if (evse_state.get("iec61851_state")->asUint() != IEC_STATE_A) {
            evse_slots.get(CHARGING_SLOT_AUTOSTART_BUTTON)->get("max_current")->updateUint(32000);
            bs_evse_start_charging(); // TODO scheduled function to start stop depending on slots?!
        }
    }, true);

#if MODULE_WS_AVAILABLE()
    server.on("/evse/start_debug", HTTP_GET, [this](WebServerRequest request) {
        task_scheduler.scheduleOnce([this](){
            ws.pushRawStateUpdate(this->get_evse_debug_header(), "evse/debug_header");
            debug = true;
        }, 0);
        return request.send(200);
    });

    server.on("/evse/stop_debug", HTTP_GET, [this](WebServerRequest request){
        task_scheduler.scheduleOnce([this](){
            debug = false;
        }, 0);
        return request.send(200);
    });
#endif

    // TODO: indicator led update as API?

    api.addState("evse/external_current", &evse_external_current, {}, 1000);
    api.addCommand("evse/external_current_update", &evse_external_current_update, {}, [this](){
        evse_slots.get(CHARGING_SLOT_EXTERNAL)->get("max_current")->updateUint(evse_external_current_update.get("current")->asUint());
    }, false);

    api.addState("evse/external_clear_on_disconnect", &evse_external_clear_on_disconnect, {}, 1000);
    api.addCommand("evse/external_clear_on_disconnect_update", &evse_external_clear_on_disconnect_update, {}, [this](){
        evse_slots.get(CHARGING_SLOT_EXTERNAL)->get("clear_on_disconnect")->updateBool(SLOT_CLEAR_ON_DISCONNECT(evse_external_clear_on_disconnect_update.get("clear_on_disconnect")->asBool()));
    }, false);

    api.addState("evse/management_current", &evse_management_current, {}, 1000);
    api.addCommand("evse/management_current_update", &evse_management_current_update, {}, [this](){
        this->set_managed_current(evse_management_current_update.get("current")->asUint());
    }, false);

    // Configurations. Note that those are _not_ configs in the api.addPersistentConfig sense:
    // The configs are stored on the EVSE itself, not the ESP's flash.
    // All _update APIs that write the EVSEs flash without checking first if this was a change
    // are marked as actions to make sure the flash is not written unnecessarily.
    api.addState("evse/gpio_configuration", &evse_gpio_configuration, {}, 1000);
    api.addCommand("evse/gpio_configuration_update", &evse_gpio_configuration_update, {}, [this](){
        /* is_in_bootloader(tf_evse_v2_set_gpio_configuration(&device, evse_gpio_configuration_update.get("shutdown_input")->asUint(), */
        /*                                                             evse_gpio_configuration_update.get("input")->asUint(), */
        /*                                                             evse_gpio_configuration_update.get("output")->asUint())); */
    }, true);

    api.addState("evse/button_configuration", &evse_button_configuration, {}, 1000);
    api.addCommand("evse/button_configuration_update", &evse_button_configuration_update, {}, [this](){
        /* is_in_bootloader(tf_evse_v2_set_button_configuration(&device, evse_button_configuration_update.get("button")->asUint())); */
    }, true);

    api.addState("evse/control_pilot_configuration", &evse_control_pilot_configuration, {}, 1000);
    api.addCommand("evse/control_pilot_configuration_update", &evse_control_pilot_configuration_update, {}, [this](){
        auto cp = evse_control_pilot_configuration_update.get("control_pilot")->asUint();
        /* int rc = tf_evse_v2_set_control_pilot_configuration(&device, cp, nullptr); */
        logger.printfln("updating control pilot to %u.", cp);
        /* is_in_bootloader(rc); */
    }, true);

    /* api.addCommand("evse/current_limit", &evse_current_limit, {}, [this](){ */
    /*     bs_evse_set_max_charging_current(evse_current_limit.get("current")->asUint()); */
    /* }, false); */

    api.addState("evse/auto_start_charging", &evse_auto_start_charging, {}, 1000);
    api.addCommand("evse/auto_start_charging_update", &evse_auto_start_charging_update, {}, [this](){
        //bs_evse_set_charging_autostart(evse_auto_start_charging_update.get("auto_start_charging")->asBool());
        // 1. set auto start
        // 2. make persistent
        // 3. fake a start/stop charging

        bool enable_auto_start = evse_auto_start_charging_update.get("auto_start_charging")->asBool();

        if (enable_auto_start) {
            this->apply_slot_default(CHARGING_SLOT_AUTOSTART_BUTTON, 32000, true, false);
        } else {
            this->apply_slot_default(CHARGING_SLOT_AUTOSTART_BUTTON, 0, true, true);
        }

        if (enable_auto_start) {
            evse_slots.get(CHARGING_SLOT_AUTOSTART_BUTTON)->get("max_current")->updateUint(32000);
        } else {
            // Only "stop" charging if no car is currently plugged in.
            // Clear on disconnect only triggers once, so we have to zero the current manually here.
            uint8_t iec_state = evse_state.get("iec61851_state")->asUint();
            if (iec_state != 2 && iec_state != 3)
                evse_slots.get(CHARGING_SLOT_AUTOSTART_BUTTON)->get("max_current")->updateUint(0);
        }
    }, false);

    api.addState("evse/global_current", &evse_global_current, {}, 1000);
    api.addCommand("evse/global_current_update", &evse_global_current_update, {}, [this](){
        uint16_t current = evse_global_current_update.get("current")->asUint();
        //is_in_bootloader(tf_evse_v2_set_charging_slot_max_current(&device, CHARGING_SLOT_GLOBAL, current));
        this->apply_slot_default(CHARGING_SLOT_GLOBAL, current, true, false);
    }, false);

    api.addState("evse/management_enabled", &evse_management_enabled, {}, 1000);
    api.addCommand("evse/management_enabled_update", &evse_management_enabled_update, {}, [this](){
        bool enabled = evse_management_enabled_update.get("enabled")->asBool();

        if (enabled == evse_management_enabled.get("enabled")->asBool())
            return;

        if (enabled)
            this->apply_slot_default(CHARGING_SLOT_CHARGE_MANAGER, 0, true, true);
        else
            this->apply_slot_default(CHARGING_SLOT_CHARGE_MANAGER, 32000, false, false);
    }, false);

    api.addState("evse/user_current", &evse_user_current, {}, 1000);
    api.addState("evse/user_enabled", &evse_user_enabled, {}, 1000);
    api.addCommand("evse/user_enabled_update", &evse_user_enabled_update, {}, [this](){
        bool enabled = evse_user_enabled_update.get("enabled")->asBool();

        if (enabled == evse_user_enabled.get("enabled")->asBool())
            return;

        if (enabled) {
            users.stop_charging(0, true);
        }

        if (enabled)
            this->apply_slot_default(CHARGING_SLOT_USER, 0, true, true);
        else
            this->apply_slot_default(CHARGING_SLOT_USER, 32000, false, false);
    }, false);

    api.addState("evse/external_enabled", &evse_external_enabled, {}, 1000);
    api.addCommand("evse/external_enabled_update", &evse_external_enabled_update, {}, [this](){
        bool enabled = evse_external_enabled_update.get("enabled")->asBool();

        if (enabled == evse_external_enabled.get("enabled")->asBool())
            return;

        this->apply_slot_default(CHARGING_SLOT_EXTERNAL, 32000, enabled, false);
    }, false);

    api.addState("evse/external_defaults", &evse_external_defaults, {}, 1000);
    api.addCommand("evse/external_defaults_update", &evse_external_defaults_update, {}, [this](){
        bool enabled;
        //tf_evse_v2_get_charging_slot_default(&device, CHARGING_SLOT_EXTERNAL, nullptr, &enabled, nullptr);
        this->apply_slot_default(CHARGING_SLOT_EXTERNAL, evse_external_defaults_update.get("current")->asUint(), evse_slots.get(CHARGING_SLOT_EXTERNAL)->get("active")->asBool(), evse_external_defaults_update.get("clear_on_disconnect")->asBool());
    }, false);

    api.addState("evse/modbus_tcp_enabled", &evse_modbus_enabled, {}, 1000);
    api.addCommand("evse/modbus_tcp_enabled_update", &evse_modbus_enabled_update, {}, [this](){
        bool enabled = evse_modbus_enabled_update.get("enabled")->asBool();

        if (enabled == evse_modbus_enabled.get("enabled")->asBool())
            return;

        if (enabled) {
            //tf_evse_v2_set_charging_slot(&device, CHARGING_SLOT_MODBUS_TCP, 32000, true, false);
            this->apply_slot_default(CHARGING_SLOT_MODBUS_TCP, 32000, true, false);

            //tf_evse_v2_set_charging_slot(&device, CHARGING_SLOT_MODBUS_TCP_ENABLE, 32000, true, false);
            this->apply_slot_default(CHARGING_SLOT_MODBUS_TCP_ENABLE, 32000, true, false);
        }
        else {
            //tf_evse_v2_set_charging_slot(&device, CHARGING_SLOT_MODBUS_TCP, 32000, false, false);
            this->apply_slot_default(CHARGING_SLOT_MODBUS_TCP, 32000, false, false);

            //tf_evse_v2_set_charging_slot(&device, CHARGING_SLOT_MODBUS_TCP_ENABLE, 32000, false, false);
            this->apply_slot_default(CHARGING_SLOT_MODBUS_TCP_ENABLE, 32000, false, false);
        }
    }, false);

    api.addState("evse/ocpp_enabled", &evse_ocpp_enabled, {}, 1000);
    api.addCommand("evse/ocpp_enabled_update", &evse_ocpp_enabled_update, {}, [this](){
        bool enabled = evse_ocpp_enabled_update.get("enabled")->asBool();

        if (enabled == evse_ocpp_enabled.get("enabled")->asBool())
            return;

        if (enabled) {
            this->apply_slot_default(CHARGING_SLOT_OCPP, 32000, true, false);
        }
        else {
            this->apply_slot_default(CHARGING_SLOT_OCPP, 32000, false, false);
        }
    }, false);

    this->register_my_urls();
}

void AC011K::loop()
{
    myloop();

#if MODULE_WS_AVAILABLE()
    static uint32_t last_debug = 0;
    if (debug && deadline_elapsed(last_debug + 50)) {
        last_debug = millis();
        ws.pushRawStateUpdate(this->get_evse_debug_line(), "evse/debug");
    }
#endif
}

void AC011K::setup_evse()
{
    my_setup_evse();

    this->apply_defaults();
    initialized = true;
}


void AC011K::update_all_data()
{
    if(!initialized)
        return;

    // get_all_data_1 - 51 byte
    uint8_t iec61851_state;
    uint8_t charger_state;
    uint8_t contactor_state;
    uint8_t contactor_error;
    uint16_t allowed_charging_current;
    uint16_t last_allowed_charging_current = evse_state.get("allowed_charging_current")->asUint();
    uint8_t error_state;
    uint8_t lock_state;
    uint8_t dc_fault_current_state;
    uint8_t jumper_configuration;
    bool has_lock_switch;
    uint8_t evse_version;
    uint8_t energy_meter_type;
    float power;
    float energy_relative;
    float energy_absolute;
    bool phases_active[3];
    bool phases_connected[3];
    uint32_t error_count[6];

    // get_all_data_2 - 19 byte
    uint8_t shutdown_input_configuration;
    uint8_t input_configuration;
    uint8_t output_configuration;
    int16_t indication;
    uint16_t duration;
    uint8_t button_configuration;
    uint32_t button_press_time;
    uint32_t button_release_time;
    bool button_pressed;
    uint8_t control_pilot;
    bool control_pilot_connected;

    // get_low_level_state - 57 byte
    uint8_t led_state;
    uint16_t cp_pwm_duty_cycle;
    uint16_t adc_values[7];
    int16_t voltages[7];
    uint32_t resistances[2];
    bool gpio[24];
    uint32_t charging_time;
    uint32_t time_since_state_change;
    uint32_t uptime;

    // get_all_charging_slots - 60 byte
    uint16_t max_current[20];
    uint8_t active_and_clear_on_disconnect[20];

    int rc = bs_evse_get_state(
        &iec61851_state,
        &charger_state,
        &contactor_state,
        &contactor_error,
        &allowed_charging_current,
        &error_state,
        &lock_state,
        &time_since_state_change,
        &uptime);

    evse_state.get("iec61851_state")->updateUint(iec61851_state);
    evse_state.get("charger_state")->updateUint(charger_state);
//logger.printfln("EVSE: charger_state %d", charger_state);
    evse_state.get("contactor_state")->updateUint(contactor_state);
    bool contactor_error_changed = evse_state.get("contactor_error")->updateUint(contactor_error);
    // TODO: implement current changes during charging (if possible)
    if(last_allowed_charging_current != allowed_charging_current) {
        evse_state.get("allowed_charging_current")->updateUint(allowed_charging_current);
        logger.printfln("EVSE: allowed_charging_current %d", allowed_charging_current);
        //bs_evse_set_max_charging_current(allowed_charging_current);  // Uwe: not needed here since requested by GD during start charging sequence
        logger.printfln("---->   bs_evse_set_max_charging_current function call dropped!");
    }

    bool low_level_mode_enabled;

    low_level_mode_enabled = true;
    led_state = 1;
    /* cp_pwm_duty_cycle = 100; */
    /* adc_values[0] = 200; */
    /* adc_values[1] = 201; */
    /* voltages[0] = 300; */
    /* voltages[1] = 301; */
    /* voltages[2] = 302; */
    /* resistances[0] = 400; */
    /* resistances[1] = 401; */
    /* gpio[0] = false; */
    /* gpio[1] = false; */
    /* gpio[2] = false; */
    /* gpio[3] = false; */
    /* gpio[4] = false; */

    uint16_t external_default_current;
    bool external_default_enabled;
    bool external_default_clear_on_disconnect;


    // We don't allow firmware updates when a vehicle is connected,
    // to be sure a potential EVSE firmware update does not interrupt a
    // charge and/or does strange stuff with the vehicle while updating.
    // However if we are in an error state, either after the EVSE update
    // the error is still there (this is fine for us) or it is cleared,
    // then the EVSE could potentially start to charge, which is okay,
    // as the ESP firmware is already running, so we can for example
    // track the charge.
    firmware_update_allowed = charger_state == 0 || charger_state == 4;

    // get_state

    /* We just can not get all the data and therefore ignore it and stick to our own fake default values. */
    /*
    evse_state.get("iec61851_state")->updateUint(iec61851_state);
    evse_state.get("charger_state")->updateUint(charger_state);
    evse_state.get("contactor_state")->updateUint(contactor_state);
    bool contactor_error_changed = evse_state.get("contactor_error")->updateUint(contactor_error);
    evse_state.get("allowed_charging_current")->updateUint(allowed_charging_current);
    bool error_state_changed = evse_state.get("error_state")->updateUint(error_state);
    evse_state.get("lock_state")->updateUint(lock_state);
    bool dc_fault_current_state_changed = evse_state.get("dc_fault_current_state")->updateUint(dc_fault_current_state);

    if (contactor_error_changed) {
        if (contactor_error != 0) {
            logger.printfln("EVSE: Contactor error %d", contactor_error);
        } else {
            logger.printfln("EVSE: Contactor error cleared");
        }
    }

    if (error_state_changed) {
        if (error_state != 0) {
            logger.printfln("EVSE: Error state %d", error_state);
        } else {
            logger.printfln("EVSE: Error state cleared");
        }
    }

    if (dc_fault_current_state_changed) {
        if (dc_fault_current_state != 0) {
            logger.printfln("EVSE: DC Fault current state %d", dc_fault_current_state);
        } else {
            logger.printfln("EVSE: DC Fault current state cleared");
        }
    }

    // get_hardware_configuration
    evse_hardware_configuration.get("jumper_configuration")->updateUint(jumper_configuration);
    evse_hardware_configuration.get("has_lock_switch")->updateBool(has_lock_switch);
    evse_hardware_configuration.get("evse_version")->updateUint(evse_version);
    evse_hardware_configuration.get("energy_meter_type")->updateUint(energy_meter_type);

    // get_low_level_state
    evse_low_level_state.get("led_state")->updateUint(led_state);
    evse_low_level_state.get("cp_pwm_duty_cycle")->updateUint(cp_pwm_duty_cycle);

    for (int i = 0; i < sizeof(adc_values) / sizeof(adc_values[0]); ++i)
        evse_low_level_state.get("adc_values")->get(i)->updateUint(adc_values[i]);

    for (int i = 0; i < sizeof(voltages) / sizeof(voltages[0]); ++i)
        evse_low_level_state.get("voltages")->get(i)->updateInt(voltages[i]);

    for (int i = 0; i < sizeof(resistances) / sizeof(resistances[0]); ++i)
        evse_low_level_state.get("resistances")->get(i)->updateUint(resistances[i]);

    for (int i = 0; i < sizeof(gpio) / sizeof(gpio[0]); ++i)
        evse_low_level_state.get("gpio")->get(i)->updateBool(gpio[i]);
    */

    evse_low_level_state.get("charging_time")->updateUint(charging_time);
    evse_low_level_state.get("time_since_state_change")->updateUint(time_since_state_change);
    //evse_state.get("time_since_state_change")->updateUint(time_since_state_change);
    evse_low_level_state.get("uptime")->updateUint(uptime);

    /*
    for (int i = 0; i < CHARGING_SLOT_COUNT; ++i) {
        evse_slots.get(i)->get("max_current")->updateUint(max_current[i]);
        evse_slots.get(i)->get("active")->updateBool(SLOT_ACTIVE(active_and_clear_on_disconnect[i]));
        evse_slots.get(i)->get("clear_on_disconnect")->updateBool(SLOT_CLEAR_ON_DISCONNECT(active_and_clear_on_disconnect[i]));
    }
    */

    evse_auto_start_charging.get("auto_start_charging")->updateBool(
        !evse_slots.get(CHARGING_SLOT_AUTOSTART_BUTTON)->get("clear_on_disconnect")->asBool());

    /*
    // get_energy_meter_values
    evse_energy_meter_values.get("power")->updateFloat(power);
    evse_energy_meter_values.get("energy_rel")->updateFloat(energy_relative);
    evse_energy_meter_values.get("energy_abs")->updateFloat(energy_absolute);

    for (int i = 0; i < 3; ++i)
        evse_energy_meter_values.get("phases_active")->get(i)->updateBool(phases_active[i]);

    for (int i = 0; i < 3; ++i)
        evse_energy_meter_values.get("phases_connected")->get(i)->updateBool(phases_connected[i]);

    // get_energy_meter_errors
    evse_energy_meter_errors.get("local_timeout")->updateUint(error_count[0]);
    evse_energy_meter_errors.get("global_timeout")->updateUint(error_count[1]);
    evse_energy_meter_errors.get("illegal_function")->updateUint(error_count[2]);
    evse_energy_meter_errors.get("illegal_data_access")->updateUint(error_count[3]);
    evse_energy_meter_errors.get("illegal_data_value")->updateUint(error_count[4]);
    evse_energy_meter_errors.get("slave_device_failure")->updateUint(error_count[5]);

    // get_gpio_configuration
    evse_gpio_configuration.get("shutdown_input")->updateUint(shutdown_input_configuration);
    evse_gpio_configuration.get("input")->updateUint(input_configuration);
    evse_gpio_configuration.get("output")->updateUint(output_configuration);

    // get_button_configuration
    evse_button_configuration.get("button")->updateUint(button_configuration);

    // get_button_state
    evse_button_state.get("button_press_time")->updateUint(button_press_time);
    evse_button_state.get("button_release_time")->updateUint(button_release_time);
    evse_button_state.get("button_pressed")->updateBool(button_pressed);

    // get_control_pilot
    evse_control_pilot_configuration.get("control_pilot")->updateUint(control_pilot);
    evse_control_pilot_connected.get("connected")->updateBool(control_pilot_connected);

    // get_indicator_led
    evse_indicator_led.get("indication")->updateInt(indication);
    evse_indicator_led.get("duration")->updateUint(duration);
    */

    evse_auto_start_charging.get("auto_start_charging")->updateBool(!SLOT_CLEAR_ON_DISCONNECT(active_and_clear_on_disconnect[CHARGING_SLOT_AUTOSTART_BUTTON]));

    evse_management_enabled.get("enabled")->updateBool(SLOT_ACTIVE(active_and_clear_on_disconnect[CHARGING_SLOT_CHARGE_MANAGER]));

    evse_user_enabled.get("enabled")->updateBool(SLOT_ACTIVE(active_and_clear_on_disconnect[CHARGING_SLOT_USER]));

    evse_modbus_enabled.get("enabled")->updateBool(SLOT_ACTIVE(active_and_clear_on_disconnect[CHARGING_SLOT_MODBUS_TCP]));
    evse_ocpp_enabled.get("enabled")->updateBool(SLOT_ACTIVE(active_and_clear_on_disconnect[CHARGING_SLOT_OCPP]));

    evse_external_enabled.get("enabled")->updateBool(SLOT_ACTIVE(active_and_clear_on_disconnect[CHARGING_SLOT_EXTERNAL]));
    evse_external_clear_on_disconnect.get("clear_on_disconnect")->updateBool(SLOT_CLEAR_ON_DISCONNECT(active_and_clear_on_disconnect[CHARGING_SLOT_EXTERNAL]));

    evse_global_current.get("current")->updateUint(max_current[CHARGING_SLOT_GLOBAL]);
    evse_management_current.get("current")->updateUint(max_current[CHARGING_SLOT_CHARGE_MANAGER]);
    evse_external_current.get("current")->updateUint(max_current[CHARGING_SLOT_EXTERNAL]);
    evse_user_current.get("current")->updateUint(max_current[CHARGING_SLOT_USER]);

    evse_external_defaults.get("current")->updateUint(external_default_current);
    evse_external_defaults.get("clear_on_disconnect")->updateBool(external_default_clear_on_disconnect);
}



/********************************************************************/
/* The code above is a copy of the TinkerForge evse_v2 module with  */ 
/* the needed alterations to allow it to work on th EN+ hardware.   */
/* This part may need to be in sync with the original for the other */ 
/* modules (backend as well as frontend) to work properly.          */
/*                                                                  */
/* Here starts the code that is specific to the EN+ hardware.       */
/********************************************************************/


bool ready_for_next_chunk = false;
size_t MAXLENGTH;
byte flash_seq;
uint32_t last_flash = 0;
bool phases_active[3];
bool phases_connected[3];
bool log_heartbeat = false;
//bool log_heartbeat = true;

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
byte Init10[] = {0xAA, 0x18, 0x12, 0x01, 0x00, 0x03, 0x3B, 0x9C}; // is Init10 the same as Init12? Probably 0x3B, 0x9C accidently copied from crc

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
byte TimeAck[] = {'c', 'a', 'y', 'm', 'd', 'h', 'm', 's', 0, 0, 0, 0};

// ctrl_cmd set start power mode done
byte Init15[] = {0xAA, 0x18, 0x09, 0x01, 0x00, 0x00};
//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 40 05 00 18 09 01 00 00 F9 36
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 40 05 00 14 09 01 00 00 11 30
//I (2021-04-11 18:36:27) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower: 3150080

//privCommCmdA7StartTransAck GD Firmware before 1.1.212?
byte StartChargingA6[] = {0xA6, 'W', 'A', 'R', 'P', ' ', 'c', 'h', 'a', 'r', 'g', 'e', 'r', ' ', 'f', 'o', 'r', ' ', 'E', 'N', '+', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x30, 0, 0, 0, 0, 0, 0, 0, 0};
byte StopChargingA6[]  = {0xA6, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, '0', '0', '0', '0', '0', '0', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x40, 0, 0, 0, 0, 0, 0, 0, 0};

byte StartChargingA7[] = {0xA7, '0', '0', '0', '0', '0', '0', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte StopChargingA7[]  = {0xA7, '0', '0', '0', '0', '0', '0', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x10, 0};

byte TransactionAck[] = {0xA9, 'W', 'A', 'R', 'P', ' ', 'c', 'h', 'a', 'r', 'g', 'e', 'r', ' ', 'f', 'o', 'r', ' ', 'E', 'N', '+', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

//privCommCmdAFSmartCurrCtl
byte ChargingLimit1[] = {0xAF, 0, 'y', 'm', 'd', 'h', 'm', 's', 0x80, 0x51, 0x01, 0, 0x01, 0, 0, 0, 0, 'A', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte ChargingLimit2[] = {0xAD, 0, 0x44, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 'y', 'm', 'd', 'h', 'm', 's', 0, 0, 0, 0, 'A', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x03, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
byte ChargingLimit3[] = {0xAD, 1, 0x91, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 'Y', 'm', 'd', 'h', 'm', 's', 0, 0, 0, 0, 0xFF, 0xFF, 0xFF, 0xFF, 'A', 0, 0, 0, 0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//byte ChargingLimit1b[] = {0xAF, 0, 'y', 'm', 'd', 'h', 'm', 's', 0x80, 0x51, 0x01, 0, 0x02, 0, 0, 0, 0, 0x10, 0x01, 0, 0, 0, 'A', 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//GD1.1.538  [2,"e3031518-5180-4c86-98ea-77fe478b09d1","SetChargingProfile",{"connectorId":1,"csChargingProfiles":{"chargingProfileKind":"Relative","chargingProfilePurpose": "TxDefaultProfile","chargingSchedule":{"startSchedule":"2022-01-10T20:29:10Z","chargingSchedulePeriod":[{"startPeriod":0,"limit":6.0,"numberPhases":3}],"chargingRateUnit":"A"},"chargingProfileId":2013,"stackLevel":0}}]
// Tx(cmd_AD len:122) : 0xAD,     0, 0xDD, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, y, m, d, h, m, s, 0, 0, 0, 0, ChargeLimitA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x03, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
// Rx(cmd_0D, len:13) : 0x0D,     0, 0, 0
// Rx(cmd_0F, len:41) : 0x0F,     0, y, m, d, h, m, s, 0x80, 0x51, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
// Tx(cmd_AF, len:292) : 0xAF,    0, y, m, d, h, m, s, 0x80, 0x51, 0x01, 0, 0x02, 0, 0, 0, 0, 0x10, 0x01, 0, 0, 0, ChargeLimitA, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0

//privCommCmdADSmartchargeCtl
//byte ChargingLimit3[] = {0xAD, 0x01, 0x91, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x01, 0, 0, 0,  0, 0x79, 0x08, 0x16, 0x10, 0x04, 0x10, 0, 0, 0,  0, 0xFF, 0xFF, 0xFF, 0xFF, 0x06, 0, 0, 0, 0x03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//byte ChargingLimit4[] = {0xAD,    0, 0xDD,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x02, 0xFF, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,    0, 0, 0, 0, 'y', 'm',  'd',  'h',  'm',  's',    0, 0, 0, 0, 'A',  0,    0,    0,    0,    0,  0, 0, 0,    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0x03, 0x01, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

// Enter boot mode: Tx(cmd_AB len: 20): FA 03 00 00 AB 14 0A 00 00 00 00 00 00 00 05 00 00 00 62 B2
// ESP> W (2021-10-04 10:04:35) [PRIV_COMM, 1875]: Tx(cmd_AB len:20) :  FA 03 00 00 AB 15 0A 00 00 00 00 00 00 00 05 00 00 00 60 33  ^M
// Enter boot mode acknowledge: Rx(cmd_0B len: 16): FA 03 00 00 0B 14 06 00 00 00 05 00 00 00 F1 3A
// erase flash
// ESP> W (2021-10-04 10:04:39) [PRIV_COMM, 1875]: Tx(cmd_AB len:20) :  FA 03 00 00 AB 17 0A 00 00 00 00 00 00 00 02 00 00 00 66 05  ^M
//byte EnterBootMode[] = {0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00};
byte RemoteUpdate[] = {0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00}; // #7 is the operation command [0-5]
// Exit boot mode, enter application mode: Tx (cmd_AB len: 24): FA 03 00 00 AB 5A 0E 00 00 00 03 08 FC FF 00 00 02 00 4F 4B 00 00 E1 98
byte EnterAppMode[] = {0xAB, 0x00, 0x00, 0x03, 0x08, 0xFC, 0xFF, 0x00, 0x00, 0x02, 0x00, 0x4F, 0x4B, 0x00, 0x00};
//Handshake: //Tx (cmd_AB len:20): FA 03 00 00 AB 15 0A 00 00 00 00 00 00 00 01 00 00 00 61 03
// ESP> W (2021-10-04 10:04:39) [PRIV_COMM, 1875]: Tx(cmd_AB len:20) :  FA 03 00 00 AB 16 0A 00 00 00 00 00 00 00 01 00 00 00 64 C0  ^M
byte Handshake[] =   {0xAB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
//Flash verify:
//byte FlashVerify[] = {0xAB, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x04, 0x00, 40, 0x00, /* 40 words */
byte FlashVerify[811] = {0xAB, 0x00, 0x00, 0x00, 0x08, 0x00, 0x00, 0x04, 0x00, 0x90, 0x01, /* 400 words */
     /*  0x00  */     0x68, 0x16, 0x00, 0x20, 0x1d, 0x25, 0x00, 0x08, 0x3b, 0x0e, 0x00, 0x08, 0x3d, 0x0e, 0x00, 0x08, /* example data */
     /*  0x10  */     0x41, 0x0e, 0x00, 0x08, 0x45, 0x0e, 0x00, 0x08, 0x49, 0x0e, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00,
     /*  0x20  */     0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4d, 0x0e, 0x00, 0x08,
     /*  0x30  */     0x4f, 0x0e, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x51, 0x0e, 0x00, 0x08, 0x29, 0x0b, 0x00, 0x08,
     /*  0x40  */     0x33, 0x25, 0x00, 0x08, 0x33, 0x25, 0x00, 0x08, 0x33, 0x25, 0x00, 0x08, 0x33, 0x25, 0x00, 0x08  /* filled with zeros automatically */ };

/*

37977                 000: FA 03 00 00 AB 5C 5A 00 00 00 00 00 00 00 04 00 28 00 68 16
                      020: 00 20 1D 25 00 08 3B 0E 00 08 3D 0E 00 08 41 0E 00 08 45 0E
                      040: 00 08 49 0E 00 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00
                      060: 00 00 4D 0E 00 08 4F 0E 00 08 00 00 00 00 51 0E 00 08 29 0B
                      080: 00 08 33 25 00 08 33 25 00 08 33 25 00 08 33 25 00 08 E8 14
38006       Tx cmd_AB seq:5C, len:100, crc:14E8
38073                 000: FA 03 00 00 0B 5C 06 00 00 00 04 00 01 00 A7 55
38073       Rx cmd_0B seq:5C len:6 crc:55A7

*/



#ifdef GD_FLASH
#include "enplus_firmware.h"
//#include "enplus_firmware.1.0.1435h"  // RFID, 1 Ampere limit steps
#include "enplus_firmware.1.1.212.h"  // no RFID but climatization possible after charging completed, charging limits 8A/10A/13A/16A only
//#include "enplus_firmware.1.1.258.h"  // RFID, no climatization possible after charging completed, 1 Ampere limit steps
//#include "enplus_firmware.1.1.538.h"  // RFID, no climatization possible after charging completed, 1 Ampere limit steps
//#include "enplus_firmware.1.1.805.h"  // RFID, no climatization possible after charging completed, 1 Ampere limit steps
#include "enplus_firmware.1.1.888.h"  // RFID, no climatization possible after charging completed, 1 Ampere limit steps
#endif

#include "HardwareSerial.h"
#include <time.h>

/* experimental: JSON data sender */
#include <HTTPClient.h>
const char* JSON_DECODER = "";  // e.g. "http://xxx.xxx.xxx.xxx/test.php", should be defined via configuration settings
HTTPClient httpClient;  // for sending JSON data

void send_http (String jmessage) {
    if (JSON_DECODER[0] != '\0') {  // if target url present
        httpClient.begin(JSON_DECODER);
        httpClient.addHeader("Content-Type", "application/json");

        int httpResponseCode = httpClient.POST("{\"version\":1,\"device\":\"" + String((uint32_t)ESP.getEfuseMac(), HEX) + "\"" + jmessage);  // prepend chip ID and send data
        logger.printfln("JSON message sent. Response code: %d", httpResponseCode);
        //logger.printfln("Response: %s", http.getString().c_str());  // print response
        httpClient.end();  // disconnect
    }
}

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

String AC011K::get_hex_privcomm_line(byte *data) {
    #define LOG_LEN 4048 //TODO work without such a big buffer by writing line by line
    char log[LOG_LEN] = {0};

    if (!(data[4] == 0xAB and data[14] == 3)) { // suppress logging of the whole GD firmware during flashing

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
    
    }
    
    return String(log);
}

void AC011K::Serial2write(byte *data, int size) {
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

char timeString[20];  // global since local variable could not be used as return value
const char* AC011K::timeStr(byte *data, uint8_t offset=0) {
    sprintf(timeString, "%04d/%02d/%02d %02d:%02d:%02d",
        data[offset++]+2000, data[offset++], data[offset++],
        data[offset++], data[offset++], data[offset]
    );
    return timeString;
}

void AC011K::sendCommand(byte *data, int datasize, byte sendSequenceNumber) {
    PrivCommTxBuffer[4] = data[0]; // command code
    PrivCommTxBuffer[5] = sendSequenceNumber;
    PrivCommTxBuffer[6] = (datasize-1) & 0xFF;
    PrivCommTxBuffer[7] = (datasize-1) >> 8;
    memcpy(PrivCommTxBuffer+8, data+1, datasize-1);

    uint16_t crc = crc16_modbus(PrivCommTxBuffer, datasize + 7);

    PrivCommTxBuffer[datasize+7] = crc & 0xFF;
    PrivCommTxBuffer[datasize+8] = crc >> 8;

    get_hex_privcomm_line(PrivCommTxBuffer); // PrivCommHexBuffer now holds the hex representation of the buffer
    String cmdText = "";
    switch (PrivCommTxBuffer[4]) {
        case 0xA3: cmdText = "- Status data ack"; break;
        case 0xA4: cmdText = "- Heartbeat ack " + String(timeStr(PrivCommTxBuffer+9)); break;
        case 0xA6: if (PrivCommTxBuffer[72] == 0x40) cmdText = "- Stop charging request"; else cmdText = "- Start charging request"; break;
        case 0xA7: if (PrivCommTxBuffer[40] == 0x10) cmdText = "- Stop charging command"; else cmdText = "- Start charging command"; break;
        case 0xA8: cmdText = "- Power data ack"; break;
        case 0xA9: cmdText = "- Transaction data ack"; break;
        case 0xAF: cmdText = "- Limit1: " + String(PrivCommTxBuffer[24]) + " Ampere"; break;
        case 0xAD: if (PrivCommTxBuffer[8] == 0) cmdText = "- Limit2: " + String(PrivCommTxBuffer[72]) + " Ampere"; else cmdText = "- Limit3: " + String(PrivCommTxBuffer[77]) + " Ampere"; break;
    }
    logger.printfln("Tx cmd_%.2X seq:%.2X len:%d crc:%.4X %s", PrivCommTxBuffer[4], PrivCommTxBuffer[5], datasize+9, crc, cmdText.c_str());

    Serial2write(data, datasize + 9);
}

void AC011K::PrivCommSend(byte cmd, uint16_t datasize, byte *data) {
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
}

void AC011K::PrivCommAck(byte cmd, byte *data) {
    // the first 4 bytes never change and should be set already
    data[4] = cmd ^ 0xA0;  // complement command for ack
    data[6] = 1; //len
    //data[5]=seq; //should already be copied from RxBuffer
    data[7] = 0; //len
    data[8] = 0; //payload
    uint16_t crc = crc16_modbus(data, 9);
    data[9] = crc & 0xFF;
    data[10] = crc >> 8;

    if(log_heartbeat || cmd!=2) { // be silent for the heartbeat //TODO show it at first and after an hour?
        get_hex_privcomm_line(data); // PrivCommHexBuffer now holds the hex representation of the buffer
        logger.printfln("Tx cmd_%.2X seq:%.2X, crc:%.4X", data[4], data[5], crc);
    }

    Serial2write(data, 11);
}

void AC011K::sendTime(byte cmd, byte action, byte len, byte sendSequenceNumber) {
    TimeAck[0] = cmd;
    TimeAck[1] = action;
    filltime(&TimeAck[2], &TimeAck[3], &TimeAck[4], &TimeAck[5], &TimeAck[6], &TimeAck[7]);
    // TimeAck[8] to TimeAck[11] are always 0
    sendCommand(TimeAck, len, sendSequenceNumber);
}

void AC011K::sendTimeLong(byte sendSequenceNumber) {
    PrivCommTxBuffer[5] = sendSequenceNumber;
    PrivCommTxBuffer[PayloadStart + 0] = 0x18;
    PrivCommTxBuffer[PayloadStart + 1] = 0x02;
    PrivCommTxBuffer[PayloadStart + 2] = 0x06;
    PrivCommTxBuffer[PayloadStart + 3] = 0x00;
    filltime(&PrivCommTxBuffer[PayloadStart + 4], &PrivCommTxBuffer[PayloadStart + 5], &PrivCommTxBuffer[PayloadStart + 6], &PrivCommTxBuffer[PayloadStart + 7], &PrivCommTxBuffer[PayloadStart + 8], &PrivCommTxBuffer[PayloadStart + 9]);
    logger.printfln("set GD time to: %s", timeStr(&PrivCommTxBuffer[PayloadStart + 4]));
    PrivCommSend(0xAA, 10, PrivCommTxBuffer);
}

void AC011K::sendChargingLimit1(uint8_t currentLimit, byte sendSequenceNumber) {  // AF 00 date/time
    filltime(&ChargingLimit1[2], &ChargingLimit1[3], &ChargingLimit1[4], &ChargingLimit1[5], &ChargingLimit1[6], &ChargingLimit1[7]);
    ChargingLimit1[17] = currentLimit;
    sendCommand(ChargingLimit1, sizeof(ChargingLimit1), sendSequenceNumber);
}

void AC011K::sendChargingLimit2(uint8_t currentLimit, byte sendSequenceNumber) {  // AD 00
//    ChargingLimit2[2] = 8;  // charging profile ID - 0x41 for 1.0.1435 ?
    filltime(&ChargingLimit2[55], &ChargingLimit2[56], &ChargingLimit2[57], &ChargingLimit2[58], &ChargingLimit2[59], &ChargingLimit2[60]);
    ChargingLimit2[65] = currentLimit;
    sendCommand(ChargingLimit2, sizeof(ChargingLimit2), sendSequenceNumber);
}

void AC011K::sendChargingLimit3(uint8_t currentLimit, byte sendSequenceNumber) {  //  AD 01 91
    filltime(&ChargingLimit3[56], &ChargingLimit3[57], &ChargingLimit3[58], &ChargingLimit3[59], &ChargingLimit3[60], &ChargingLimit3[61]);
    ChargingLimit3[56] = ChargingLimit3[56] +100;  // adds 100 to the year, because it starts at the year 1900
    ChargingLimit3[70] = currentLimit;
    sendCommand(ChargingLimit3, sizeof(ChargingLimit3), sendSequenceNumber);
}


int AC011K::bs_evse_start_charging() {
    //evse_state.get("allowed_charging_current")->updateUint(11000);//TODO this is just a test and does not belong here at all
    uint8_t allowed_charging_current = uint8_t(evse_state.get("allowed_charging_current")->asUint()/1000);
    logger.printfln("EVSE start charging with max %d Ampere", allowed_charging_current);

    switch (evse_hardware_configuration.get("GDFirmwareVersion")->asUint()) {
        case 212:
            sendCommand(StartChargingA6, sizeof(StartChargingA6), sendSequenceNumber++);  // max current is switched to 16A when plugged out to ensure full current range available
            sendChargingLimit3(allowed_charging_current, sendSequenceNumber++);  // reduce to intended value
            break;
        default:
            logger.printfln("Unknown firmware version. Trying commands for latest version.");
        case 258:
        case 538:
        case 805:
        case 812:
        case 888:
        case 1435:
            sendCommand(StartChargingA6, sizeof(StartChargingA6), sendSequenceNumber++);
            break;
    }
    return 0;
}

int AC011K::bs_evse_stop_charging() {
    logger.printfln("EVSE stop charging");
    sendCommand(StopChargingA6, sizeof(StopChargingA6), sendSequenceNumber++);
    return 0;
}

int AC011K::bs_evse_persist_config() {
    String error = api.callCommand("evse/config_update", Config::ConfUpdateObject{{
        {"auto_start_charging", evse_auto_start_charging.get("auto_start_charging")->asBool()},
        {"max_current_configured", evse_max_charging_current.get("max_current_configured")->asUint()},
        //{"managed", evse_managed.get("managed")->asBool()}
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

int AC011K::bs_evse_set_charging_autostart(bool autostart) {
    logger.printfln("EVSE set auto start charging to %s", autostart ? "true" :"false");
    evse_auto_start_charging.get("auto_start_charging")->updateBool(autostart);
    bs_evse_persist_config();
    return 0;
}

int AC011K::bs_evse_set_max_charging_current(uint16_t max_current) {
    evse_max_charging_current.get("max_current_configured")->updateUint(max_current);
    bs_evse_persist_config();
    update_all_data();
    uint8_t allowed_charging_current = evse_state.get("allowed_charging_current")->asUint()/1000;
    logger.printfln("EVSE set configured charging limit to %d Ampere", uint8_t(max_current/1000));
    logger.printfln("EVSE calculated allowed charging limit is %d Ampere", allowed_charging_current);
    switch (evse_hardware_configuration.get("GDFirmwareVersion")->asUint()) {
        case 212:
            sendChargingLimit3(allowed_charging_current, sendSequenceNumber++);
            break;
        default:
            logger.printfln("Unknown firmware version. Trying commands for latest version.");
        case 258:
        case 538:
        case 805:
        case 812:
        case 888:
        case 1435:
            sendChargingLimit1(allowed_charging_current, sendSequenceNumber++);
    }
    return 0;
}

int AC011K::bs_evse_get_state(uint8_t *ret_iec61851_state, uint8_t *ret_charger_state, uint8_t *ret_contactor_state, uint8_t *ret_contactor_error, uint16_t *ret_allowed_charging_current, uint8_t *ret_error_state, uint8_t *ret_lock_state, uint32_t *ret_time_since_state_change, uint32_t *ret_uptime) {
//    bool response_expected = true;
//    tf_tfp_prepare_send(evse->tfp, TF_EVSE_FUNCTION_GET_STATE, 0, 17, response_expected);

    *ret_iec61851_state = evse_state.get("iec61851_state")->asUint();
    *ret_charger_state = evse_state.get("iec61851_state")->asUint(); // == 1 ? // charging ? 2 : 1; // 1 verbunden 2 leadt
    *ret_contactor_state = 2; // 1 - Stromführend vor, aber nicht stromführend nach dem Schütz, 3 - Stromführend vor und nach dem Schütz
    *ret_contactor_error = 0;
    //*ret_charge_release = 1; // manuell 0 automatisch
    // find the charging current maximum
    uint16_t allowed_charging_current = 32000;
    for (int i = 0; i < CHARGING_SLOT_COUNT; ++i) {
        if (!evse_slots.get(i)->get("active")->asBool())
            continue;
        allowed_charging_current = min(allowed_charging_current, (uint16_t)evse_slots.get(i)->get("max_current")->asUint());
    }
    *ret_allowed_charging_current = allowed_charging_current;
    *ret_error_state = 0;
    *ret_lock_state = 0;
    *ret_time_since_state_change = evse_state.get("time_since_state_change")->asUint();
    *ret_uptime = millis();

    return TF_E_OK;
}

void AC011K::update_evseStatus(uint8_t evseStatus) {
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
            for (int i = 0; i < 3; ++i)
                phases_active[i] = phases_connected[i];
            break;
        case 4:                                              // Suspended by charger (started but no power available)
            evse_state.get("iec61851_state")->updateUint(1); // Verbunden
            break;
        case 5:                                              // Suspended by EV (power available but waiting for the EV response)
            evse_state.get("iec61851_state")->updateUint(1); // Verbunden
            break;
        case 6:                                              // Finishing, charging acomplished (RFID stop or EMS control stop)
            evse_state.get("iec61851_state")->updateUint(1); // Verbunden
            for (int i = 0; i < 3; ++i)
                phases_active[i] = false;
                    // clear meter values
                    // voltages
                    meter.updateMeterAllValues(METER_ALL_VALUES_LINE_TO_NEUTRAL_VOLTS_L1, 0);
                    meter.updateMeterAllValues(METER_ALL_VALUES_LINE_TO_NEUTRAL_VOLTS_L2, 0);
                    meter.updateMeterAllValues(METER_ALL_VALUES_LINE_TO_NEUTRAL_VOLTS_L3, 0);
                    // current
                    meter.updateMeterAllValues(METER_ALL_VALUES_CURRENT_L1_A, 0);
                    meter.updateMeterAllValues(METER_ALL_VALUES_CURRENT_L2_A, 0);
                    meter.updateMeterAllValues(METER_ALL_VALUES_CURRENT_L3_A, 0);

                    /* // meter power */
                    /* meter.updateMeterValues( */
                    /*           0,  // charging power W  (power) */
                    /*           PrivCommRxBuffer[84]+256*PrivCommRxBuffer[85],  // charged energy Wh (energy_rel) */
                    /*           PrivCommRxBuffer[88]+256*PrivCommRxBuffer[89]   // charged energy Wh (energy_abs) */
                    /*           ); */

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
    meter.updateMeterPhases(phases_connected, phases_active);
    
    if(last_iec61851_state != evse_state.get("iec61851_state")->asUint()) {
        evse_state.get("last_state_change")->updateUint(millis());
        evse_state.get("time_since_state_change")->updateUint(millis() - evse_state.get("last_state_change")->asUint());
	if((evseStatus != last_evseStatus == 1) && (evseStatus == 1)) { // plugged out
            if (evse_hardware_configuration.get("GDFirmwareVersion")->asUint() == 212)
                //sendChargingLimit2(16, sendSequenceNumber++);  // hack to ensure full current range is available in next charging session 
                sendChargingLimit3(16, sendSequenceNumber++);  // hack to ensure full current range is available in next charging session
        }
	if(evseStatus == 2 && last_evseStatus == 1) { // just plugged in
            transactionNumber++;
            char buffer[13];
            sprintf(buffer, "%06d", transactionNumber);
            for (int i=0; i<6; i++) {  // patch transaction number into command templates
                StartChargingA7[i+1] = byte(buffer[i]);
                StopChargingA7[i+1] = byte(buffer[i]);
                StopChargingA6[i+33] = byte(buffer[i]);
            }
            logger.printfln("New transaction number %05d", transactionNumber);
        }
	if(evse_auto_start_charging.get("auto_start_charging")->asBool()
           && evseStatus == 2 || (evseStatus == 6 && last_evseStatus == 0)) { // just plugged in or already plugged in at startup
            logger.printfln("Start charging automatically");
            //update_evseStatus(evseStatus); TODO stört dies beim ladestart?
            bs_evse_start_charging();
        }
    }
    evse_state.get("charger_state")->updateUint(evse_state.get("iec61851_state")->asUint());
}


#ifdef GD_FLASH
/* GD Firmware updater */

bool AC011K::handle_update_chunk(int command, WebServerRequest request, size_t chunk_index, uint8_t *data, size_t chunk_length, bool final, size_t complete_length) {

    if(chunk_index == 0) {
 /* [PRIV_COMM, 1875]: Tx(cmd_AB len:820) :  FA 03 00 00 AB 18 2A 03 00 00 00 08 00 00 03 00 90 01 68 16 00 20 1D 25 00 08 3B 0E 00 08 3D 0E 00 08 41 0E 00 08 45 0E 00 08 49 0E 00 08 00 00 00 00 00 00 */
        //sendCommand(EnterBootMode, sizeof(EnterBootMode), sendSequenceNumber++);
        logger.printfln("EVSE RemoteUpdate, reset into boot mode");
        RemoteUpdate[7] = 5; // Reset into boot mode
        sendCommand(RemoteUpdate, sizeof(RemoteUpdate), sendSequenceNumber++);
        /* logger.printfln("Failed to start update: %s", Update.errorString()); */
        /* request.send(400, "text/plain", Update.errorString()); */
        /* update_aborted = true; */
        /* return true; */
    }

    size_t chunk_offset = 0;
    size_t length = chunk_length;

    FlashVerify[7] = command; // flash write (3=write, 4=verify)

    while (length > 0) {
        while (!ready_for_next_chunk) {
            loop(); //TODO make this more elegant
        }

        //calculate maxlength
        size_t maxlength = MIN(length, length % 800); // 800 bytes is the max flash verify/write size
        maxlength = maxlength > 0 ? maxlength : 800;  // process the reminder first, then 800b chunks
        FlashVerify[9]  = (maxlength/2 & 0x000000FF); // number of words to process (therefore divided by 2)
        FlashVerify[10] = (maxlength/2 & 0x0000FF00) >> 8;

        //calculate address
        uint32_t gd_address = chunk_index + chunk_offset + 0x8000000; // 0x8000000 is the start address for the GD chip
        FlashVerify[5] = (gd_address & 0x000000FF);
        FlashVerify[6] = (gd_address & 0x0000FF00) >> 8;
        FlashVerify[3] = (gd_address & 0x00FF0000) >> 16;
        FlashVerify[4] = (gd_address & 0xFF000000) >> 24;

        //logger.printfln("Processing update chunk with: chunk_index %.6X (%d), gd(%.2x %.2x %.2x %.2x) chunk_l %d, chunk_offset %d, complete_l %d, final: %s", chunk_index, chunk_index, FlashVerify[3],FlashVerify[4],FlashVerify[5],FlashVerify[6], chunk_length, chunk_offset, complete_length, final?"true":"false");
        logger.printfln("c_index %d, gd(%.2x %.2x %.2x %.2x) chunk_l %d, chunk_offset %d, l %d, ml %d, ll %d, final: %s", chunk_index, FlashVerify[3],FlashVerify[4],FlashVerify[5],FlashVerify[6], chunk_length, chunk_offset, length, maxlength, complete_length, final?"true":"false");

        if (update_aborted)
            return true;

        // copy data
        memcpy(FlashVerify+11, data + chunk_offset, maxlength);

        MAXLENGTH = maxlength;
        sendCommand(FlashVerify, maxlength+11, sendSequenceNumber++); // next chunk (11 bytes header) 
        flash_seq = PrivCommTxBuffer[5];
        last_flash = millis();
        ready_for_next_chunk = false;

        chunk_offset = chunk_offset + maxlength;
        length = length - maxlength;
    } // iterate through big chunks

    if(final) {
        this->firmware_update_running = false;
        logger.printfln("   scheduling GD chip app mode in 3s");
        // after last chunk, get out of flash mode
        task_scheduler.scheduleOnce([this](){
            logger.printfln("   getting the GD chip back into app mode");
            sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++);
        }, 3000);
    }

    return true;
}

bool AC011K::handle_update_chunk1(int command, WebServerRequest request, size_t chunk_index, uint8_t *data, size_t chunk_length, bool final, size_t complete_length) {

    if(chunk_index == 0) {
 /* [PRIV_COMM, 1875]: Tx(cmd_AB len:820) :  FA 03 00 00 AB 18 2A 03 00 00 00 08 00 00 03 00 90 01 68 16 00 20 1D 25 00 08 3B 0E 00 08 3D 0E 00 08 41 0E 00 08 45 0E 00 08 49 0E 00 08 00 00 00 00 00 00 */
        //sendCommand(EnterBootMode, sizeof(EnterBootMode), sendSequenceNumber++);
        logger.printfln("EVSE RemoteUpdate, reset into boot mode");
        RemoteUpdate[7] = 5; // Reset into boot mode
        sendCommand(RemoteUpdate, sizeof(RemoteUpdate), sendSequenceNumber++);
        /* logger.printfln("Failed to start update: %s", Update.errorString()); */
        /* request.send(400, "text/plain", Update.errorString()); */
        /* update_aborted = true; */
        /* return true; */

        size_t chunk_offset = 0 + 0x8000;
        size_t length = gd_firmware_len - 0x8000;

        FlashVerify[7] = command; // flash write (3=write, 4=verify)

        while (length > 0) {
            while (!ready_for_next_chunk) {
                loop(); //TODO make this more elegant
            }

            //calculate maxlength
            //size_t maxlength = 800;               // 800 byte chunks
            size_t maxlength = 512;               // 512 byte chunks
            //if (length < 800) maxlength = length; // reminder
            if (length < 512) maxlength = length; // reminder
            FlashVerify[9]  = (maxlength/2 & 0x000000FF); // number of words to process (therefore divided by 2)
            FlashVerify[10] = (maxlength/2 & 0x0000FF00) >> 8;

            //calculate address
            uint32_t gd_address = chunk_index + chunk_offset + 0x8000000; // 0x8000000 is the start address for the GD chip
            FlashVerify[5] = (gd_address & 0x000000FF);
            FlashVerify[6] = (gd_address & 0x0000FF00) >> 8;
            FlashVerify[3] = (gd_address & 0x00FF0000) >> 16;
            FlashVerify[4] = (gd_address & 0xFF000000) >> 24;

            //logger.printfln("Processing update chunk with: chunk_index %.6X (%d), gd(%.2x %.2x %.2x %.2x) chunk_l %d, chunk_offset %d, complete_l %d, final: %s", chunk_index, chunk_index, FlashVerify[3],FlashVerify[4],FlashVerify[5],FlashVerify[6], chunk_length, chunk_offset, complete_length, final?"true":"false");
            logger.printfln("gd(%.2x %.2x %.2x %.2x) binhex(%.2x%.2x) chunk_offset %d, l %d, ml %d, ll %d, final: %s", FlashVerify[3],FlashVerify[4],FlashVerify[5],FlashVerify[6], FlashVerify[6],FlashVerify[5], chunk_offset, length, maxlength, complete_length, final?"true":"false");

            if (update_aborted)
                return true;

            // copy data
            memcpy(FlashVerify+11, gd_firmware_1_1_212 + chunk_offset, maxlength);  // firmware file for upload button

            MAXLENGTH = maxlength;
            sendCommand(FlashVerify, maxlength+11, sendSequenceNumber++); // next chunk (11 bytes header) 
            flash_seq = PrivCommTxBuffer[5];
            last_flash = millis();
            ready_for_next_chunk = false;

            chunk_offset = chunk_offset + maxlength;
            length = length - maxlength;
        } // iterate through big chunks
    } // first chunk

    if(final) {
        this->firmware_update_running = false;
        logger.printfln("   scheduling GD chip app mode in 3s");
        // after last chunk, get out of flash mode
        task_scheduler.scheduleOnce([this](){
            logger.printfln("   getting the GD chip back into app mode");
            sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++);
        }, 3000);
    }

    return true;
}

bool AC011K::handle_update_chunk2(int command, WebServerRequest request, size_t chunk_index, uint8_t *data, size_t chunk_length) {

    if(chunk_index == 0) {
 /* [PRIV_COMM, 1875]: Tx(cmd_AB len:820) :  FA 03 00 00 AB 18 2A 03 00 00 00 08 00 00 03 00 90 01 68 16 00 20 1D 25 00 08 3B 0E 00 08 3D 0E 00 08 41 0E 00 08 45 0E 00 08 49 0E 00 08 00 00 00 00 00 00 */
        //sendCommand(EnterBootMode, sizeof(EnterBootMode), sendSequenceNumber++);
        logger.printfln("EVSE RemoteUpdate, reset into boot mode");
        RemoteUpdate[7] = 5; // Reset into boot mode
        sendCommand(RemoteUpdate, sizeof(RemoteUpdate), sendSequenceNumber++);
        /* logger.printfln("Failed to start update: %s", Update.errorString()); */
        /* request.send(400, "text/plain", Update.errorString()); */
        /* update_aborted = true; */
        /* return true; */

        size_t chunk_offset = 0 + 0x8000;
        size_t length = gd_firmware_len - 0x8000;

        FlashVerify[7] = command; // flash write (3=write, 4=verify)

        while (length > 0) {
            while (!ready_for_next_chunk) {
                loop(); //TODO make this more elegant
            }

            //calculate maxlength
            //size_t maxlength = 800;               // 800 byte chunks
            size_t maxlength = 512;               // 512 byte chunks
            //if (length < 800) maxlength = length; // reminder
            if (length < 512) maxlength = length; // reminder
            FlashVerify[9]  = (maxlength/2 & 0x000000FF); // number of words to process (therefore divided by 2)
            FlashVerify[10] = (maxlength/2 & 0x0000FF00) >> 8;

            //calculate address
            uint32_t gd_address = chunk_index + chunk_offset + 0x8000000; // 0x8000000 is the start address for the GD chip
            FlashVerify[5] = (gd_address & 0x000000FF);
            FlashVerify[6] = (gd_address & 0x0000FF00) >> 8;
            FlashVerify[3] = (gd_address & 0x00FF0000) >> 16;
            FlashVerify[4] = (gd_address & 0xFF000000) >> 24;

            logger.printfln("gd(%.2x%.2x%.2x%.2x) chunk_offset %d, l %d, ml %d", FlashVerify[3],FlashVerify[4],FlashVerify[5],FlashVerify[6], chunk_offset, length, maxlength);

            if (update_aborted)
                return true;

            // copy data
            memcpy(FlashVerify+11, gd_firmware_1_1_888 + chunk_offset, maxlength);  //  firmware file for verify button

            MAXLENGTH = maxlength;
            sendCommand(FlashVerify, maxlength+11, sendSequenceNumber++); // next chunk (11 bytes header) 
            flash_seq = PrivCommTxBuffer[5];
            last_flash = millis();
            ready_for_next_chunk = false;

            chunk_offset = chunk_offset + maxlength;
            length = length - maxlength;
        } // iterate through big chunks
    } // first chunk

    /* this->firmware_update_running = false; */
    /* logger.printfln("   scheduling GD chip app mode in 3s"); */
    /* // after last chunk, get out of flash mode */
    /* task_scheduler.scheduleOnce([this](){ */
    /*     logger.printfln("   getting the GD chip back into app mode (scheduled 3s before)"); */
    /*     sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++); */
    /* }, 3000); */

    return true;
}
#endif


void AC011K::filltime(byte *year, byte *month, byte *day, byte *hour, byte *minute, byte *second)
{
    struct timeval tv_now;
    struct tm timeinfo;

    if (clock_synced(&tv_now)) {
        localtime_r(&tv_now.tv_sec, &timeinfo);

        *year   = (byte)(timeinfo.tm_year - 100);
        *month  = (byte)(timeinfo.tm_mon + 1);
        *day    = (byte)(timeinfo.tm_mday);
        *hour   = (byte)(timeinfo.tm_hour);
        *minute = (byte)(timeinfo.tm_min);
        *second = (byte)(timeinfo.tm_sec);

        //logger.printfln("time fill success %d/%d/%d %d:%d:%d", *year, *month, *day, *hour, *minute, *second);
    } else {

        *year   = 22;
        *month  = 1;
        *day    = 2;
        *hour   = 3;
        *minute = 4;
        *second = 5;
    /*     auto now = millis(); */
    /*     auto secs = now / 1000; */
        logger.printfln("time fill FAKE %d/%d/%d %d:%d:%d", *year, *month, *day, *hour, *minute, *second);
    }
}

void AC011K::my_setup_evse()
{

    Serial2.setRxBufferSize(1024);
    Serial2.begin(115200, SERIAL_8N1, 26, 27); // PrivComm to EVSE GD32 Chip
    //Serial2.setTimeout(90);
    logger.printfln("Set up PrivComm: 115200, SERIAL_8N1, RX 26, TX 27");

    /* // TODO start: look out for this on a unconfigured box ( no wifi ) - if it still works, delete the code */
    /* setTime(23,59,00,31,12,2018); */
    /* switch (timeStatus()){ */
    /*     case timeNotSet: */
    /*         logger.printfln("the time has never been set, the clock started on Jan 1, 1970"); */
    /*         break; */
    /*     case timeNeedsSync: */
    /*         logger.printfln("the time had been set but a sync attempt did not succeed"); */
    /*         break; */
    /*     case timeSet: */
    /*         logger.printfln("the time is set and is synced"); */
    /*         break; */
    /* } */
    /* logger.printfln("the time is %d", now()); */
    /* logger.printfln("the now() call was not blocking"); */
    /* // TODO end: look out for this on a unconfigured box ( no wifi ) - if it still works, delete the code */


// V3.2.589 init sequence
 /* W (1970-01-01 00:00:00) [PRIV_COMM, 1875]: Tx(cmd_AC len:15) :  FA 03 00 00  AC  01   05 00    11 0B 01 00 00 CA D3 */
 /* W (1970-01-01 00:00:00) [PRIV_COMM, 1875]: Tx(cmd_AC len:15) :  FA 03 00 00  AC  02   05 00    11 09 01 00 01 4A BE */
 /* W (1970-01-01 00:00:00) [PRIV_COMM, 1875]: Tx(cmd_AC len:15) :  FA 03 00 00  AC  03   05 00    11 0A 01 00 00 4A F6 */
 /* W (1970-01-01 00:00:00) [PRIV_COMM, 1875]: Tx(cmd_AC len:15) :  FA 03 00 00  AC  04   05 00    11 0C 01 00 00 0B 98 */
 /* W (1970-01-01 00:00:00) [PRIV_COMM, 1875]: Tx(cmd_AA len:18) :  FA 03 00 00  AA  05   08 00    18 3E 04 00 00 00 00 00 54 F0 */
 /* W (1970-01-01 00:00:00) [PRIV_COMM, 1875]: Tx(cmd_AC len:18) :  FA 03 00 00  AC  06   08 00    11 0D 04 00 B8 0B 00 00 C5 B7 */
 /* W (1970-01-01 00:00:01) [PRIV_COMM, 1875]: Tx(cmd_AA len:18) :  FA 03 00 00  AA  07   08 00    18 3F 04 00 1E 00 00 00 49 A0 */
 /* W (1970-01-01 00:00:01) [PRIV_COMM, 1875]: Tx(cmd_AA len:28) :  FA 03 00 00  AA  08   12 00    18 25 0E 00 05 00 00 00 05 00 00 00 00 03 00 00 00 02 EC 31 */
 /* W (1970-01-01 00:00:01) [PRIV_COMM, 1875]: Tx(cmd_AA len:15) :  FA 03 00 00  AA  09   05 00    18 12 01 00 03 7B 89 */
 /* W (1970-01-01 00:00:03) [PRIV_COMM, 1875]: Tx(cmd_AA len:14) :  FA 03 00 00  AA  0A   04 00    18 2A 00 00 52 B6 */

    sendCommand(Init1,  sizeof(Init1), sendSequenceNumber++);
    sendCommand(Init2,  sizeof(Init2), sendSequenceNumber++);
    sendCommand(Init3,  sizeof(Init3), sendSequenceNumber++);
    sendCommand(Init4,  sizeof(Init4), sendSequenceNumber++);
    sendCommand(Init5,  sizeof(Init5), sendSequenceNumber++);
    sendCommand(Init6,  sizeof(Init6), sendSequenceNumber++);
    sendCommand(Init7,  sizeof(Init7), sendSequenceNumber++);
    sendCommand(Init8,  sizeof(Init8), sendSequenceNumber++);
    sendCommand(Init9,  sizeof(Init9), sendSequenceNumber++);
    sendCommand(Init10, sizeof(Init10), sendSequenceNumber++);  // last two bytes correct?

//W (1970-01-01 00:08:53) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 08 05 00 18 12 01 00 03 BA 45
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 08 05 00 14 12 01 00 00 12 42
//I (2021-04-11 18:36:27) [PRIV_COMM, 51]: ctrl_cmd set ack done, type:0

    // ctrl_cmd set ack done, type:0 // this triggers 0x02 SN, Hardware, Version
    sendCommand(Init12, sizeof(Init12), sendSequenceNumber++);


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

    sendTimeLong(sendSequenceNumber++);


//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 40 05 00 18 09 01 00 00 F9 36
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 40 05 00 14 09 01 00 00 11 30
//I (2021-04-11 18:36:27) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower:  3.150.080

//W (2021-04-11 18:36:30) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 42 05 00 18 09 01 00 00 78 EF
//W (2021-04-11 18:36:31) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 42 05 00 14 09 01 00 00 90 E9
//I (2021-04-11 18:36:31) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower: 15.306.752

    // ctrl_cmd set start power mode done
    sendCommand(Init15, sizeof(Init15), sendSequenceNumber++);


//W (1970-01-01 00:00:03) [PRIV_COMM, 1764]: Tx(cmd_AA len:14) :  FA 03 00 00 AA 02 04 00 18 2A 00 00 DB 76
//W (1970-01-01 00:00:03) [PRIV_COMM, 1919]: Rx(cmd_0A len:14) :  FA 03 00 00 0A 02 04 00 14 2A 00 00 D2 5E
//E (1970-01-01 00:00:03) [PRIV_COMM, 78]: cmdAACtrlcantestsetAck test cancom...111

    // cmdAACtrlcantestsetAck test cancom...111
    sendCommand(Init11, sizeof(Init11), sendSequenceNumber++);

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

        /* task_scheduler.scheduleOnce("reboot_GD", [this](){ */
        /*     if(!initialized) { */
        /*         logger.printfln("   try to reset GD chip, cycle boot mode, app mode"); */
        /*         RemoteUpdate[7] = 5; // Reset into boot mode */
        /*         sendCommand(RemoteUpdate, sizeof(RemoteUpdate), sendSequenceNumber++); */
        /*         sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++); */
        /*     } */
        /* }, 5000); */

    char buffer[13];
    sprintf(buffer, "%06d", transactionNumber);
    for (int i=0; i<6; i++) {  // patch transaction number into command templates
        StartChargingA7[i+1] = byte(buffer[i]);
        StopChargingA7[i+1] = byte(buffer[i]);
        StopChargingA6[i+33] = byte(buffer[i]);
    }
    
    // TODO fix this, the api.call way is probably the right way, but it does not work like below.
    // therefore the call to the module directly (meter.update...)
    String error = api.callCommand("meter/state_update", Config::ConfUpdateObject{{
        {"state", 2}, // meter connected
        {"type", 99},
    }});
    if (error == "") {
        logger.printfln("internal meter enabeled");
    } else {
        logger.printfln("could not enable the internal meter: %s", error.c_str());
    }

    meter.updateMeterState(2, 99);

    logger.printfln("Initial transaction number %05d", transactionNumber);
}
            

void AC011K::myloop()
{
    static uint32_t last_check = 0;
    static uint32_t nextMillis = 2000;
    static bool ntp_clock_synced = false;
    static struct timeval tv_now;
    static uint8_t evseStatus = 0;
    static uint8_t cmd;
    static uint8_t seq;
    static uint16_t len;
    uint16_t crc;
    static bool cmd_to_process = false;
    static byte PrivCommRxState = PRIVCOMM_MAGIC;
    static int PrivCommRxBufferPointer = 0;
    byte rxByte;

    if (!ntp_clock_synced && clock_synced(&tv_now)) {
        sendTimeLong(sendSequenceNumber++);
        ntp_clock_synced = true;
    }

    if(evse_found && !initialized && deadline_elapsed(last_check + 10000)) {
        last_check = millis();
        setup_evse();
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
                            if(log_heartbeat || cmd!=2) { // be silent for the heartbeat //TODO show it at first and after an hour?
                                get_hex_privcomm_line(PrivCommRxBuffer); // PrivCommHexBuffer now holds the hex representation of the buffer
                            }
                            cmd_to_process = true;
                        }
                    }
                    break;
                case PRIVCOMM_CRC:
                    if(PrivCommRxBufferPointer == len + 10) {
            //Serial.println();
                        PrivCommRxState = PRIVCOMM_MAGIC;
                        if(log_heartbeat || cmd!=2) { // be silent for the heartbeat //TODO show it at first and after an hour?
                            get_hex_privcomm_line(PrivCommRxBuffer); // PrivCommHexBuffer now holds the hex representation of the buffer
                        }
                        crc = (uint16_t)(PrivCommRxBuffer[len + 9] << 8 | PrivCommRxBuffer[len + 8]);
                        uint16_t checksum = crc16_modbus(PrivCommRxBuffer, len+8);
                        if(crc == checksum) {
                            if(!evse_found) {
                                logger.printfln("EN+ GD EVSE found. Enabling EVSE support.");
                                evse_found = true;
                                evse_hardware_configuration.get("evse_found")->updateBool(evse_found);
                            }
                        } else {
                            logger.printfln("CRC ERROR Rx cmd_%.2X seq:%.2X len:%d crc:%.4X checksum:%.4X", cmd, seq, len, crc, checksum);
                            break;
                        }
                        cmd_to_process = true;
                    }
                    break;
            }  //switch read packet
        } while((Serial2.available() > 0) && !cmd_to_process && PrivCommRxBufferPointer<sizeof(PrivCommRxBuffer)/sizeof(PrivCommRxBuffer[0])); // one command at a time
    }

    char str[20];
    uint8_t allowed_charging_current = uint8_t(evse_state.get("allowed_charging_current")->asUint()/1000);
    //time_t t=now();     // get current time

    if(cmd_to_process) {
        switch( cmd ) {
            case 0x02: // Info: Serial number, Version
//W (1970-01-01 00:08:52) [PRIV_COMM, 1919]: Rx(cmd_02 len:135) :  FA 03 00 00 02 26 7D 00 53 4E 31 30 30 35 32 31 30 31 31 39 33 35 37 30 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 24 D1 00 41 43 30 31 31 4B 2D 41 55 2D 32 35 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 31 2E 31 2E 32 37 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 09 00 00 00 00 00 5A 00 1E 00 00 00 00 00 00 00 00 00 D9 25
//W (1970-01-01 00:08:52) [PRIV_COMM, 1764]: Tx(cmd_A2 len:11) :  FA 03 00 00 A2 26 01 00 00 99 E0
                if(log_heartbeat || cmd!=2) { // be silent for the heartbeat //TODO show it at first and after an hour?
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Serial number and version.", cmd, seq, len, crc);
                }
                sprintf(str, "%s", PrivCommRxBuffer+8);
                evse_hardware_configuration.get("SerialNumber")->updateString(str);
                sprintf(str, "%s",PrivCommRxBuffer+43);
                evse_hardware_configuration.get("Hardware")->updateString(str);
                sprintf(str, "%s",PrivCommRxBuffer+91);
                evse_hardware_configuration.get("FirmwareVersion")->updateString(str);
                if(!evse_hardware_configuration.get("initialized")->asBool()) {
                    initialized =
                        ((evse_hardware_configuration.get("Hardware")->asString().compareTo("AC011K-AU-25") == 0 || evse_hardware_configuration.get("Hardware")->asString().compareTo("AC011K-AE-25") == 0)
                         && (evse_hardware_configuration.get("FirmwareVersion")->asString().startsWith("1.1.", 0)  // known working: 1.1.27, 1.1.212, 1.1.258, 1.1.525, 1.1.538, 1.1.805, 1.1.812, 1.1.888
                             && evse_hardware_configuration.get("FirmwareVersion")->asString().substring(4).toInt() <= 888  // higest known working version (we assume earlier versions work as well)
                            )
                            || (evse_hardware_configuration.get("FirmwareVersion")->asString().startsWith("1.0.", 0)  // known working: 1.0.1435
                                && evse_hardware_configuration.get("FirmwareVersion")->asString().substring(4).toInt() <= 1435  // higest known working version (we assume earlier versions work as well)
                            )
                        );
                    evse_hardware_configuration.get("initialized")->updateBool(initialized);
                    evse_hardware_configuration.get("GDFirmwareVersion")->updateUint(evse_hardware_configuration.get("FirmwareVersion")->asString().substring(4).toInt());
                    logger.printfln("EVSE serial: %s hw: %s fw: %s", 
                        evse_hardware_configuration.get("SerialNumber")->asString().c_str(),
                        evse_hardware_configuration.get("Hardware")->asString().c_str(),
                        evse_hardware_configuration.get("FirmwareVersion")->asString().c_str());
                    if(initialized) {
                        logger.printfln("EN+ GD EVSE initialized.");
                    } else {
                        logger.printfln("EN+ GD EVSE Firmware Version or Hardware is not supported.");
                    }
                }
                PrivCommAck(cmd, PrivCommTxBuffer); // privCommCmdA2InfoSynAck
                break;

            case 0x03:
//W (1970-01-01 00:08:52) [PRIV_COMM, 1919]: Rx(cmd_03 len:24) :  FA 03 00 00 03 27 0E 00 00 09 09 0D 00 00 02 00 00 00 00 00 04 00 80 BC
//W (1970-01-01 00:08:52) [PRIV_COMM, 1764]: Tx(cmd_A3 len:17) :  FA 03 00 00 A3 27 07 00 00 E2 01 01 00 08 34 CF 2F
// 00 00 02 02 00 00 00 00 00 00 00 00 00
/* 853540                000: FA 03 00 00 03 0C 0E 00 50 02 02 05 00 00 00 00 00 00 00 00 */
/* 853540      Rx cmd_03 seq:0C len:14 crc:3DE8 */
/* (2021-10-04 10:04:39) [PRIV_COMM, 2033]: Rx(cmd_03 len:24) :  FA 03 00 00 03 03 0E 00 00 02 02 00 00 00 00 00 00 00 00 00 00 00 9C 52 */
/* (2021-10-04 10:04:40) [PRIV_COMM, 1875]: Tx(cmd_A3 len:17) :  FA 03 00 00 A3 03 07 00 10 15 0A 04 0A 04 27 4A D4 */
                evseStatus = PrivCommRxBuffer[9];
                if( PrivCommRxBuffer[8]==0x50 ) { // TODO this 0x50 is a wild guess, I've seen it work, and I'm sure there is more than just the evseStatus byte needed for a well founded decission
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Status %d: %s", cmd, seq, len, crc, evseStatus, evse_status_text[evseStatus]);
                    update_evseStatus(evseStatus);
                } else {
                    //TODO figure out what substatus (PrivCommRxBuffer[8]) is or should be
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Status %d: %s but substatus %.2X not 0x50.", cmd, seq, len, crc, evseStatus, evse_status_text[evseStatus], PrivCommRxBuffer[8]);
                    if(evse_hardware_configuration.get("initialized")->asBool()) { // only update the EVSE status if we support the hardware
                        update_evseStatus(evseStatus);
                    }
                }
                // I think there is the time in the 03 cmd - log it!
                logger.printfln("       time?: %d/%d/%d %d:%d.%d", PrivCommRxBuffer[14],PrivCommRxBuffer[15],PrivCommRxBuffer[16],PrivCommRxBuffer[17],PrivCommRxBuffer[18],PrivCommRxBuffer[19]);
                sendTime(0xA3, 0x10, 8, seq);  // send ack
                break;

            case 0x04: // time request / ESP32-GD32 communication heartbeat
//W (1970-01-01 00:08:52) [PRIV_COMM, 1919]: Rx(cmd_04 len:16) :  FA 03 00 00 04 28 06 00 09 00 00 00 00 00 C0 5E
//W (1970-01-01 00:08:52) [PRIV_COMM, 1764]: Tx(cmd_A4 len:17) :  FA 03 00 00 A4 28 07 00 00 E2 01 01 00 08 34 E5 6B
                evseStatus = PrivCommRxBuffer[8];
                update_evseStatus(evseStatus);
                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Heartbeat request, Status %d: %s, value:%d", cmd, seq, len, crc, evseStatus, evse_status_text[evseStatus], PrivCommRxBuffer[12]);
// experimental:
                send_http(String(",\"type\":\"en+04\",\"data\":{")
                    +"\"status\":"+String(PrivCommRxBuffer[8])  // status
                    +",\"value\":"+String(PrivCommRxBuffer[12])
                    +"}}"
                );
// end experimental
                sendTime(0xA4, 0x01, 8, seq);  // send ack
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
		// der GD sendet nur ein cmd_05 wenn er in einem bestimmten mode ist?
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
                //Tx(cmd_A5 len:47) :  FA 03 00 00 A5 1D 25 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 D0 00 00 00 00 A4 86
		// privCommCmdA5CardAuthAck PrivCommTxBuffer+40 = 0x40; // allow charging
		// privCommCmdA5CardAuthAck PrivCommTxBuffer+40 = 0xD0; // decline charging

                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - RFID card detected. ID: %s", cmd, seq, len, crc, PrivCommRxBuffer + PayloadStart); //str);
                // Start/stop test with any RFID card:
                if (evseStatus == 2 || evseStatus == 6) sendCommand(StartChargingA6, sizeof(StartChargingA6), sendSequenceNumber++); 
                if (evseStatus == 3 || evseStatus == 4) sendCommand(StopChargingA6, sizeof(StopChargingA6), sendSequenceNumber++);
		break;

            case 0x06:
		// ack for cmd_A6 srvOcppRemoteStartTransactionReq (triggers cmd_07)
                //logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - cp call srv_ocpp ack srv remote ctrl ack", cmd, seq, len, crc);
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
                if (PrivCommRxBuffer[72] == 0x40) logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Stop charging request ack", cmd, seq, len, crc);
                else  logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Start charging request ack", cmd, seq, len, crc);
                break;

            case 0x07:
                //if (PrivCommRxBuffer[72] == 0x10) cmdText = "- Stop charging approval"; else cmdText = "- Start charging approval";
                if (PrivCommRxBuffer[72] == 0) {
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Start charging approval", cmd, seq, len, crc);
                    sendCommand(StartChargingA7, sizeof(StartChargingA7), seq);
// experimental:
                    send_http(String(",\"type\":\"en+07\",\"data\":{")
                      +"\"transaction\":"+String(transactionNumber)
                      +",\"meterStart\":"+String(PrivCommRxBuffer[79]+256*PrivCommRxBuffer[80])  // status ?
                      //+",\"startMode\":"+String(PrivCommRxBuffer[72])
                      +"}}"
                    );
// end experimental
                } else {
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Stop charging approval", cmd, seq, len, crc);
                    sendCommand(StopChargingA7, sizeof(StopChargingA7), seq);
                }
                break;

            case 0x08:
// W (2021-08-07 07:55:19) [PRIV_COMM, 1764]: Tx(cmd_A8 len:21) :  FA 03 00 00 A8 25 0B 00 40 15 08 07 07 37 13 00 00|[2021-08-07 07:55:18] Rx(cmd_A8 len:21) : FA 03 00 00 A8 25 0B 00 40 15 08 07 07 37 13 00 00 00 00 1B BE 00 00 1B BE                                                                                                      |[2021-08-07 07:55:18] cmd_A8 [privCommCmdA8RTDataAck]!
//D (2021-08-07 07:55:19) [OCPP_SRV, 3550]: ocpp_sevice_ntc_evt: 9, chan:0,sts:8                                    |[2021-08-07 07:55:18] charger A8 settime:21-8-7 7:55:19
//D (2021-08-07 07:55:19) [OCPP_SRV, 3031]: startMode(0:app 1:card 2:vin):1, stopreson:Remote timestamp:2021-08-07T0|[2021-08-07 07:55:19] [comm] cmd03 cpNowSts=0, gunNowSts=1,gunPreSts=0,chargerreson=6
//7:55:17Z idTag:50a674e1                                                                                           |[2021-08-07 07:55:19] Tx(cmd_03 len:24) : FA 03 00 00 03 18 0E 00 80 01 01 06 00 00 00 00 [2021-08-07 07:55:19] [
                if (PrivCommRxBuffer[77] < 10) {  // statistics
                    // TODO is it true that PrivCommRxBuffer[77] is the evseStatus?
                    evseStatus = PrivCommRxBuffer[77];
                    update_evseStatus(evseStatus);
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Status %d: %s", cmd, seq, len, crc, evseStatus, evse_status_text[evseStatus]);
                    logger.printfln("\t%dWh\t%d\t%dWh\t%d\t%d\t%d\t%dW\t%d\t%.1fV\t%.1fV\t%.1fV\t%.1fA\t%.1fA\t%.1fA\t%d minutes",
                              PrivCommRxBuffer[84]+256*PrivCommRxBuffer[85],  // charged energy Wh
                              PrivCommRxBuffer[86]+256*PrivCommRxBuffer[87],
                              PrivCommRxBuffer[88]+256*PrivCommRxBuffer[89],  // charged energy Wh
                              PrivCommRxBuffer[90]+256*PrivCommRxBuffer[91],
                              PrivCommRxBuffer[92]+256*PrivCommRxBuffer[93],
                              PrivCommRxBuffer[94]+256*PrivCommRxBuffer[95],
                              PrivCommRxBuffer[96]+256*PrivCommRxBuffer[97],  // charging power
                              PrivCommRxBuffer[98]+256*PrivCommRxBuffer[99],
                              float(PrivCommRxBuffer[100]+256*PrivCommRxBuffer[101])/10,  // L1 plug voltage * 10
                              float(PrivCommRxBuffer[102]+256*PrivCommRxBuffer[103])/10,  // L2 plug voltage * 10
                              float(PrivCommRxBuffer[104]+256*PrivCommRxBuffer[105])/10,  // L3 plug voltage * 10
                              float(PrivCommRxBuffer[106]+256*PrivCommRxBuffer[107])/10,  // L1 charging current * 10
                              float(PrivCommRxBuffer[108]+256*PrivCommRxBuffer[109])/10,  // L2 charging current * 10
                              float(PrivCommRxBuffer[110]+256*PrivCommRxBuffer[111])/10,  // L3 charging current * 10
                              PrivCommRxBuffer[113]+256*PrivCommRxBuffer[114]
                              );
                    // push values to the meter
                    // voltages
                    meter.updateMeterAllValues(METER_ALL_VALUES_LINE_TO_NEUTRAL_VOLTS_L1, float(PrivCommRxBuffer[100]+256*PrivCommRxBuffer[101])/10);
                    meter.updateMeterAllValues(METER_ALL_VALUES_LINE_TO_NEUTRAL_VOLTS_L2, float(PrivCommRxBuffer[102]+256*PrivCommRxBuffer[103])/10);
                    meter.updateMeterAllValues(METER_ALL_VALUES_LINE_TO_NEUTRAL_VOLTS_L3, float(PrivCommRxBuffer[104]+256*PrivCommRxBuffer[105])/10);
                    // current
                    meter.updateMeterAllValues(METER_ALL_VALUES_CURRENT_L1_A, float(PrivCommRxBuffer[106]+256*PrivCommRxBuffer[107])/10);
                    meter.updateMeterAllValues(METER_ALL_VALUES_CURRENT_L2_A, float(PrivCommRxBuffer[108]+256*PrivCommRxBuffer[109])/10);
                    meter.updateMeterAllValues(METER_ALL_VALUES_CURRENT_L3_A, float(PrivCommRxBuffer[110]+256*PrivCommRxBuffer[111])/10);

                    // meter power
                    meter.updateMeterValues(
                              PrivCommRxBuffer[96]+256*PrivCommRxBuffer[97],  // charging power W  (power)
                              PrivCommRxBuffer[84]+256*PrivCommRxBuffer[85],  // charged energy Wh (energy_rel)
                              PrivCommRxBuffer[88]+256*PrivCommRxBuffer[89]   // charged energy Wh (energy_abs)
                              );
                    /*
                    meter.updateMeterAllValues(i, all_values_update.get(i)->asFloat());
                    */

                    /* fill phases status values */
                    /* consider a voltage > 10V as real, below that it is probably a faulty reading */
                    if ((PrivCommRxBuffer[100]+256*PrivCommRxBuffer[101]) > 100) { phases_connected[0] = true; } // L1 plug voltage
                    if ((PrivCommRxBuffer[102]+256*PrivCommRxBuffer[103]) > 100) { phases_connected[1] = true; } // L2 plug voltage
                    if ((PrivCommRxBuffer[104]+256*PrivCommRxBuffer[105]) > 100) { phases_connected[2] = true; } // L3 plug voltage
                    meter.updateMeterPhases(phases_connected, phases_active);
// experimental:
                    send_http(String(",\"type\":\"en+08\",\"data\":{")
                        +"\"status\":"+String(PrivCommRxBuffer[77])  // status
                        +",\"transaction\":"+String(transactionNumber)
                        +",\"energy1\":"+String(PrivCommRxBuffer[84]+256*PrivCommRxBuffer[85])  // charged energy Wh
                        +",\"aaa\":"+String(PrivCommRxBuffer[86]+256*PrivCommRxBuffer[87])
                        +",\"energy2\":"+String(PrivCommRxBuffer[88]+256*PrivCommRxBuffer[89])  // charged energy Wh
                        +",\"bbb\":"+String(PrivCommRxBuffer[90]+256*PrivCommRxBuffer[91])
                        +",\"ccc\":"+String(PrivCommRxBuffer[92]+256*PrivCommRxBuffer[93])
                        +",\"ddd\":"+String(PrivCommRxBuffer[94]+256*PrivCommRxBuffer[95])
                        +",\"power\":"+String(PrivCommRxBuffer[96]+256*PrivCommRxBuffer[97])  // charging power
                        +",\"eee\":"+String(PrivCommRxBuffer[98]+256*PrivCommRxBuffer[99])
                        +",\"u1\":"+String(float(PrivCommRxBuffer[100]+256*PrivCommRxBuffer[101])/10,1)  // L1 plug voltage * 10
                        +",\"u2\":"+String(float(PrivCommRxBuffer[102]+256*PrivCommRxBuffer[103])/10,1)  // L2 plug voltage * 10
                        +",\"u3\":"+String(float(PrivCommRxBuffer[104]+256*PrivCommRxBuffer[105])/10,1)  // L3 plug voltage * 10
                        +",\"i1\":"+String(float(PrivCommRxBuffer[106]+256*PrivCommRxBuffer[107])/10,1)  // L1 charging current * 10
                        +",\"i2\":"+String(float(PrivCommRxBuffer[108]+256*PrivCommRxBuffer[109])/10,1)  // L2 charging current * 10
                        +",\"i3\":"+String(float(PrivCommRxBuffer[110]+256*PrivCommRxBuffer[111])/10,1)  // L3 charging current * 10
                        +",\"minutes\":"+String(PrivCommRxBuffer[113]+256*PrivCommRxBuffer[114])
                        +"}}"
                    );
// end experimental
                } else {
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - type:%.2X", cmd, seq, len, crc, PrivCommRxBuffer[77]);
                    if (PrivCommRxBuffer[77] == 0x10) {  // RFID card
                        String rfid = "";
                        for (int i=0; i<8; i++) {rfid += PrivCommRxBuffer[40 + i];}  // Card number in bytes 40..47
                        logger.printfln("RFID Card %s", rfid);
                    }
                }
                sendTime(0xA8, 0x40, 12, seq); // send ack
                break;

            case 0x09:
                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Charging stop reason: %d - %s",
                    cmd, seq, len, crc,
                    PrivCommRxBuffer[77], PrivCommRxBuffer[77]<=3 ? stop_reason_text[PrivCommRxBuffer[77]] : stop_reason_text[0]);  // "stopreson": 1 = Remote, 3 = EVDisconnected
                logger.printfln("start:%s stop:%s meter:%dWh value1:%d value2:%d value3:%d",
                    timeStr(PrivCommRxBuffer+80),
                    timeStr(PrivCommRxBuffer+86),
                    PrivCommRxBuffer[96]+256*PrivCommRxBuffer[97],
                    PrivCommRxBuffer[78]+256*PrivCommRxBuffer[79],
                    PrivCommRxBuffer[92]+256*PrivCommRxBuffer[93],
                    PrivCommRxBuffer[94]+256*PrivCommRxBuffer[95]);
// experimental:
                send_http(String(",\"type\":\"en+09\",\"data\":{")
                    +"\"transaction\":"+String(transactionNumber)
                    +",\"meterStop\":"+String(PrivCommRxBuffer[96]+256*PrivCommRxBuffer[97])  // status
                    +",\"stopReason\":"+String(PrivCommRxBuffer[77])
                    +"}}"
                );
                //PrivCommAck(cmd, PrivCommTxBuffer); // privCommCmdA9RecordAck
                sendCommand(TransactionAck, sizeof(TransactionAck), seq);
// end experimental
                break;

            case 0x0A:
                switch( PrivCommRxBuffer[9] ) { // 8: always 14, 9: answertype?
                    case 0x02: // answer to set time
//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:20) :  FA 03 00 00 AA 09 0A 00 18 02 06 00 15 04 0B 12 24 1B 5C 78
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:20) :  FA 03 00 00 0A 09 0A 00 14 02 06 00 15 04 0B 12 24 1B 3C E7
//I (2021-04-11 18:36:27) [PRIV_COMM, 94]: ctrl_cmd set time done -> time: 2021-04-11 18:36:27
    // ctrl_cmd set start power mode done
                        logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Set Time done", cmd, seq, len, crc);
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
                        logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Heartbeat Timeout:%ds", cmd, seq, len, crc, PrivCommRxBuffer[12]);
                        break;
                    case 0x09: // answer to ctrl_cmd set start power mode
//W (2021-04-11 18:36:27) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 40 05 00 18 09 01 00 00 F9 36
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 40 05 00 14 09 01 00 00 11 30
//I (2021-04-11 18:36:27) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower:  3.150.080

//W (2021-04-11 18:36:30) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 42 05 00 18 09 01 00 00 78 EF
//W (2021-04-11 18:36:31) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 42 05 00 14 09 01 00 00 90 E9
//I (2021-04-11 18:36:31) [PRIV_COMM, 279]: ctrl_cmd set start power mode done -> minpower: 15.306.752
                        logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - ctrl_cmd set start power mode done", cmd, seq, len, crc);
                        break;
                    case 0x12: // ctrl_cmd set ack done, type:0
//W (1970-01-01 00:08:53) [PRIV_COMM, 1764]: Tx(cmd_AA len:15) :  FA 03 00 00 AA 08 05 00 18 12 01 00 03 BA 45
//W (2021-04-11 18:36:27) [PRIV_COMM, 1919]: Rx(cmd_0A len:15) :  FA 03 00 00 0A 08 05 00 14 12 01 00 00 12 42
//I (2021-04-11 18:36:27) [PRIV_COMM, 51]: ctrl_cmd set ack done, type:0
                        logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - ctrl_cmd set ack done, type:0", cmd, seq, len, crc);
                        break;
                    case 0x2A: // answer to cmdAACtrlcantestsetAck test cancom...111
//W (1970-01-01 00:00:03) [PRIV_COMM, 1764]: Tx(cmd_AA len:14) :  FA 03 00 00 AA 02 04 00 18 2A 00 00 DB 76
//W (1970-01-01 00:00:03) [PRIV_COMM, 1919]: Rx(cmd_0A len:14) :  FA 03 00 00 0A 02 04 00 14 2A 00 00 D2 5E
//E (1970-01-01 00:00:03) [PRIV_COMM, 78]: cmdAACtrlcantestsetAck test cancom...111
    // cmdAACtrlcantestsetAck test cancom...111
                        logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - cmdAACtrlcantestsetAck test cancom...111 done", cmd, seq, len, crc);
                        break;
                    default:
                        logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X -  I don't know what %.2X means.", cmd, seq, len, crc, PrivCommRxBuffer[9]);
                        break;
                }  //switch cmdAA answer processing
                break;

            case 0x0B:
                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - update reply: %.2X%.2X%.2X%.2X  %d (%s) - %s, fw update running: %s", cmd, seq, len, crc, FlashVerify[3], FlashVerify[4], FlashVerify[5], FlashVerify[6], PrivCommRxBuffer[10], cmd_0B_text[PrivCommRxBuffer[10]], PrivCommRxBuffer[12]==0 ?"success":"failure", this->firmware_update_running==true ?"true":"false");
                if( PrivCommRxBuffer[12]==0 ) { // success
                    switch( PrivCommRxBuffer[10] ) {
                        case 5: // reset into boot mode
                            if(this->firmware_update_running) {
                                logger.printfln("   reset into boot mode complete, handshake next");
                                //sendCommand(Handshake, sizeof(Handshake), sendSequenceNumber++);
                                RemoteUpdate[7] = 1; // handshake
                                sendCommand(RemoteUpdate, sizeof(RemoteUpdate), sendSequenceNumber++);
                            } else {
                                logger.printfln("   reset into app mode complete");
                            }
                            break;
                        case 1: // handshake
                            if(FlashVerify[7] == 3) { // flash write
                                logger.printfln("   handshake complete, flash erase next");
                                RemoteUpdate[7] = 2; // flash erase
                                sendCommand(RemoteUpdate, sizeof(RemoteUpdate), sendSequenceNumber++);
                            } else if(FlashVerify[7] == 4) { // flash verify
                                logger.printfln("   handshake complete, flash verify next");
                                ready_for_next_chunk = true;
                            }
                            break;
                        case 2: // flash erase
                            logger.printfln("   flash erase complete, flash write next");
                            ready_for_next_chunk = true;
                            break;
                        case 3: // flash write
                        case 4: // verify
                            //if (*((unsigned int) FlashVerify[3]) == 0x030800FE) {
                            if (FlashVerify[3] == 0x03 && FlashVerify[4] == 0x08 && FlashVerify[5] == 0x00 && FlashVerify[6] == 0xFE) {
                                logger.printfln("   finished flashing, getting the GD chip back into app mode");
                                sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++);
                            }
                            ready_for_next_chunk = true;
                            break;
                        default:
                            logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - privCommCmdABUpdateReq: %.2X%.2X%.2X%.2X  %d (%s) - %s", cmd, seq, len, crc, FlashVerify[3], FlashVerify[4], FlashVerify[5], FlashVerify[6], PrivCommRxBuffer[10], cmd_0B_text[PrivCommRxBuffer[10]], PrivCommRxBuffer[12]==0 ?"success":"failure");
                            logger.printfln("   getting the GD chip back into app mode");
                            sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++);
                            break;
                    }  //switch privCommCmdABUpdateReq success
                } else {
                    logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - privCommCmdABUpdateReq: %.2X%.2X%.2X%.2X  %d (%s) - %s", cmd, seq, len, crc, FlashVerify[3], FlashVerify[4], FlashVerify[5], FlashVerify[6], PrivCommRxBuffer[10], cmd_0B_text[PrivCommRxBuffer[10]], PrivCommRxBuffer[12]==0 ?"success":"failure");
                    logger.printfln("   getting the GD chip back into app mode");
                    update_aborted = true;
                    sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++);
                }
                break;

            case 0x0D:
                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Limit ack", cmd, seq, len, crc);
                break;

            case 0x0E:
// [2021-08-07 07:55:05] Tx(cmd_0E len:76) : FA 03 00 00 0E 11 42 00 00 00 00 00 00 00 00 00 00 0A 01 77 02 37 35 32 30 33 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 66 08 97 08 14 00 7A 01 01 00 00 00 00 00 00 00 00 00 00 DE 91
// [2021-08-07 07:55:05] cmd0E_DutyData pwmMax:266
                evse_low_level_state.get("cp_pwm_duty_cycle")->updateUint(PrivCommRxBuffer[17]+256*PrivCommRxBuffer[18]); //duty
                evse_low_level_state.get("adc_values")->get(6)->updateUint(PrivCommRxBuffer[19]+256*PrivCommRxBuffer[20]); //cpVolt
                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Duty data duty:%d cpVolt:%d",
                    cmd, seq, len, crc,
                    PrivCommRxBuffer[17]+256*PrivCommRxBuffer[18],
                    PrivCommRxBuffer[19]+256*PrivCommRxBuffer[20]);
                logger.printfln("power factors:%d/%d/%d/%d offset0:%d offset1:%d leakcurr:%d AMBTemp:%d lock:%d",
                    PrivCommRxBuffer[9]+256*PrivCommRxBuffer[10],PrivCommRxBuffer[11]+256*PrivCommRxBuffer[12],PrivCommRxBuffer[13]+256*PrivCommRxBuffer[14],PrivCommRxBuffer[15]+256*PrivCommRxBuffer[16],
                    PrivCommRxBuffer[55]+256*PrivCommRxBuffer[56],
                    PrivCommRxBuffer[57]+256*PrivCommRxBuffer[58],
                    PrivCommRxBuffer[59]+256*PrivCommRxBuffer[60],
                    PrivCommRxBuffer[61]+256*PrivCommRxBuffer[62],
                    PrivCommRxBuffer[63]);
                //PrivCommAck(cmd, PrivCommTxBuffer); // Ack?
// experimental:
                send_http(String(",\"type\":\"en+0E\",\"data\":{")
                    +"\"transaction\":"+String(transactionNumber)
                    +",\"duty\":"+String(PrivCommRxBuffer[17]+256*PrivCommRxBuffer[18])
                    +",\"cpVolt\":"+String(PrivCommRxBuffer[19]+256*PrivCommRxBuffer[20])
                    +",\"pf1\":"+String(PrivCommRxBuffer[9]+256*PrivCommRxBuffer[10])  // power factor 1
                    +",\"pf2\":"+String(PrivCommRxBuffer[11]+256*PrivCommRxBuffer[12])  // power factor 2
                    +",\"pf3\":"+String(PrivCommRxBuffer[13]+256*PrivCommRxBuffer[14])  // power factor 3
                    +",\"pft\":"+String(PrivCommRxBuffer[15]+256*PrivCommRxBuffer[16])  // power factor total
                    +",\"offset0\":"+String(PrivCommRxBuffer[55]+256*PrivCommRxBuffer[56])
                    +",\"offset1\":"+String(PrivCommRxBuffer[57]+256*PrivCommRxBuffer[58])
                    +",\"leakcurr\":"+String(PrivCommRxBuffer[59]+256*PrivCommRxBuffer[60])
                    +",\"AMBTemp\":"+String(PrivCommRxBuffer[61]+256*PrivCommRxBuffer[62])
                    +",\"lock\":"+String(PrivCommRxBuffer[63])
                    +"}}"
                );
// end experimental
                break;

            case 0x0F:
                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - Charging limit request", cmd, seq, len, crc);
                sendChargingLimit1(allowed_charging_current, seq);
                break;

            default:
                logger.printfln("Rx cmd_%.2X seq:%.2X len:%d crc:%.4X - I don't know what to do about it.", cmd, seq, len, crc);
                break;
        }  //switch process cmd
        cmd_to_process = false;
    }

    evse_state.get("time_since_state_change")->updateUint(millis() - evse_state.get("last_state_change")->asUint());
    
    //resend flash commands if needed
    if(this->firmware_update_running && flash_seq == PrivCommTxBuffer[5] && !ready_for_next_chunk && deadline_elapsed(last_flash + 2000)) {
        last_flash = millis();
        logger.printfln("resend the last chunk fseq: %d, seq: %d rfnc: %s", flash_seq, PrivCommTxBuffer[5], ready_for_next_chunk?"true":"false");
        sendCommand(FlashVerify, MAXLENGTH+11, sendSequenceNumber++); // next chunk (11 bytes header) 
    }


// experimental: UDP command receiver for testing
    int packetSize = UdpListener.parsePacket();
    if (packetSize) {
        logger.printfln("Received UDP packet of size %d from %s:%d.", packetSize, UdpListener.remoteIP().toString().c_str(), UdpListener.remotePort());
        // read the packet into packetBufffer
        UdpListener.read(receiveCommandBuffer, UDP_RX_PACKET_MAX_SIZE);
        receiveCommandBuffer[packetSize] = '\0';
        logger.printfln("Content: %s", receiveCommandBuffer);
        switch (receiveCommandBuffer[0]) {
            case 'L':
                uint8_t limit;
                if (receiveCommandBuffer[2] >= 'A') limit = receiveCommandBuffer[2]+10-'A';  // e.g. L2F for 15 Ampere;
                else limit = receiveCommandBuffer[2]-'0';  // e.g. L29 for 9A

                if (receiveCommandBuffer[1] == '1') sendChargingLimit1(limit, sendSequenceNumber++);  // working for all except for 1.1.212
                else if (receiveCommandBuffer[1] == '2') sendChargingLimit2(limit, sendSequenceNumber++);
                else if (receiveCommandBuffer[1] == '3') sendChargingLimit3(limit, sendSequenceNumber++);  // working for 1.1.212

                if (receiveCommandBuffer[1] == 'P') {
                    ChargingLimit3[74] = 1;
                    sendChargingLimit3(limit, sendSequenceNumber++);  // e.g. LPA for 1 phase, 10 Ampere. Sadly not working here :-( 
                    ChargingLimit3[74] = 3;
                }
                allowed_charging_current = limit;
                break;
            case 'C':
                if (receiveCommandBuffer[1] == '6') {
                    if (receiveCommandBuffer[2] == '0') sendCommand(StopChargingA6, sizeof(StopChargingA6), sendSequenceNumber++);  // stop charging handshake
                    else if (receiveCommandBuffer[2] == '1') sendCommand(StartChargingA6, sizeof(StartChargingA6), sendSequenceNumber++);  // start charging handshake
                } else if (receiveCommandBuffer[1] == '7') {
                    if (receiveCommandBuffer[2] == '0') sendCommand(StopChargingA7, sizeof(StopChargingA7), sendSequenceNumber++);  // stop charging directly
                    else if (receiveCommandBuffer[2] == '1') sendCommand(StartChargingA7, sizeof(StartChargingA7), sendSequenceNumber++);  // start charging directly
                }
                break;
            case 'O':
                // output as comment
                break;
            case 'R':
                RemoteUpdate[7] = 5; // Reset GD into boot mode
                //sendCommand(RemoteUpdate, sizeof(RemoteUpdate), sendSequenceNumber++);
                sendCommand(EnterAppMode, sizeof(EnterAppMode), sendSequenceNumber++);  // Restart GD
                break;
            case 'V':
                sendCommand(Init12, sizeof(Init12), sendSequenceNumber++);  // triggers 0x02 reply: SN, Hardware, Version
        }
    }
// end experimental
}

void AC011K::register_my_urls()
{
#ifdef GD_FLASH
    server.on("/update_gd", HTTP_GET, [this](WebServerRequest request){
        //request.send(200, "text/html", "<form><input id=\"firmware\"type=\"file\"> <button id=\"u_firmware\"type=\"button\"onclick='u(\"firmware\")'>Flash GD Firmware</button> <label id=\"p_firmware\"></label><button id=\"u_verify\"type=\"button\"onclick='u(\"verify\")'>Verify GD Firmware</button> <label id=\"p_verify\"></label></form><script>function u(e){var t,n,d,o=document.getElementById(\"firmware\").files;0==o.length?alert(\"No file selected!\"):(document.getElementById(\"firmware\").disabled=!0,document.getElementById(\"u_firmware\").disabled=!0,document.getElementById(\"u_verify\").disabled=!0,t=o[0],n=new XMLHttpRequest,d=document.getElementById(\"p_\"+e),n.onreadystatechange=function(){4==n.readyState&&(200==n.status?(document.open(),document.write(n.responseText),document.close()):(0==n.status?alert(\"Server closed the connection abruptly!\"):alert(n.status+\" Error!\\n\"+n.responseText),location.reload()))},n.upload.addEventListener(\"progress\",function(e){e.lengthComputable&&(d.innerHTML=e.loaded/e.total*100+\"% (\"+e.loaded+\" / \"+e.total+\")\")},!1),n.open(\"POST\",\"/flash_\"+e,!0),n.send(t))}</script>");
        return request.send(200, "text/html", "<form><input id=\"gd_firmware\"type=\"file\"> <button id=\"u_firmware\"type=\"button\"onclick='u(\"gd_firmware\")'>Upload GD Firmware</button> <label id=\"p_gd_firmware\"></label></form><form><input id=\"verify\"type=\"file\"> <button id=\"u_verify\"type=\"button\"onclick='u(\"verify\")'>Verify GD Firmware</button> <label id=\"p_verify\"></label></form><script>function u(e){var t,n,d,o=document.getElementById(e).files;0==o.length?alert(\"No file selected!\"):(document.getElementById(\"gd_firmware\").disabled=!0,document.getElementById(\"u_firmware\").disabled=!0,document.getElementById(\"verify\").disabled=!0,document.getElementById(\"u_verify\").disabled=!0,t=o[0],n=new XMLHttpRequest,d=document.getElementById(\"p_\"+e),n.onreadystatechange=function(){4==n.readyState&&(200==n.status?(document.open(),document.write(n.responseText),document.close()):(0==n.status?alert(\"Server closed the connection abruptly!\"):alert(n.status+\" Error!\\n\"+n.responseText),location.reload()))},n.upload.addEventListener(\"progress\",function(e){e.lengthComputable&&(d.innerHTML=e.loaded/e.total*100+\"% (\"+e.loaded+\" / \"+e.total+\")\")},!1),n.open(\"POST\",\"/flash_\"+e,!0),n.send(t))}</script>");
    });
    server.on("/flash_gd_firmware", HTTP_POST, [this](WebServerRequest request){
        if (update_aborted)
            return request.unsafe_ResponseAlreadySent(); // Already sent in upload callback.
        this->firmware_update_running = false;
        if (!firmware_update_allowed) {
            request.send(423, "text/plain", "vehicle connected");
            return request.unsafe_ResponseAlreadySent(); // Already sent in upload callback.
        }
        /* request.send(Update.hasError() ? 400: 200, "text/plain", Update.hasError() ? Update.errorString() : "Update OK"); */
        return request.send(200, "text/plain", "Update OK");
    },[this](WebServerRequest request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if (!firmware_update_allowed) {
            request.send(423, "text/plain", "vehicle connected");
            this->firmware_update_running = false;
            return false;
        }
        this->firmware_update_running = true;
        return handle_update_chunk1(3, request, index, data, len, final, request.contentLength());
    });

    server.on("/evse/reflash", HTTP_PUT, [this](WebServerRequest request){
        if (update_aborted)
            return request.unsafe_ResponseAlreadySent(); // Already sent in upload callback.
        this->firmware_update_running = false;
        if (!firmware_update_allowed) {
            request.send(423, "text/plain", "vehicle connected");
            return request.unsafe_ResponseAlreadySent(); // Already sent in upload callback.
        }
        /* request.send(Update.hasError() ? 400: 200, "text/plain", Update.hasError() ? Update.errorString() : "Update OK"); */
        return request.send(200, "text/plain", "Update OK");
    },[this](WebServerRequest request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if (!firmware_update_allowed) {
            request.send(423, "text/plain", "vehicle connected");
            this->firmware_update_running = false;
            return false;
        }
        this->firmware_update_running = true;
        logger.printfln("/evse/reflash %d (%d)", index, len);
        return handle_update_chunk2(3, request, index, data, len);
    });

    server.on("/flash_verify", HTTP_POST, [this](WebServerRequest request){
        if (update_aborted)
            return request.unsafe_ResponseAlreadySent(); // Already sent in upload callback.
        this->firmware_update_running = false;
        if (!firmware_update_allowed) {
            request.send(423, "text/plain", "vehicle connected");
            return request.unsafe_ResponseAlreadySent(); // Already sent in upload callback.
        }
        /* request.send(Update.hasError() ? 400: 200, "text/plain", Update.hasError() ? Update.errorString() : "Update OK"); */
        request.send(200, "text/plain", "Update OK");
    },[this](WebServerRequest request, String filename, size_t index, uint8_t *data, size_t len, bool final){
        if (!firmware_update_allowed) {
            request.send(423, "text/plain", "vehicle connected");
            this->firmware_update_running = false;
            return false;
        }
        this->firmware_update_running = true;
        return handle_update_chunk1(4, request, index, data, len, final, request.contentLength());
    });

    /* server.on("/flash_verify", HTTP_POST, [this](WebServerRequest request){ */
    /*     request.send(200, "text/plain", "Update OK"); */
    /* },[this](WebServerRequest request, String filename, size_t index, uint8_t *data, size_t len, bool final){ */
    /*     return handle_update_chunk(4, request, index, data, len, final, request.contentLength()); */
    /* }); */
#endif

}

/* void AC011K:tf_evse_v2_set_charging_slot_max_current:(uint16_t current) */
/* { */
/*     evse_slots.get(CHARGING_SLOT_CHARGE_MANAGER)->get("max_current")->updateUint(current); */
/* } */
