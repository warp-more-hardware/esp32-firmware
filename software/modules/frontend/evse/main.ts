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

import $ from "jquery";

import * as util from "../util";

interface EVSEState {
    iec61851_state: number,
    vehicle_state: number,
    contactor_state: number,
    contactor_error: number,
    allowed_charging_current: number,
    error_state: number,
    lock_state: number,
    time_since_state_change: number
    uptime: number
}

function update_evse_state(state: EVSEState) {
    util.update_button_group("btn_group_iec_state", state.iec61851_state);

    util.update_button_group("btn_group_evse_state", state.vehicle_state);

    $('#status_start_charging').prop("disabled", state.vehicle_state != 1);
    $('#status_stop_charging').prop("disabled", state.vehicle_state != 2);

    let allowed_charging_current = util.toLocaleFixed(state.allowed_charging_current / 1000.0, 3) + " A";
    $('#allowed_charging_current').val(allowed_charging_current);

    if($('#status_charging_current_save').prop("disabled")) {
        $('#status_charging_current').val(state.allowed_charging_current / 1000);
    }

    util.update_button_group("btn_group_ac1", (state.contactor_state & 1) == 1 ? 1 : 0);
    util.update_button_group("btn_group_ac2", state.contactor_state > 1 ? 1 : 0);
    util.update_button_group("btn_group_contactor_error", state.contactor_error != 0 ? 1 : 0, state.contactor_error != 0 ? __("evse.script.error_code") + " " + state.contactor_error : null);
    util.update_button_group("btn_group_error_state", state.error_state == 0 ? 0 : state.error_state - 1); // 1 is not a valid error state
    util.update_button_group("btn_group_lock_state", state.lock_state);

    $('#uptime').val(util.format_timespan(Math.floor(state.uptime / 1000)));
    $('#time_since_state_change').val(util.format_timespan(Math.floor(state.time_since_state_change / 1000)));

    if (state.iec61851_state == 2) {
        $("#evse_state_charging_text").html(__("evse.script.charging_for") + " ");
        $('#evse_state_charging_charge_time').html(util.format_timespan(Math.floor(state.time_since_state_change / 1000)));
    } else if ($('#evse_state_charging_charge_time').html() != "") {
        $("#evse_state_charging_text").html(__("evse.script.last_charge_took") + " ");
    } else {
        $("#evse_state_charging_text").html(__("evse.status.charging"));
    }
}

interface EVSEHardwareConfiguration {
    jumper_configuration: number,
    has_lock_switch: boolean,
}

function update_evse_hardware_configuration(cfg: EVSEHardwareConfiguration) {
    util.update_button_group("btn_group_has_lock_switch", cfg.has_lock_switch ? 1 : 0);
    util.update_button_group("btn_group_jumper_config", cfg.jumper_configuration);
}

interface EVSELowLevelState {
    low_level_mode_enabled: boolean,
    led_state: number,
    cp_pwm_duty_cycle: number,
    adc_values: Uint16Array,
    voltages: Int16Array,
    resistances: Uint32Array,
    gpio: boolean[]
}

function update_evse_low_level_state(state: EVSELowLevelState) {
    util.update_button_group("btn_group_low_level_mode_enabled", state.low_level_mode_enabled ? 1 : 0);
    util.update_button_group("btn_group_led_state", state.led_state);

    for(let i = 0; i < 5; ++i) {
        util.update_button_group("btn_group_gpio" + i, state.gpio[i] ? 1 : 0);
        util.update_button_group("btn_group_gpio" + i, state.gpio[i] ? 1 : 0);
        util.update_button_group("btn_group_gpio" + i, state.gpio[i] ? 1 : 0);
        util.update_button_group("btn_group_gpio" + i, state.gpio[i] ? 1 : 0);
        util.update_button_group("btn_group_gpio" + i, state.gpio[i] ? 1 : 0);
    }

    $('#pwm_duty_cycle').val(util.toLocaleFixed(state.cp_pwm_duty_cycle / 10, 1) + " %");

    $('#adc_value_0').val(state.adc_values[0]);
    $('#adc_value_1').val(state.adc_values[1]);

    $('#voltage_0').val(util.toLocaleFixed(state.voltages[0] / 1000.0, 3) + " V");
    $('#voltage_1').val(util.toLocaleFixed(state.voltages[1] / 1000.0, 3) + " V");
    $('#voltage_2').val(util.toLocaleFixed(state.voltages[2] / 1000.0, 3) + " V");

    $('#resistance_0').val(state.resistances[0] + " Ω");
    $('#resistance_1').val(state.resistances[1] + " Ω");
}

interface EVSEMaxChargingCurrent {
    max_current_configured: number,
    max_current_incoming_cable: number,
    max_current_outgoing_cable: number
}

function update_evse_max_charging_current(state: EVSEMaxChargingCurrent) {
    $('#max_current_configured').val(util.toLocaleFixed(state.max_current_configured / 1000.0, 3) + " A");
    $('#max_current_incoming_cable').val(util.toLocaleFixed(state.max_current_incoming_cable / 1000.0, 3) + " A");
    $('#max_current_outgoing_cable').val(util.toLocaleFixed(state.max_current_outgoing_cable / 1000.0, 3) + " A");

    let theoretical_maximum = Math.min(state.max_current_incoming_cable, state.max_current_outgoing_cable);
    let theoretical_maximum_str = util.toLocaleFixed(theoretical_maximum / 1000.0, 0) + " A";
    $('#status_charging_current').prop("max", theoretical_maximum / 1000);
    $('#status_charging_current_maximum').html(theoretical_maximum_str);
}

function set_charging_current(current: number) {
    $.ajax({
        url: '/evse/current_limit',
        method: 'PUT',
        contentType: 'application/json',
        data: JSON.stringify({"current": current}),
        success: () => $('#status_charging_current_save').prop("disabled", true),
        error: (xhr, status, error) => util.show_alert("alert-danger", __("evse.script.set_charging_current_failed"), error + ": " + xhr.responseText)
    });
}

interface EVSEAutoStart {
    auto_start_charging: boolean
}

function update_evse_auto_start_charging(x: EVSEAutoStart) {
    $('#status_auto_start_charging').prop("checked", x.auto_start_charging);
}

function set_auto_start_charging(auto_start_charging: boolean) {
    $.ajax({
        url: '/evse/auto_start_charging_update',
        method: 'PUT',
        contentType: 'application/json',
        data: JSON.stringify({"auto_start_charging": auto_start_charging}),
        error: (xhr, status, error) => util.show_alert("alert-danger",  __("evse.script.auto_start_charging_update"), error + ": " + xhr.responseText)
    });
}

function start_charging() {
    $.ajax({
        url: '/evse/start_charging',
        method: 'PUT',
        contentType: 'application/json',
        data: JSON.stringify(null),
        error: (xhr, status, error) => util.show_alert("alert-danger", __("evse.script.start_charging_failed"), error + ": " + xhr.responseText)
    });
}

function stop_charging() {
    $.ajax({
        url: '/evse/stop_charging',
        method: 'PUT',
        contentType: 'application/json',
        data: JSON.stringify(null),
        error: (xhr, status, error) => util.show_alert("alert-danger",  __("evse.script.stop_charging_failed"), error + ": " + xhr.responseText)
    });
}


export function init() {
    $("#status_charging_current_minimum").on("click", () => set_charging_current(6000));
    $("#status_charging_current_maximum").on("click", () => set_charging_current(32000));

    $("#status_stop_charging").on("click", stop_charging);
    $("#status_start_charging").on("click", start_charging);

    $('#status_auto_start_charging').on("change", () => set_auto_start_charging($('#status_auto_start_charging').prop('checked')));

    let input = $('#status_charging_current');
    let save_btn = $('#status_charging_current_save');
    input.on("input", () => save_btn.prop("disabled", false));

    save_btn.on("click", () => {
        if(input.val() >= 6 || input.val() <= 32)
            set_charging_current(Number(input.val() * 1000));
    });

    let form = <HTMLFormElement>$('#evse_status_charging_current_form')[0];
    form.addEventListener('submit', function (event: Event) {
        event.preventDefault();
        event.stopPropagation();

        if(input.val() >= 6 || input.val() <= 32)
            set_charging_current(Number(input.val() * 1000));
    }, false);

}

export function addEventListeners(source: EventSource) {
    source.addEventListener('evse/state', function (e: util.SSE) {
        update_evse_state(<EVSEState>(JSON.parse(e.data)));
    }, false);

    source.addEventListener('evse/low_level_state', function (e: util.SSE) {
        update_evse_low_level_state(<EVSELowLevelState>(JSON.parse(e.data)));
    }, false);

    source.addEventListener('evse/max_charging_current', function (e: util.SSE) {
        update_evse_max_charging_current(<EVSEMaxChargingCurrent>(JSON.parse(e.data)));
    }, false);

    source.addEventListener('evse/hardware_configuration', function (e: util.SSE) {
        update_evse_hardware_configuration(<EVSEHardwareConfiguration>(JSON.parse(e.data)));
    }, false);

    source.addEventListener('evse/auto_start_charging', function (e: util.SSE) {
        update_evse_auto_start_charging(<EVSEAutoStart>(JSON.parse(e.data)));
    }, false);
}

export function updateLockState(module_init) {
    $('#sidebar-evse').prop('hidden', !module_init.evse);
    $('#status-evse').prop('hidden', !module_init.evse);
}

export function getTranslation(lang: string) {
    return {
        "de": {
            "evse": {
                "status": {
                    "evse": "Ladestatus",
                    "not_connected": "Getrennt",
                    "connected": "Verbunden",
                    "charging": "Lädt",
                    "error": "Fehler",
                    "charging_current": "Ladestrom",
                    "charging_current_set": "Set",
                    "charging_current_minimum": "6 A",
                    "charging_current_maximum": "Max",
                    "charge_control": "Ladekontrolle",
                    "auto_start_charging": "Autostart",
                    "start_charging": "Start",
                    "stop_charging": "Stop"
                },
                "navbar": {
                    "evse": "Ladecontroller"
                },
                "content": {
                    "evse": "Ladecontroller (EVSE)",
                    "state": "Zustand",
                    "iec_state": "IEC-61851-Zustand",
                    "iec_state_a": "A (getrennt)",
                    "iec_state_b": "B (verbunden)",
                    "iec_state_c": "C (lädt)",
                    "iec_state_d": "D (nicht unterstützt)",
                    "iec_state_ef": "E/F (Fehler)",
                    "contactor_state": "Schützprüfung",
                    "contactor_names": "vor Schütz, nach Schütz, Zustand",
                    "contactor_not_live": "Stromlos",
                    "contactor_live": "Stromführend",
                    "contactor_ok": "OK",
                    "contactor_error": "Fehler",
                    "allowed_charging_current": "Erlaubter Ladestrom",
                    "error_state": "Fehlerzustand",
                    "error_state_desc": "<a href=\"https://www.warp-charger.com/#documents\">siehe Betriebsanleitung für Details</a>",
                    "error_ok": "OK",
                    "error_switch": "Schalterfehler",
                    "error_calibration": "Kalibrierungsfehler",
                    "error_contactor": "Schützfehler",
                    "error_communication": "Kommunikationsfehler",
                    "lock_state": "Kabelverriegelung",
                    "lock_init": "Start",
                    "lock_open": "Offen",
                    "lock_closing": "Schließend",
                    "lock_close": "Geschlossen",
                    "lock_opening": "Öffnend",
                    "lock_error": "Fehler",
                    "time_since_state_change": "Zeit seit Zustandswechsel",
                    "uptime": "Laufzeit",
                    "configuration": "Hardware-Konfiguration",
                    "has_lock_switch": "Kabelverriegelung vorhanden",
                    "lock_no": "Nein",
                    "lock_yes": "Ja",
                    "jumper_config_max_current": "Maximalstrom der Zuleitung",
                    "jumper_config": "durch Schalter konfiguriert",
                    "jumper_config_software": "Software",
                    "jumper_config_unconfigured": "Unkonfiguriert",
                    "charging_current": "Erlaubter Ladestrom",
                    "charging_current_configured": "Konfiguriert",
                    "charging_current_max_incoming": "Zuleitung",
                    "charging_current_max_outgoing": "Typ-2-Ladekabel",
                    "low_level_state": "Low-Level-Zustand",
                    "low_level_state_show": "Anzeigen / Verstecken",
                    "low_level_mode": "Low-Level-Modus",
                    "low_level_state_disabled": "Deaktiviert",
                    "low_level_state_enabled": "Aktiviert",
                    "led_state": "LED-Zustand",
                    "led_state_off": "Aus",
                    "led_state_on": "An",
                    "led_state_blinking": "Blinkend",
                    "led_state_flickering": "Flackernd",
                    "led_state_breathing": "Atmend",
                    "cp_pwm_dc": "CP-PWM-Tastverhältnis",
                    "adc_values": "ADC-Werte",
                    "adc_names": "PE-CP, PE-PP",
                    "voltages": "Spannungen",
                    "voltage_names": "PE-CP, PE-PP, Maximalspannung PE-CP",
                    "resistances": "Widerstände",
                    "resistance_names": "PE-CP, PE-PP",
                    "gpios": "GPIOs",
                    "gpio_names": "Eingang, Ausgang, Motoreingangsschalter, Relais, Motorfehler",
                    "gpio_low": "Low",
                    "gpio_high": "High",
                },
                "script": {
                    "error_code": "Fehlercode",
                    "charging_for": "Lädt seit",
                    "last_charge_took": "Letztes Laden dauerte",
                    "charging": "Lädt",
                    "set_charging_current_failed": "Konnte Ladestrom nicht setzen",
                    "not_implemented": "Noch nicht implementiert",

                    "auto_start_charging_update": "Lade-Autostart setzen fehlgeschlagen.",
                    "start_charging_failed": "Ladestart auslösen fehlgeschlagen",
                    "stop_charging_failed": "Ladestop auslösen fehlgeschlagen"
                }
            }
        },
        "en": {
            "evse": {
                "status": {
                    "evse": "Charge state",
                    "not_connected": "Disconnected",
                    "connected": "Connected",
                    "charging": "Charging",
                    "error": "Error",
                    "charging_current": "Charge current",
                    "charging_current_set": "Set",
                    "charging_current_minimum": "6 A",
                    "charging_current_maximum": "Max",
                    "charge_control": "Charge control",
                    "auto_start_charging": "Auto-start",
                    "start_charging": "Start",
                    "stop_charging": "Stop"
                },
                "navbar": {
                    "evse": "Charge Controller"
                },
                "content": {
                    "evse": "Charge Controller (EVSE)",
                    "state": "State",
                    "iec_state": "IEC 61851 State",
                    "iec_state_a": "A (disconnected)",
                    "iec_state_b": "B (connected)",
                    "iec_state_c": "C (charging)",
                    "iec_state_d": "D (not supported)",
                    "iec_state_ef": "E/F (error)",
                    "contactor_state": "Contactor check",
                    "contactor_names": "before contactor, after contactor, state",
                    "contactor_not_live": "Not live",
                    "contactor_live": "Live",
                    "contactor_ok": "OK",
                    "contactor_error": "Error",
                    "allowed_charging_current": "Allowed charging current",
                    "error_state": "Error state",
                    "error_state_desc": "<a href=\"https://www.warp-charger.com/#documents\">see manual for details</a>",
                    "error_ok": "OK",
                    "error_switch": "Switch error",
                    "error_calibration": "Calibration error",
                    "error_contactor": "Contactor error",
                    "error_communication": "Communication error",
                    "lock_state": "Cable lock",
                    "lock_init": "Init",
                    "lock_open": "Open",
                    "lock_closing": "Closing",
                    "lock_close": "Close",
                    "lock_opening": "Opening",
                    "lock_error": "Error",
                    "time_since_state_change": "Time since state change",
                    "uptime": "Uptime",
                    "configuration": "Hardware configuration",
                    "has_lock_switch": "Cable lock available",
                    "lock_no": "No",
                    "lock_yes": "Yes",
                    "jumper_config_max_current": "Max current of supply cable",
                    "jumper_config": "switch configured",
                    "jumper_config_software": "Software",
                    "jumper_config_unconfigured": "Unconfigured",
                    "charging_current": "Allowed charging current",
                    "charging_current_configured": "Configured",
                    "charging_current_max_incoming": "Supply cable",
                    "charging_current_max_outgoing": "Type 2 cable",
                    "low_level_state": "Low Level State",
                    "low_level_state_show": "Show / Hide",
                    "low_level_mode": "Low Level Mode",
                    "low_level_state_disabled": "Disabled",
                    "low_level_state_enabled": "Enabled",
                    "led_state": "LED state",
                    "led_state_off": "Off",
                    "led_state_on": "On",
                    "led_state_blinking": "Blinking",
                    "led_state_flickering": "Flickering",
                    "led_state_breathing": "Breathing",
                    "cp_pwm_dc": "CP PWM duty cycle",
                    "adc_values": "ADC values",
                    "adc_names": "PE-CP, PE-PP",
                    "voltages": "Voltages",
                    "voltage_names": "PE-CP, PE-PP, high voltage PE-CP",
                    "resistances": "Resistances",
                    "resistance_names": "PE-CP, PE-PP",
                    "gpios": "GPIOs",
                    "gpio_names": "Input, Output, Motor Input Switch, Relay, Motor Fault",
                    "gpio_low": "Low",
                    "gpio_high": "High",
                },
                "script": {
                    "error_code": "Error code",
                    "charging_for": "Charging for",
                    "last_charge_took": "Last charge took",
                    "charging": "Charging",
                    "set_charging_current_failed": "Failed to set charging current",
                    "not_implemented": "Not implemented yet",
                    "auto_start_charging_update": "Failed to set auto charge.",
                    "start_charging_failed": "Failed to start charging",
                    "stop_charging_failed": "Failed to stop charging"
                }
            }
        }
    }[lang];
}
