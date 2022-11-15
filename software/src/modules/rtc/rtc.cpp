/* esp32-firmware
 * Copyright (C) 2022 Frederic Henrichs <frederic@tinkerforge.com>
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

#include "rtc.h"
#include "build_timestamp.h"
#include "esp_sntp.h"
#include "modules.h"
#include <ctime>

extern API api;
extern TaskScheduler task_scheduler;
extern NTP ntp;

void Rtc::pre_setup()
{
    time = Config::Object({
        {"year", Config::Uint16(0)},
        {"month", Config::Uint8(0)},
        {"day", Config::Uint8(0)},
        {"hour", Config::Uint8(0)},
        {"minute", Config::Uint8(0)},
        {"second", Config::Uint8(0)},
        {"centisecond", Config::Uint8(0)},
        {"weekday", Config::Uint8(0)},
    });

    time_update = Config::Object({
        {"year", Config::Uint16(0)},
        {"month", Config::Uint8(0)},
        {"day", Config::Uint8(0)},
        {"hour", Config::Uint8(0)},
        {"minute", Config::Uint8(0)},
        {"second", Config::Uint8(0)},
        {"centisecond", Config::Uint8(0)},
        {"weekday", Config::Uint8(0)},
    });

    config = Config::Object({
        {"sync_enabled", Config::Bool(false)},
    });
}

void Rtc::update_system_time()
{
    uint32_t count;
    {
        std::lock_guard<std::mutex> lock{ntp.mtx};
        count = ntp.mtx_count;
    }

    struct timeval t = this->get_time();

    {
        std::lock_guard<std::mutex> lock{ntp.mtx};
        if (count != ntp.mtx_count)
            return;
        settimeofday(&t, nullptr);
    }
    logger.printfln("Updated system time");
}

void Rtc::setup()
{
    setup_rtc();
    if (device_found)
    {
        struct timeval time = get_time();
        if (time.tv_sec)
        {
            settimeofday(&time, nullptr);
            logger.printfln("Got time via RTC-Bricklet");
        }
        api.restorePersistentConfig("rtc/config", &config);
    }
}

void Rtc::update_time()
{
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t centisecond;
    uint8_t weekday;
    if (tf_real_time_clock_v2_get_date_time(&device,
                                        &year,
                                        &month,
                                        &day,
                                        &hour,
                                        &minute,
                                        &second,
                                        &centisecond,
                                        &weekday,
                                        NULL))
        logger.printfln("Update time failed");
    time.get("year")->updateUint(year);
    time.get("month")->updateUint(month);
    time.get("day")->updateUint(day);
    time.get("hour")->updateUint(hour);
    time.get("minute")->updateUint(minute);
    time.get("second")->updateUint(second);
    time.get("centisecond")->updateUint(centisecond);
    time.get("weekday")->updateUint(weekday);
}

void Rtc::set_time()
{
    logger.printfln("Updating Time via Web/API");
    tf_real_time_clock_v2_set_date_time(&device,
                                    (uint16_t)time_update.get("year")->asUint(),
                                    (uint8_t)time_update.get("month")->asUint(),
                                    (uint8_t)time_update.get("day")->asUint(),
                                    (uint8_t)time_update.get("hour")->asUint(),
                                    (uint8_t)time_update.get("minute")->asUint(),
                                    (uint8_t)time_update.get("second")->asUint(),
                                    (uint8_t)time_update.get("centisecond")->asUint(),
                                    (uint8_t)time_update.get("weekday")->asUint());
    ntp.set_last_sync();
    task_scheduler.scheduleOnce([this]() {
        update_system_time();
    }, 500);
}

void Rtc::set_time(time_t time)
{
    tm *date_time = gmtime(&time);

    date_time->tm_year += 1900;
    int ret = tf_real_time_clock_v2_set_date_time(&device,
                                        (uint16_t)date_time->tm_year,
                                        (uint8_t)date_time->tm_mon + 1,
                                        (uint8_t)date_time->tm_mday,
                                        (uint8_t)date_time->tm_hour,
                                        (uint8_t)date_time->tm_min,
                                        (uint8_t)date_time->tm_sec,
                                        (uint8_t)0,
                                        (uint8_t)date_time->tm_wday);
    if (ret)
        logger.printfln("Setting rtc failed with code %i", ret);
    free(date_time);
    ntp.set_last_sync();
}

struct timeval Rtc::get_time()
{
    int64_t ts;
    int ret = tf_real_time_clock_v2_get_timestamp(&device, &ts);
    if (ret)
    {
        logger.printfln("Reading rtc failed with code %i", ret);
        struct timeval tmp;
        tmp.tv_sec = 0;
        tmp.tv_usec = 0;
        return tmp;
    }

    struct timeval time;
    time.tv_usec = ts % 1000 * 1000;
    time.tv_sec = ts / 1000;

    time.tv_sec += 946684800;

    if (time.tv_sec < BUILD_TIMESTAMP)
    {
        struct timeval tmp;
        tmp.tv_sec = 0;
        tmp.tv_usec = 0;
        return tmp;
    }
    return time;
}

void Rtc::register_urls()
{
    if (!device_found)
        return;

    DeviceModule::register_urls();

    api.addPersistentConfig("rtc/config", &config, {}, 1000);

    api.addState("rtc/state", &time, {}, 1000);
    api.addCommand("rtc/state_update", &time_update, {}, [this]() {
        set_time();
    }, true);

    api.addFeature("rtc");

    task_scheduler.scheduleWithFixedDelay([this]() {
        update_time();
    }, 0, 1000);

    task_scheduler.scheduleWithFixedDelay([this]() {
        update_system_time();
    }, 1000 * 60 * 10, 1000 * 60 * 60 * 25);
}

void Rtc::loop()
{
}

void Rtc::setup_rtc()
{
    if (!this->DeviceModule::setup_device())
        return;

    tf_real_time_clock_v2_set_response_expected(&device, TF_REAL_TIME_CLOCK_V2_FUNCTION_SET_DATE_TIME, true);

    initialized = true;
}