/* ***********************************************************
 * This file was automatically generated on 2021-11-26.      *
 *                                                           *
 * C/C++ for Microcontrollers Bindings Version 2.0.0         *
 *                                                           *
 * If you have a bugfix for this file and want to commit it, *
 * please fix the bug in the generator. You can find a link  *
 * to the generators git repository on tinkerforge.com       *
 *************************************************************/


#include "bricklet_air_quality.h"
#include "base58.h"
#include "endian_convert.h"
#include "errors.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


#if TF_IMPLEMENT_CALLBACKS != 0
static bool tf_air_quality_callback_handler(void *device, uint8_t fid, TF_PacketBuffer *payload) {
    TF_AirQuality *air_quality = (TF_AirQuality *)device;
    TF_HALCommon *hal_common = tf_hal_get_common(air_quality->tfp->spitfp->hal);
    (void)payload;

    switch (fid) {
        case TF_AIR_QUALITY_CALLBACK_ALL_VALUES: {
            TF_AirQuality_AllValuesHandler fn = air_quality->all_values_handler;
            void *user_data = air_quality->all_values_user_data;
            if (fn == NULL) {
                return false;
            }

            int32_t iaq_index = tf_packet_buffer_read_int32_t(payload);
            uint8_t iaq_index_accuracy = tf_packet_buffer_read_uint8_t(payload);
            int32_t temperature = tf_packet_buffer_read_int32_t(payload);
            int32_t humidity = tf_packet_buffer_read_int32_t(payload);
            int32_t air_pressure = tf_packet_buffer_read_int32_t(payload);
            hal_common->locked = true;
            fn(air_quality, iaq_index, iaq_index_accuracy, temperature, humidity, air_pressure, user_data);
            hal_common->locked = false;
            break;
        }

        case TF_AIR_QUALITY_CALLBACK_IAQ_INDEX: {
            TF_AirQuality_IAQIndexHandler fn = air_quality->iaq_index_handler;
            void *user_data = air_quality->iaq_index_user_data;
            if (fn == NULL) {
                return false;
            }

            int32_t iaq_index = tf_packet_buffer_read_int32_t(payload);
            uint8_t iaq_index_accuracy = tf_packet_buffer_read_uint8_t(payload);
            hal_common->locked = true;
            fn(air_quality, iaq_index, iaq_index_accuracy, user_data);
            hal_common->locked = false;
            break;
        }

        case TF_AIR_QUALITY_CALLBACK_TEMPERATURE: {
            TF_AirQuality_TemperatureHandler fn = air_quality->temperature_handler;
            void *user_data = air_quality->temperature_user_data;
            if (fn == NULL) {
                return false;
            }

            int32_t temperature = tf_packet_buffer_read_int32_t(payload);
            hal_common->locked = true;
            fn(air_quality, temperature, user_data);
            hal_common->locked = false;
            break;
        }

        case TF_AIR_QUALITY_CALLBACK_HUMIDITY: {
            TF_AirQuality_HumidityHandler fn = air_quality->humidity_handler;
            void *user_data = air_quality->humidity_user_data;
            if (fn == NULL) {
                return false;
            }

            int32_t humidity = tf_packet_buffer_read_int32_t(payload);
            hal_common->locked = true;
            fn(air_quality, humidity, user_data);
            hal_common->locked = false;
            break;
        }

        case TF_AIR_QUALITY_CALLBACK_AIR_PRESSURE: {
            TF_AirQuality_AirPressureHandler fn = air_quality->air_pressure_handler;
            void *user_data = air_quality->air_pressure_user_data;
            if (fn == NULL) {
                return false;
            }

            int32_t air_pressure = tf_packet_buffer_read_int32_t(payload);
            hal_common->locked = true;
            fn(air_quality, air_pressure, user_data);
            hal_common->locked = false;
            break;
        }

        default:
            return false;
    }

    return true;
}
#else
static bool tf_air_quality_callback_handler(void *device, uint8_t fid, TF_PacketBuffer *payload) {
    return false;
}
#endif
int tf_air_quality_create(TF_AirQuality *air_quality, const char *uid, TF_HAL *hal) {
    if (air_quality == NULL || hal == NULL) {
        return TF_E_NULL;
    }

    static uint16_t next_tfp_index = 0;

    memset(air_quality, 0, sizeof(TF_AirQuality));

    TF_TFP *tfp;

    if (uid != NULL && *uid != '\0') {
        uint32_t uid_num = 0;
        int rc = tf_base58_decode(uid, &uid_num);

        if (rc != TF_E_OK) {
            return rc;
        }

        tfp = tf_hal_get_tfp(hal, &next_tfp_index, &uid_num, NULL, NULL);

        if (tfp == NULL) {
            return TF_E_DEVICE_NOT_FOUND;
        }

        if (tfp->device_id != TF_AIR_QUALITY_DEVICE_IDENTIFIER) {
            return TF_E_WRONG_DEVICE_TYPE;
        }
    } else {
        uint16_t device_id = TF_AIR_QUALITY_DEVICE_IDENTIFIER;

        tfp = tf_hal_get_tfp(hal, &next_tfp_index, NULL, NULL, &device_id);

        if (tfp == NULL) {
            return TF_E_DEVICE_NOT_FOUND;
        }
    }

    if (tfp->device != NULL) {
        return TF_E_DEVICE_ALREADY_IN_USE;
    }

    air_quality->tfp = tfp;
    air_quality->tfp->device = air_quality;
    air_quality->tfp->cb_handler = tf_air_quality_callback_handler;
    air_quality->response_expected[0] = 0x3E;
    air_quality->response_expected[1] = 0x00;

    return TF_E_OK;
}

int tf_air_quality_destroy(TF_AirQuality *air_quality) {
    if (air_quality == NULL || air_quality->tfp == NULL) {
        return TF_E_NULL;
    }

    air_quality->tfp->cb_handler = NULL;
    air_quality->tfp->device = NULL;
    air_quality->tfp = NULL;

    return TF_E_OK;
}

int tf_air_quality_get_response_expected(TF_AirQuality *air_quality, uint8_t function_id, bool *ret_response_expected) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    switch (function_id) {
        case TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_OFFSET:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 0)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_ALL_VALUES_CALLBACK_CONFIGURATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 1)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_IAQ_INDEX_CALLBACK_CONFIGURATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 2)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_CALLBACK_CONFIGURATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 3)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_HUMIDITY_CALLBACK_CONFIGURATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 4)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_AIR_PRESSURE_CALLBACK_CONFIGURATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 5)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_REMOVE_CALIBRATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 6)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_BACKGROUND_CALIBRATION_DURATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[0] & (1 << 7)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_WRITE_FIRMWARE_POINTER:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[1] & (1 << 0)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_STATUS_LED_CONFIG:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[1] & (1 << 1)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_RESET:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[1] & (1 << 2)) != 0;
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_WRITE_UID:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (air_quality->response_expected[1] & (1 << 3)) != 0;
            }
            break;
        default:
            return TF_E_INVALID_PARAMETER;
    }

    return TF_E_OK;
}

int tf_air_quality_set_response_expected(TF_AirQuality *air_quality, uint8_t function_id, bool response_expected) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    switch (function_id) {
        case TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_OFFSET:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 0);
            } else {
                air_quality->response_expected[0] &= ~(1 << 0);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_ALL_VALUES_CALLBACK_CONFIGURATION:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 1);
            } else {
                air_quality->response_expected[0] &= ~(1 << 1);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_IAQ_INDEX_CALLBACK_CONFIGURATION:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 2);
            } else {
                air_quality->response_expected[0] &= ~(1 << 2);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_CALLBACK_CONFIGURATION:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 3);
            } else {
                air_quality->response_expected[0] &= ~(1 << 3);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_HUMIDITY_CALLBACK_CONFIGURATION:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 4);
            } else {
                air_quality->response_expected[0] &= ~(1 << 4);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_AIR_PRESSURE_CALLBACK_CONFIGURATION:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 5);
            } else {
                air_quality->response_expected[0] &= ~(1 << 5);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_REMOVE_CALIBRATION:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 6);
            } else {
                air_quality->response_expected[0] &= ~(1 << 6);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_BACKGROUND_CALIBRATION_DURATION:
            if (response_expected) {
                air_quality->response_expected[0] |= (1 << 7);
            } else {
                air_quality->response_expected[0] &= ~(1 << 7);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_WRITE_FIRMWARE_POINTER:
            if (response_expected) {
                air_quality->response_expected[1] |= (1 << 0);
            } else {
                air_quality->response_expected[1] &= ~(1 << 0);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_SET_STATUS_LED_CONFIG:
            if (response_expected) {
                air_quality->response_expected[1] |= (1 << 1);
            } else {
                air_quality->response_expected[1] &= ~(1 << 1);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_RESET:
            if (response_expected) {
                air_quality->response_expected[1] |= (1 << 2);
            } else {
                air_quality->response_expected[1] &= ~(1 << 2);
            }
            break;
        case TF_AIR_QUALITY_FUNCTION_WRITE_UID:
            if (response_expected) {
                air_quality->response_expected[1] |= (1 << 3);
            } else {
                air_quality->response_expected[1] &= ~(1 << 3);
            }
            break;
        default:
            return TF_E_INVALID_PARAMETER;
    }

    return TF_E_OK;
}

int tf_air_quality_set_response_expected_all(TF_AirQuality *air_quality, bool response_expected) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    memset(air_quality->response_expected, response_expected ? 0xFF : 0, 2);

    return TF_E_OK;
}

int tf_air_quality_get_all_values(TF_AirQuality *air_quality, int32_t *ret_iaq_index, uint8_t *ret_iaq_index_accuracy, int32_t *ret_temperature, int32_t *ret_humidity, int32_t *ret_air_pressure) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_ALL_VALUES, 0, 17, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_iaq_index != NULL) { *ret_iaq_index = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_iaq_index_accuracy != NULL) { *ret_iaq_index_accuracy = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_temperature != NULL) { *ret_temperature = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_humidity != NULL) { *ret_humidity = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_air_pressure != NULL) { *ret_air_pressure = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_temperature_offset(TF_AirQuality *air_quality, int32_t offset) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_OFFSET, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_OFFSET, 4, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    offset = tf_leconvert_int32_to(offset); memcpy(send_buf + 0, &offset, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_temperature_offset(TF_AirQuality *air_quality, int32_t *ret_offset) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_TEMPERATURE_OFFSET, 0, 4, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_offset != NULL) { *ret_offset = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_all_values_callback_configuration(TF_AirQuality *air_quality, uint32_t period, bool value_has_to_change) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_ALL_VALUES_CALLBACK_CONFIGURATION, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_ALL_VALUES_CALLBACK_CONFIGURATION, 5, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    period = tf_leconvert_uint32_to(period); memcpy(send_buf + 0, &period, 4);
    send_buf[4] = value_has_to_change ? 1 : 0;

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_all_values_callback_configuration(TF_AirQuality *air_quality, uint32_t *ret_period, bool *ret_value_has_to_change) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_ALL_VALUES_CALLBACK_CONFIGURATION, 0, 5, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_period != NULL) { *ret_period = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_value_has_to_change != NULL) { *ret_value_has_to_change = tf_packet_buffer_read_bool(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_iaq_index(TF_AirQuality *air_quality, int32_t *ret_iaq_index, uint8_t *ret_iaq_index_accuracy) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_IAQ_INDEX, 0, 5, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_iaq_index != NULL) { *ret_iaq_index = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_iaq_index_accuracy != NULL) { *ret_iaq_index_accuracy = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_iaq_index_callback_configuration(TF_AirQuality *air_quality, uint32_t period, bool value_has_to_change) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_IAQ_INDEX_CALLBACK_CONFIGURATION, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_IAQ_INDEX_CALLBACK_CONFIGURATION, 5, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    period = tf_leconvert_uint32_to(period); memcpy(send_buf + 0, &period, 4);
    send_buf[4] = value_has_to_change ? 1 : 0;

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_iaq_index_callback_configuration(TF_AirQuality *air_quality, uint32_t *ret_period, bool *ret_value_has_to_change) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_IAQ_INDEX_CALLBACK_CONFIGURATION, 0, 5, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_period != NULL) { *ret_period = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_value_has_to_change != NULL) { *ret_value_has_to_change = tf_packet_buffer_read_bool(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_temperature(TF_AirQuality *air_quality, int32_t *ret_temperature) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_TEMPERATURE, 0, 4, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_temperature != NULL) { *ret_temperature = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_temperature_callback_configuration(TF_AirQuality *air_quality, uint32_t period, bool value_has_to_change, char option, int32_t min, int32_t max) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_CALLBACK_CONFIGURATION, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_TEMPERATURE_CALLBACK_CONFIGURATION, 14, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    period = tf_leconvert_uint32_to(period); memcpy(send_buf + 0, &period, 4);
    send_buf[4] = value_has_to_change ? 1 : 0;
    send_buf[5] = (uint8_t)option;
    min = tf_leconvert_int32_to(min); memcpy(send_buf + 6, &min, 4);
    max = tf_leconvert_int32_to(max); memcpy(send_buf + 10, &max, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_temperature_callback_configuration(TF_AirQuality *air_quality, uint32_t *ret_period, bool *ret_value_has_to_change, char *ret_option, int32_t *ret_min, int32_t *ret_max) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_TEMPERATURE_CALLBACK_CONFIGURATION, 0, 14, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_period != NULL) { *ret_period = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_value_has_to_change != NULL) { *ret_value_has_to_change = tf_packet_buffer_read_bool(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_option != NULL) { *ret_option = tf_packet_buffer_read_char(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_min != NULL) { *ret_min = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_max != NULL) { *ret_max = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_humidity(TF_AirQuality *air_quality, int32_t *ret_humidity) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_HUMIDITY, 0, 4, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_humidity != NULL) { *ret_humidity = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_humidity_callback_configuration(TF_AirQuality *air_quality, uint32_t period, bool value_has_to_change, char option, int32_t min, int32_t max) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_HUMIDITY_CALLBACK_CONFIGURATION, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_HUMIDITY_CALLBACK_CONFIGURATION, 14, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    period = tf_leconvert_uint32_to(period); memcpy(send_buf + 0, &period, 4);
    send_buf[4] = value_has_to_change ? 1 : 0;
    send_buf[5] = (uint8_t)option;
    min = tf_leconvert_int32_to(min); memcpy(send_buf + 6, &min, 4);
    max = tf_leconvert_int32_to(max); memcpy(send_buf + 10, &max, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_humidity_callback_configuration(TF_AirQuality *air_quality, uint32_t *ret_period, bool *ret_value_has_to_change, char *ret_option, int32_t *ret_min, int32_t *ret_max) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_HUMIDITY_CALLBACK_CONFIGURATION, 0, 14, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_period != NULL) { *ret_period = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_value_has_to_change != NULL) { *ret_value_has_to_change = tf_packet_buffer_read_bool(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_option != NULL) { *ret_option = tf_packet_buffer_read_char(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_min != NULL) { *ret_min = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_max != NULL) { *ret_max = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_air_pressure(TF_AirQuality *air_quality, int32_t *ret_air_pressure) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_AIR_PRESSURE, 0, 4, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_air_pressure != NULL) { *ret_air_pressure = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_air_pressure_callback_configuration(TF_AirQuality *air_quality, uint32_t period, bool value_has_to_change, char option, int32_t min, int32_t max) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_AIR_PRESSURE_CALLBACK_CONFIGURATION, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_AIR_PRESSURE_CALLBACK_CONFIGURATION, 14, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    period = tf_leconvert_uint32_to(period); memcpy(send_buf + 0, &period, 4);
    send_buf[4] = value_has_to_change ? 1 : 0;
    send_buf[5] = (uint8_t)option;
    min = tf_leconvert_int32_to(min); memcpy(send_buf + 6, &min, 4);
    max = tf_leconvert_int32_to(max); memcpy(send_buf + 10, &max, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_air_pressure_callback_configuration(TF_AirQuality *air_quality, uint32_t *ret_period, bool *ret_value_has_to_change, char *ret_option, int32_t *ret_min, int32_t *ret_max) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_AIR_PRESSURE_CALLBACK_CONFIGURATION, 0, 14, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_period != NULL) { *ret_period = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_value_has_to_change != NULL) { *ret_value_has_to_change = tf_packet_buffer_read_bool(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_option != NULL) { *ret_option = tf_packet_buffer_read_char(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_min != NULL) { *ret_min = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_max != NULL) { *ret_max = tf_packet_buffer_read_int32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_remove_calibration(TF_AirQuality *air_quality) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_REMOVE_CALIBRATION, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_REMOVE_CALIBRATION, 0, 0, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_background_calibration_duration(TF_AirQuality *air_quality, uint8_t duration) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_BACKGROUND_CALIBRATION_DURATION, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_BACKGROUND_CALIBRATION_DURATION, 1, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    send_buf[0] = (uint8_t)duration;

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_background_calibration_duration(TF_AirQuality *air_quality, uint8_t *ret_duration) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_BACKGROUND_CALIBRATION_DURATION, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_duration != NULL) { *ret_duration = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_spitfp_error_count(TF_AirQuality *air_quality, uint32_t *ret_error_count_ack_checksum, uint32_t *ret_error_count_message_checksum, uint32_t *ret_error_count_frame, uint32_t *ret_error_count_overflow) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_SPITFP_ERROR_COUNT, 0, 16, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_error_count_ack_checksum != NULL) { *ret_error_count_ack_checksum = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_error_count_message_checksum != NULL) { *ret_error_count_message_checksum = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_error_count_frame != NULL) { *ret_error_count_frame = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_error_count_overflow != NULL) { *ret_error_count_overflow = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_bootloader_mode(TF_AirQuality *air_quality, uint8_t mode, uint8_t *ret_status) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_BOOTLOADER_MODE, 1, 1, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    send_buf[0] = (uint8_t)mode;

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_status != NULL) { *ret_status = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_bootloader_mode(TF_AirQuality *air_quality, uint8_t *ret_mode) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_BOOTLOADER_MODE, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_mode != NULL) { *ret_mode = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_write_firmware_pointer(TF_AirQuality *air_quality, uint32_t pointer) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_WRITE_FIRMWARE_POINTER, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_WRITE_FIRMWARE_POINTER, 4, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    pointer = tf_leconvert_uint32_to(pointer); memcpy(send_buf + 0, &pointer, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_write_firmware(TF_AirQuality *air_quality, const uint8_t data[64], uint8_t *ret_status) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_WRITE_FIRMWARE, 64, 1, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    memcpy(send_buf + 0, data, 64);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_status != NULL) { *ret_status = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_set_status_led_config(TF_AirQuality *air_quality, uint8_t config) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_SET_STATUS_LED_CONFIG, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_SET_STATUS_LED_CONFIG, 1, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    send_buf[0] = (uint8_t)config;

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_status_led_config(TF_AirQuality *air_quality, uint8_t *ret_config) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_STATUS_LED_CONFIG, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_config != NULL) { *ret_config = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_chip_temperature(TF_AirQuality *air_quality, int16_t *ret_temperature) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_CHIP_TEMPERATURE, 0, 2, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_temperature != NULL) { *ret_temperature = tf_packet_buffer_read_int16_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 2); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_reset(TF_AirQuality *air_quality) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_RESET, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_RESET, 0, 0, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_write_uid(TF_AirQuality *air_quality, uint32_t uid) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_air_quality_get_response_expected(air_quality, TF_AIR_QUALITY_FUNCTION_WRITE_UID, &response_expected);
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_WRITE_UID, 4, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(air_quality->tfp);

    uid = tf_leconvert_uint32_to(uid); memcpy(send_buf + 0, &uid, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_read_uid(TF_AirQuality *air_quality, uint32_t *ret_uid) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_READ_UID, 0, 4, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_uid != NULL) { *ret_uid = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_air_quality_get_identity(TF_AirQuality *air_quality, char ret_uid[8], char ret_connected_uid[8], char *ret_position, uint8_t ret_hardware_version[3], uint8_t ret_firmware_version[3], uint16_t *ret_device_identifier) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(air_quality->tfp, TF_AIR_QUALITY_FUNCTION_GET_IDENTITY, 0, 25, response_expected);

    size_t i;
    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(air_quality->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(air_quality->tfp);
        if (ret_uid != NULL) { tf_packet_buffer_pop_n(recv_buf, (uint8_t *)ret_uid, 8);} else { tf_packet_buffer_remove(recv_buf, 8); }
        if (ret_connected_uid != NULL) { tf_packet_buffer_pop_n(recv_buf, (uint8_t *)ret_connected_uid, 8);} else { tf_packet_buffer_remove(recv_buf, 8); }
        if (ret_position != NULL) { *ret_position = tf_packet_buffer_read_char(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_hardware_version != NULL) { for (i = 0; i < 3; ++i) ret_hardware_version[i] = tf_packet_buffer_read_uint8_t(recv_buf);} else { tf_packet_buffer_remove(recv_buf, 3); }
        if (ret_firmware_version != NULL) { for (i = 0; i < 3; ++i) ret_firmware_version[i] = tf_packet_buffer_read_uint8_t(recv_buf);} else { tf_packet_buffer_remove(recv_buf, 3); }
        if (ret_device_identifier != NULL) { *ret_device_identifier = tf_packet_buffer_read_uint16_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 2); }
        tf_tfp_packet_processed(air_quality->tfp);
    }

    result = tf_tfp_finish_send(air_quality->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}
#if TF_IMPLEMENT_CALLBACKS != 0
int tf_air_quality_register_all_values_callback(TF_AirQuality *air_quality, TF_AirQuality_AllValuesHandler handler, void *user_data) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    if (handler == NULL) {
        air_quality->tfp->needs_callback_tick = false;
        air_quality->tfp->needs_callback_tick |= air_quality->iaq_index_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->temperature_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->humidity_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->air_pressure_handler != NULL;
    } else {
        air_quality->tfp->needs_callback_tick = true;
    }

    air_quality->all_values_handler = handler;
    air_quality->all_values_user_data = user_data;

    return TF_E_OK;
}


int tf_air_quality_register_iaq_index_callback(TF_AirQuality *air_quality, TF_AirQuality_IAQIndexHandler handler, void *user_data) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    if (handler == NULL) {
        air_quality->tfp->needs_callback_tick = false;
        air_quality->tfp->needs_callback_tick |= air_quality->all_values_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->temperature_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->humidity_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->air_pressure_handler != NULL;
    } else {
        air_quality->tfp->needs_callback_tick = true;
    }

    air_quality->iaq_index_handler = handler;
    air_quality->iaq_index_user_data = user_data;

    return TF_E_OK;
}


int tf_air_quality_register_temperature_callback(TF_AirQuality *air_quality, TF_AirQuality_TemperatureHandler handler, void *user_data) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    if (handler == NULL) {
        air_quality->tfp->needs_callback_tick = false;
        air_quality->tfp->needs_callback_tick |= air_quality->all_values_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->iaq_index_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->humidity_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->air_pressure_handler != NULL;
    } else {
        air_quality->tfp->needs_callback_tick = true;
    }

    air_quality->temperature_handler = handler;
    air_quality->temperature_user_data = user_data;

    return TF_E_OK;
}


int tf_air_quality_register_humidity_callback(TF_AirQuality *air_quality, TF_AirQuality_HumidityHandler handler, void *user_data) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    if (handler == NULL) {
        air_quality->tfp->needs_callback_tick = false;
        air_quality->tfp->needs_callback_tick |= air_quality->all_values_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->iaq_index_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->temperature_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->air_pressure_handler != NULL;
    } else {
        air_quality->tfp->needs_callback_tick = true;
    }

    air_quality->humidity_handler = handler;
    air_quality->humidity_user_data = user_data;

    return TF_E_OK;
}


int tf_air_quality_register_air_pressure_callback(TF_AirQuality *air_quality, TF_AirQuality_AirPressureHandler handler, void *user_data) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    if (handler == NULL) {
        air_quality->tfp->needs_callback_tick = false;
        air_quality->tfp->needs_callback_tick |= air_quality->all_values_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->iaq_index_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->temperature_handler != NULL;
        air_quality->tfp->needs_callback_tick |= air_quality->humidity_handler != NULL;
    } else {
        air_quality->tfp->needs_callback_tick = true;
    }

    air_quality->air_pressure_handler = handler;
    air_quality->air_pressure_user_data = user_data;

    return TF_E_OK;
}
#endif
int tf_air_quality_callback_tick(TF_AirQuality *air_quality, uint32_t timeout_us) {
    if (air_quality == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = air_quality->tfp->spitfp->hal;

    return tf_tfp_callback_tick(air_quality->tfp, tf_hal_current_time_us(hal) + timeout_us);
}

#ifdef __cplusplus
}
#endif
