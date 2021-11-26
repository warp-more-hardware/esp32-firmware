/* ***********************************************************
 * This file was automatically generated on 2021-11-26.      *
 *                                                           *
 * C/C++ for Microcontrollers Bindings Version 2.0.0         *
 *                                                           *
 * If you have a bugfix for this file and want to commit it, *
 * please fix the bug in the generator. You can find a link  *
 * to the generators git repository on tinkerforge.com       *
 *************************************************************/


#include "brick_hat_zero.h"
#include "base58.h"
#include "endian_convert.h"
#include "errors.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


#if TF_IMPLEMENT_CALLBACKS != 0
static bool tf_hat_zero_callback_handler(void *device, uint8_t fid, TF_PacketBuffer *payload) {
    TF_HATZero *hat_zero = (TF_HATZero *)device;
    TF_HALCommon *hal_common = tf_hal_get_common(hat_zero->tfp->spitfp->hal);
    (void)payload;

    switch (fid) {
        case TF_HAT_ZERO_CALLBACK_USB_VOLTAGE: {
            TF_HATZero_USBVoltageHandler fn = hat_zero->usb_voltage_handler;
            void *user_data = hat_zero->usb_voltage_user_data;
            if (fn == NULL) {
                return false;
            }

            uint16_t voltage = tf_packet_buffer_read_uint16_t(payload);
            hal_common->locked = true;
            fn(hat_zero, voltage, user_data);
            hal_common->locked = false;
            break;
        }

        default:
            return false;
    }

    return true;
}
#else
static bool tf_hat_zero_callback_handler(void *device, uint8_t fid, TF_PacketBuffer *payload) {
    return false;
}
#endif
int tf_hat_zero_create(TF_HATZero *hat_zero, const char *uid, TF_HAL *hal) {
    if (hat_zero == NULL || hal == NULL) {
        return TF_E_NULL;
    }

    static uint16_t next_tfp_index = 0;

    memset(hat_zero, 0, sizeof(TF_HATZero));

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

        if (tfp->device_id != TF_HAT_ZERO_DEVICE_IDENTIFIER) {
            return TF_E_WRONG_DEVICE_TYPE;
        }
    } else {
        uint16_t device_id = TF_HAT_ZERO_DEVICE_IDENTIFIER;

        tfp = tf_hal_get_tfp(hal, &next_tfp_index, NULL, NULL, &device_id);

        if (tfp == NULL) {
            return TF_E_DEVICE_NOT_FOUND;
        }
    }

    if (tfp->device != NULL) {
        return TF_E_DEVICE_ALREADY_IN_USE;
    }

    hat_zero->tfp = tfp;
    hat_zero->tfp->device = hat_zero;
    hat_zero->tfp->cb_handler = tf_hat_zero_callback_handler;
    hat_zero->response_expected[0] = 0x01;

    return TF_E_OK;
}

int tf_hat_zero_destroy(TF_HATZero *hat_zero) {
    if (hat_zero == NULL || hat_zero->tfp == NULL) {
        return TF_E_NULL;
    }

    hat_zero->tfp->cb_handler = NULL;
    hat_zero->tfp->device = NULL;
    hat_zero->tfp = NULL;

    return TF_E_OK;
}

int tf_hat_zero_get_response_expected(TF_HATZero *hat_zero, uint8_t function_id, bool *ret_response_expected) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    switch (function_id) {
        case TF_HAT_ZERO_FUNCTION_SET_USB_VOLTAGE_CALLBACK_CONFIGURATION:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (hat_zero->response_expected[0] & (1 << 0)) != 0;
            }
            break;
        case TF_HAT_ZERO_FUNCTION_SET_WRITE_FIRMWARE_POINTER:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (hat_zero->response_expected[0] & (1 << 1)) != 0;
            }
            break;
        case TF_HAT_ZERO_FUNCTION_SET_STATUS_LED_CONFIG:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (hat_zero->response_expected[0] & (1 << 2)) != 0;
            }
            break;
        case TF_HAT_ZERO_FUNCTION_RESET:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (hat_zero->response_expected[0] & (1 << 3)) != 0;
            }
            break;
        case TF_HAT_ZERO_FUNCTION_WRITE_UID:
            if (ret_response_expected != NULL) {
                *ret_response_expected = (hat_zero->response_expected[0] & (1 << 4)) != 0;
            }
            break;
        default:
            return TF_E_INVALID_PARAMETER;
    }

    return TF_E_OK;
}

int tf_hat_zero_set_response_expected(TF_HATZero *hat_zero, uint8_t function_id, bool response_expected) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    switch (function_id) {
        case TF_HAT_ZERO_FUNCTION_SET_USB_VOLTAGE_CALLBACK_CONFIGURATION:
            if (response_expected) {
                hat_zero->response_expected[0] |= (1 << 0);
            } else {
                hat_zero->response_expected[0] &= ~(1 << 0);
            }
            break;
        case TF_HAT_ZERO_FUNCTION_SET_WRITE_FIRMWARE_POINTER:
            if (response_expected) {
                hat_zero->response_expected[0] |= (1 << 1);
            } else {
                hat_zero->response_expected[0] &= ~(1 << 1);
            }
            break;
        case TF_HAT_ZERO_FUNCTION_SET_STATUS_LED_CONFIG:
            if (response_expected) {
                hat_zero->response_expected[0] |= (1 << 2);
            } else {
                hat_zero->response_expected[0] &= ~(1 << 2);
            }
            break;
        case TF_HAT_ZERO_FUNCTION_RESET:
            if (response_expected) {
                hat_zero->response_expected[0] |= (1 << 3);
            } else {
                hat_zero->response_expected[0] &= ~(1 << 3);
            }
            break;
        case TF_HAT_ZERO_FUNCTION_WRITE_UID:
            if (response_expected) {
                hat_zero->response_expected[0] |= (1 << 4);
            } else {
                hat_zero->response_expected[0] &= ~(1 << 4);
            }
            break;
        default:
            return TF_E_INVALID_PARAMETER;
    }

    return TF_E_OK;
}

int tf_hat_zero_set_response_expected_all(TF_HATZero *hat_zero, bool response_expected) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    memset(hat_zero->response_expected, response_expected ? 0xFF : 0, 1);

    return TF_E_OK;
}

int tf_hat_zero_get_usb_voltage(TF_HATZero *hat_zero, uint16_t *ret_voltage) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_GET_USB_VOLTAGE, 0, 2, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_voltage != NULL) { *ret_voltage = tf_packet_buffer_read_uint16_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 2); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_set_usb_voltage_callback_configuration(TF_HATZero *hat_zero, uint32_t period, bool value_has_to_change, char option, uint16_t min, uint16_t max) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_hat_zero_get_response_expected(hat_zero, TF_HAT_ZERO_FUNCTION_SET_USB_VOLTAGE_CALLBACK_CONFIGURATION, &response_expected);
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_SET_USB_VOLTAGE_CALLBACK_CONFIGURATION, 10, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(hat_zero->tfp);

    period = tf_leconvert_uint32_to(period); memcpy(send_buf + 0, &period, 4);
    send_buf[4] = value_has_to_change ? 1 : 0;
    send_buf[5] = (uint8_t)option;
    min = tf_leconvert_uint16_to(min); memcpy(send_buf + 6, &min, 2);
    max = tf_leconvert_uint16_to(max); memcpy(send_buf + 8, &max, 2);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_get_usb_voltage_callback_configuration(TF_HATZero *hat_zero, uint32_t *ret_period, bool *ret_value_has_to_change, char *ret_option, uint16_t *ret_min, uint16_t *ret_max) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_GET_USB_VOLTAGE_CALLBACK_CONFIGURATION, 0, 10, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_period != NULL) { *ret_period = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_value_has_to_change != NULL) { *ret_value_has_to_change = tf_packet_buffer_read_bool(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_option != NULL) { *ret_option = tf_packet_buffer_read_char(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_min != NULL) { *ret_min = tf_packet_buffer_read_uint16_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 2); }
        if (ret_max != NULL) { *ret_max = tf_packet_buffer_read_uint16_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 2); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_get_spitfp_error_count(TF_HATZero *hat_zero, uint32_t *ret_error_count_ack_checksum, uint32_t *ret_error_count_message_checksum, uint32_t *ret_error_count_frame, uint32_t *ret_error_count_overflow) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_GET_SPITFP_ERROR_COUNT, 0, 16, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_error_count_ack_checksum != NULL) { *ret_error_count_ack_checksum = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_error_count_message_checksum != NULL) { *ret_error_count_message_checksum = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_error_count_frame != NULL) { *ret_error_count_frame = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        if (ret_error_count_overflow != NULL) { *ret_error_count_overflow = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_set_bootloader_mode(TF_HATZero *hat_zero, uint8_t mode, uint8_t *ret_status) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_SET_BOOTLOADER_MODE, 1, 1, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(hat_zero->tfp);

    send_buf[0] = (uint8_t)mode;

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_status != NULL) { *ret_status = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_get_bootloader_mode(TF_HATZero *hat_zero, uint8_t *ret_mode) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_GET_BOOTLOADER_MODE, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_mode != NULL) { *ret_mode = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_set_write_firmware_pointer(TF_HATZero *hat_zero, uint32_t pointer) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_hat_zero_get_response_expected(hat_zero, TF_HAT_ZERO_FUNCTION_SET_WRITE_FIRMWARE_POINTER, &response_expected);
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_SET_WRITE_FIRMWARE_POINTER, 4, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(hat_zero->tfp);

    pointer = tf_leconvert_uint32_to(pointer); memcpy(send_buf + 0, &pointer, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_write_firmware(TF_HATZero *hat_zero, const uint8_t data[64], uint8_t *ret_status) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_WRITE_FIRMWARE, 64, 1, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(hat_zero->tfp);

    memcpy(send_buf + 0, data, 64);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_status != NULL) { *ret_status = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_set_status_led_config(TF_HATZero *hat_zero, uint8_t config) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_hat_zero_get_response_expected(hat_zero, TF_HAT_ZERO_FUNCTION_SET_STATUS_LED_CONFIG, &response_expected);
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_SET_STATUS_LED_CONFIG, 1, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(hat_zero->tfp);

    send_buf[0] = (uint8_t)config;

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_get_status_led_config(TF_HATZero *hat_zero, uint8_t *ret_config) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_GET_STATUS_LED_CONFIG, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_config != NULL) { *ret_config = tf_packet_buffer_read_uint8_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_get_chip_temperature(TF_HATZero *hat_zero, int16_t *ret_temperature) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_GET_CHIP_TEMPERATURE, 0, 2, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_temperature != NULL) { *ret_temperature = tf_packet_buffer_read_int16_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 2); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_reset(TF_HATZero *hat_zero) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_hat_zero_get_response_expected(hat_zero, TF_HAT_ZERO_FUNCTION_RESET, &response_expected);
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_RESET, 0, 0, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_write_uid(TF_HATZero *hat_zero, uint32_t uid) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_hat_zero_get_response_expected(hat_zero, TF_HAT_ZERO_FUNCTION_WRITE_UID, &response_expected);
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_WRITE_UID, 4, 0, response_expected);

    uint8_t *send_buf = tf_tfp_get_send_payload_buffer(hat_zero->tfp);

    uid = tf_leconvert_uint32_to(uid); memcpy(send_buf + 0, &uid, 4);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_read_uid(TF_HATZero *hat_zero, uint32_t *ret_uid) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_READ_UID, 0, 4, response_expected);

    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_uid != NULL) { *ret_uid = tf_packet_buffer_read_uint32_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 4); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}

int tf_hat_zero_get_identity(TF_HATZero *hat_zero, char ret_uid[8], char ret_connected_uid[8], char *ret_position, uint8_t ret_hardware_version[3], uint8_t ret_firmware_version[3], uint16_t *ret_device_identifier) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    if (tf_hal_get_common(hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(hat_zero->tfp, TF_HAT_ZERO_FUNCTION_GET_IDENTITY, 0, 25, response_expected);

    size_t i;
    uint32_t deadline = tf_hal_current_time_us(hal) + tf_hal_get_common(hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_send_packet(hat_zero->tfp, response_expected, deadline, &error_code);

    if (result < 0) {
        return result;
    }

    if (result & TF_TICK_TIMEOUT) {
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        TF_PacketBuffer *recv_buf = tf_tfp_get_receive_buffer(hat_zero->tfp);
        if (ret_uid != NULL) { tf_packet_buffer_pop_n(recv_buf, (uint8_t *)ret_uid, 8);} else { tf_packet_buffer_remove(recv_buf, 8); }
        if (ret_connected_uid != NULL) { tf_packet_buffer_pop_n(recv_buf, (uint8_t *)ret_connected_uid, 8);} else { tf_packet_buffer_remove(recv_buf, 8); }
        if (ret_position != NULL) { *ret_position = tf_packet_buffer_read_char(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 1); }
        if (ret_hardware_version != NULL) { for (i = 0; i < 3; ++i) ret_hardware_version[i] = tf_packet_buffer_read_uint8_t(recv_buf);} else { tf_packet_buffer_remove(recv_buf, 3); }
        if (ret_firmware_version != NULL) { for (i = 0; i < 3; ++i) ret_firmware_version[i] = tf_packet_buffer_read_uint8_t(recv_buf);} else { tf_packet_buffer_remove(recv_buf, 3); }
        if (ret_device_identifier != NULL) { *ret_device_identifier = tf_packet_buffer_read_uint16_t(recv_buf); } else { tf_packet_buffer_remove(recv_buf, 2); }
        tf_tfp_packet_processed(hat_zero->tfp);
    }

    result = tf_tfp_finish_send(hat_zero->tfp, result, deadline);

    if (result < 0) {
        return result;
    }

    return tf_tfp_get_error(error_code);
}
#if TF_IMPLEMENT_CALLBACKS != 0
int tf_hat_zero_register_usb_voltage_callback(TF_HATZero *hat_zero, TF_HATZero_USBVoltageHandler handler, void *user_data) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    if (handler == NULL) {
        hat_zero->tfp->needs_callback_tick = false;
    } else {
        hat_zero->tfp->needs_callback_tick = true;
    }

    hat_zero->usb_voltage_handler = handler;
    hat_zero->usb_voltage_user_data = user_data;

    return TF_E_OK;
}
#endif
int tf_hat_zero_callback_tick(TF_HATZero *hat_zero, uint32_t timeout_us) {
    if (hat_zero == NULL) {
        return TF_E_NULL;
    }

    TF_HAL *hal = hat_zero->tfp->spitfp->hal;

    return tf_tfp_callback_tick(hat_zero->tfp, tf_hal_current_time_us(hal) + timeout_us);
}

#ifdef __cplusplus
}
#endif
