/* ***********************************************************
 * This file was automatically generated on 2021-11-12.      *
 *                                                           *
 * C/C++ for Microcontrollers Bindings Version 2.0.0         *
 *                                                           *
 * If you have a bugfix for this file and want to commit it, *
 * please fix the bug in the generator. You can find a link  *
 * to the generators git repository on tinkerforge.com       *
 *************************************************************/


#include "bricklet_rotary_encoder_v2.h"
#include "base58.h"
#include "endian_convert.h"
#include "errors.h"

#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif


#if TF_IMPLEMENT_CALLBACKS != 0
static bool tf_rotary_encoder_v2_callback_handler(void *dev, uint8_t fid, TF_Packetbuffer *payload) {
    TF_RotaryEncoderV2 *rotary_encoder_v2 = (TF_RotaryEncoderV2 *) dev;
    (void)payload;

    switch(fid) {

        case TF_ROTARY_ENCODER_V2_CALLBACK_COUNT: {
            TF_RotaryEncoderV2CountHandler fn = rotary_encoder_v2->count_handler;
            void *user_data = rotary_encoder_v2->count_user_data;
            if (fn == NULL)
                return false;

            int32_t count = tf_packetbuffer_read_int32_t(payload);
            TF_HalCommon *common = tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal);
            common->locked = true;
            fn(rotary_encoder_v2, count, user_data);
            common->locked = false;
            break;
        }

        case TF_ROTARY_ENCODER_V2_CALLBACK_PRESSED: {
            TF_RotaryEncoderV2PressedHandler fn = rotary_encoder_v2->pressed_handler;
            void *user_data = rotary_encoder_v2->pressed_user_data;
            if (fn == NULL)
                return false;


            TF_HalCommon *common = tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal);
            common->locked = true;
            fn(rotary_encoder_v2, user_data);
            common->locked = false;
            break;
        }

        case TF_ROTARY_ENCODER_V2_CALLBACK_RELEASED: {
            TF_RotaryEncoderV2ReleasedHandler fn = rotary_encoder_v2->released_handler;
            void *user_data = rotary_encoder_v2->released_user_data;
            if (fn == NULL)
                return false;


            TF_HalCommon *common = tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal);
            common->locked = true;
            fn(rotary_encoder_v2, user_data);
            common->locked = false;
            break;
        }
        default:
            return false;
    }

    return true;
}
#else
static bool tf_rotary_encoder_v2_callback_handler(void *dev, uint8_t fid, TF_Packetbuffer *payload) {
    return false;
}
#endif
int tf_rotary_encoder_v2_create(TF_RotaryEncoderV2 *rotary_encoder_v2, const char *uid, TF_HalContext *hal) {
    if (rotary_encoder_v2 == NULL || uid == NULL || hal == NULL)
        return TF_E_NULL;

    memset(rotary_encoder_v2, 0, sizeof(TF_RotaryEncoderV2));

    uint32_t numeric_uid;
    int rc = tf_base58_decode(uid, &numeric_uid);
    if (rc != TF_E_OK) {
        return rc;
    }

    uint8_t port_id;
    uint8_t inventory_index;
    rc = tf_hal_get_port_id(hal, numeric_uid, &port_id, &inventory_index);
    if (rc < 0) {
        return rc;
    }

    rc = tf_hal_get_tfp(hal, &rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_DEVICE_IDENTIFIER, inventory_index);
    if (rc != TF_E_OK) {
        return rc;
    }
    rotary_encoder_v2->tfp->device = rotary_encoder_v2;
    rotary_encoder_v2->tfp->uid = numeric_uid;
    rotary_encoder_v2->tfp->cb_handler = tf_rotary_encoder_v2_callback_handler;
    rotary_encoder_v2->response_expected[0] = 0x01;
    return TF_E_OK;
}

int tf_rotary_encoder_v2_destroy(TF_RotaryEncoderV2 *rotary_encoder_v2) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    int result = tf_tfp_destroy(rotary_encoder_v2->tfp);
    rotary_encoder_v2->tfp = NULL;
    return result;
}

int tf_rotary_encoder_v2_get_response_expected(TF_RotaryEncoderV2 *rotary_encoder_v2, uint8_t function_id, bool *ret_response_expected) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    switch(function_id) {
        case TF_ROTARY_ENCODER_V2_FUNCTION_SET_COUNT_CALLBACK_CONFIGURATION:
            if(ret_response_expected != NULL)
                *ret_response_expected = (rotary_encoder_v2->response_expected[0] & (1 << 0)) != 0;
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_SET_WRITE_FIRMWARE_POINTER:
            if(ret_response_expected != NULL)
                *ret_response_expected = (rotary_encoder_v2->response_expected[0] & (1 << 1)) != 0;
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_SET_STATUS_LED_CONFIG:
            if(ret_response_expected != NULL)
                *ret_response_expected = (rotary_encoder_v2->response_expected[0] & (1 << 2)) != 0;
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_RESET:
            if(ret_response_expected != NULL)
                *ret_response_expected = (rotary_encoder_v2->response_expected[0] & (1 << 3)) != 0;
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_WRITE_UID:
            if(ret_response_expected != NULL)
                *ret_response_expected = (rotary_encoder_v2->response_expected[0] & (1 << 4)) != 0;
            break;
        default:
            return TF_E_INVALID_PARAMETER;
    }
    return TF_E_OK;
}

int tf_rotary_encoder_v2_set_response_expected(TF_RotaryEncoderV2 *rotary_encoder_v2, uint8_t function_id, bool response_expected) {
    switch(function_id) {
        case TF_ROTARY_ENCODER_V2_FUNCTION_SET_COUNT_CALLBACK_CONFIGURATION:
            if (response_expected) {
                rotary_encoder_v2->response_expected[0] |= (1 << 0);
            } else {
                rotary_encoder_v2->response_expected[0] &= ~(1 << 0);
            }
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_SET_WRITE_FIRMWARE_POINTER:
            if (response_expected) {
                rotary_encoder_v2->response_expected[0] |= (1 << 1);
            } else {
                rotary_encoder_v2->response_expected[0] &= ~(1 << 1);
            }
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_SET_STATUS_LED_CONFIG:
            if (response_expected) {
                rotary_encoder_v2->response_expected[0] |= (1 << 2);
            } else {
                rotary_encoder_v2->response_expected[0] &= ~(1 << 2);
            }
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_RESET:
            if (response_expected) {
                rotary_encoder_v2->response_expected[0] |= (1 << 3);
            } else {
                rotary_encoder_v2->response_expected[0] &= ~(1 << 3);
            }
            break;
        case TF_ROTARY_ENCODER_V2_FUNCTION_WRITE_UID:
            if (response_expected) {
                rotary_encoder_v2->response_expected[0] |= (1 << 4);
            } else {
                rotary_encoder_v2->response_expected[0] &= ~(1 << 4);
            }
            break;
        default:
            return TF_E_INVALID_PARAMETER;
    }
    return TF_E_OK;
}

void tf_rotary_encoder_v2_set_response_expected_all(TF_RotaryEncoderV2 *rotary_encoder_v2, bool response_expected) {
    memset(rotary_encoder_v2->response_expected, response_expected ? 0xFF : 0, 1);
}

int tf_rotary_encoder_v2_get_count(TF_RotaryEncoderV2 *rotary_encoder_v2, bool reset, int32_t *ret_count) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_GET_COUNT, 1, 4, response_expected);

    uint8_t *buf = tf_tfp_get_payload_buffer(rotary_encoder_v2->tfp);

    buf[0] = reset ? 1 : 0;

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_count != NULL) { *ret_count = tf_packetbuffer_read_int32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_set_count_callback_configuration(TF_RotaryEncoderV2 *rotary_encoder_v2, uint32_t period, bool value_has_to_change, char option, int32_t min, int32_t max) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_rotary_encoder_v2_get_response_expected(rotary_encoder_v2, TF_ROTARY_ENCODER_V2_FUNCTION_SET_COUNT_CALLBACK_CONFIGURATION, &response_expected);
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_SET_COUNT_CALLBACK_CONFIGURATION, 14, 0, response_expected);

    uint8_t *buf = tf_tfp_get_payload_buffer(rotary_encoder_v2->tfp);

    period = tf_leconvert_uint32_to(period); memcpy(buf + 0, &period, 4);
    buf[4] = value_has_to_change ? 1 : 0;
    buf[5] = (uint8_t)option;
    min = tf_leconvert_int32_to(min); memcpy(buf + 6, &min, 4);
    max = tf_leconvert_int32_to(max); memcpy(buf + 10, &max, 4);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_get_count_callback_configuration(TF_RotaryEncoderV2 *rotary_encoder_v2, uint32_t *ret_period, bool *ret_value_has_to_change, char *ret_option, int32_t *ret_min, int32_t *ret_max) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_GET_COUNT_CALLBACK_CONFIGURATION, 0, 14, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_period != NULL) { *ret_period = tf_packetbuffer_read_uint32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        if (ret_value_has_to_change != NULL) { *ret_value_has_to_change = tf_packetbuffer_read_bool(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        if (ret_option != NULL) { *ret_option = tf_packetbuffer_read_char(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        if (ret_min != NULL) { *ret_min = tf_packetbuffer_read_int32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        if (ret_max != NULL) { *ret_max = tf_packetbuffer_read_int32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_is_pressed(TF_RotaryEncoderV2 *rotary_encoder_v2, bool *ret_pressed) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_IS_PRESSED, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_pressed != NULL) { *ret_pressed = tf_packetbuffer_read_bool(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_get_spitfp_error_count(TF_RotaryEncoderV2 *rotary_encoder_v2, uint32_t *ret_error_count_ack_checksum, uint32_t *ret_error_count_message_checksum, uint32_t *ret_error_count_frame, uint32_t *ret_error_count_overflow) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_GET_SPITFP_ERROR_COUNT, 0, 16, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_error_count_ack_checksum != NULL) { *ret_error_count_ack_checksum = tf_packetbuffer_read_uint32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        if (ret_error_count_message_checksum != NULL) { *ret_error_count_message_checksum = tf_packetbuffer_read_uint32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        if (ret_error_count_frame != NULL) { *ret_error_count_frame = tf_packetbuffer_read_uint32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        if (ret_error_count_overflow != NULL) { *ret_error_count_overflow = tf_packetbuffer_read_uint32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_set_bootloader_mode(TF_RotaryEncoderV2 *rotary_encoder_v2, uint8_t mode, uint8_t *ret_status) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_SET_BOOTLOADER_MODE, 1, 1, response_expected);

    uint8_t *buf = tf_tfp_get_payload_buffer(rotary_encoder_v2->tfp);

    buf[0] = (uint8_t)mode;

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_status != NULL) { *ret_status = tf_packetbuffer_read_uint8_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_get_bootloader_mode(TF_RotaryEncoderV2 *rotary_encoder_v2, uint8_t *ret_mode) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_GET_BOOTLOADER_MODE, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_mode != NULL) { *ret_mode = tf_packetbuffer_read_uint8_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_set_write_firmware_pointer(TF_RotaryEncoderV2 *rotary_encoder_v2, uint32_t pointer) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_rotary_encoder_v2_get_response_expected(rotary_encoder_v2, TF_ROTARY_ENCODER_V2_FUNCTION_SET_WRITE_FIRMWARE_POINTER, &response_expected);
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_SET_WRITE_FIRMWARE_POINTER, 4, 0, response_expected);

    uint8_t *buf = tf_tfp_get_payload_buffer(rotary_encoder_v2->tfp);

    pointer = tf_leconvert_uint32_to(pointer); memcpy(buf + 0, &pointer, 4);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_write_firmware(TF_RotaryEncoderV2 *rotary_encoder_v2, const uint8_t data[64], uint8_t *ret_status) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_WRITE_FIRMWARE, 64, 1, response_expected);

    uint8_t *buf = tf_tfp_get_payload_buffer(rotary_encoder_v2->tfp);

    memcpy(buf + 0, data, 64);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_status != NULL) { *ret_status = tf_packetbuffer_read_uint8_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_set_status_led_config(TF_RotaryEncoderV2 *rotary_encoder_v2, uint8_t config) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_rotary_encoder_v2_get_response_expected(rotary_encoder_v2, TF_ROTARY_ENCODER_V2_FUNCTION_SET_STATUS_LED_CONFIG, &response_expected);
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_SET_STATUS_LED_CONFIG, 1, 0, response_expected);

    uint8_t *buf = tf_tfp_get_payload_buffer(rotary_encoder_v2->tfp);

    buf[0] = (uint8_t)config;

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_get_status_led_config(TF_RotaryEncoderV2 *rotary_encoder_v2, uint8_t *ret_config) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_GET_STATUS_LED_CONFIG, 0, 1, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_config != NULL) { *ret_config = tf_packetbuffer_read_uint8_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_get_chip_temperature(TF_RotaryEncoderV2 *rotary_encoder_v2, int16_t *ret_temperature) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_GET_CHIP_TEMPERATURE, 0, 2, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_temperature != NULL) { *ret_temperature = tf_packetbuffer_read_int16_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 2); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_reset(TF_RotaryEncoderV2 *rotary_encoder_v2) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_rotary_encoder_v2_get_response_expected(rotary_encoder_v2, TF_ROTARY_ENCODER_V2_FUNCTION_RESET, &response_expected);
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_RESET, 0, 0, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_write_uid(TF_RotaryEncoderV2 *rotary_encoder_v2, uint32_t uid) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_rotary_encoder_v2_get_response_expected(rotary_encoder_v2, TF_ROTARY_ENCODER_V2_FUNCTION_WRITE_UID, &response_expected);
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_WRITE_UID, 4, 0, response_expected);

    uint8_t *buf = tf_tfp_get_payload_buffer(rotary_encoder_v2->tfp);

    uid = tf_leconvert_uint32_to(uid); memcpy(buf + 0, &uid, 4);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_read_uid(TF_RotaryEncoderV2 *rotary_encoder_v2, uint32_t *ret_uid) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_READ_UID, 0, 4, response_expected);

    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        if (ret_uid != NULL) { *ret_uid = tf_packetbuffer_read_uint32_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 4); }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}

int tf_rotary_encoder_v2_get_identity(TF_RotaryEncoderV2 *rotary_encoder_v2, char ret_uid[8], char ret_connected_uid[8], char *ret_position, uint8_t ret_hardware_version[3], uint8_t ret_firmware_version[3], uint16_t *ret_device_identifier) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if(tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->locked) {
        return TF_E_LOCKED;
    }

    bool response_expected = true;
    tf_tfp_prepare_send(rotary_encoder_v2->tfp, TF_ROTARY_ENCODER_V2_FUNCTION_GET_IDENTITY, 0, 25, response_expected);

    size_t i;
    uint32_t deadline = tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + tf_hal_get_common((TF_HalContext*)rotary_encoder_v2->tfp->hal)->timeout;

    uint8_t error_code = 0;
    int result = tf_tfp_transmit_packet(rotary_encoder_v2->tfp, response_expected, deadline, &error_code);
    if(result < 0)
        return result;

    if (result & TF_TICK_TIMEOUT) {
        //return -result;
        return TF_E_TIMEOUT;
    }

    if (result & TF_TICK_PACKET_RECEIVED && error_code == 0) {
        char tmp_connected_uid[8] = {0};
        if (ret_uid != NULL) { tf_packetbuffer_pop_n(&rotary_encoder_v2->tfp->spitfp->recv_buf, (uint8_t*)ret_uid, 8);} else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 8); }
        tf_packetbuffer_pop_n(&rotary_encoder_v2->tfp->spitfp->recv_buf, (uint8_t*)tmp_connected_uid, 8);
        if (ret_position != NULL) { *ret_position = tf_packetbuffer_read_char(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 1); }
        if (ret_hardware_version != NULL) { for (i = 0; i < 3; ++i) ret_hardware_version[i] = tf_packetbuffer_read_uint8_t(&rotary_encoder_v2->tfp->spitfp->recv_buf);} else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 3); }
        if (ret_firmware_version != NULL) { for (i = 0; i < 3; ++i) ret_firmware_version[i] = tf_packetbuffer_read_uint8_t(&rotary_encoder_v2->tfp->spitfp->recv_buf);} else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 3); }
        if (ret_device_identifier != NULL) { *ret_device_identifier = tf_packetbuffer_read_uint16_t(&rotary_encoder_v2->tfp->spitfp->recv_buf); } else { tf_packetbuffer_remove(&rotary_encoder_v2->tfp->spitfp->recv_buf, 2); }
        if (tmp_connected_uid[0] == 0 && ret_position != NULL) {
            *ret_position = tf_hal_get_port_name((TF_HalContext*)rotary_encoder_v2->tfp->hal, rotary_encoder_v2->tfp->spitfp->port_id);
        }
        if (ret_connected_uid != NULL) {
            memcpy(ret_connected_uid, tmp_connected_uid, 8);
        }
        tf_tfp_packet_processed(rotary_encoder_v2->tfp);
    }

    result = tf_tfp_finish_send(rotary_encoder_v2->tfp, result, deadline);
    if(result < 0)
        return result;

    return tf_tfp_get_error(error_code);
}
#if TF_IMPLEMENT_CALLBACKS != 0
int tf_rotary_encoder_v2_register_count_callback(TF_RotaryEncoderV2 *rotary_encoder_v2, TF_RotaryEncoderV2CountHandler handler, void *user_data) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if (handler == NULL) {
        rotary_encoder_v2->tfp->needs_callback_tick = false;
        rotary_encoder_v2->tfp->needs_callback_tick |= rotary_encoder_v2->pressed_handler != NULL;
        rotary_encoder_v2->tfp->needs_callback_tick |= rotary_encoder_v2->released_handler != NULL;
    } else {
        rotary_encoder_v2->tfp->needs_callback_tick = true;
    }
    rotary_encoder_v2->count_handler = handler;
    rotary_encoder_v2->count_user_data = user_data;
    return TF_E_OK;
}


int tf_rotary_encoder_v2_register_pressed_callback(TF_RotaryEncoderV2 *rotary_encoder_v2, TF_RotaryEncoderV2PressedHandler handler, void *user_data) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if (handler == NULL) {
        rotary_encoder_v2->tfp->needs_callback_tick = false;
        rotary_encoder_v2->tfp->needs_callback_tick |= rotary_encoder_v2->count_handler != NULL;
        rotary_encoder_v2->tfp->needs_callback_tick |= rotary_encoder_v2->released_handler != NULL;
    } else {
        rotary_encoder_v2->tfp->needs_callback_tick = true;
    }
    rotary_encoder_v2->pressed_handler = handler;
    rotary_encoder_v2->pressed_user_data = user_data;
    return TF_E_OK;
}


int tf_rotary_encoder_v2_register_released_callback(TF_RotaryEncoderV2 *rotary_encoder_v2, TF_RotaryEncoderV2ReleasedHandler handler, void *user_data) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    if (handler == NULL) {
        rotary_encoder_v2->tfp->needs_callback_tick = false;
        rotary_encoder_v2->tfp->needs_callback_tick |= rotary_encoder_v2->count_handler != NULL;
        rotary_encoder_v2->tfp->needs_callback_tick |= rotary_encoder_v2->pressed_handler != NULL;
    } else {
        rotary_encoder_v2->tfp->needs_callback_tick = true;
    }
    rotary_encoder_v2->released_handler = handler;
    rotary_encoder_v2->released_user_data = user_data;
    return TF_E_OK;
}
#endif
int tf_rotary_encoder_v2_callback_tick(TF_RotaryEncoderV2 *rotary_encoder_v2, uint32_t timeout_us) {
    if (rotary_encoder_v2 == NULL)
        return TF_E_NULL;

    return tf_tfp_callback_tick(rotary_encoder_v2->tfp, tf_hal_current_time_us((TF_HalContext*)rotary_encoder_v2->tfp->hal) + timeout_us);
}

#ifdef __cplusplus
}
#endif