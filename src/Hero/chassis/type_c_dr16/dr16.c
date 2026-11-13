#include "dr16.h"

#include <string.h>

/* Primary references:
 * DJI DT7/DR16 manual V1.0 (2016-09), protocol table: 18 bytes, 100000 baud,
 * 8 data bits, even parity, one stop bit; nominal 14 ms frame interval.
 * https://rm-static.djicdn.com/tem/4.RoboMaster%20%E6%9C%BA%E5%99%A8%E4%BA%BA%E4%B8%93%E7%94%A8%E9%81%A5%E6%8E%A7%E5%99%A8%EF%BC%88%E6%8E%A5%E6%94%B6%E6%9C%BA%EF%BC%89%E7%94%A8%E6%88%B7%E6%89%8B%E5%86%8C.pdf
 * Official C-board example: wire channel order, physical left/right switch
 * names, and optional wheel at bytes16..17 (reserved in the older manual):
 * https://github.com/RoboMaster/Development-Board-C-Examples/blob/master/20.standard_robot/application/remote_control.c
 * Physical channel signs/configuration still require receiver readback.
 */

typedef char dr16_frame_size_check[(sizeof(dr16_frame_t) == 100u) ? 1 : -1];

static uint32_t little_u16(const uint8_t *p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8);
}

static int32_t little_i16(const uint8_t *p) {
    uint32_t value = little_u16(p);
    return value >= 32768u ? (int32_t)value - 65536 : (int32_t)value;
}

static int32_t normalized(int32_t centered) {
    int32_t magnitude = centered < 0 ? -centered : centered;
    int32_t value;
    if (magnitude <= (int32_t)DR16_DEADZONE_RAW) return 0;
    value = (magnitude - (int32_t)DR16_DEADZONE_RAW) * 1000 /
            (int32_t)(DR16_CHANNEL_MAX - DR16_CHANNEL_CENTER - DR16_DEADZONE_RAW);
    return centered < 0 ? -value : value;
}

dr16_result_t dr16_decode(const uint8_t *frame, size_t length,
                          uint32_t now_ms, dr16_frame_t *out) {
    dr16_frame_t decoded;
    unsigned i;
    if (frame == NULL || out == NULL) return DR16_ERROR_ARGUMENT;
    if (length != DR16_FRAME_BYTES) return DR16_ERROR_LENGTH;
    memset(&decoded, 0, sizeof(decoded));
    decoded.raw[0] = ((uint32_t)frame[0] | ((uint32_t)frame[1] << 8)) & 0x7ffu;
    decoded.raw[1] = (((uint32_t)frame[1] >> 3) | ((uint32_t)frame[2] << 5)) & 0x7ffu;
    decoded.raw[2] = (((uint32_t)frame[2] >> 6) | ((uint32_t)frame[3] << 2) |
                       ((uint32_t)frame[4] << 10)) & 0x7ffu;
    decoded.raw[3] = (((uint32_t)frame[4] >> 1) | ((uint32_t)frame[5] << 7)) & 0x7ffu;
    decoded.raw[4] = little_u16(frame + 16);
    decoded.switch_left = ((uint32_t)frame[5] >> 4) & 3u;
    decoded.switch_right = ((uint32_t)frame[5] >> 6) & 3u;
    if (decoded.switch_left == 0u || decoded.switch_right == 0u)
        return DR16_ERROR_SWITCH;
    for (i = 0; i < 4; ++i)
        if (decoded.raw[i] < DR16_CHANNEL_MIN || decoded.raw[i] > DR16_CHANNEL_MAX)
            return DR16_ERROR_CHANNEL;
    if (decoded.raw[4] != 0u &&
        (decoded.raw[4] < DR16_CHANNEL_MIN || decoded.raw[4] > DR16_CHANNEL_MAX))
        return DR16_ERROR_WHEEL;
    if (frame[12] > 1u || frame[13] > 1u)
        return DR16_ERROR_MOUSE_BUTTON;
    decoded.wheel_valid = decoded.raw[4] != 0u;
    for (i = 0; i < 5; ++i) {
        if (i == DR16_WHEEL && !decoded.wheel_valid) continue;
        decoded.centered[i] = (int32_t)decoded.raw[i] - (int32_t)DR16_CHANNEL_CENTER;
        decoded.normalized[i] = normalized(decoded.centered[i]);
    }
    decoded.mouse_x = little_i16(frame + 6);
    decoded.mouse_y = little_i16(frame + 8);
    decoded.mouse_z = little_i16(frame + 10);
    decoded.mouse_left = frame[12];
    decoded.mouse_right = frame[13];
    decoded.keyboard = little_u16(frame + 14);
    decoded.last_frame_ms = now_ms;
    *out = decoded;
    return DR16_OK;
}
