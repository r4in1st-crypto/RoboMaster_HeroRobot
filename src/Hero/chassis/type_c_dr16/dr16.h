#ifndef TYPE_C_DR16_H
#define TYPE_C_DR16_H

#include <stddef.h>
#include <stdint.h>

#define DR16_FRAME_BYTES 18u
#define DR16_CHANNEL_MIN 364u
#define DR16_CHANNEL_CENTER 1024u
#define DR16_CHANNEL_MAX 1684u
#define DR16_DEADZONE_RAW 20u
#define DR16_RIGHT_HORIZONTAL 0u
#define DR16_RIGHT_VERTICAL 1u
#define DR16_LEFT_HORIZONTAL 2u
#define DR16_LEFT_VERTICAL 3u
#define DR16_WHEEL 4u
#define DR16_SWITCH_UP 1u
#define DR16_SWITCH_DOWN 2u
#define DR16_SWITCH_MIDDLE 3u

typedef enum {
    DR16_OK = 0,
    DR16_ERROR_ARGUMENT = 1,
    DR16_ERROR_LENGTH = 2,
    DR16_ERROR_CHANNEL = 3,
    DR16_ERROR_SWITCH = 4,
    DR16_ERROR_WHEEL = 5,
    DR16_ERROR_MOUSE_BUTTON = 6
} dr16_result_t;

/* Fixed ABI: 25 32-bit words, 100 bytes; no pointers, no packed accesses.
 * Channels retain WIRE order, unlike the old HERO project's renamed fields:
 * [0]=right horizontal, [1]=right vertical, [2]=left horizontal,
 * [3]=left vertical, [4]=wheel (optional in older receivers).
 * normalized uses -1000..1000 after a +/-20 raw-count deadzone; centered
 * preserves raw minus 1024 without a deadzone. Physical direction signs must
 * be confirmed against the transmitter's actual channel configuration.
 * Official C-board example names byte5[5:4] left, byte5[7:6] right.
 * No arming, frame-continuity, connection, or actuator state is owned here.
 */
typedef struct {
    uint32_t raw[5];
    int32_t centered[5];
    int32_t normalized[5];
    uint32_t switch_left;
    uint32_t switch_right;
    int32_t mouse_x;
    int32_t mouse_y;
    int32_t mouse_z;
    uint32_t mouse_left;
    uint32_t mouse_right;
    uint32_t keyboard;
    uint32_t last_frame_ms;
    uint32_t wheel_valid;
} dr16_frame_t;

/* Decode one complete, externally framed UART packet (100000 baud, 8E1).
 * Return DR16_OK only for exactly 18 bytes and valid fields. On ANY error,
 * *out is unchanged: the caller MUST invalidate control immediately, not
 * continue applying the old out value. UART errors/overflow/framing and
 * frame cadence/freshness are the platform/controller's responsibility.
 * Legacy bytes16..17 == 0 are accepted as wheel_valid=0, wheel centered and
 * normalized=0. Any other wheel value must satisfy the normal channel range.
 * DBUS has no CRC; this parser cannot prove authenticity or frame alignment.
 */
dr16_result_t dr16_decode(const uint8_t *frame, size_t length,
                          uint32_t now_ms, dr16_frame_t *out);

#endif
