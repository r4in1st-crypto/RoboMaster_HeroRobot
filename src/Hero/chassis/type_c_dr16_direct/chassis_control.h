#ifndef TYPE_C_DIRECT_CHASSIS_CONTROL_H
#define TYPE_C_DIRECT_CHASSIS_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include "dr16.h"
#include "wheel_config.h"

#define CHASSIS_DISABLED 0u
#define CHASSIS_READY 1u /* Reserved diagnostic value, unused in direct mode. */
#define CHASSIS_ACTIVE 2u
#define CHASSIS_FAULT 3u /* Reserved diagnostic value, unused in direct mode. */
#define CHASSIS_DIAG_MAGIC UINT32_C(0x44524348)
#define CHASSIS_DIAG_VERSION 2u
#define CHASSIS_MAX_ROTOR_RPM WHEEL_FULL_STICK_ROTOR_RPM
#define CHASSIS_MAX_CURRENT_RAW C620_PROTOCOL_CURRENT_MAX

/* Preserve numeric names/layout for diagnostic readers. Direct mode never
 * creates a fault or enters READY/FAULT; only STARTUP/SWITCH/NONE are used. */
enum chassis_fault {
    CHASSIS_FAULT_NONE, CHASSIS_FAULT_CAN, CHASSIS_FAULT_STOP,
    CHASSIS_FAULT_FEEDBACK, CHASSIS_FAULT_TEMPERATURE, CHASSIS_FAULT_FRAME,
    CHASSIS_FAULT_OVERSPEED, CHASSIS_FAULT_TIMING, CHASSIS_FAULT_STALL
};
enum chassis_stop_reason {
    CHASSIS_STOP_STARTUP, CHASSIS_STOP_RC_TIMEOUT, CHASSIS_STOP_RC_INVALID,
    CHASSIS_STOP_SWITCH, CHASSIS_STOP_NEED_OFF_NEUTRAL,
    CHASSIS_STOP_MOTORS, CHASSIS_STOP_NONE,
    CHASSIS_STOP_WAIT_DRIVE_NEUTRAL, CHASSIS_STOP_WAIT_WHEELS_STILL,
    CHASSIS_STOP_WAIT_DOWN_DWELL, CHASSIS_STOP_READY_WAIT_UP,
    CHASSIS_STOP_NEED_DOWN_SEQUENCE
};

typedef struct {
    uint32_t can_id, feedback_count, last_feedback_ms, encoder_raw;
    int32_t rotor_rpm, measured_current_raw;
    uint32_t temperature_c;
    int32_t target_millirpm, command_current_raw;
    uint32_t fresh;
} chassis_motor_diag_t;

/* Same 396-byte layout as version 1, deliberately different semantics:
 * rc_valid means a legal RC sample exists (not a live link); fresh, rc_age_ms,
 * online_wheels and consecutive are informational only. Public writes do not
 * change the private controller. Main owns externally stable snapshots. */
typedef struct {
    uint32_t magic, version, size_bytes, seq;
    uint32_t state, fault_code, fault_can_id, now_ms;
    uint32_t rc_valid, rc_frame_count, rc_bad_count, rc_bad_reason;
    uint32_t rc_age_ms, online_wheels, arm_count, disarm_count;
    uint32_t ready_since_ms, rc_consecutive, stop_reason, max_rotor_rpm;
    uint32_t max_current_raw;
    int32_t desired_right_milli, desired_forward_milli, desired_cw_milli;
    chassis_motor_diag_t motors[5];
    dr16_frame_t rc;
} chassis_diag_t;
extern volatile chassis_diag_t chassis_diag;

void chassis_init(void);
/* Unknown IDs and malformed payloads (NULL/encoder>=8192) leave last data. */
void chassis_feedback(uint16_t id, const uint8_t data[8], uint32_t now_ms);
/* Call with a successfully decoded DR16 sample. UP immediately selects ACTIVE;
 * every other switch position immediately selects zero output. No dwell,
 * neutral, motor/RC frame count, age, temperature or speed qualification. */
void chassis_remote_update(const dr16_frame_t *frame);
/* Diagnostic count only: preserves last legal RC and current drive state. */
void chassis_remote_invalid(uint32_t now_ms, uint32_t reason);
/* can_healthy and user_stop are compatibility inputs and are ignored.
 * Missing/old RC and feedback continue to supply their last legal values.
 * PI has no acceleration ramp or former 1500-current cap. Its output is encoded
 * within C620 +/-16384; conditional integration prevents PI windup at that
 * protocol endpoint. This algorithm is not a fault/stop mechanism. */
void chassis_step(uint32_t now_ms, bool can_healthy, bool user_stop);
void chassis_currents(int16_t out[8]);
bool chassis_output_enabled(void);
/* Outputs ordered RR, FR, FL, LR; slot assignment follows wheel_config.h. */
void chassis_mix(float right, float forward, float clockwise, float out_rpm[4]);
#endif
