#ifndef TYPE_C_CHASSIS_CONTROL_H
#define TYPE_C_CHASSIS_CONTROL_H
#include <stdbool.h>
#include <stdint.h>
#include "dr16.h"

#define CHASSIS_DISABLED 0u
#define CHASSIS_READY 1u
#define CHASSIS_ACTIVE 2u
#define CHASSIS_FAULT 3u
#define CHASSIS_DIAG_MAGIC UINT32_C(0x44524348)
#define CHASSIS_DIAG_VERSION 1u
#define CHASSIS_RC_TIMEOUT_MS 50u
#define CHASSIS_MOTOR_TIMEOUT_MS 30u
#define CHASSIS_ARM_DWELL_MS 500u
#define CHASSIS_MAX_ROTOR_RPM 300
#define CHASSIS_MAX_CURRENT_RAW 1500

enum chassis_fault {
    CHASSIS_FAULT_NONE, CHASSIS_FAULT_CAN, CHASSIS_FAULT_STOP,
    CHASSIS_FAULT_FEEDBACK, CHASSIS_FAULT_TEMPERATURE,
    CHASSIS_FAULT_FRAME, /* Reserved ABI value; invalid payloads are discarded. */
    CHASSIS_FAULT_OVERSPEED,
    CHASSIS_FAULT_TIMING, CHASSIS_FAULT_STALL
};
enum chassis_stop_reason {
    CHASSIS_STOP_STARTUP, CHASSIS_STOP_RC_TIMEOUT, CHASSIS_STOP_RC_INVALID,
    CHASSIS_STOP_SWITCH, CHASSIS_STOP_NEED_OFF_NEUTRAL,
    CHASSIS_STOP_MOTORS, CHASSIS_STOP_NONE,
    /* Append-only diagnostic values: preserve the original 0..6 ABI. */
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

/* All fields are 32-bit, with no pointers. Private controller state cannot be
 * armed by writing this diagnostic object. Stable copies belong to main.c. */
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
/* Malformed payloads (NULL or encoder >=8192) are discarded without updating
 * accepted motor data/freshness. The valid-feedback timeout remains active. */
void chassis_feedback(uint16_t id, const uint8_t data[8], uint32_t now_ms);
void chassis_remote_update(const dr16_frame_t *frame);
void chassis_remote_invalid(uint32_t now_ms, uint32_t reason);
void chassis_step(uint32_t now_ms, bool can_healthy, bool user_stop);
void chassis_currents(int16_t out[8]);
bool chassis_output_enabled(void);
/* Communication-fault recovery (CAN or FEEDBACK; API names retained). Queries
 * only private controller state. BSP owns the separate OFF/link-clean dwell and
 * acknowledged-zero requirements when a platform CAN fault is present; main
 * may clear only after that recovery succeeds. A pure FEEDBACK fault is cleared
 * by chassis_step only while can_healthy and !user_stop. Clearing revalidates
 * safe DOWN/neutral/still/fresh conditions and returns to DISABLED, requiring a
 * new full DOWN dwell before an explicit UP can enable output. A later
 * noncommunication fault supersedes a communication fault and cannot clear. */
bool chassis_can_recovery_requested(uint32_t now_ms);
bool chassis_clear_can_fault(uint32_t now_ms);
/* ID order: 201 RR, 202 FR, 203 FL, 204 LR. Standard X roller arrangement,
 * outward-facing M3508 output shafts. Inputs: right, forward, clockwise.
 * This pure mixer is also tested independently from the arming/PID logic. */
void chassis_mix(float right, float forward, float clockwise, float out_rpm[4]);
#endif
