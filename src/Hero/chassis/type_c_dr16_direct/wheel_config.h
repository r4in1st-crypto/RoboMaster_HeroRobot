#ifndef TYPE_C_DIRECT_WHEEL_CONFIG_H
#define TYPE_C_DIRECT_WHEEL_CONFIG_H

/* Physical wheel placement, independent of CAN cable/connector order.
 * C620 drive slots must be a permutation of 0x201..0x204. Slot 0x207 is unused.
 * Polarity converts wheel-forward rotation to each outward-facing motor's
 * encoder/current direction. Geometry is the standard X mecanum arrangement. */
#define WHEEL_RR_CAN_ID 0x201u
#define WHEEL_FR_CAN_ID 0x202u
#define WHEEL_FL_CAN_ID 0x203u
#define WHEEL_LR_CAN_ID 0x204u
#define WHEEL_FEEDER_CAN_ID 0x207u
#define WHEEL_RR_POLARITY (-1.0f)
#define WHEEL_FR_POLARITY (-1.0f)
#define WHEEL_FL_POLARITY ( 1.0f)
#define WHEEL_LR_POLARITY ( 1.0f)

#define CHASSIS_RIGHT_CHANNEL 0u
#define CHASSIS_FORWARD_CHANNEL 1u
#define CHASSIS_CW_CHANNEL 2u
#define CHASSIS_RIGHT_POLARITY 1
#define CHASSIS_FORWARD_POLARITY 1
#define CHASSIS_CW_POLARITY 1
#define WHEEL_FULL_STICK_ROTOR_RPM 300
#define WHEEL_YAW_SCALE 0.65f
#define WHEEL_SPEED_KP 8.0f
#define WHEEL_SPEED_KI 20.0f

/* C620 protocol encoding endpoint, not the previous 1500-raw output limit.
 * PI conditional integration uses this same endpoint for anti-windup. */
#define C620_PROTOCOL_CURRENT_MAX 16384
/* Telemetry annotation only. No control path tests the age/fresh fields. */
#define WHEEL_DIAGNOSTIC_FRESH_MS 30u

#endif
