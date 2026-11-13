#include "chassis_control.h"
#include <stddef.h>
#include <string.h>

volatile chassis_diag_t chassis_diag;
typedef char diag_size_check[(sizeof(chassis_diag_t)==396u) ? 1 : -1];
#define DRIVE_ID_VALID(id) ((id)>=0x201u && (id)<=0x204u)
typedef char drive_id_range_check[(DRIVE_ID_VALID(WHEEL_RR_CAN_ID) &&
    DRIVE_ID_VALID(WHEEL_FR_CAN_ID) && DRIVE_ID_VALID(WHEEL_FL_CAN_ID) &&
    DRIVE_ID_VALID(WHEEL_LR_CAN_ID)) ? 1 : -1];
typedef char drive_id_unique_check[(WHEEL_RR_CAN_ID!=WHEEL_FR_CAN_ID &&
    WHEEL_RR_CAN_ID!=WHEEL_FL_CAN_ID && WHEEL_RR_CAN_ID!=WHEEL_LR_CAN_ID &&
    WHEEL_FR_CAN_ID!=WHEEL_FL_CAN_ID && WHEEL_FR_CAN_ID!=WHEEL_LR_CAN_ID &&
    WHEEL_FL_CAN_ID!=WHEEL_LR_CAN_ID) ? 1 : -1];
static const uint16_t ids[5]={WHEEL_RR_CAN_ID,WHEEL_FR_CAN_ID,WHEEL_FL_CAN_ID,
                              WHEEL_LR_CAN_ID,WHEEL_FEEDER_CAN_ID};
static struct {
    chassis_diag_t d;
    float integral[4];
    uint32_t last_step;
    bool have_step, have_rc;
} ctl;

static float absf(float value) { return value<0.0f ? -value : value; }
static float clamp(float value,float endpoint) {
    return value>endpoint ? endpoint : value< -endpoint ? -endpoint : value;
}
static int32_t signed16(const uint8_t *p) {
    uint32_t value=((uint32_t)p[0]<<8)|p[1];
    return value>=32768u ? (int32_t)value-65536 : (int32_t)value;
}
static void publish(void) {
    ++ctl.d.seq; chassis_diag.seq=ctl.d.seq;
    chassis_diag=ctl.d;
    ++ctl.d.seq; chassis_diag.seq=ctl.d.seq;
}
static void zero(void) {
    for(uint32_t i=0;i<5u;++i) {
        ctl.d.motors[i].command_current_raw=0;
        ctl.d.motors[i].target_millirpm=0;
    }
    for(uint32_t i=0;i<4u;++i) ctl.integral[i]=0.0f;
    ctl.d.desired_right_milli=ctl.d.desired_forward_milli=ctl.d.desired_cw_milli=0;
}

void chassis_init(void) {
    memset(&ctl,0,sizeof(ctl));
    ctl.d.magic=CHASSIS_DIAG_MAGIC; ctl.d.version=CHASSIS_DIAG_VERSION;
    ctl.d.size_bytes=sizeof(chassis_diag_t); ctl.d.state=CHASSIS_DISABLED;
    ctl.d.stop_reason=CHASSIS_STOP_STARTUP;
    ctl.d.max_rotor_rpm=CHASSIS_MAX_ROTOR_RPM;
    ctl.d.max_current_raw=CHASSIS_MAX_CURRENT_RAW;
    for(uint32_t i=0;i<5u;++i) ctl.d.motors[i].can_id=ids[i];
    publish();
}
void chassis_remote_invalid(uint32_t now_ms,uint32_t reason) {
    ctl.d.now_ms=now_ms; ++ctl.d.rc_bad_count; ctl.d.rc_bad_reason=reason;
    ctl.d.rc_consecutive=0u;
    publish();
}
void chassis_remote_update(const dr16_frame_t *frame) {
    if(!frame) { chassis_remote_invalid(ctl.d.now_ms,DR16_ERROR_ARGUMENT); return; }
    ctl.d.rc=*frame; ctl.have_rc=true; ctl.d.rc_valid=1u;
    ctl.d.now_ms=frame->last_frame_ms; ++ctl.d.rc_frame_count;
    if(ctl.d.rc_consecutive!=UINT32_MAX) ++ctl.d.rc_consecutive;
    if(frame->switch_left==DR16_SWITCH_UP) {
        if(ctl.d.state!=CHASSIS_ACTIVE) ++ctl.d.arm_count;
        ctl.d.state=CHASSIS_ACTIVE; ctl.d.stop_reason=CHASSIS_STOP_NONE;
    } else {
        if(ctl.d.state==CHASSIS_ACTIVE) ++ctl.d.disarm_count;
        ctl.d.state=CHASSIS_DISABLED; ctl.d.stop_reason=CHASSIS_STOP_SWITCH;
        zero();
    }
    publish();
}
void chassis_feedback(uint16_t id,const uint8_t data[8],uint32_t now_ms) {
    uint32_t i;
    for(i=0;i<5u && ids[i]!=id;++i) {}
    if(i==5u || !data) return;
    uint32_t angle=((uint32_t)data[0]<<8)|data[1];
    if(angle>=8192u) return;
    chassis_motor_diag_t *m=&ctl.d.motors[i];
    m->encoder_raw=angle; m->rotor_rpm=signed16(data+2);
    m->measured_current_raw=signed16(data+4); m->temperature_c=data[6];
    m->last_feedback_ms=now_ms;
    if(m->feedback_count!=UINT32_MAX) ++m->feedback_count;
    ctl.d.now_ms=now_ms;
    publish();
}
void chassis_mix(float right,float forward,float clockwise,float out[4]) {
    float yaw=clockwise*WHEEL_YAW_SCALE;
    out[0]=(forward+right-yaw)*WHEEL_RR_POLARITY;
    out[1]=(forward-right-yaw)*WHEEL_FR_POLARITY;
    out[2]=(forward+right+yaw)*WHEEL_FL_POLARITY;
    out[3]=(forward-right+yaw)*WHEEL_LR_POLARITY;
    float largest=1.0f;
    for(uint32_t i=0;i<4u;++i) if(absf(out[i])>largest) largest=absf(out[i]);
    for(uint32_t i=0;i<4u;++i)
        out[i]=out[i]/largest*(float)WHEEL_FULL_STICK_ROTOR_RPM;
}
void chassis_step(uint32_t now_ms,bool can_healthy,bool user_stop) {
    (void)can_healthy; (void)user_stop;
    uint32_t elapsed=ctl.have_step ? now_ms-ctl.last_step : 1u;
    ctl.last_step=now_ms; ctl.have_step=true; ctl.d.now_ms=now_ms;
    ctl.d.rc_age_ms=ctl.have_rc ? now_ms-ctl.d.rc.last_frame_ms : UINT32_MAX;
    ctl.d.online_wheels=0u;
    for(uint32_t i=0;i<5u;++i) {
        chassis_motor_diag_t *m=&ctl.d.motors[i];
        m->fresh=m->feedback_count!=0u &&
                 now_ms-m->last_feedback_ms<=WHEEL_DIAGNOSTIC_FRESH_MS;
        if(i<4u && m->fresh) ++ctl.d.online_wheels;
    }
    if(ctl.d.state!=CHASSIS_ACTIVE) { zero(); publish(); return; }
    ctl.d.desired_right_milli=ctl.d.rc.normalized[CHASSIS_RIGHT_CHANNEL]*CHASSIS_RIGHT_POLARITY;
    ctl.d.desired_forward_milli=ctl.d.rc.normalized[CHASSIS_FORWARD_CHANNEL]*CHASSIS_FORWARD_POLARITY;
    ctl.d.desired_cw_milli=ctl.d.rc.normalized[CHASSIS_CW_CHANNEL]*CHASSIS_CW_POLARITY;
    float target[4];
    chassis_mix((float)ctl.d.desired_right_milli/1000.0f,
                (float)ctl.d.desired_forward_milli/1000.0f,
                (float)ctl.d.desired_cw_milli/1000.0f,target);
    float dt=(float)elapsed*0.001f;
    for(uint32_t i=0;i<4u;++i) {
        chassis_motor_diag_t *m=&ctl.d.motors[i];
        float error=target[i]-(float)m->rotor_rpm;
        float proposed=clamp(ctl.integral[i]+WHEEL_SPEED_KI*error*dt,
                             (float)C620_PROTOCOL_CURRENT_MAX);
        float candidate=WHEEL_SPEED_KP*error+proposed;
        /* Conditional integration at the protocol endpoint is PI anti-windup,
         * not a fault, a stop condition or the old 1500-raw current limit. */
        if(absf(candidate)<=C620_PROTOCOL_CURRENT_MAX ||
           (candidate>C620_PROTOCOL_CURRENT_MAX && error<0.0f) ||
           (candidate< -C620_PROTOCOL_CURRENT_MAX && error>0.0f))
            ctl.integral[i]=proposed;
        m->target_millirpm=(int32_t)(target[i]*1000.0f);
        m->command_current_raw=(int32_t)clamp(WHEEL_SPEED_KP*error+ctl.integral[i],
                                             (float)C620_PROTOCOL_CURRENT_MAX);
    }
    publish();
}
bool chassis_output_enabled(void) { return ctl.d.state==CHASSIS_ACTIVE; }
void chassis_currents(int16_t out[8]) {
    if(!out) return;
    for(uint32_t i=0;i<8u;++i) out[i]=0;
    if(ctl.d.state!=CHASSIS_ACTIVE) return;
    for(uint32_t i=0;i<4u;++i)
        out[ids[i]-0x201u]=(int16_t)ctl.d.motors[i].command_current_raw;
    /* C620 slots 5..8, including the unused feeder slot 7, remain zero. */
}
