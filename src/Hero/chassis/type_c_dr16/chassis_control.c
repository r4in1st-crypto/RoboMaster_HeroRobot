#include "chassis_control.h"
#include <stddef.h>
#include <string.h>

#define MIN_RC_FRAMES 5u
#define MIN_MOTOR_FRAMES 8u
#define RPM_ACCEL_PER_S 300.0f
#define SPEED_KP 8.0f
#define SPEED_KI 20.0f
#define INTEGRAL_LIMIT 400.0f
#define MAX_STEP_MS 10u

volatile chassis_diag_t chassis_diag;
typedef char diag_size_check[(sizeof(chassis_diag_t) == 396u) ? 1 : -1];
static struct {
    chassis_diag_t d;
    float target[4], integral[4];
    uint32_t stall_since[4], still_since[4];
    bool stall_timing[4], still_timing[4];
    uint32_t last_step, last_safe_off_ms;
    bool have_step, off_timing, have_rc;
} ctl;
static const uint16_t ids[5] = {0x201u,0x202u,0x203u,0x204u,0x207u};

static float absf(float v) { return v < 0.0f ? -v : v; }
static float clamp(float v,float lim) { return v > lim ? lim : v < -lim ? -lim : v; }
static int32_t signed16(const uint8_t *p) {
    uint32_t v=((uint32_t)p[0]<<8)|p[1];
    return v >= 32768u ? (int32_t)v-65536 : (int32_t)v;
}
static void publish(void) {
    ++ctl.d.seq;
    chassis_diag.seq=ctl.d.seq;
    chassis_diag=ctl.d;
    ++ctl.d.seq;
    chassis_diag.seq=ctl.d.seq;
}
static void zero(void) {
    for (uint32_t i=0;i<5u;++i) {
        ctl.d.motors[i].command_current_raw=0;
        ctl.d.motors[i].target_millirpm=0;
    }
    for (uint32_t i=0;i<4u;++i) {
        ctl.target[i]=0.0f; ctl.integral[i]=0.0f;
        ctl.stall_timing[i]=false; ctl.still_timing[i]=false;
    }
    ctl.d.desired_right_milli=ctl.d.desired_forward_milli=ctl.d.desired_cw_milli=0;
}
static void disarm(uint32_t reason) {
    if (ctl.d.state==CHASSIS_ACTIVE) ++ctl.d.disarm_count;
    if (ctl.d.state!=CHASSIS_FAULT) ctl.d.state=CHASSIS_DISABLED;
    ctl.off_timing=false; ctl.d.ready_since_ms=0;
    ctl.d.stop_reason=reason;
    zero();
}
static void fault(uint32_t code,uint32_t id) {
    /* A later noncommunication fault must not be hidden by a recoverable first
     * communication fault: cooling down or restoring the link cannot clear it. */
    if (ctl.d.state!=CHASSIS_FAULT ||
        ((ctl.d.fault_code==CHASSIS_FAULT_CAN ||
          ctl.d.fault_code==CHASSIS_FAULT_FEEDBACK) &&
         code!=CHASSIS_FAULT_CAN && code!=CHASSIS_FAULT_FEEDBACK)) {
        ctl.d.fault_code=code; ctl.d.fault_can_id=id;
    }
    disarm(CHASSIS_STOP_NONE); ctl.d.state=CHASSIS_FAULT;
}
static bool neutral(void) {
    /* The unused left vertical channel remains validated by the parser, but
     * cannot command this chassis and therefore must not inhibit arming. */
    return ctl.d.rc.normalized[DR16_RIGHT_HORIZONTAL]==0 &&
           ctl.d.rc.normalized[DR16_RIGHT_VERTICAL]==0 &&
           ctl.d.rc.normalized[DR16_LEFT_HORIZONTAL]==0;
}

bool chassis_can_recovery_requested(uint32_t now_ms) {
    /* Preserve the API name for callers, but both first-fault communication
     * codes must be eligible when a later platform CAN fault also needs its
     * separate clean-link recovery handshake. */
    if(ctl.d.state!=CHASSIS_FAULT ||
       (ctl.d.fault_code!=CHASSIS_FAULT_CAN &&
        ctl.d.fault_code!=CHASSIS_FAULT_FEEDBACK) ||
       !ctl.have_rc || !ctl.d.rc_valid || ctl.d.rc_consecutive<MIN_RC_FRAMES ||
       now_ms-ctl.d.rc.last_frame_ms>=CHASSIS_RC_TIMEOUT_MS ||
       ctl.d.rc.switch_left!=DR16_SWITCH_DOWN || !neutral()) return false;
    for(uint32_t i=0;i<4u;++i) {
        const chassis_motor_diag_t *m=&ctl.d.motors[i];
        if(m->feedback_count<MIN_MOTOR_FRAMES ||
           now_ms-m->last_feedback_ms>CHASSIS_MOTOR_TIMEOUT_MS ||
           absf((float)m->rotor_rpm)>5.0f || m->temperature_c>=70u) return false;
    }
    return true;
}

bool chassis_clear_can_fault(uint32_t now_ms) {
    if(!chassis_can_recovery_requested(now_ms)) return false;
    zero();
    ctl.d.state=CHASSIS_DISABLED;
    ctl.d.fault_code=CHASSIS_FAULT_NONE;
    ctl.d.fault_can_id=0u;
    ctl.d.now_ms=now_ms;
    ctl.d.stop_reason=CHASSIS_STOP_WAIT_DOWN_DWELL;
    ctl.off_timing=false;
    ctl.d.ready_since_ms=0u;
    ctl.last_safe_off_ms=0u;
    publish();
    return true;
}

void chassis_init(void) {
    memset(&ctl,0,sizeof(ctl));
    ctl.d.magic=CHASSIS_DIAG_MAGIC; ctl.d.version=CHASSIS_DIAG_VERSION;
    ctl.d.size_bytes=sizeof(chassis_diag_t); ctl.d.state=CHASSIS_DISABLED;
    ctl.d.max_rotor_rpm=CHASSIS_MAX_ROTOR_RPM;
    ctl.d.max_current_raw=CHASSIS_MAX_CURRENT_RAW;
    for (uint32_t i=0;i<5u;++i) ctl.d.motors[i].can_id=ids[i];
    publish();
}
void chassis_remote_invalid(uint32_t now_ms,uint32_t reason) {
    ctl.d.now_ms=now_ms; ctl.d.rc_valid=0; ctl.d.rc_consecutive=0;
    ++ctl.d.rc_bad_count; ctl.d.rc_bad_reason=reason;
    disarm(CHASSIS_STOP_RC_INVALID); publish();
}
void chassis_remote_update(const dr16_frame_t *frame) {
    if (!frame) { chassis_remote_invalid(ctl.d.now_ms,DR16_ERROR_ARGUMENT); return; }
    /* Preserve an interrupted stream as disarmed even if a fresh packet arrives
     * before the next control tick; old commands are never resumed on reconnect. */
    if (ctl.have_rc) {
        uint32_t gap=frame->last_frame_ms-ctl.d.rc.last_frame_ms;
        if (gap<5u || gap>40u) {
            disarm(CHASSIS_STOP_RC_INVALID); ctl.d.rc_consecutive=0;
            if (gap<5u) {
                chassis_remote_invalid(frame->last_frame_ms,0x100u); return;
            }
        }
    }
    ctl.d.rc=*frame; ctl.have_rc=true; ctl.d.rc_valid=1;
    ctl.d.now_ms=frame->last_frame_ms;
    ++ctl.d.rc_frame_count;
    if (ctl.d.rc_consecutive<1000u) ++ctl.d.rc_consecutive;
    if (frame->switch_left!=DR16_SWITCH_UP && ctl.d.state==CHASSIS_ACTIVE)
        disarm(CHASSIS_STOP_SWITCH);
    publish();
}
void chassis_feedback(uint16_t id,const uint8_t data[8],uint32_t now_ms) {
    uint32_t i;
    for(i=0;i<5u && ids[i]!=id;++i) {}
    if(i==5u) return;
    /* A malformed payload is not a new measurement. Keep the last accepted
     * fields and timestamp so sustained corruption still reaches the normal
     * valid-feedback timeout; an isolated bad frame must not latch a fault. */
    if(!data) return;
    uint32_t angle=((uint32_t)data[0]<<8)|data[1];
    if(angle>=8192u) return;
    ctl.d.now_ms=now_ms;
    chassis_motor_diag_t *m=&ctl.d.motors[i];
    if(i<4u && m->feedback_count && ctl.d.state==CHASSIS_ACTIVE &&
       now_ms-m->last_feedback_ms>CHASSIS_MOTOR_TIMEOUT_MS)
        fault(CHASSIS_FAULT_FEEDBACK,id);
    m->encoder_raw=angle; m->rotor_rpm=signed16(data+2);
    m->measured_current_raw=signed16(data+4); m->temperature_c=data[6];
    m->last_feedback_ms=now_ms;
    if(m->feedback_count!=UINT32_MAX) ++m->feedback_count;
    if(i<4u && m->temperature_c>=70u) fault(CHASSIS_FAULT_TEMPERATURE,id);
    if(i<4u && ctl.d.state==CHASSIS_ACTIVE && absf((float)m->rotor_rpm)>900.0f)
        fault(CHASSIS_FAULT_OVERSPEED,id);
    publish();
}

void chassis_mix(float right,float forward,float clockwise,float out[4]) {
    /* Body-relative standard X mecanum mixer. No gimbal/yaw sensor is needed.
     * User-confirmed mapping: RR201 FR202 FL203 LR204. Positive C620 current
     * is default shaft CCW; left and right outward shafts face opposite ways. */
    float yaw=clockwise*0.65f;
    out[0]=-forward-right+yaw; out[1]=-forward+right+yaw;
    out[2]= forward+right+yaw; out[3]= forward-right+yaw;
    float largest=1.0f;
    for(uint32_t i=0;i<4u;++i) if(absf(out[i])>largest) largest=absf(out[i]);
    for(uint32_t i=0;i<4u;++i) out[i]=out[i]/largest*(float)CHASSIS_MAX_ROTOR_RPM;
}

void chassis_step(uint32_t now_ms,bool can_healthy,bool user_stop) {
    uint32_t elapsed=ctl.have_step ? now_ms-ctl.last_step : 1u;
    ctl.last_step=now_ms; ctl.have_step=true; ctl.d.now_ms=now_ms;
    if(!can_healthy) fault(CHASSIS_FAULT_CAN,0);
    if(user_stop) fault(CHASSIS_FAULT_STOP,0);
    if(elapsed>MAX_STEP_MS && ctl.d.state==CHASSIS_ACTIVE)
        fault(CHASSIS_FAULT_TIMING,0);
    bool still=true;
    ctl.d.online_wheels=0;
    for(uint32_t i=0;i<5u;++i) {
        chassis_motor_diag_t *m=&ctl.d.motors[i];
        m->fresh=m->feedback_count>=MIN_MOTOR_FRAMES &&
                 now_ms-m->last_feedback_ms<=CHASSIS_MOTOR_TIMEOUT_MS;
        if(i<4u) {
            if(m->fresh) ++ctl.d.online_wheels;
            if(!m->fresh || absf((float)m->rotor_rpm)>5.0f) still=false;
            if(!m->fresh && ctl.d.state==CHASSIS_ACTIVE)
                fault(CHASSIS_FAULT_FEEDBACK,m->can_id);
        }
    }
    ctl.d.rc_age_ms=ctl.have_rc ? now_ms-ctl.d.rc.last_frame_ms : UINT32_MAX;
    /* A restored feedback stream may release its stop only with a healthy
     * platform and an explicit safe DOWN position. Continue through the normal
     * DISABLED path so this tick begins a new full 500 ms DOWN dwell. CAN faults
     * still require the BSP's separate acknowledged-zero/link-clean handshake. */
    if(ctl.d.state==CHASSIS_FAULT && ctl.d.fault_code==CHASSIS_FAULT_FEEDBACK &&
       can_healthy && !user_stop)
        (void)chassis_clear_can_fault(now_ms);
    if(ctl.d.state==CHASSIS_FAULT) { zero(); publish(); return; }
    if(!ctl.d.rc_valid || ctl.d.rc_age_ms>=CHASSIS_RC_TIMEOUT_MS ||
       ctl.d.rc_consecutive<MIN_RC_FRAMES) {
        if(ctl.d.rc_age_ms>=CHASSIS_RC_TIMEOUT_MS) {
            ctl.d.rc_valid=0; ctl.d.rc_consecutive=0;
        }
        disarm(CHASSIS_STOP_RC_TIMEOUT); publish(); return;
    }
    if(ctl.d.online_wheels!=4u) {
        disarm(CHASSIS_STOP_MOTORS); publish(); return;
    }
    uint32_t sw=ctl.d.rc.switch_left;
    if(ctl.d.state!=CHASSIS_ACTIVE) {
        zero();
        if(sw==DR16_SWITCH_DOWN && neutral() && still) {
            ctl.last_safe_off_ms=now_ms;
            if(!ctl.off_timing) { ctl.off_timing=true; ctl.d.ready_since_ms=now_ms; }
            if(now_ms-ctl.d.ready_since_ms>=CHASSIS_ARM_DWELL_MS)
                ctl.d.state=CHASSIS_READY;
            ctl.d.stop_reason=ctl.d.state==CHASSIS_READY ?
                CHASSIS_STOP_READY_WAIT_UP : CHASSIS_STOP_WAIT_DOWN_DWELL;
        } else if(sw==DR16_SWITCH_MIDDLE && ctl.d.state==CHASSIS_READY &&
                  neutral() && still && now_ms-ctl.last_safe_off_ms<=2000u) {
            /* A three-position lever must pass through MID on its way from
             * DOWN to UP. Keep the completed neutral qualification briefly;
             * MID never enables output. ACTIVE->MID still disarms immediately. */
            ctl.d.stop_reason=CHASSIS_STOP_READY_WAIT_UP;
        } else if(sw==DR16_SWITCH_UP && ctl.d.state==CHASSIS_READY && neutral() && still &&
                  now_ms-ctl.last_safe_off_ms<=2000u) {
            ctl.d.state=CHASSIS_ACTIVE; ++ctl.d.arm_count;
            ctl.off_timing=false; ctl.d.stop_reason=CHASSIS_STOP_NONE;
        } else {
            uint32_t reason=CHASSIS_STOP_NEED_DOWN_SEQUENCE;
            if(!neutral()) reason=CHASSIS_STOP_WAIT_DRIVE_NEUTRAL;
            else if(!still) reason=CHASSIS_STOP_WAIT_WHEELS_STILL;
            disarm(reason);
        }
        publish(); return;
    }
    if(sw!=DR16_SWITCH_UP) { disarm(CHASSIS_STOP_SWITCH); publish(); return; }
    ctl.d.stop_reason=CHASSIS_STOP_NONE;
    ctl.d.desired_right_milli=ctl.d.rc.normalized[DR16_RIGHT_HORIZONTAL];
    ctl.d.desired_forward_milli=ctl.d.rc.normalized[DR16_RIGHT_VERTICAL];
    ctl.d.desired_cw_milli=ctl.d.rc.normalized[DR16_LEFT_HORIZONTAL];
    float desired[4];
    chassis_mix((float)ctl.d.desired_right_milli/1000.0f,
                (float)ctl.d.desired_forward_milli/1000.0f,
                (float)ctl.d.desired_cw_milli/1000.0f,desired);
    float dt=(float)elapsed*0.001f;
    for(uint32_t i=0;i<4u;++i) {
        chassis_motor_diag_t *m=&ctl.d.motors[i];
        ctl.target[i]+=clamp(desired[i]-ctl.target[i],RPM_ACCEL_PER_S*dt);
        float error=ctl.target[i]-(float)m->rotor_rpm;
        float proposed_i=clamp(ctl.integral[i]+SPEED_KI*error*dt,INTEGRAL_LIMIT);
        float candidate=SPEED_KP*error+proposed_i;
        if(absf(candidate)<=CHASSIS_MAX_CURRENT_RAW ||
           (candidate>CHASSIS_MAX_CURRENT_RAW && error<0.0f) ||
           (candidate< -CHASSIS_MAX_CURRENT_RAW && error>0.0f)) ctl.integral[i]=proposed_i;
        float current=clamp(SPEED_KP*error+ctl.integral[i],(float)CHASSIS_MAX_CURRENT_RAW);
        if(absf(ctl.target[i])<0.01f && absf((float)m->rotor_rpm)<=3.0f) {
            if(!ctl.still_timing[i]) { ctl.still_timing[i]=true;ctl.still_since[i]=now_ms; }
            if(now_ms-ctl.still_since[i]>=100u) { current=0;ctl.integral[i]=0; }
        } else ctl.still_timing[i]=false;
        m->target_millirpm=(int32_t)(ctl.target[i]*1000.0f);
        m->command_current_raw=(int32_t)current;
        if(absf(ctl.target[i])>80.0f && absf((float)m->rotor_rpm)<5.0f && absf(current)>1000.0f) {
            if(!ctl.stall_timing[i]) {ctl.stall_timing[i]=true;ctl.stall_since[i]=now_ms;}
            else if(now_ms-ctl.stall_since[i]>=700u) {fault(CHASSIS_FAULT_STALL,m->can_id);break;}
        } else ctl.stall_timing[i]=false;
    }
    publish();
}
bool chassis_output_enabled(void) { return ctl.d.state==CHASSIS_ACTIVE; }
void chassis_currents(int16_t out[8]) {
    if(!out) return;
    for(uint32_t i=0;i<8u;++i) out[i]=0;
    if(ctl.d.state!=CHASSIS_ACTIVE) return;
    for(uint32_t i=0;i<4u;++i) out[i]=(int16_t)ctl.d.motors[i].command_current_raw;
    /* IDs5..8, including feeder207, are always unpowered. */
}
