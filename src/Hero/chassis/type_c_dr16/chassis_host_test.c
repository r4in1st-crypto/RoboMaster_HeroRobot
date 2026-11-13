/* Tests the actual chassis_control.c on a host. This is software evidence,
 * not confirmation of transmitter polarity, wheel installation, or hardware.
 */
#include "chassis_control.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
static uint32_t now_ms, last_rc_sent;
static bool rc_sent, rc_enabled, can_ok, key_stop, plant_enabled;
static unsigned motor_mask;
static dr16_frame_t remote;
static int32_t encoder[5], motor_rpm[5];
static uint8_t temperature[5];
static double plant_rpm[4], plant_position[4];
static int32_t previous_target[4];
static uint32_t previous_state;
static const uint16_t motor_ids[5] = {0x201,0x202,0x203,0x204,0x207};

#define CHECK(c) do { ++checks; if (!(c)) { \
    fprintf(stderr,"FAIL line %d: %s; now=%lu state=%lu fault=%lu id=%lx stop=%lu\n", \
        __LINE__,#c,(unsigned long)now_ms,(unsigned long)chassis_diag.state, \
        (unsigned long)chassis_diag.fault_code,(unsigned long)chassis_diag.fault_can_id, \
        (unsigned long)chassis_diag.stop_reason); exit(1); } } while (0)

static float absf(float x) { return x < 0 ? -x : x; }
static int32_t rounded(double x) { return (int32_t)(x >= 0 ? x + 0.5 : x - 0.5); }

static void command(unsigned axis, int32_t normalized) {
    int32_t magnitude = normalized < 0 ? -normalized : normalized;
    /* Round away from the deadzone so normalized +/-1 represents raw +/-21,
     * matching a realizable parser output rather than a centered raw value. */
    int32_t raw_delta = magnitude ? 20 + (magnitude * 640 + 999) / 1000 : 0;
    if (normalized < 0) raw_delta = -raw_delta;
    remote.normalized[axis] = normalized;
    remote.centered[axis] = raw_delta;
    remote.raw[axis] = (uint32_t)(1024 + raw_delta);
}

static void reset_at(uint32_t start) {
    unsigned i;
    chassis_init();
    now_ms = start; last_rc_sent = 0; rc_sent = false;
    rc_enabled = can_ok = true; key_stop = plant_enabled = false; motor_mask = 15;
    previous_state=CHASSIS_DISABLED;
    memset(previous_target,0,sizeof(previous_target));
    memset(&remote,0,sizeof(remote));
    remote.switch_left = DR16_SWITCH_DOWN;
    remote.switch_right = DR16_SWITCH_MIDDLE;
    for (i=0;i<5;++i) {
        command(i,0); encoder[i]=(int32_t)(1000+i*1000); motor_rpm[i]=0;
        temperature[i]=25;
        if(i<4) {plant_rpm[i]=0;plant_position[i]=encoder[i];}
    }
    remote.wheel_valid=1;
    CHECK(chassis_diag.magic==CHASSIS_DIAG_MAGIC);
    CHECK(chassis_diag.size_bytes==sizeof(chassis_diag_t));
    CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(!chassis_output_enabled());
}

static void feedback(unsigned i) {
    uint8_t data[8]={0};
    uint16_t raw=(uint16_t)((uint32_t)encoder[i]&8191u), rpm=(uint16_t)motor_rpm[i];
    data[0]=(uint8_t)(raw>>8); data[1]=(uint8_t)raw;
    data[2]=(uint8_t)(rpm>>8); data[3]=(uint8_t)rpm; data[6]=temperature[i];
    chassis_feedback(motor_ids[i],data,now_ms);
}

static void verify_current_limits(void) {
    int16_t out[8]; unsigned i;
    chassis_currents(out);
    for(i=0;i<8;++i) CHECK(out[i]>=-CHASSIS_MAX_CURRENT_RAW && out[i]<=CHASSIS_MAX_CURRENT_RAW);
    for(i=4;i<8;++i) CHECK(out[i]==0); /* Includes feeder 0x207. */
    if(!chassis_output_enabled()) for(i=0;i<4;++i) CHECK(out[i]==0);
    for(i=0;i<4;++i) {
        int32_t target=chassis_diag.motors[i].target_millirpm;
        CHECK(target>=-300000 && target<=300000);
        if(previous_state==CHASSIS_ACTIVE && chassis_diag.state==CHASSIS_ACTIVE) {
            int32_t delta=target-previous_target[i];
            /* One-millisecond ticks, plus one count for float-to-int rounding. */
            CHECK(delta>=-301 && delta<=301);
        }
        previous_target[i]=target;
    }
    previous_state=chassis_diag.state;
}

static void zero_output(void) {
    int16_t out[8]; unsigned i;
    chassis_currents(out);
    for(i=0;i<8;++i) CHECK(out[i]==0);
}

static void tick(void) {
    unsigned i;
    if(plant_enabled) {
        int16_t out[8]; chassis_currents(out);
        for(i=0;i<4;++i) {
            /* A simple independent inertial plant, not a hardware model. */
            /* Steady current at 300 rpm is 240 raw, within the deliberately
             * conservative 400-raw integral cap. Higher loads can retain
             * speed error and need hardware tuning, not a more permissive test. */
            plant_rpm[i] += ((double)out[i]*0.25 - plant_rpm[i]*0.2)*0.001;
            plant_position[i] += plant_rpm[i]*8192.0/60000.0;
            encoder[i]=rounded(plant_position[i]); motor_rpm[i]=rounded(plant_rpm[i]);
        }
    }
    for(i=0;i<5;++i) if(motor_mask&(1u<<i)) feedback(i);
    if(rc_enabled && (!rc_sent || (uint32_t)(now_ms-last_rc_sent)>=14u)) {
        remote.last_frame_ms=now_ms; chassis_remote_update(&remote);
        last_rc_sent=now_ms; rc_sent=true;
    }
    chassis_step(now_ms,can_ok,key_stop);
    verify_current_limits();
    ++now_ms;
}

static void run(unsigned ms) { while(ms--) tick(); }

static void make_ready(void) {
    unsigned n;
    remote.switch_left=DR16_SWITCH_DOWN;
    for(n=0;n<4;++n) command(n,0);
    for(n=0;n<1200 && chassis_diag.state!=CHASSIS_READY;++n) tick();
    CHECK(chassis_diag.state==CHASSIS_READY);
    CHECK(chassis_diag.online_wheels==4);
    CHECK(!chassis_output_enabled()); zero_output();
}

static void arm(void) {
    unsigned n;
    CHECK(chassis_diag.state==CHASSIS_READY);
    remote.switch_left=DR16_SWITCH_UP;
    for(n=0;n<30 && chassis_diag.state!=CHASSIS_ACTIVE;++n) tick();
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    CHECK(chassis_output_enabled()); zero_output();
}

static void test_mixer(void) {
    float out[4]; unsigned i;
    const float forward[4]={-300,-300,300,300};
    const float right[4]={-300,300,300,-300};
    chassis_mix(0,1,0,out);
    for(i=0;i<4;++i) CHECK(absf(out[i]-forward[i])<0.001f);
    chassis_mix(1,0,0,out);
    for(i=0;i<4;++i) CHECK(absf(out[i]-right[i])<0.001f);
    chassis_mix(0,0,1,out);
    for(i=0;i<4;++i) CHECK(absf(out[i]-195.0f)<0.001f);
    chassis_mix(0,0,-1,out);
    for(i=0;i<4;++i) CHECK(absf(out[i]+195.0f)<0.001f);
    chassis_mix(1,1,1,out);
    CHECK(absf(out[2]-300.0f)<0.001f);
    CHECK(absf(out[0]/out[2]-(-1.35f/2.65f))<0.00001f);
    CHECK(absf(out[1]/out[2]-(0.65f/2.65f))<0.00001f);
    CHECK(absf(out[3]-out[1])<0.001f);
    for(int r=-2;r<=2;++r) for(int f=-2;f<=2;++f) for(int w=-2;w<=2;++w) {
        float reverse[4]; chassis_mix(r*0.5f,f*0.5f,w*0.5f,out);
        chassis_mix(-r*0.5f,-f*0.5f,-w*0.5f,reverse);
        for(i=0;i<4;++i) {
            CHECK(absf(out[i])<=CHASSIS_MAX_ROTOR_RPM+0.001f);
            CHECK(absf(out[i]+reverse[i])<0.001f);
        }
    }
    puts("PASS FL203/FR202/RR201/LR204 mixer signs, proportional saturation and limits");
}

static void test_arming_sequence(void) {
    reset_at(0); remote.switch_left=DR16_SWITCH_UP;
    run(800); CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_NEED_DOWN_SEQUENCE);
    CHECK(chassis_diag.arm_count==0); zero_output();
    make_ready(); arm(); CHECK(chassis_diag.arm_count==1);
    reset_at(0);
    run(556); CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DOWN_DWELL);
    tick(); CHECK(chassis_diag.state==CHASSIS_READY);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_READY_WAIT_UP);
    CHECK(chassis_diag.ready_since_ms==56);
    arm();
    reset_at(0); motor_mask=7; run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.online_wheels==3);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_MOTORS);
    motor_mask=15; make_ready(); arm();
    reset_at(0); command(DR16_RIGHT_HORIZONTAL,1); run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DRIVE_NEUTRAL);
    make_ready();
    command(DR16_RIGHT_HORIZONTAL,1000); remote.switch_left=DR16_SWITCH_UP;
    run(50); CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DRIVE_NEUTRAL);
    command(DR16_RIGHT_HORIZONTAL,0); run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==0);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_NEED_DOWN_SEQUENCE);
    make_ready(); arm();
    puts("PASS startup UP inhibited, full OFF/neutral dwell, four fresh wheels, nonneutral UP rejection");
}

static void test_only_drive_axes_inhibit_arming(void) {
    const unsigned drive_axes[3]={DR16_RIGHT_HORIZONTAL,DR16_RIGHT_VERTICAL,
                                  DR16_LEFT_HORIZONTAL};
    for(int sign=-1;sign<=1;sign+=2) {
        reset_at(0); command(DR16_LEFT_VERTICAL,sign*1000);
        run(556);
        CHECK(chassis_diag.state==CHASSIS_DISABLED);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DOWN_DWELL);
        tick();
        CHECK(chassis_diag.state==CHASSIS_READY);
        CHECK(chassis_diag.ready_since_ms==56u);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_READY_WAIT_UP);
        CHECK(chassis_diag.rc.normalized[DR16_LEFT_VERTICAL]==sign*1000);
        zero_output();
        remote.switch_left=DR16_SWITCH_MIDDLE; run(250);
        CHECK(chassis_diag.state==CHASSIS_READY);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_READY_WAIT_UP);
        arm(); run(100);
        CHECK(chassis_diag.state==CHASSIS_ACTIVE && chassis_diag.arm_count==1u);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_NONE);
        zero_output();
    }
    for(unsigned axis=0;axis<3u;++axis) for(int sign=-1;sign<=1;sign+=2) {
        reset_at(0);
        command(DR16_LEFT_VERTICAL,1000);
        command(drive_axes[axis],sign); /* First nonzero normalized command. */
        run(800);
        CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==0u);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DRIVE_NEUTRAL);
        CHECK(chassis_diag.ready_since_ms==0u); zero_output();
        command(drive_axes[axis],0); run(600);
        CHECK(chassis_diag.state==CHASSIS_READY);
        command(drive_axes[axis],sign*1000); remote.switch_left=DR16_SWITCH_UP;
        run(30);
        CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==0u);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DRIVE_NEUTRAL);
        zero_output();
        command(drive_axes[axis],0); run(100);
        CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==0u);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_NEED_DOWN_SEQUENCE);
        zero_output();
    }
    puts("PASS unused left vertical permits OFF/MID/UP arming; each drive axis still requires neutral");
}

static void test_wait_for_still_wheels_reason(void) {
    reset_at(0); motor_rpm[2]=6;
    run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.online_wheels==4u && chassis_diag.ready_since_ms==0u);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_WHEELS_STILL); zero_output();
    motor_rpm[2]=0; tick();
    uint32_t stopped_at=chassis_diag.ready_since_ms;
    CHECK(stopped_at==800u);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DOWN_DWELL);
    run(499);
    CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DOWN_DWELL);
    tick();
    CHECK(chassis_diag.state==CHASSIS_READY);
    CHECK(chassis_diag.ready_since_ms==stopped_at);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_READY_WAIT_UP); zero_output();
    puts("PASS moving-wheel wait is distinct from the unchanged 500 ms DOWN dwell and READY");
}

static void test_no_drive_from_left_vertical_or_switch_right(void) {
    reset_at(0); make_ready(); arm();
    command(DR16_LEFT_VERTICAL,1000); remote.switch_right=DR16_SWITCH_UP;
    run(300); CHECK(chassis_diag.state==CHASSIS_ACTIVE); zero_output();
    CHECK(chassis_diag.desired_right_milli==0);
    CHECK(chassis_diag.desired_forward_milli==0 && chassis_diag.desired_cw_milli==0);
    command(DR16_LEFT_VERTICAL,-1000); remote.switch_right=DR16_SWITCH_DOWN;
    run(300); CHECK(chassis_diag.state==CHASSIS_ACTIVE); zero_output();
    puts("PASS left vertical and right switch do not issue drive/feeder commands");
}

static void test_middle_switch_transit(void) {
    reset_at(0); make_ready();
    remote.switch_left=DR16_SWITCH_MIDDLE; run(500);
    CHECK(chassis_diag.state==CHASSIS_READY); zero_output();
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_READY_WAIT_UP);
    arm(); CHECK(chassis_diag.arm_count==1);
    remote.switch_left=DR16_SWITCH_MIDDLE; run(30);
    CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    remote.switch_left=DR16_SWITCH_UP; run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==1);
    reset_at(0); make_ready();
    remote.switch_left=DR16_SWITCH_MIDDLE; run(2050);
    CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_NEED_DOWN_SEQUENCE);
    remote.switch_left=DR16_SWITCH_UP; run(100);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==0);
    reset_at(0); make_ready();
    command(DR16_LEFT_HORIZONTAL,1); remote.switch_left=DR16_SWITCH_MIDDLE; run(30);
    CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DRIVE_NEUTRAL);
    command(DR16_LEFT_HORIZONTAL,0); remote.switch_left=DR16_SWITCH_UP; run(100);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==0);
    puts("PASS neutral DOWN-MID-UP transit, two-second expiry and nonneutral MID rejection");
}

static void test_remote_loss_invalid_and_rearm(void) {
    reset_at(0); make_ready(); arm();
    command(DR16_RIGHT_VERTICAL,1000); run(200);
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    rc_enabled=false;
    while((uint32_t)(now_ms-last_rc_sent)<50u) tick();
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    tick(); CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_RC_TIMEOUT);
    rc_enabled=true; command(DR16_RIGHT_VERTICAL,0); run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==1);
    make_ready(); arm(); CHECK(chassis_diag.arm_count==2);
    command(DR16_RIGHT_HORIZONTAL,1000); run(200);
    chassis_remote_invalid(now_ms,DR16_ERROR_CHANNEL);
    CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    CHECK(chassis_diag.rc_valid==0 && chassis_diag.rc_consecutive==0);
    command(DR16_RIGHT_HORIZONTAL,0); run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==2);
    make_ready(); arm(); CHECK(chassis_diag.arm_count==3);
    puts("PASS 50 ms RC timeout, immediate invalidation, no reconnect auto-arm, OFF rearm");
}

static void assert_hard_fault(uint32_t expected) {
    CHECK(expected!=CHASSIS_FAULT_FEEDBACK);
    CHECK(chassis_diag.state==CHASSIS_FAULT);
    CHECK(chassis_diag.fault_code==expected); zero_output();
    can_ok=true; key_stop=false; motor_mask=15;
    for(unsigned i=0;i<4;++i) {motor_rpm[i]=0;temperature[i]=25;command(i,0);}
    remote.switch_left=DR16_SWITCH_DOWN; run(800);
    if(expected!=CHASSIS_FAULT_CAN) {
        CHECK(!chassis_can_recovery_requested(now_ms));
        CHECK(!chassis_clear_can_fault(now_ms));
        CHECK(chassis_diag.state==CHASSIS_FAULT && chassis_diag.fault_code==expected);
    }
    remote.switch_left=DR16_SWITCH_UP; run(100);
    CHECK(chassis_diag.state==CHASSIS_FAULT);
    CHECK(chassis_diag.fault_code==expected); zero_output();
}

static void assert_feedback_fault_holds_in_up(void) {
    CHECK(chassis_diag.state==CHASSIS_FAULT);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK); zero_output();
    can_ok=true; key_stop=false; motor_mask=15;
    for(unsigned i=0;i<4u;++i) {motor_rpm[i]=0;temperature[i]=25;command(i,0);}
    remote.switch_left=DR16_SWITCH_UP; run(800);
    CHECK(chassis_diag.state==CHASSIS_FAULT);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK);
    CHECK(!chassis_can_recovery_requested(now_ms));
    CHECK(!chassis_clear_can_fault(now_ms)); zero_output();
}

static void test_motor_can_key_and_stall_faults(void) {
    reset_at(0); make_ready(); arm();
    motor_mask=14; run(31);
    CHECK(chassis_diag.fault_can_id==0x201);
    assert_feedback_fault_holds_in_up();
    reset_at(0); make_ready(); arm();
    can_ok=false; tick(); assert_hard_fault(CHASSIS_FAULT_CAN);
    reset_at(0); make_ready(); arm();
    key_stop=true; tick(); assert_hard_fault(CHASSIS_FAULT_STOP);
    reset_at(0); make_ready(); arm(); command(DR16_RIGHT_VERTICAL,1000);
    run(600);
    {
        int16_t saturated[8]; chassis_currents(saturated);
        CHECK(chassis_diag.state==CHASSIS_ACTIVE);
        CHECK(saturated[0]==-1500 && saturated[1]==-1500);
        CHECK(saturated[2]==1500 && saturated[3]==1500);
    }
    run(1200); assert_hard_fault(CHASSIS_FAULT_STALL);
    reset_at(0); make_ready(); arm(); temperature[2]=70;
    tick(); assert_hard_fault(CHASSIS_FAULT_TEMPERATURE);
    reset_at(0); make_ready(); arm(); motor_rpm[1]=901;
    tick(); assert_hard_fault(CHASSIS_FAULT_OVERSPEED);
    reset_at(0); make_ready(); arm(); now_ms+=11;
    tick(); assert_hard_fault(CHASSIS_FAULT_TIMING);
    puts("PASS motor loss stops without UP auto-recovery; CAN, KEY, stall, temperature, overspeed and timing stay latched");
}

static void test_isolated_bad_motor_payload_is_discarded(void) {
    const uint8_t invalid_payloads[2][8]={
        {0x20,0x00,0x7f,0xff,0x7f,0xff,255,0},
        {0xff,0xff,0x80,0x00,0x80,0x00,255,0}
    };
    reset_at(0); make_ready(); arm();
    command(DR16_RIGHT_VERTICAL,1000); run(200);
    CHECK(chassis_diag.motors[0].command_current_raw!=0);
    for(unsigned i=0;i<5u;++i) {
        chassis_motor_diag_t accepted=chassis_diag.motors[i];
        chassis_feedback(motor_ids[i],NULL,now_ms);
        chassis_motor_diag_t after=chassis_diag.motors[i];
        CHECK(memcmp(&accepted,&after,sizeof(accepted))==0);
        for(unsigned bad=0;bad<2u;++bad) {
            chassis_feedback(motor_ids[i],invalid_payloads[bad],now_ms);
            after=chassis_diag.motors[i];
            CHECK(memcmp(&accepted,&after,sizeof(accepted))==0);
            CHECK(chassis_diag.state==CHASSIS_ACTIVE);
            CHECK(chassis_diag.fault_code==CHASSIS_FAULT_NONE);
        }
        encoder[i]=(int32_t)((accepted.encoder_raw+1u)&8191u);
        feedback(i);
        CHECK(chassis_diag.motors[i].feedback_count==accepted.feedback_count+1u);
        CHECK(chassis_diag.motors[i].last_feedback_ms==now_ms);
        CHECK(chassis_diag.motors[i].encoder_raw==(uint32_t)encoder[i]);
        tick();
        CHECK(chassis_diag.state==CHASSIS_ACTIVE && chassis_output_enabled());
        CHECK(chassis_diag.motors[0].command_current_raw!=0);
    }
    puts("PASS isolated NULL/out-of-range motor payloads preserve measurements and ACTIVE drive; next good frame resumes");
}

static void test_continuous_bad_motor_payload_times_out(void) {
    const uint8_t invalid_payload[8]={0x20,0x00,0x7f,0xff,0x7f,0xff,255,0};
    reset_at(0); make_ready(); arm();
    chassis_motor_diag_t accepted=chassis_diag.motors[0];
    motor_mask=14u;
    for(uint32_t age=1u;age<=31u;++age) {
        CHECK(now_ms-accepted.last_feedback_ms==age);
        chassis_feedback(0x201u,(age&1u)!=0u ? invalid_payload : NULL,now_ms);
        CHECK(chassis_diag.motors[0].last_feedback_ms==accepted.last_feedback_ms);
        CHECK(chassis_diag.motors[0].feedback_count==accepted.feedback_count);
        CHECK(chassis_diag.motors[0].encoder_raw==accepted.encoder_raw);
        CHECK(chassis_diag.motors[0].rotor_rpm==accepted.rotor_rpm);
        CHECK(chassis_diag.motors[0].measured_current_raw==accepted.measured_current_raw);
        CHECK(chassis_diag.motors[0].temperature_c==accepted.temperature_c);
        tick();
        if(age<=30u) CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    }
    CHECK(chassis_diag.state==CHASSIS_FAULT);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK && chassis_diag.fault_can_id==0x201u);
    CHECK(chassis_diag.motors[0].fresh==0u); zero_output();
    feedback(0);
    CHECK(chassis_diag.state==CHASSIS_FAULT && chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK);
    assert_feedback_fault_holds_in_up();
    puts("PASS sustained bad payloads never refresh motor data and stop at the existing valid-feedback timeout");
}

static void make_recoverable_can_fault(void) {
    reset_at(0); make_ready(); arm();
    can_ok=false; tick(); can_ok=true;
    CHECK(chassis_diag.state==CHASSIS_FAULT && chassis_diag.fault_code==CHASSIS_FAULT_CAN);
    remote.switch_left=DR16_SWITCH_DOWN; run(100);
    CHECK(chassis_can_recovery_requested(now_ms));
    CHECK(chassis_diag.state==CHASSIS_FAULT); zero_output();
}

static void reject_can_recovery(void) {
    uint32_t state=chassis_diag.state, code=chassis_diag.fault_code;
    CHECK(!chassis_can_recovery_requested(now_ms));
    CHECK(!chassis_clear_can_fault(now_ms));
    CHECK(chassis_diag.state==state && chassis_diag.fault_code==code);
    zero_output();
}

static void test_can_recovery_rechecks_private_safety_conditions(void) {
    reset_at(0); run(100); reject_can_recovery(); /* Not a fault. */
    make_ready(); arm(); key_stop=true; tick(); key_stop=false;
    remote.switch_left=DR16_SWITCH_DOWN; run(100);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_STOP); reject_can_recovery();

    make_recoverable_can_fault();
    remote.switch_left=DR16_SWITCH_UP; run(30); reject_can_recovery();
    /* A writable diagnostic must not manufacture a private safe-OFF request. */
    chassis_diag.rc.switch_left=DR16_SWITCH_DOWN;
    CHECK(!chassis_can_recovery_requested(now_ms));
    CHECK(!chassis_clear_can_fault(now_ms));
    remote.switch_left=DR16_SWITCH_MIDDLE; run(30); reject_can_recovery();
    remote.switch_left=DR16_SWITCH_DOWN; run(30);
    CHECK(chassis_can_recovery_requested(now_ms));
    for(unsigned axis=0;axis<3u;++axis) {
        command(axis,1); run(30); reject_can_recovery();
        command(axis,0); run(30); CHECK(chassis_can_recovery_requested(now_ms));
    }
    command(DR16_LEFT_VERTICAL,1000); run(30);
    CHECK(chassis_can_recovery_requested(now_ms));
    for(unsigned i=0;i<4u;++i) {
        motor_rpm[i]=6; tick(); reject_can_recovery();
        motor_rpm[i]=0; tick(); CHECK(chassis_can_recovery_requested(now_ms));
        temperature[i]=70; tick(); reject_can_recovery();
        CHECK(chassis_diag.fault_code==CHASSIS_FAULT_TEMPERATURE);
        temperature[i]=25; tick(); reject_can_recovery();
        make_recoverable_can_fault();
        motor_mask=15u & ~(1u<<i); run(31); reject_can_recovery();
        motor_mask=15u; tick(); CHECK(chassis_can_recovery_requested(now_ms));
    }
    /* Clear revalidates, even if a previous query was true in the same tick. */
    CHECK(chassis_can_recovery_requested(now_ms));
    chassis_remote_invalid(now_ms,DR16_ERROR_CHANNEL); reject_can_recovery();
    run(14); reject_can_recovery(); /* One valid packet is not five. */
    run(70); CHECK(chassis_can_recovery_requested(now_ms));
    rc_enabled=false; run(60); reject_can_recovery();
    rc_enabled=true; run(80); CHECK(chassis_can_recovery_requested(now_ms));
    puts("PASS CAN recovery rejects other faults, UP/MID, drive input, moving/hot/missing wheels and bad/stale RC");
}

static void test_can_recovery_freshness_boundaries(void) {
    make_recoverable_can_fault();
    uint32_t rc_time=chassis_diag.rc.last_frame_ms;
    uint32_t saved_now=now_ms;
    now_ms=rc_time+49u;
    for(unsigned i=0;i<4u;++i) feedback(i);
    CHECK(chassis_can_recovery_requested(now_ms));
    ++now_ms; reject_can_recovery(); /* RC age must be strictly below 50 ms. */
    now_ms=saved_now;

    make_recoverable_can_fault();
    for(unsigned i=0;i<4u;++i) feedback(i);
    uint32_t motor_time=now_ms;
    now_ms+=30u;
    remote.last_frame_ms=now_ms; chassis_remote_update(&remote);
    CHECK(chassis_can_recovery_requested(now_ms));
    ++now_ms; reject_can_recovery(); /* Motor age may be 30, never 31 ms. */
    CHECK(now_ms-motor_time==31u);

    reset_at(0); motor_mask=0u; can_ok=false; run(70); can_ok=true;
    CHECK(chassis_diag.rc_consecutive>=5u);
    for(unsigned sample=0;sample<7u;++sample) {
        for(unsigned i=0;i<4u;++i) feedback(i);
        reject_can_recovery();
    }
    for(unsigned i=0;i<4u;++i) feedback(i);
    CHECK(chassis_can_recovery_requested(now_ms));
    puts("PASS CAN recovery enforces RC<50 ms, wheel<=30 ms and at least eight wheel frames");
}

static void test_can_recovery_requires_new_full_arm_sequence(void) {
    make_recoverable_can_fault();
    uint32_t arms=chassis_diag.arm_count, disarms=chassis_diag.disarm_count;
    /* No public diagnostic fields are consulted by the recovery decision. */
    chassis_diag.state=CHASSIS_ACTIVE;
    chassis_diag.fault_code=CHASSIS_FAULT_STOP;
    chassis_diag.rc_valid=0;
    chassis_diag.motors[0].fresh=0;
    CHECK(chassis_can_recovery_requested(now_ms));
    CHECK(chassis_clear_can_fault(now_ms));
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.fault_code==CHASSIS_FAULT_NONE);
    CHECK(chassis_diag.fault_can_id==0u && chassis_diag.ready_since_ms==0u);
    CHECK(chassis_diag.arm_count==arms && chassis_diag.disarm_count==disarms);
    CHECK(!chassis_output_enabled()); zero_output();
    CHECK(!chassis_can_recovery_requested(now_ms));
    CHECK(!chassis_clear_can_fault(now_ms));
    uint32_t dwell_start=now_ms;
    tick();
    CHECK(chassis_diag.ready_since_ms==dwell_start);
    run(499);
    CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DOWN_DWELL); zero_output();
    tick(); CHECK(chassis_diag.state==CHASSIS_READY);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_READY_WAIT_UP); zero_output();
    arm(); CHECK(chassis_diag.arm_count==arms+1u);
    CHECK(chassis_diag.disarm_count==disarms);

    make_recoverable_can_fault(); arms=chassis_diag.arm_count;
    CHECK(chassis_clear_can_fault(now_ms));
    remote.switch_left=DR16_SWITCH_UP; run(100);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==arms);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_NEED_DOWN_SEQUENCE); zero_output();
    make_ready(); arm(); CHECK(chassis_diag.arm_count==arms+1u);
    puts("PASS explicit CAN clear preserves history, ignores writable diagnostics and requires a fresh 500 ms DOWN then UP");
}

static void make_feedback_fault(unsigned lost_wheel) {
    reset_at(0); make_ready(); arm();
    motor_mask=15u & ~(1u<<lost_wheel); run(31);
    CHECK(chassis_diag.state==CHASSIS_FAULT);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK);
    CHECK(chassis_diag.fault_can_id==motor_ids[lost_wheel]);
    CHECK(chassis_diag.arm_count==1u && chassis_diag.disarm_count==1u);
    motor_mask=15u; zero_output();
}

static uint32_t await_feedback_clear(void) {
    for(unsigned n=0;n<100u && chassis_diag.state==CHASSIS_FAULT;++n) tick();
    CHECK(chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_NONE && chassis_diag.fault_can_id==0u);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_WAIT_DOWN_DWELL);
    CHECK(chassis_diag.ready_since_ms==now_ms-1u);
    CHECK(chassis_diag.arm_count==1u && chassis_diag.disarm_count==1u);
    CHECK(!chassis_output_enabled()); zero_output();
    return chassis_diag.ready_since_ms;
}

static void assert_feedback_fault(void) {
    CHECK(chassis_diag.state==CHASSIS_FAULT);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK);
    CHECK(chassis_diag.arm_count==1u && chassis_diag.disarm_count==1u);
    CHECK(!chassis_output_enabled()); zero_output();
}

static void test_feedback_recovery_requires_new_full_arm_sequence(void) {
    for(unsigned lost=0;lost<4u;++lost) {
        make_feedback_fault(lost);
        run(800); assert_feedback_fault(); reject_can_recovery();
        remote.switch_left=DR16_SWITCH_MIDDLE; run(100);
        assert_feedback_fault(); reject_can_recovery();
        remote.switch_left=DR16_SWITCH_DOWN;
        uint32_t cleared_at=await_feedback_clear();
        run(499);
        CHECK(chassis_diag.state==CHASSIS_DISABLED);
        CHECK(chassis_diag.ready_since_ms==cleared_at); zero_output();
        tick(); CHECK(chassis_diag.state==CHASSIS_READY);
        CHECK(chassis_diag.stop_reason==CHASSIS_STOP_READY_WAIT_UP);
        run(100); CHECK(chassis_diag.state==CHASSIS_READY); zero_output();
        arm(); CHECK(chassis_diag.arm_count==2u && chassis_diag.disarm_count==1u);
        command(DR16_RIGHT_VERTICAL,1000); run(100);
        CHECK(chassis_diag.state==CHASSIS_ACTIVE);
        CHECK(chassis_diag.motors[0].command_current_raw!=0);
    }
    make_feedback_fault(0);
    remote.switch_left=DR16_SWITCH_DOWN; (void)await_feedback_clear();
    remote.switch_left=DR16_SWITCH_UP; run(800);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==1u);
    CHECK(chassis_diag.stop_reason==CHASSIS_STOP_NEED_DOWN_SEQUENCE); zero_output();
    make_ready(); arm(); CHECK(chassis_diag.arm_count==2u);
    puts("PASS each lost wheel recovers only via safe DOWN, a new complete 500 ms dwell, then explicit UP; early UP cancels");
}

static void test_feedback_recovery_safety_guards(void) {
    for(unsigned axis=0;axis<3u;++axis) for(int sign=-1;sign<=1;sign+=2) {
        make_feedback_fault(0);
        command(axis,sign); remote.switch_left=DR16_SWITCH_DOWN; run(100);
        assert_feedback_fault(); reject_can_recovery();
        /* Public diagnostics cannot authorize the new automatic path either. */
        for(unsigned i=0;i<3u;++i) chassis_diag.rc.normalized[i]=0;
        chassis_diag.rc.switch_left=DR16_SWITCH_DOWN;
        tick(); assert_feedback_fault(); reject_can_recovery();
        command(axis,0); (void)await_feedback_clear();
    }
    for(unsigned wheel=0;wheel<4u;++wheel) {
        make_feedback_fault(wheel);
        motor_rpm[wheel]=-6; remote.switch_left=DR16_SWITCH_DOWN; run(100);
        assert_feedback_fault(); reject_can_recovery();
        motor_rpm[wheel]=6; run(100);
        assert_feedback_fault(); reject_can_recovery();
        motor_rpm[wheel]=0; (void)await_feedback_clear();

        make_feedback_fault(wheel);
        temperature[wheel]=70; remote.switch_left=DR16_SWITCH_DOWN; run(100);
        CHECK(chassis_diag.fault_code==CHASSIS_FAULT_TEMPERATURE);
        CHECK(chassis_diag.fault_can_id==motor_ids[wheel]); reject_can_recovery();
        temperature[wheel]=69; assert_hard_fault(CHASSIS_FAULT_TEMPERATURE);

        make_feedback_fault(wheel);
        motor_mask=15u & ~(1u<<wheel);
        remote.switch_left=DR16_SWITCH_DOWN; run(800);
        assert_feedback_fault(); reject_can_recovery();
        motor_mask=15u; (void)await_feedback_clear();
    }
    make_feedback_fault(0);
    remote.switch_left=DR16_SWITCH_DOWN;
    chassis_remote_invalid(now_ms,DR16_ERROR_CHANNEL);
    run(14); assert_feedback_fault(); reject_can_recovery();
    CHECK(chassis_diag.rc_consecutive==1u);
    run(42); assert_feedback_fault(); reject_can_recovery();
    CHECK(chassis_diag.rc_consecutive==4u);
    (void)await_feedback_clear();

    make_feedback_fault(0);
    can_ok=false; remote.switch_left=DR16_SWITCH_DOWN; run(100);
    rc_enabled=false; run(60); can_ok=true; tick();
    assert_feedback_fault(); reject_can_recovery();
    rc_enabled=true; (void)await_feedback_clear();

    make_feedback_fault(0);
    key_stop=true; remote.switch_left=DR16_SWITCH_DOWN; run(800);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_STOP);
    reject_can_recovery(); key_stop=false; assert_hard_fault(CHASSIS_FAULT_STOP);

    make_feedback_fault(0);
    command(DR16_LEFT_VERTICAL,1000); remote.switch_left=DR16_SWITCH_DOWN;
    (void)await_feedback_clear();
    CHECK(chassis_diag.rc.normalized[DR16_LEFT_VERTICAL]==1000);
    puts("PASS feedback recovery rejects drive input, moving/missing wheels and invalid/short/stale RC; later heat/STOP stay latched; unused axis permits recovery");
}

static void test_feedback_first_then_can_recovery(void) {
    make_feedback_fault(2);
    can_ok=false; remote.switch_left=DR16_SWITCH_DOWN; run(1500);
    assert_feedback_fault();
    CHECK(chassis_diag.fault_can_id==0x203u);
    CHECK(chassis_can_recovery_requested(now_ms));
    /* A CAN error after a feedback stop must not make the first-fault code
     * ineligible for main's BSP recovery handshake. This host test models BSP
     * completion; the separate policy tests own its 1 s / 20 zero-TX gate. */
    can_ok=true;
    CHECK(chassis_clear_can_fault(now_ms));
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.fault_code==CHASSIS_FAULT_NONE);
    CHECK(chassis_diag.ready_since_ms==0u && chassis_diag.fault_can_id==0u);
    CHECK(chassis_diag.arm_count==1u && chassis_diag.disarm_count==1u); zero_output();
    uint32_t cleared_at=now_ms;
    tick(); CHECK(chassis_diag.ready_since_ms==cleared_at);
    run(499); CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    tick(); CHECK(chassis_diag.state==CHASSIS_READY); zero_output();
    arm(); CHECK(chassis_diag.arm_count==2u && chassis_diag.disarm_count==1u);
    puts("PASS FEEDBACK-first plus CAN fault stays stopped while unhealthy and permits the existing explicit recovery handshake");
}

static void test_later_noncommunication_fault_supersedes_communication(void) {
    make_feedback_fault(0);
    temperature[2]=70; tick();
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_TEMPERATURE);
    CHECK(chassis_diag.fault_can_id==0x203u);
    temperature[2]=25; assert_hard_fault(CHASSIS_FAULT_TEMPERATURE);

    make_recoverable_can_fault();
    temperature[3]=70; tick();
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_TEMPERATURE);
    CHECK(chassis_diag.fault_can_id==0x204u);
    temperature[3]=25; assert_hard_fault(CHASSIS_FAULT_TEMPERATURE);

    make_recoverable_can_fault();
    key_stop=true; tick();
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_STOP);
    key_stop=false; assert_hard_fault(CHASSIS_FAULT_STOP);
    puts("PASS later temperature/STOP faults supersede CAN/FEEDBACK and remain latched after cooling or release");
}

static void test_bad_and_old_frames_cannot_authorize(void) {
    dr16_frame_t forged;
    reset_at(0); make_ready(); arm();
    forged=remote; forged.last_frame_ms=last_rc_sent;
    chassis_remote_update(&forged);
    CHECK(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.rc_valid==0); zero_output();
    run(800); CHECK(chassis_diag.state==CHASSIS_DISABLED);
    reset_at(0); make_ready(); arm();
    forged=remote; forged.last_frame_ms=last_rc_sent-1u;
    chassis_remote_update(&forged); chassis_step(now_ms,true,false);
    CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    run(800); CHECK(chassis_diag.state==CHASSIS_DISABLED);
    reset_at(0); make_ready(); arm();
    forged=remote; forged.last_frame_ms=now_ms+1000u;
    chassis_remote_update(&forged); chassis_step(now_ms,true,false);
    CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    run(800); CHECK(chassis_diag.state==CHASSIS_DISABLED);
    reset_at(0); make_ready();
    rc_enabled=false;
    forged=remote; forged.switch_left=DR16_SWITCH_UP;
    forged.last_frame_ms=last_rc_sent-100u;
    chassis_remote_update(&forged); chassis_step(now_ms,true,false);
    CHECK(chassis_diag.state!=CHASSIS_ACTIVE); zero_output();
    puts("PASS duplicate, stale, backwards and future timestamps never authorize motion");
}

static void test_feedback_gap_not_hidden_by_recovery(void) {
    reset_at(0); make_ready(); arm();
    uint32_t last_good=chassis_diag.motors[3].last_feedback_ms;
    const uint8_t invalid_payload[8]={0xff,0xff,0,0,0,0,25,0};
    now_ms+=31u;
    chassis_feedback(0x204u,invalid_payload,now_ms);
    CHECK(chassis_diag.motors[3].last_feedback_ms==last_good);
    feedback(3);
    CHECK(chassis_diag.fault_can_id==0x204);
    assert_feedback_fault_holds_in_up();
    puts("PASS a late feedback frame cannot conceal the previous timeout gap");
}

static void test_closed_loop_plant(void) {
    unsigned i;
    reset_at(0); make_ready(); arm(); plant_enabled=true;
    command(DR16_RIGHT_VERTICAL,1000); run(5000);
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    for(i=0;i<4;++i) {
        int sign=i<2 ? -1 : 1;
        CHECK(plant_rpm[i]*sign>250.0 && plant_rpm[i]*sign<350.0);
        CHECK(chassis_diag.motors[i].target_millirpm==sign*300000);
    }
    command(DR16_RIGHT_VERTICAL,0); command(DR16_RIGHT_HORIZONTAL,1000); run(5000);
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    for(i=0;i<4;++i) {
        int sign=(i==0 || i==3) ? -1 : 1;
        CHECK(plant_rpm[i]*sign>250.0 && plant_rpm[i]*sign<350.0);
    }
    command(DR16_RIGHT_HORIZONTAL,0); command(DR16_LEFT_HORIZONTAL,1000); run(5000);
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    for(i=0;i<4;++i) CHECK(plant_rpm[i]>165.0 && plant_rpm[i]<225.0);
    command(DR16_LEFT_HORIZONTAL,0); run(5000);
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    for(i=0;i<4;++i) CHECK(plant_rpm[i]>-3.1 && plant_rpm[i]<3.1);
    zero_output();
    remote.switch_left=DR16_SWITCH_MIDDLE; run(20);
    CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    puts("PASS forward/strafe/yaw PI signs, acceleration/current limits and neutral coast-down in a host plant");
}

static void test_diagnostic_isolation_and_wrap(void) {
    reset_at(UINT32_MAX-300u); make_ready(); arm();
    CHECK(now_ms<1000u);
    CHECK(chassis_diag.arm_count==1);
    chassis_diag.state=CHASSIS_DISABLED;
    tick(); CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    reset_at(0);
    chassis_diag.state=CHASSIS_ACTIVE;
    chassis_diag.rc_valid=1;
    chassis_diag.motors[0].command_current_raw=1500;
    CHECK(!chassis_output_enabled()); zero_output();
    tick(); CHECK(chassis_diag.state==CHASSIS_DISABLED); zero_output();
    puts("PASS clock wraparound and SWD diagnostic writes cannot arm controller");
}

int main(void) {
    test_mixer();
    test_arming_sequence();
    test_only_drive_axes_inhibit_arming();
    test_wait_for_still_wheels_reason();
    test_no_drive_from_left_vertical_or_switch_right();
    test_middle_switch_transit();
    test_remote_loss_invalid_and_rearm();
    test_motor_can_key_and_stall_faults();
    test_isolated_bad_motor_payload_is_discarded();
    test_continuous_bad_motor_payload_times_out();
    test_can_recovery_rechecks_private_safety_conditions();
    test_can_recovery_freshness_boundaries();
    test_can_recovery_requires_new_full_arm_sequence();
    test_feedback_recovery_requires_new_full_arm_sequence();
    test_feedback_recovery_safety_guards();
    test_feedback_first_then_can_recovery();
    test_later_noncommunication_fault_supersedes_communication();
    test_bad_and_old_frames_cannot_authorize();
    test_feedback_gap_not_hidden_by_recovery();
    test_closed_loop_plant();
    test_diagnostic_isolation_and_wrap();
    printf("ALL PASS: %u assertions; software tests only, no hardware validation\n",checks);
    return 0;
}
