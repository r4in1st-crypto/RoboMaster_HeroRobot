/* Native tests of the actual direct controller and DR16 decoder. Virtual
 * elapsed time and synthetic feedback are not physical board validation. */
#include "chassis_control.h"
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
static uint32_t now_ms;
static const uint16_t ids[5]={WHEEL_RR_CAN_ID,WHEEL_FR_CAN_ID,WHEEL_FL_CAN_ID,
                              WHEEL_LR_CAN_ID,WHEEL_FEEDER_CAN_ID};
#define CHECK(c) do { ++checks; if(!(c)) { \
    fprintf(stderr,"FAIL line %d: %s; now=%lu state=%lu fault=%lu\n", \
        __LINE__,#c,(unsigned long)now_ms,(unsigned long)chassis_diag.state, \
        (unsigned long)chassis_diag.fault_code); exit(1); } } while(0)
static float absf(float value) { return value<0.0f ? -value : value; }

static void reset(uint32_t start) {
    now_ms=start; chassis_init();
    CHECK(sizeof(chassis_diag_t)==396u);
    CHECK(offsetof(chassis_diag_t,motors)==96u);
    CHECK(offsetof(chassis_diag_t,rc)==296u);
    CHECK(chassis_diag.magic==CHASSIS_DIAG_MAGIC && chassis_diag.version==2u);
    CHECK(chassis_diag.size_bytes==396u && chassis_diag.state==CHASSIS_DISABLED);
    CHECK(chassis_diag.max_rotor_rpm==300u && chassis_diag.max_current_raw==16384u);
    for(unsigned i=0;i<5u;++i) CHECK(chassis_diag.motors[i].can_id==ids[i]);
}
static void packet(uint8_t bytes[18],uint32_t left,uint32_t right,
                   uint32_t ch0,uint32_t ch1,uint32_t ch2,uint32_t ch3) {
    const uint32_t raw[4]={ch0,ch1,ch2,ch3};
    memset(bytes,0,18u);
    for(unsigned channel=0;channel<4u;++channel)
        for(unsigned bit=0;bit<11u;++bit) {
            unsigned wire_bit=channel*11u+bit;
            if((raw[channel]&(1u<<bit))!=0u)
                bytes[wire_bit/8u]|=(uint8_t)(1u<<(wire_bit%8u));
        }
    bytes[5]|=(uint8_t)((left<<4)|(right<<6));
    bytes[16]=0u; bytes[17]=4u; /* Optional wheel centered at 1024. */
}
static void remote(uint32_t left,uint32_t ch0,uint32_t ch1,uint32_t ch2,uint32_t ch3) {
    uint8_t bytes[18]; dr16_frame_t decoded;
    packet(bytes,left,DR16_SWITCH_MIDDLE,ch0,ch1,ch2,ch3);
    CHECK(dr16_decode(bytes,sizeof(bytes),now_ms,&decoded)==DR16_OK);
    chassis_remote_update(&decoded);
}
static void feedback(unsigned motor,int32_t rpm,uint8_t temperature) {
    uint16_t speed=(uint16_t)rpm;
    uint8_t bytes[8]={0x12,0x34,(uint8_t)(speed>>8),(uint8_t)speed,
                       0xff,0xfe,temperature,0};
    chassis_feedback(ids[motor],bytes,now_ms);
}
static void zero_output(void) {
    int16_t out[8]; chassis_currents(out);
    for(unsigned i=0;i<8u;++i) CHECK(out[i]==0);
}
static void check_outputs(void) {
    int16_t out[8]; chassis_currents(out);
    for(unsigned i=0;i<8u;++i)
        CHECK(out[i]>=-C620_PROTOCOL_CURRENT_MAX && out[i]<=C620_PROTOCOL_CURRENT_MAX);
    for(unsigned i=4;i<8u;++i) CHECK(out[i]==0);
    for(unsigned i=0;i<4u;++i) {
        CHECK(chassis_diag.motors[i].target_millirpm>=-300000);
        CHECK(chassis_diag.motors[i].target_millirpm<=300000);
        CHECK(out[ids[i]-0x201u]==chassis_diag.motors[i].command_current_raw);
    }
    CHECK(chassis_diag.motors[4].command_current_raw==0);
    CHECK(chassis_diag.fault_code==CHASSIS_FAULT_NONE);
    CHECK(chassis_diag.state==CHASSIS_DISABLED || chassis_diag.state==CHASSIS_ACTIVE);
}
static void step(uint32_t elapsed,bool healthy,bool stop) {
    now_ms+=elapsed; chassis_step(now_ms,healthy,stop); check_outputs();
}

static void test_mapping_and_mixer(void) {
    const uint16_t expected_ids[5]={0x201,0x202,0x203,0x204,0x207};
    const float forward[4]={-300,-300,300,300};
    const float right[4]={-300,300,300,-300};
    float out[4];
    reset(0);
    for(unsigned i=0;i<5u;++i) CHECK(ids[i]==expected_ids[i]);
    chassis_mix(0,1,0,out);
    for(unsigned i=0;i<4u;++i) CHECK(absf(out[i]-forward[i])<0.001f);
    chassis_mix(1,0,0,out);
    for(unsigned i=0;i<4u;++i) CHECK(absf(out[i]-right[i])<0.001f);
    chassis_mix(0,0,1,out);
    for(unsigned i=0;i<4u;++i) CHECK(absf(out[i]-195.0f)<0.001f);
    for(int r=-4;r<=4;++r) for(int f=-4;f<=4;++f) for(int y=-4;y<=4;++y) {
        float reverse[4]; chassis_mix(r*0.25f,f*0.25f,y*0.25f,out);
        chassis_mix(-r*0.25f,-f*0.25f,-y*0.25f,reverse);
        for(unsigned i=0;i<4u;++i) {
            CHECK(absf(out[i])<=300.001f);
            CHECK(absf(out[i]+reverse[i])<0.001f);
        }
    }
    chassis_mix(1,1,1,out);
    CHECK(absf(out[2]-300.0f)<0.001f);
    CHECK(absf(out[0]/out[2]-(-1.35f/2.65f))<0.00001f);
    puts("PASS fixed RR201 FR202 FL203 LR204 mapping, forward/strafe/yaw signs and proportional mixer");
}

static void test_first_up_and_direct_switch(void) {
    reset(100);
    step(1,false,true); zero_output();
    CHECK(chassis_diag.state==CHASSIS_DISABLED);
    remote(DR16_SWITCH_UP,1024,1684,1024,1684);
    CHECK(chassis_output_enabled() && chassis_diag.state==CHASSIS_ACTIVE);
    CHECK(chassis_diag.rc_frame_count==1u && chassis_diag.rc_consecutive==1u);
    CHECK(chassis_diag.online_wheels==0u && chassis_diag.ready_since_ms==0u);
    CHECK(chassis_diag.arm_count==1u);
    step(1,false,true);
    for(unsigned i=0;i<4u;++i) {
        CHECK(chassis_diag.motors[i].feedback_count==0u);
        CHECK(chassis_diag.motors[i].target_millirpm==(i<2u ? -300000 : 300000));
        CHECK(absf((float)chassis_diag.motors[i].command_current_raw)>1500.0f);
    }
    remote(DR16_SWITCH_MIDDLE,1024,1684,1024,1024);
    CHECK(!chassis_output_enabled()); zero_output();
    CHECK(chassis_diag.disarm_count==1u);
    remote(DR16_SWITCH_UP,1024,364,1024,1024);
    CHECK(chassis_output_enabled() && chassis_diag.arm_count==2u);
    step(1,true,false);
    CHECK(chassis_diag.motors[0].target_millirpm==300000);
    CHECK(chassis_diag.motors[2].target_millirpm==-300000);
    remote(DR16_SWITCH_DOWN,1684,1684,1684,1024);
    CHECK(!chassis_output_enabled()); zero_output();
    CHECK(chassis_diag.disarm_count==2u);
    remote(DR16_SWITCH_UP,1024,1024,1024,1684);
    step(1,true,false); zero_output();
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    puts("PASS first nonneutral UP enables immediately without feedback/dwell; target has no ramp; MID/DOWN zero immediately");
}

static void test_invalid_loss_and_timestamps_keep_last(void) {
    reset(0); remote(DR16_SWITCH_UP,1684,1024,1024,1024); step(1,true,false);
    dr16_frame_t accepted=chassis_diag.rc;
    int16_t before[8], after[8]; chassis_currents(before);
    uint8_t bad[18]; dr16_frame_t decoded=accepted;
    packet(bad,DR16_SWITCH_DOWN,DR16_SWITCH_MIDDLE,0,1024,1024,1024);
    CHECK(dr16_decode(bad,sizeof(bad),now_ms,&decoded)==DR16_ERROR_CHANNEL);
    CHECK(memcmp(&decoded,&accepted,sizeof(accepted))==0);
    chassis_remote_invalid(now_ms,DR16_ERROR_CHANNEL);
    chassis_currents(after); CHECK(memcmp(before,after,sizeof(before))==0);
    chassis_remote_update(NULL);
    CHECK(chassis_diag.rc_bad_count==2u && chassis_diag.rc_valid==1u);
    CHECK(chassis_diag.rc_consecutive==0u && chassis_output_enabled());
    decoded=chassis_diag.rc; CHECK(memcmp(&decoded,&accepted,sizeof(accepted))==0);
    step(60000,false,true);
    CHECK(chassis_diag.rc_age_ms==now_ms-accepted.last_frame_ms);
    CHECK(chassis_diag.online_wheels==0u && chassis_output_enabled());
    CHECK(chassis_diag.motors[0].target_millirpm==-300000);
    CHECK(chassis_diag.motors[1].target_millirpm==300000);
    /* Timestamp age/cadence has no authority in this development variant. */
    decoded.last_frame_ms=UINT32_MAX; chassis_remote_update(&decoded);
    step(20000,false,true); CHECK(chassis_output_enabled());
    decoded.last_frame_ms=0u; chassis_remote_update(&decoded);
    chassis_remote_update(&decoded); step(0,false,true);
    CHECK(chassis_output_enabled() && chassis_diag.rc.last_frame_ms==0u);
    puts("PASS malformed/null RC, missing RC/feedback, old/duplicate timestamps, CAN-health and STOP arguments never latch or stop");
}

static void test_feedback_payload_and_protocol_endpoint(void) {
    reset(0); remote(DR16_SWITCH_UP,1024,1024,1024,1024);
    for(unsigned i=0;i<5u;++i) feedback(i,i%2u ? -32768 : 32767,255);
    step(1,false,true);
    for(unsigned i=0;i<4u;++i) {
        CHECK(chassis_diag.motors[i].temperature_c==255u);
        CHECK(chassis_diag.motors[i].command_current_raw==(i%2u ? 16384 : -16384));
    }
    int16_t out[8]; chassis_currents(out);
    CHECK((uint16_t)out[0]==0xc000u && (uint16_t)out[1]==0x4000u);
    CHECK(out[6]==0 && chassis_diag.motors[4].temperature_c==255u);
    chassis_motor_diag_t accepted[5];
    for(unsigned i=0;i<5u;++i) accepted[i]=chassis_diag.motors[i];
    const uint8_t invalid[8]={0x20,0x00,0,0,0,0,0,0};
    for(unsigned i=0;i<5u;++i) {
        chassis_feedback(ids[i],NULL,now_ms+100u);
        chassis_feedback(ids[i],invalid,now_ms+100u);
    }
    chassis_feedback(0x205u,invalid,now_ms+100u);
    for(unsigned i=0;i<5u;++i) {
        chassis_motor_diag_t after=chassis_diag.motors[i];
        CHECK(memcmp(&after,&accepted[i],sizeof(after))==0);
    }
    for(unsigned ms=0;ms<5000u;++ms) step(1,false,true);
    CHECK(chassis_output_enabled() && chassis_diag.online_wheels==0u);
    /* Saturation has not accumulated an integral opposing a recovered zero
     * error. This is PI anti-windup, without an arm/stop/fault transition. */
    for(unsigned i=0;i<4u;++i) feedback(i,0,25);
    step(1,true,false); zero_output();
    CHECK(chassis_diag.state==CHASSIS_ACTIVE);
    remote(DR16_SWITCH_UP,1024,1684,1024,1024); step(1,true,false);
    CHECK(chassis_diag.motors[0].command_current_raw< -1500);
    CHECK(chassis_diag.motors[2].command_current_raw>1500);
    puts("PASS hot/overspeed feedback remains data; malformed payloads keep last sample; signed C620 +/-16384 endpoints and PI anti-windup");
}

static void test_twenty_minutes_without_new_samples(void) {
    const uint32_t duration=20u*60u*1000u;
    const uint32_t start=UINT32_MAX-1000u;
    reset(start); remote(DR16_SWITCH_UP,1024,1684,1024,1024);
    for(uint32_t elapsed=1u;elapsed<=duration;++elapsed) {
        now_ms=start+elapsed;
        if(elapsed%1000u==0u) chassis_remote_invalid(now_ms,DR16_ERROR_LENGTH);
        chassis_step(now_ms,false,true);
        CHECK(chassis_output_enabled());
        CHECK(chassis_diag.fault_code==CHASSIS_FAULT_NONE);
        if(elapsed%1000u==0u) {
            check_outputs();
            CHECK(chassis_diag.motors[0].target_millirpm==-300000);
            CHECK(chassis_diag.motors[2].target_millirpm==300000);
            CHECK(chassis_diag.motors[0].command_current_raw< -1500);
            CHECK(chassis_diag.motors[2].command_current_raw>1500);
        }
    }
    CHECK(chassis_diag.rc_age_ms==duration && chassis_diag.rc_frame_count==1u);
    CHECK(chassis_diag.rc_bad_count==1200u && chassis_diag.online_wheels==0u);
    CHECK(chassis_diag.arm_count==1u && chassis_diag.disarm_count==0u);
    remote(DR16_SWITCH_DOWN,1024,1684,1024,1024);
    CHECK(!chassis_output_enabled()); zero_output();
    remote(DR16_SWITCH_UP,1024,1684,1024,1024);
    step(1,false,true); CHECK(chassis_output_enabled());
    puts("PASS 20 minutes at 1 ms virtual steps across uint32 wrap: one UP sample, no feedback, 1200 invalid frames, no latch; DOWN still zero");
}

static void test_diagnostic_isolation(void) {
    reset(0);
    chassis_diag.state=CHASSIS_ACTIVE;
    chassis_diag.rc_valid=1u;
    chassis_diag.motors[0].command_current_raw=16384;
    CHECK(!chassis_output_enabled()); zero_output();
    step(1,true,false); CHECK(chassis_diag.state==CHASSIS_DISABLED);
    remote(DR16_SWITCH_UP,1024,1684,1024,1024); step(1,true,false);
    chassis_diag.state=CHASSIS_FAULT;
    chassis_diag.fault_code=CHASSIS_FAULT_STOP;
    step(1,false,true); CHECK(chassis_output_enabled());
    CHECK(chassis_diag.state==CHASSIS_ACTIVE && chassis_diag.fault_code==0u);
    puts("PASS public diagnostic writes cannot change private control; ABI remains 396 bytes with version 2 semantics");
}

int main(void) {
    test_mapping_and_mixer();
    test_first_up_and_direct_switch();
    test_invalid_loss_and_timestamps_keep_last();
    test_feedback_payload_and_protocol_endpoint();
    test_twenty_minutes_without_new_samples();
    test_diagnostic_isolation();
    printf("ALL PASS: %u assertions; direct-mode host tests only, no hardware validation\n",checks);
    return 0;
}
