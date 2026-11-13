/* Software-only closed-loop scenarios using the production parser/controller.
 * The first-order plant is an explicit assumption, not calibrated hardware.
 * No BSP, device driver, debugger, network, UART, CAN adapter, or UI is used.
 */
#include "chassis_control.h"
#include "dr16.h"
#include "can_policy.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CASES 14
#define PLANT_GAIN_RPM_PER_RAW 0.8
#define PLANT_TAU_SECONDS 0.180
#define DT_SECONDS 0.001

typedef struct {
    const char *name;
    uint32_t rows, checks, failures, active_rows, saturated_rows;
    uint32_t first_fault_ms, final_state, final_fault, final_arms;
    int peak_current;
    double peak_rpm, error_squared;
    char first_failure[128];
    int left_vertical_bias_armed;
    int uses_can_policy;
    uint32_t policy_clears, controller_clears, recovery_clear_ms;
    uint32_t recovery_elapsed_ms, recovery_zero_tx_delta;
    uint32_t feedback_clears, feedback_clear_ms;
} result_t;

static struct {
    uint32_t ms, next_rc;
    int command[4], switch_left;
    unsigned feedback_mask;
    int rc_enabled, can_healthy, jammed, bad_then_good_once;
    int feedback_temperature, forced_feedback_rpm, corrupt_203;
    double rpm[4], position[4];
    int16_t applied[4], currents[8];
    uint8_t latest_frame[18];
    uint32_t last_state;
    int32_t last_target[4];
    const char *phase;
    struct {
        int enabled, pending, queued_zero, auto_complete;
        int stuff_once, timeout_once, stuffed;
        uint32_t queued_ms, suppress_until, esr, mailbox_status, fault_flags;
        uint32_t error_generation, zero_tx_count, last_zero_tx_ms;
        int have_zero_tx, safe_off;
        can_policy_recovery_t recovery;
    } can;
} rig;
static FILE *csv;
static const char *job_id, *nonce, *left_policy="record-only";
static result_t results[CASES];
static result_t *result;
static const uint16_t ids[5]={0x201,0x202,0x203,0x204,0x207};

static double absd(double x) { return x<0 ? -x : x; }
static int rounded(double x) { return (int)(x>=0 ? x+0.5 : x-0.5); }

static void check(int condition,const char *name) {
    ++result->checks;
    if(!condition) {
        ++result->failures;
        if(!result->first_failure[0])
            snprintf(result->first_failure,sizeof(result->first_failure),
                     "%s at %lu ms",name,(unsigned long)rig.ms);
    }
}

static void bits(uint8_t frame[18],unsigned offset,unsigned width,unsigned value) {
    for(unsigned i=0;i<width;++i) {
        unsigned at=offset+i; uint8_t mask=(uint8_t)(1u<<(at%8u));
        if((value>>i)&1u) frame[at/8u]|=mask;
        else frame[at/8u]&=(uint8_t)~mask;
    }
}

static void make_remote(uint8_t frame[18]) {
    memset(frame,0,18);
    for(unsigned i=0;i<4;++i) {
        int v=rig.command[i], magnitude=v<0 ? -v : v;
        int delta=magnitude ? 20+magnitude*640/1000 : 0;
        if(v<0) delta=-delta;
        bits(frame,i*11,11,(unsigned)(1024+delta));
    }
    bits(frame,44,2,(unsigned)rig.switch_left);
    bits(frame,46,2,DR16_SWITCH_MIDDLE);
    bits(frame,128,16,1024);
}

static void deliver_remote(const uint8_t frame[18]) {
    dr16_frame_t decoded;
    dr16_result_t decoded_result=dr16_decode(frame,18,rig.ms,&decoded);
    if(decoded_result==DR16_OK) chassis_remote_update(&decoded);
    else chassis_remote_invalid(rig.ms,(uint32_t)decoded_result);
}

static void motor_feedback(unsigned i) {
    uint8_t frame[8]={0};
    uint16_t angle=i<4 ? (uint16_t)((uint32_t)rounded(rig.position[i])&8191u) : 4000u;
    uint16_t speed=i<4 ? (uint16_t)(rig.forced_feedback_rpm ? rig.forced_feedback_rpm : rounded(rig.rpm[i])) : 0u;
    uint16_t current=i<4 ? (uint16_t)rig.applied[i] : 0u;
    frame[0]=(uint8_t)(angle>>8); frame[1]=(uint8_t)angle;
    frame[2]=(uint8_t)(speed>>8); frame[3]=(uint8_t)speed;
    frame[4]=(uint8_t)(current>>8); frame[5]=(uint8_t)current;
    frame[6]=(uint8_t)rig.feedback_temperature;
    if(i==2u && rig.corrupt_203) {frame[0]=0x20u;frame[1]=0u;}
    chassis_feedback(ids[i],frame,rig.ms);
}

/* These are simulated peripheral observations, not a replacement policy.
 * Each safety classification and recovery permission calls production code. */
static void transport_service(void) {
    if(!rig.can.enabled) return;
    rig.can.mailbox_status=0;
    if(rig.can.stuff_once || rig.can.timeout_once) {
        check(!rig.can.pending,"fault_injection_mailbox_idle");
        rig.can.pending=1; rig.can.queued_ms=rig.ms; rig.can.queued_zero=0;
        if(rig.can.stuff_once) {
            rig.can.esr=UINT32_C(0x01000010); /* REC=1, LEC=stuff, no warning. */
            rig.can.mailbox_status=8u; /* TERR while request is still pending. */
            rig.can.stuffed=1; ++rig.can.error_generation; rig.can.stuff_once=0;
        }
        if(rig.can.timeout_once) {
            rig.can.suppress_until=rig.ms+4u; rig.can.timeout_once=0;
        }
    }
    if(rig.can.pending && rig.can.auto_complete && rig.ms>rig.can.queued_ms &&
       rig.ms>=rig.can.suppress_until) {
        rig.can.mailbox_status=3u; /* RQCP | TXOK after successful retry. */
        rig.can.pending=0;
        if(rig.can.stuffed) {rig.can.esr=0;rig.can.stuffed=0;}
        if(rig.can.queued_zero) {
            rig.can.zero_tx_count+=2u; rig.can.last_zero_tx_ms=rig.ms;
            rig.can.have_zero_tx=1;
        }
    }
    if(can_policy_esr_fatal(rig.can.esr)) rig.can.fault_flags|=8u;
    if(can_policy_tx_failed(rig.can.mailbox_status,false)) {
        rig.can.fault_flags|=64u; ++rig.can.error_generation;
    }
    if(rig.can.pending && can_policy_tx_expired(rig.ms,rig.can.queued_ms)) {
        rig.can.fault_flags|=128u; ++rig.can.error_generation;
        /* The board aborts pending work before sending zeros. This is an
         * intentional abort, and the production helper must not call it TERR. */
        check(!can_policy_tx_failed(9u,true),"intentional_abort_not_final_tx_error");
        rig.can.pending=0;
    }
}

static void transport_recovery(void) {
    if(!rig.can.enabled) return;
    int16_t prior[8]; chassis_currents(prior);
    bool output_zero=true;
    for(unsigned i=0;i<8;++i) if(prior[i]) output_zero=false;
    rig.can.safe_off=chassis_can_recovery_requested(rig.ms);
    can_policy_sample_t sample={
        .now_ms=rig.ms,.fault_flags=rig.can.fault_flags,.esr=rig.can.esr,
        .zero_tx_count=rig.can.zero_tx_count,.last_zero_tx_ms=rig.can.last_zero_tx_ms,
        .error_generation=rig.can.error_generation,.safe_off=rig.can.safe_off!=0,
        .output_zero=output_zero,.can_ready=true,.tx_ready=true,
        .have_zero_tx=rig.can.have_zero_tx!=0,.pending_empty=rig.can.pending==0
    };
    uint32_t started=rig.can.recovery.start_ms;
    uint32_t zero_started=rig.can.recovery.start_zero_tx_count;
    if(can_policy_recovery_step(&rig.can.recovery,&sample)) {
        ++result->policy_clears;
        result->recovery_elapsed_ms=rig.ms-started;
        result->recovery_zero_tx_delta=rig.can.zero_tx_count-zero_started;
        check(result->recovery_elapsed_ms>=CAN_POLICY_RECOVERY_MS,"production_policy_full_clean_dwell");
        check(result->recovery_zero_tx_delta>=CAN_POLICY_RECOVERY_TX_MIN,"production_policy_zero_tx_minimum");
        check(rig.can.safe_off && output_zero,"recovery_private_safe_off_and_zero");
        rig.can.fault_flags&=~CAN_POLICY_RECOVERABLE_ERRORS;
        bool cleared=chassis_clear_can_fault(rig.ms);
        check(cleared,"controller_accepts_qualified_clear");
        if(cleared) {
            ++result->controller_clears; result->recovery_clear_ms=rig.ms;
            check(chassis_diag.state==CHASSIS_DISABLED && !chassis_output_enabled(),"clear_does_not_arm");
        }
    }
}

static void transport_queue(void) {
    if(!rig.can.enabled || rig.can.pending || rig.ms%5u) return;
    rig.can.pending=1; rig.can.queued_ms=rig.ms; rig.can.queued_zero=1;
    for(unsigned i=0;i<4;++i) if(rig.currents[i]) rig.can.queued_zero=0;
}

static void record(int sent,int bad_injected) {
    fprintf(csv,"%s,%s,%s,%lu,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%u,%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
            job_id,nonce,result->name,(unsigned long)rig.ms,rig.phase,
            sent,rig.rc_enabled,bad_injected,rig.switch_left,
            rig.command[0],rig.command[1],rig.command[2],rig.command[3],
            rig.can_healthy,rig.feedback_mask,rig.jammed,
            (unsigned long)chassis_diag.state,(unsigned long)chassis_diag.fault_code,
            (unsigned long)chassis_diag.stop_reason,(unsigned long)chassis_diag.rc_valid,
            (unsigned long)chassis_diag.rc_age_ms,(unsigned long)chassis_diag.rc_consecutive,
            (unsigned long)chassis_diag.arm_count,(unsigned long)chassis_diag.disarm_count);
    for(unsigned i=0;i<4;++i)
        fprintf(csv,",%ld,%.6f,%ld,%d,%d,%lu",
                (long)chassis_diag.motors[i].target_millirpm,rig.rpm[i],
                (long)chassis_diag.motors[i].rotor_rpm,rig.currents[i],rig.applied[i],
                (unsigned long)chassis_diag.motors[i].feedback_count);
    for(unsigned i=4;i<8;++i) fprintf(csv,",%d",rig.currents[i]);
    fprintf(csv,",%d,%lu,%lu,%d,%lu,%lu,%lu,%lu,%lu,%d,%d,%lu,%lu,%lu,%lu,%lu",
            rig.can.enabled,(unsigned long)rig.can.esr,(unsigned long)rig.can.mailbox_status,
            rig.can.pending,(unsigned long)(rig.can.pending ? rig.ms-rig.can.queued_ms : 0u),
            (unsigned long)rig.can.fault_flags,(unsigned long)rig.can.error_generation,
            (unsigned long)rig.can.zero_tx_count,(unsigned long)rig.can.last_zero_tx_ms,
            rig.can.safe_off,rig.can.recovery.qualifying,
            (unsigned long)(rig.can.recovery.qualifying ? rig.ms-rig.can.recovery.start_ms : 0u),
            (unsigned long)result->policy_clears,(unsigned long)result->controller_clears,
            (unsigned long)result->recovery_elapsed_ms,(unsigned long)result->recovery_zero_tx_delta);
    fprintf(csv,",%d,%d,%d,%lu,%lu\n",rig.feedback_temperature,rig.forced_feedback_rpm,
            rig.corrupt_203,(unsigned long)result->feedback_clears,
            (unsigned long)result->feedback_clear_ms);
    ++result->rows;
}

static void tick(void) {
    int sent=0,bad_injected=0;
    for(unsigned i=0;i<4;++i) {
        if(rig.jammed) rig.rpm[i]=0;
        else rig.rpm[i]+=(PLANT_GAIN_RPM_PER_RAW*rig.applied[i]-rig.rpm[i])*
                         DT_SECONDS/PLANT_TAU_SECONDS;
        rig.position[i]+=rig.rpm[i]*8192.0/60000.0;
    }
    for(unsigned i=0;i<5;++i)
        if((rig.feedback_mask&(1u<<i)) && (i!=2 || rig.ms%8u==0u)) motor_feedback(i);
    if(rig.rc_enabled && rig.ms>=rig.next_rc) {
        make_remote(rig.latest_frame);
        if(rig.bad_then_good_once) {
            uint8_t corrupted[18]; memcpy(corrupted,rig.latest_frame,18);
            corrupted[5]&=0xcfu; /* left switch zero is invalid */
            deliver_remote(corrupted); bad_injected=1; rig.bad_then_good_once=0;
        }
        deliver_remote(rig.latest_frame); sent=1; rig.next_rc=rig.ms+14u;
    }
    transport_service();
    transport_recovery();
    bool feedback_fault_before_step=chassis_diag.state==CHASSIS_FAULT &&
                                    chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK;
    chassis_step(rig.ms,rig.can_healthy!=0 && (!rig.can.enabled || rig.can.fault_flags==0u),false);
    if(feedback_fault_before_step && chassis_diag.state==CHASSIS_DISABLED) {
        ++result->feedback_clears;result->feedback_clear_ms=rig.ms;
        check(chassis_diag.fault_code==CHASSIS_FAULT_NONE,"feedback_clear_has_no_fault");
    }
    chassis_currents(rig.currents);
    for(unsigned i=0;i<8;++i) {
        int magnitude=rig.currents[i]<0 ? -rig.currents[i] : rig.currents[i];
        check(magnitude<=CHASSIS_MAX_CURRENT_RAW,"current_limit");
        if(magnitude>result->peak_current) result->peak_current=magnitude;
        if(i>=4) check(rig.currents[i]==0,"unused_and_feeder_zero");
        if(chassis_diag.state!=CHASSIS_ACTIVE) check(rig.currents[i]==0,"disabled_zero");
    }
    int saturated=0;
    for(unsigned i=0;i<4;++i) {
        int32_t target=chassis_diag.motors[i].target_millirpm;
        check(target>=-CHASSIS_MAX_ROTOR_RPM*1000 && target<=CHASSIS_MAX_ROTOR_RPM*1000,"target_limit");
        if(chassis_diag.state==CHASSIS_ACTIVE && rig.last_state==CHASSIS_ACTIVE) {
            int32_t delta=target-rig.last_target[i];
            check(delta>=-301 && delta<=301,"target_acceleration_limit");
        }
        rig.last_target[i]=target;
        double magnitude=absd(rig.rpm[i]);
        if(magnitude>result->peak_rpm) result->peak_rpm=magnitude;
        double error=(double)target/1000.0-rig.rpm[i];
        result->error_squared+=error*error;
        if(absd(rig.currents[i])==CHASSIS_MAX_CURRENT_RAW) saturated=1;
        /* Commands are held for the physical design's 5 ms TX interval;
         * disabling output immediately replaces the hold with zeros. */
        if(chassis_diag.state!=CHASSIS_ACTIVE) rig.applied[i]=0;
        else if(rig.ms%5u==0u) rig.applied[i]=rig.currents[i];
    }
    result->saturated_rows+=(uint32_t)saturated;
    result->active_rows+=(chassis_diag.state==CHASSIS_ACTIVE);
    if(chassis_diag.state==CHASSIS_FAULT && result->first_fault_ms==UINT32_MAX)
        result->first_fault_ms=rig.ms;
    rig.last_state=chassis_diag.state;
    transport_queue();
    record(sent,bad_injected); ++rig.ms;
}

static void command(int right,int forward,int yaw,int unused_left_vertical) {
    rig.command[0]=right; rig.command[1]=forward;
    rig.command[2]=yaw; rig.command[3]=unused_left_vertical;
}
static void run(const char *phase,unsigned duration,int must_be_active) {
    rig.phase=phase;
    for(unsigned i=0;i<duration;++i) {
        tick();
        if(must_be_active) check(chassis_diag.state==CHASSIS_ACTIVE,"continuous_active");
    }
}
static void begin_case(unsigned index,const char *name) {
    memset(&rig,0,sizeof(rig)); memset(&results[index],0,sizeof(results[index]));
    result=&results[index]; result->name=name; result->first_fault_ms=UINT32_MAX;
    result->recovery_clear_ms=UINT32_MAX;
    result->feedback_clear_ms=UINT32_MAX;
    result->left_vertical_bias_armed=-1;
    rig.feedback_mask=31; rig.rc_enabled=rig.can_healthy=1;
    rig.feedback_temperature=25;
    rig.switch_left=DR16_SWITCH_DOWN;
    for(unsigned i=0;i<4;++i) rig.position[i]=1000.0+i*1000.0;
    chassis_init();
}
static void enable_policy(void) {
    rig.can.enabled=1;rig.can.auto_complete=1;result->uses_can_policy=1;
    can_policy_recovery_reset(&rig.can.recovery);
}
static void finish_case(void) {
    result->final_state=chassis_diag.state; result->final_fault=chassis_diag.fault_code;
    result->final_arms=chassis_diag.arm_count;
    printf("%s: %s (%lu rows, %lu checks, %lu failures)\n",result->name,
           result->failures ? "FAIL" : "PASS",(unsigned long)result->rows,
           (unsigned long)result->checks,(unsigned long)result->failures);
}
static void arm_fixture(void) {
    command(0,0,0,0); rig.switch_left=DR16_SWITCH_DOWN;
    run("off_neutral",700,0); check(chassis_diag.state==CHASSIS_READY,"ready_after_off_dwell");
    rig.switch_left=DR16_SWITCH_MIDDLE; run("middle_transit",80,0);
    check(chassis_diag.state==CHASSIS_READY,"middle_keeps_qualification");
    rig.switch_left=DR16_SWITCH_UP; run("arm_neutral",100,0);
    check(chassis_diag.state==CHASSIS_ACTIVE,"explicit_arm");
}
static void confirm_latched(uint32_t expected) {
    check(chassis_diag.state==CHASSIS_FAULT,"expected_hard_fault");
    check(chassis_diag.fault_code==expected,"expected_fault_reason");
    rig.can_healthy=rig.rc_enabled=1; rig.feedback_mask=31; rig.jammed=0;
    command(0,0,0,0); rig.switch_left=DR16_SWITCH_DOWN;
    run("recover_off_attempt",1300,0); rig.switch_left=DR16_SWITCH_UP;
    run("recover_up_attempt",200,0);
    check(chassis_diag.state==CHASSIS_FAULT,"hard_fault_remains_latched");
}

static void scenarios(void) {
    static const int moves[6][3]={{0,1000,0},{0,-1000,0},{1000,0,0},
                                {-1000,0,0},{0,0,1000},{0,0,-1000}};
    static const char *names[6]={"forward","reverse","right","left","clockwise","counterclockwise"};
    begin_case(0,"continuous_switching_20min"); arm_fixture();
    /* 240 x 5 seconds = 20 minutes of simulated time, with production control
     * called every millisecond. This is not 20 minutes of physical testing. */
    for(unsigned phase=0;phase<240;++phase) {
        unsigned p=phase%6; command(moves[p][0],moves[p][1],moves[p][2],0);
        run(names[p],5000,1);
        float expected[4]; chassis_mix((float)moves[p][0]/1000.0f,
                                      (float)moves[p][1]/1000.0f,
                                      (float)moves[p][2]/1000.0f,expected);
        for(unsigned i=0;i<4;++i) check(rig.rpm[i]*expected[i]>0,"phase_end_rotation_sign");
    }
    check(chassis_diag.arm_count==1,"no_spontaneous_rearm");
    command(0,0,0,0); run("neutral_settle",2000,1);
    rig.switch_left=DR16_SWITCH_MIDDLE; run("explicit_stop",30,0);
    check(chassis_diag.state==CHASSIS_DISABLED,"switch_stop"); finish_case();

    begin_case(1,"rc_loss_and_explicit_rearm"); arm_fixture();
    command(0,1000,0,0); run("drive",1200,1);
    rig.rc_enabled=0; run("rc_missing",100,0);
    check(chassis_diag.state==CHASSIS_DISABLED,"rc_loss_disables");
    check(chassis_diag.fault_code==CHASSIS_FAULT_NONE,"rc_loss_not_hard_fault");
    rig.rc_enabled=1; command(0,0,0,0); run("reconnected_up",1200,0);
    check(chassis_diag.state==CHASSIS_DISABLED && chassis_diag.arm_count==1,"no_auto_resume_up");
    rig.switch_left=DR16_SWITCH_DOWN; run("explicit_off_neutral",1200,0);
    check(chassis_diag.state==CHASSIS_READY,"rearm_ready");
    rig.switch_left=DR16_SWITCH_UP; run("explicit_up_neutral",100,0);
    check(chassis_diag.state==CHASSIS_ACTIVE && chassis_diag.arm_count==2,"explicit_rearm");
    command(0,-1000,0,0); run("drive_after_rearm",1200,1); finish_case();

    begin_case(2,"invalid_then_valid_same_tick"); arm_fixture();
    command(1000,0,0,0); run("drive",1000,1);
    rig.bad_then_good_once=1; run("invalid_then_valid",30,0);
    check(chassis_diag.rc_bad_count>=1,"invalid_was_seen");
    check(chassis_diag.state==CHASSIS_DISABLED,"same_tick_valid_cannot_rearm");
    command(0,0,0,0); run("valid_up_recovery",1200,0);
    check(chassis_diag.state==CHASSIS_DISABLED,"invalid_recovery_stays_disabled");
    rig.switch_left=DR16_SWITCH_DOWN; run("explicit_off_neutral",1200,0);
    rig.switch_left=DR16_SWITCH_UP; run("explicit_up_neutral",100,0);
    check(chassis_diag.state==CHASSIS_ACTIVE && chassis_diag.arm_count==2,"invalid_then_off_rearms");
    command(-1000,0,0,0); run("drive_after_rearm",1000,1); finish_case();

    begin_case(3,"motor_203_feedback_loss"); arm_fixture();
    command(0,1000,0,0); run("drive",1000,1);
    rig.feedback_mask&=~4u; uint32_t drop_at=rig.ms;
    run("id203_missing",60,0);
    check(result->first_fault_ms>=drop_at && result->first_fault_ms-drop_at<=31u,"motor_loss_deadline");
    check(chassis_diag.fault_can_id==0x203u,"lost_motor_identified");
    command(0,0,0,0);rig.switch_left=DR16_SWITCH_DOWN;
    run("missing_feedback_down_attempt",1300,0);
    rig.switch_left=DR16_SWITCH_UP;run("missing_feedback_up_attempt",200,0);
    check(chassis_diag.state==CHASSIS_FAULT && result->feedback_clears==0,
          "continued_missing_feedback_cannot_recover");finish_case();

    begin_case(4,"can_unhealthy"); arm_fixture();
    command(0,0,1000,0); run("drive",1000,1);
    rig.can_healthy=0; run("can_error",1,0);
    confirm_latched(CHASSIS_FAULT_CAN); finish_case();

    begin_case(5,"jam_and_current_limit"); arm_fixture();
    rig.jammed=1; command(0,1000,0,0); uint32_t jam_at=rig.ms;
    run("jammed_wheels",2200,0);
    check(result->peak_current==CHASSIS_MAX_CURRENT_RAW,"current_limit_was_exercised");
    check(result->saturated_rows>0,"saturation_recorded");
    check(result->first_fault_ms>=jam_at && result->first_fault_ms-jam_at<=2000u,"stall_deadline");
    confirm_latched(CHASSIS_FAULT_STALL); finish_case();

    begin_case(6,"unused_left_vertical_bias");
    command(0,0,0,100); run("off_with_unused_bias",900,0);
    rig.switch_left=DR16_SWITCH_UP; run("up_with_unused_bias",150,0);
    result->left_vertical_bias_armed=chassis_diag.state==CHASSIS_ACTIVE;
    if(!strcmp(left_policy,"ignore")) check(result->left_vertical_bias_armed,"unused_axis_ignored_for_arm");
    else if(!strcmp(left_policy,"require-neutral")) check(!result->left_vertical_bias_armed,"unused_axis_required_neutral");
    for(unsigned i=0;i<8;++i) check(rig.currents[i]==0,"unused_axis_never_drives");
    finish_case();

    begin_case(7,"can_transient_stuff_retry"); enable_policy(); arm_fixture();
    command(0,1000,0,0); run("drive_before_stuff",1000,1);
    rig.can.stuff_once=1; run("stuff_pending_retry",1,1);
    check(rig.can.esr==UINT32_C(0x01000010),"stuff_snapshot_exercised");
    check(rig.can.pending && rig.can.fault_flags==0u,"pending_retry_is_not_failure");
    run("retry_success_1ms",1,1);
    check(rig.can.mailbox_status==3u && rig.can.fault_flags==0u,"retry_txok_accepted");
    run("drive_after_retry",2000,1);
    check(chassis_diag.arm_count==1 && chassis_diag.disarm_count==0,"transient_does_not_interrupt");
    finish_case();

    begin_case(8,"can_pending_timeout_explicit_recovery"); enable_policy(); arm_fixture();
    command(1000,0,0,0); run("drive_before_timeout",1000,1);
    uint32_t queued_at=rig.ms; rig.can.timeout_once=1;
    run("pending_below_deadline",3,1);
    run("pending_reaches_3ms",1,0);
    check(chassis_diag.state==CHASSIS_FAULT && chassis_diag.fault_code==CHASSIS_FAULT_CAN,"3ms_pending_stops");
    check(result->first_fault_ms==queued_at+3u,"exact_production_pending_deadline");
    command(0,0,0,0); run("link_clean_but_up",1800,0);
    check(chassis_diag.state==CHASSIS_FAULT && result->policy_clears==0,"healthy_up_never_clears");
    rig.switch_left=DR16_SWITCH_DOWN; rig.phase="down_link_qualification";
    unsigned waited=0;
    while(!result->controller_clears && waited++<2500u) tick();
    check(result->controller_clears==1 && result->policy_clears==1,"qualified_policy_and_controller_clear");
    check(chassis_diag.state==CHASSIS_DISABLED,"recovery_returns_disabled");
    uint32_t clear_ms=result->recovery_clear_ms;
    while(rig.ms-clear_ms<500u) {rig.phase="fresh_down_dwell";tick();check(chassis_diag.state!=CHASSIS_ACTIVE,"no_auto_arm_after_clear");}
    run("down_dwell_complete",20,0);
    check(chassis_diag.state==CHASSIS_READY,"new_500ms_down_required");
    rig.switch_left=DR16_SWITCH_UP;run("explicit_up_after_clear",100,0);
    check(chassis_diag.state==CHASSIS_ACTIVE && chassis_diag.arm_count==2,"explicit_rearm_after_policy_clear");
    command(-1000,0,0,0);run("drive_after_can_recovery",1500,1);finish_case();

    begin_case(9,"can_busoff_still_set_rejects_recovery"); enable_policy(); arm_fixture();
    command(0,0,1000,0);run("drive",1000,1);
    rig.can.esr=4u;rig.can.auto_complete=0;++rig.can.error_generation;
    run("busoff",10,0);command(0,0,0,0);rig.switch_left=DR16_SWITCH_DOWN;
    run("safe_down_but_busoff",2300,0);
    check(chassis_diag.state==CHASSIS_FAULT && result->policy_clears==0,"busoff_blocks_production_recovery");finish_case();

    begin_case(10,"can_nonrecoverable_flag_rejects_clear");enable_policy();arm_fixture();
    command(0,1000,0,0);run("drive",1000,1);
    rig.can.fault_flags=16u; /* RX overrun: outside the recoverable mask. */
    ++rig.can.error_generation;run("nonrecoverable_fault",1,0);
    command(0,0,0,0);rig.switch_left=DR16_SWITCH_DOWN;run("clean_zeros_but_nonrecoverable",2300,0);
    check(chassis_can_recovery_requested(rig.ms),"controller_safeoff_is_available");
    check(chassis_diag.state==CHASSIS_FAULT && result->policy_clears==0,"nonrecoverable_flag_blocks_clear");finish_case();

    begin_case(11,"feedback_reconnect_requires_safe_down");arm_fixture();
    command(0,1000,0,0);run("drive",1000,1);
    rig.feedback_mask&=~4u;run("id203_missing",60,0);
    check(chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK,"feedback_timeout_exercised");
    rig.feedback_mask=31;command(0,0,0,0);run("fresh_feedback_but_up",1500,0);
    check(chassis_diag.state==CHASSIS_FAULT,"reconnect_up_cannot_clear");
    rig.switch_left=DR16_SWITCH_MIDDLE;run("fresh_feedback_but_mid",100,0);
    check(chassis_diag.state==CHASSIS_FAULT,"mid_cannot_clear");
    command(300,0,0,0);rig.switch_left=DR16_SWITCH_DOWN;run("down_non_neutral",100,0);
    check(chassis_diag.state==CHASSIS_FAULT,"non_neutral_cannot_clear");
    rig.forced_feedback_rpm=20;command(0,0,0,0);run("down_wheels_moving",100,0);
    check(chassis_diag.state==CHASSIS_FAULT,"moving_cannot_clear");
    rig.rc_enabled=0;run("moving_wait_rc_stale",60,0);
    rig.forced_feedback_rpm=0;run("down_rc_stale",100,0);
    check(chassis_diag.state==CHASSIS_FAULT,"stale_rc_cannot_clear");
    rig.rc_enabled=1;rig.phase="safe_down_feedback_clear";
    unsigned feedback_wait=0;
    while(!result->feedback_clears && feedback_wait++<300u) tick();
    check(result->feedback_clears==1 && chassis_diag.state==CHASSIS_DISABLED,"safe_feedback_clear_disabled");
    uint32_t feedback_clear=result->feedback_clear_ms;
    while(rig.ms-feedback_clear<500u) {rig.phase="feedback_fresh_down_dwell";tick();
        check(chassis_diag.state==CHASSIS_DISABLED,"feedback_clear_requires_new_dwell");}
    run("feedback_down_dwell_complete",20,0);
    check(chassis_diag.state==CHASSIS_READY,"feedback_fresh_dwell_ready");
    rig.switch_left=DR16_SWITCH_UP;run("feedback_explicit_up",100,0);
    check(chassis_diag.state==CHASSIS_ACTIVE && chassis_diag.arm_count==2,"feedback_explicit_rearm");
    command(0,-1000,0,0);run("drive_after_feedback_recovery",1200,1);finish_case();

    begin_case(12,"feedback_first_then_can_recovery");enable_policy();arm_fixture();
    command(1000,0,0,0);run("drive",1000,1);
    rig.feedback_mask&=~4u;run("id203_missing",60,0);
    check(chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK,"feedback_is_first_fault");
    rig.can.timeout_once=1;run("feedback_then_pending_timeout",4,0);
    check((rig.can.fault_flags&128u) && chassis_diag.fault_code==CHASSIS_FAULT_FEEDBACK,
          "first_feedback_fault_preserved_after_can_error");
    rig.feedback_mask=31;command(0,0,0,0);run("feedback_can_recovered_but_up",1600,0);
    check(chassis_diag.state==CHASSIS_FAULT && !result->controller_clears,"combined_up_cannot_clear");
    rig.switch_left=DR16_SWITCH_DOWN;rig.phase="feedback_can_clean_down";
    unsigned combined_wait=0;
    while(!result->controller_clears && combined_wait++<2500u) tick();
    check(result->policy_clears==1 && result->controller_clears==1 && result->feedback_clears==0,
          "combined_clear_requires_production_can_policy");
    check(chassis_diag.state==CHASSIS_DISABLED,"combined_clear_disabled");
    uint32_t combined_clear=result->recovery_clear_ms;
    while(rig.ms-combined_clear<500u) {rig.phase="combined_fresh_down_dwell";tick();
        check(chassis_diag.state==CHASSIS_DISABLED,"combined_clear_new_dwell");}
    run("combined_down_dwell_complete",20,0);
    check(chassis_diag.state==CHASSIS_READY,"combined_fresh_dwell_ready");
    rig.switch_left=DR16_SWITCH_UP;run("combined_explicit_up",100,0);
    check(chassis_diag.state==CHASSIS_ACTIVE && chassis_diag.arm_count==2,"combined_explicit_rearm");
    command(-1000,0,0,0);run("drive_after_combined_recovery",1200,1);finish_case();

    begin_case(13,"isolated_bad_motor_frame_is_discarded");arm_fixture();
    command(0,1000,0,0);run("drive",1000,1);
    uint32_t good_count=chassis_diag.motors[2].feedback_count;
    uint32_t good_time=chassis_diag.motors[2].last_feedback_ms;
    rig.corrupt_203=1;run("isolated_invalid_payload",1,1);
    check(chassis_diag.motors[2].feedback_count==good_count &&
          chassis_diag.motors[2].last_feedback_ms==good_time,"bad_payload_is_not_a_measurement");
    rig.corrupt_203=0;run("good_frames_after_bad_payload",1000,1);
    check(chassis_diag.fault_code==CHASSIS_FAULT_NONE && chassis_diag.arm_count==1 &&
          chassis_diag.disarm_count==0,"bad_payload_does_not_interrupt");finish_case();
}

static int identity_valid(const char *s) {
    size_t length=strlen(s); if(length==0 || length>96) return 0;
    for(size_t i=0;i<length;++i) {
        char c=s[i];
        if(!((c>='a'&&c<='z')||(c>='A'&&c<='Z')||(c>='0'&&c<='9')||c=='-'||c=='_'||c=='.')) return 0;
    }
    return 1;
}
static int write_summary(void) {
    FILE *f=fopen("runner_summary.json","wb"); if(!f) return 2;
    unsigned total_failures=0;
    fprintf(f,"{\n  \"schema_version\":1,\n  \"job_id\":\"%s\",\n  \"nonce\":\"%s\",\n",job_id,nonce);
    fprintf(f,"  \"software_only\":true,\n  \"left_vertical_policy\":\"%s\",\n",left_policy);
    fprintf(f,"  \"limits\":{\"max_current_raw\":%d,\"max_target_rotor_rpm\":%d,\"active_state\":%u},\n",
            CHASSIS_MAX_CURRENT_RAW,CHASSIS_MAX_ROTOR_RPM,CHASSIS_ACTIVE);
    fprintf(f,"  \"model\":{\"kind\":\"assumed_first_order_rotor\",\"gain_rpm_per_raw\":%.6f,\"tau_ms\":%.3f,\"dt_ms\":1,\"can_command_hold_ms\":5,\"rc_frame_ms\":14,\"motor_feedback_ms\":[1,1,8,1,1]},\n",
            PLANT_GAIN_RPM_PER_RAW,PLANT_TAU_SECONDS*1000.0);
    fprintf(f,"  \"scenarios\":[\n");
    for(unsigned i=0;i<CASES;++i) {
        result_t *r=&results[i]; total_failures+=r->failures;
        fprintf(f,"    {\"id\":\"%s\",\"passed\":%s,\"rows\":%lu,\"checks\":%lu,\"failures\":%lu,\"active_rows\":%lu,\"saturated_rows\":%lu,\"peak_current_raw\":%d,\"peak_model_rpm\":%.6f,\"model_tracking_rmse_rpm\":%.6f,\"final_state\":%lu,\"final_fault\":%lu,\"arm_count\":%lu,\"first_fault_ms\":",
                r->name,r->failures?"false":"true",(unsigned long)r->rows,
                (unsigned long)r->checks,(unsigned long)r->failures,(unsigned long)r->active_rows,
                (unsigned long)r->saturated_rows,r->peak_current,r->peak_rpm,
                sqrt(r->error_squared/((double)r->rows*4.0)),
                (unsigned long)r->final_state,(unsigned long)r->final_fault,(unsigned long)r->final_arms);
        if(r->first_fault_ms==UINT32_MAX) fputs("null",f); else fprintf(f,"%lu",(unsigned long)r->first_fault_ms);
        fprintf(f,",\"uses_production_can_policy\":%s,\"policy_clears\":%lu,\"controller_clears\":%lu,\"recovery_elapsed_ms\":%lu,\"recovery_zero_tx_delta\":%lu,\"recovery_clear_ms\":",
                r->uses_can_policy?"true":"false",(unsigned long)r->policy_clears,
                (unsigned long)r->controller_clears,(unsigned long)r->recovery_elapsed_ms,
                (unsigned long)r->recovery_zero_tx_delta);
        if(r->recovery_clear_ms==UINT32_MAX) fputs("null",f);else fprintf(f,"%lu",(unsigned long)r->recovery_clear_ms);
        fprintf(f,",\"feedback_clears\":%lu,\"feedback_clear_ms\":",(unsigned long)r->feedback_clears);
        if(r->feedback_clear_ms==UINT32_MAX) fputs("null",f);else fprintf(f,"%lu",(unsigned long)r->feedback_clear_ms);
        fprintf(f,",\"first_failure\":\"%s\",\"left_vertical_bias_armed\":",r->first_failure);
        if(r->left_vertical_bias_armed<0) fputs("null",f); else fputs(r->left_vertical_bias_armed?"true":"false",f);
        fprintf(f,"}%s\n",i+1<CASES?",":"");
    }
    fprintf(f,"  ],\n  \"status\":\"%s\"\n}\n",total_failures?"FAIL":"PASS");
    if(fclose(f)) return 2;
    return total_failures ? 1 : 0;
}

int main(int argc,char **argv) {
    for(int i=1;i<argc;++i) {
        if(i+1>=argc) {fprintf(stderr,"Missing option value\n");return 2;}
        if(!strcmp(argv[i],"--job-id")) job_id=argv[++i];
        else if(!strcmp(argv[i],"--nonce")) nonce=argv[++i];
        else if(!strcmp(argv[i],"--left-vertical-policy")) left_policy=argv[++i];
        else {fprintf(stderr,"Unknown argument\n");return 2;}
    }
    if(!job_id||!nonce||!identity_valid(job_id)||!identity_valid(nonce) ||
       (strcmp(left_policy,"record-only") && strcmp(left_policy,"ignore") && strcmp(left_policy,"require-neutral"))) {
        fprintf(stderr,"Provide ASCII --job-id and --nonce; policy record-only/ignore/require-neutral\n");return 2;
    }
    csv=fopen("raw.csv","wb"); if(!csv) {perror("raw.csv");return 2;}
    fputs("job_id,nonce,scenario,sim_ms,phase,rc_packet_sent,rc_enabled,invalid_injected,switch_left,command_right_milli,command_forward_milli,command_cw_milli,unused_left_vertical_milli,can_healthy,feedback_mask,jammed,state,fault,stop_reason,rc_valid,rc_age_ms,rc_consecutive,arm_count,disarm_count",csv);
    for(unsigned i=0;i<4;++i) fprintf(csv,",target_%x_millirpm,model_%x_rpm,feedback_%x_rpm,command_%x_raw,applied_%x_raw,feedback_%x_count",ids[i],ids[i],ids[i],ids[i],ids[i],ids[i]);
    fputs(",command_205_raw,command_206_raw,command_207_raw,command_208_raw,policy_enabled,policy_esr,policy_mailbox_status,policy_pending,policy_pending_age_ms,policy_fault_flags,policy_error_generation,policy_zero_tx_count,policy_last_zero_tx_ms,policy_safe_off,policy_qualifying,policy_qualifying_ms,policy_clear_count,controller_clear_count,policy_clear_elapsed_ms,policy_clear_zero_tx_delta,feedback_temperature_c,forced_feedback_rpm,corrupt_203,feedback_clear_count,feedback_clear_ms\n",csv);
    scenarios();
    if(fclose(csv)) {perror("CSV close");return 2;}
    return write_summary();
}
