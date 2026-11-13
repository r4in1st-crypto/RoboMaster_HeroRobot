/* Direct-mode software observations. No BSP, hardware, or CAN adapter.
 * The rotor is an assumed first-order model, not measured machine dynamics. */
#include "chassis_control.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define CASES 7
typedef struct {
    const char *id;
    unsigned rows, checks, failures, active_rows, peak_current;
    char first_failure[100];
    unsigned final_state, final_fault;
} result_t;
static result_t results[CASES], *res;
static struct {
    unsigned ms, offset, next_rc, switch_left, rc_mode, feedback_mask;
    unsigned bad_feedback, temperature, force_rpm, can_healthy, key_stop, jam;
    int command[4];
    double rpm[4], position[4];
    int16_t applied[4], current[8];
    const char *phase;
} rig;
static FILE *raw;
static const char *job_id, *nonce;
static const unsigned ids[5]={0x201,0x202,0x203,0x204,0x207};
static int rnd(double v) {return (int)(v<0?v-0.5:v+0.5);}
static void check(int ok,const char *why) {
    ++res->checks;
    if(!ok) {++res->failures;if(!res->first_failure[0])
        snprintf(res->first_failure,sizeof(res->first_failure),"%s at %u",why,rig.ms);}
}
static void bits(uint8_t d[18],unsigned at,unsigned width,unsigned value) {
    for(unsigned i=0;i<width;++i) if(value&(1u<<i)) d[(at+i)/8]|=(uint8_t)(1u<<((at+i)%8));
}
static int remote(void) {
    if(!rig.rc_mode || rig.ms<rig.next_rc) return -1;
    uint8_t d[18]={0};dr16_frame_t decoded;
    for(unsigned i=0;i<4;++i) {
        int v=rig.command[i],delta=v?20+abs(v)*640/1000:0;
        bits(d,11*i,11,(unsigned)(1024+(v<0?-delta:delta)));
    }
    bits(d,44,2,rig.rc_mode==2?0:rig.switch_left);
    bits(d,46,2,DR16_SWITCH_MIDDLE);bits(d,128,16,1024);
    int result=(int)dr16_decode(d,18,rig.ms+rig.offset,&decoded);
    if(result==DR16_OK) chassis_remote_update(&decoded);
    else chassis_remote_invalid(rig.ms+rig.offset,(uint32_t)result);
    rig.next_rc=rig.ms+14u;
    return result;
}
static void feedback(unsigned i) {
    uint8_t d[8]={0};
    unsigned angle=i<4?(unsigned)rnd(rig.position[i])&8191u:4000u;
    int speed=i<4?(rig.force_rpm?(int)rig.force_rpm:rnd(rig.rpm[i])):0;
    if(rig.bad_feedback) angle=8192u;
    d[0]=(uint8_t)(angle>>8);d[1]=(uint8_t)angle;
    d[2]=(uint8_t)((unsigned)(uint16_t)speed>>8);d[3]=(uint8_t)speed;
    d[6]=(uint8_t)rig.temperature;
    chassis_feedback((uint16_t)ids[i],d,rig.ms+rig.offset);
}
static void tick(void) {
    for(unsigned i=0;i<4;++i) {
        if(rig.jam) rig.rpm[i]=0;
        else rig.rpm[i]+=(0.8*rig.applied[i]-rig.rpm[i])*(0.001/0.180);
        rig.position[i]+=rig.rpm[i]*8192.0/60000.0;
    }
    for(unsigned i=0;i<5;++i)
        if((rig.feedback_mask&(1u<<i)) && (i!=2 || rig.ms%8==0)) feedback(i);
    int rc_result=remote();
    chassis_step(rig.ms+rig.offset,rig.can_healthy!=0,rig.key_stop!=0);
    chassis_currents(rig.current);
    check(chassis_diag.state==CHASSIS_DISABLED || chassis_diag.state==CHASSIS_ACTIVE,"direct_states_only");
    check(chassis_diag.fault_code==0,"no_software_fault_latch");
    for(unsigned i=0;i<8;++i) {
        unsigned magnitude=(unsigned)abs(rig.current[i]);
        if(magnitude>res->peak_current) res->peak_current=magnitude;
        check(magnitude<=16384u,"C620_encoding_range");
        if(i>=4) check(rig.current[i]==0,"unused_slots_zero");
        if(chassis_diag.state!=CHASSIS_ACTIVE) check(rig.current[i]==0,"explicit_switch_zero");
    }
    for(unsigned i=0;i<4;++i) {
        check(abs(chassis_diag.motors[i].target_millirpm)<=300000,"target_300rpm_scale");
        check(isfinite(rig.rpm[i]),"finite_assumed_model");
        if(chassis_diag.state!=CHASSIS_ACTIVE) rig.applied[i]=0;
        else if(rig.ms%5u==0) rig.applied[i]=rig.current[i];
    }
    fprintf(raw,"%s,%s,%s,%u,%u,%s,%d,%u,%u,%d,%d,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u,%u",
        job_id,nonce,res->id,rig.ms,rig.ms+rig.offset,rig.phase,rc_result,rig.rc_mode,rig.switch_left,
        rig.command[0],rig.command[1],rig.command[2],rig.command[3],rig.feedback_mask,rig.bad_feedback,
        rig.temperature,rig.force_rpm,rig.can_healthy,rig.key_stop,rig.jam,
        (unsigned)chassis_diag.state,(unsigned)chassis_diag.fault_code,(unsigned)chassis_diag.rc_frame_count,
        (unsigned)chassis_diag.rc_bad_count,(unsigned)chassis_diag.rc_age_ms,(unsigned)chassis_diag.online_wheels);
    for(unsigned i=0;i<4;++i) fprintf(raw,",%d,%.6f,%d,%d,%u,%u",
        (int)chassis_diag.motors[i].target_millirpm,rig.rpm[i],(int)chassis_diag.motors[i].rotor_rpm,
        (int)rig.current[i],(unsigned)chassis_diag.motors[i].feedback_count,
        (unsigned)chassis_diag.motors[i].last_feedback_ms);
    for(unsigned i=4;i<8;++i) fprintf(raw,",%d",(int)rig.current[i]);
    fputc('\n',raw);++res->rows;res->active_rows+=chassis_diag.state==CHASSIS_ACTIVE;++rig.ms;
}
static void run(const char *phase,unsigned duration,int active) {
    rig.phase=phase;
    for(unsigned i=0;i<duration;++i) {tick();if(active>=0)check(chassis_diag.state==(active?CHASSIS_ACTIVE:CHASSIS_DISABLED),"phase_state");}
}
static void command(int right,int forward,int cw,int unused) {
    rig.command[0]=right;rig.command[1]=forward;rig.command[2]=cw;rig.command[3]=unused;
}
static void start(unsigned index,const char *id) {
    memset(&rig,0,sizeof(rig));res=&results[index];res->id=id;
    rig.rc_mode=1;rig.switch_left=DR16_SWITCH_UP;rig.feedback_mask=31;
    rig.temperature=25;rig.can_healthy=1;chassis_init();
}
static void finish(void) {
    res->final_state=chassis_diag.state;res->final_fault=chassis_diag.fault_code;
    printf("%s: %s, %u rows, %u checks, %u failures\n",res->id,res->failures?"FAIL":"PASS",res->rows,res->checks,res->failures);
}
static void scenarios(void) {
    static const int inputs[6][3]={{0,1000,0},{0,-1000,0},{1000,0,0},{-1000,0,0},{0,0,1000},{0,0,-1000}};
    static const char *phases[6]={"forward","reverse","right","left","clockwise","counterclockwise"};
    start(0,"direct_continuous_20min");
    for(unsigned n=0;n<240;++n) {unsigned j=n%6;command(inputs[j][0],inputs[j][1],inputs[j][2],0);run(phases[j],5000,1);}
    check(chassis_diag.arm_count==1 && chassis_diag.disarm_count==0,"no_timed_shutdown_in_20min");finish();

    start(1,"up_immediate_other_switch_positions_zero");
    command(0,0,0,1000);run("unused_left_vertical",20,1);
    for(unsigned i=0;i<8;++i)check(rig.current[i]==0,"unused_axis_does_not_drive");
    command(0,1000,0,0);run("up_non_neutral",300,1);
    rig.switch_left=DR16_SWITCH_MIDDLE;run("switch_middle",100,-1);
    check(chassis_diag.state==CHASSIS_DISABLED,"middle_zero");
    rig.switch_left=DR16_SWITCH_DOWN;run("switch_down",100,0);
    rig.switch_left=DR16_SWITCH_UP;run("up_again_non_neutral",200,-1);
    check(chassis_diag.state==CHASSIS_ACTIVE,"up_needs_no_neutral_or_dwell");finish();

    start(2,"rc_gap_invalid_zero_then_reverse");
    command(400,600,0,0);run("valid_drive",500,1);
    unsigned rc_count=chassis_diag.rc_frame_count;
    rig.rc_mode=0;command(-1000,0,0,0);run("rc_missing",1500,1);
    check(chassis_diag.rc_frame_count==rc_count && chassis_diag.rc_age_ms>1000,"stale_rc_still_used");
    rig.rc_mode=2;run("one_invalid_frame",1,0);
    rig.rc_mode=0;run("wait_for_new_frame",299,0);
    check(chassis_diag.rc_frame_count==rc_count && chassis_diag.rc_bad_count>0,"invalid_rc_does_not_replace_last");
    rig.rc_mode=1;run("valid_reverse_no_rearm",100,1);
    check(chassis_diag.desired_right_milli==-1000,"next_UP_applies_new_command");
    rig.switch_left=DR16_SWITCH_MIDDLE;run("valid_switch_middle",100,-1);
    check(chassis_diag.state==CHASSIS_DISABLED,"new_valid_switch_stops");finish();

    start(3,"feedback_gap_and_invalid_keep_last_valid");
    command(0,1000,0,0);run("valid_feedback",500,1);
    unsigned counts[4],times[4];int speeds[4];
    for(unsigned i=0;i<4;++i){counts[i]=chassis_diag.motors[i].feedback_count;times[i]=chassis_diag.motors[i].last_feedback_ms;speeds[i]=chassis_diag.motors[i].rotor_rpm;}
    rig.feedback_mask=0;run("feedback_missing",1500,1);
    rig.feedback_mask=31;rig.bad_feedback=1;run("invalid_feedback_frames",600,1);
    for(unsigned i=0;i<4;++i)check(counts[i]==chassis_diag.motors[i].feedback_count && times[i]==chassis_diag.motors[i].last_feedback_ms && speeds[i]==chassis_diag.motors[i].rotor_rpm,"last_feedback_retained");
    check(chassis_diag.online_wheels==0,"offline_diagnostic_is_not_a_stop");
    rig.bad_feedback=0;run("fresh_feedback_returns",200,1);finish();

    start(4,"temperature_and_overspeed_do_not_stop");
    command(0,1000,0,0);run("normal_feedback",500,1);
    rig.temperature=100;rig.force_rpm=2500;run("hot_overspeed_feedback",300,1);
    check(res->peak_current==16384u,"protocol_endpoint_exercised");finish();

    start(5,"can_key_and_tick_gap_do_not_stop");
    command(0,0,1000,0);run("normal_inputs",500,1);
    rig.can_healthy=0;rig.key_stop=1;run("can_unhealthy_and_key",800,1);
    rig.offset=5000;run("controller_clock_jump",100,1);finish();

    start(6,"stalled_model_does_not_shutdown");
    rig.jam=1;command(0,1000,0,0);run("jammed_for_three_seconds",3000,1);
    check(res->peak_current>1500u && res->peak_current<=16384u,"no_former_1500_current_cap");finish();
}
static int valid_identity(const char *s) {
    if(!s || !*s || strlen(s)>96)return 0;
    for(;*s;++s)if(!((*s>='a'&&*s<='z')||(*s>='A'&&*s<='Z')||(*s>='0'&&*s<='9')||*s=='-'||*s=='_'||*s=='.'))return 0;
    return 1;
}
int main(int argc,char **argv) {
    for(int i=1;i<argc;++i) {
        if(i+1>=argc)return 2;
        if(!strcmp(argv[i],"--job-id"))job_id=argv[++i];
        else if(!strcmp(argv[i],"--nonce"))nonce=argv[++i];else return 2;
    }
    if(!valid_identity(job_id)||!valid_identity(nonce))return 2;
    raw=fopen("raw.csv","wb");if(!raw)return 2;
    fputs("job_id,nonce,scenario,sim_ms,controller_ms,phase,rc_result,rc_mode,switch_left,input_right,input_forward,input_cw,input_unused,feedback_mask,bad_feedback,temperature,forced_rpm,can_healthy,key_stop,jammed,state,fault,rc_frames,rc_bad,rc_age_ms,online_wheels",raw);
    for(unsigned i=0;i<4;++i)fprintf(raw,",target_%x_millirpm,model_%x_rpm,feedback_%x_rpm,current_%x_raw,count_%x,last_%x_ms",ids[i],ids[i],ids[i],ids[i],ids[i],ids[i]);
    fputs(",current_205_raw,current_206_raw,current_207_raw,current_208_raw\n",raw);
    scenarios();if(fclose(raw))return 2;
    FILE *f=fopen("runner_summary.json","wb");if(!f)return 2;
    fprintf(f,"{\"schema_version\":1,\"job_id\":\"%s\",\"nonce\":\"%s\",\"software_only\":true,\"direct_mode\":true,\"scenarios\":[",job_id,nonce);
    unsigned failures=0;
    for(unsigned i=0;i<CASES;++i) {result_t *r=&results[i];failures+=r->failures;
        fprintf(f,"%s{\"id\":\"%s\",\"passed\":%s,\"rows\":%u,\"checks\":%u,\"failures\":%u,\"active_rows\":%u,\"peak_current_raw\":%u,\"final_state\":%u,\"final_fault\":%u,\"first_failure\":\"%s\"}",i?",":"",r->id,r->failures?"false":"true",r->rows,r->checks,r->failures,r->active_rows,r->peak_current,r->final_state,r->final_fault,r->first_failure);
    }
    fprintf(f,"],\"status\":\"%s\"}\n",failures?"FAIL":"PASS");if(fclose(f))return 2;return failures?1:0;
}
