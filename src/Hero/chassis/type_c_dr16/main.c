#include "platform.h"
#include "chassis_control.h"
#include "dr16.h"

volatile uint32_t g_snapshot_request;
volatile uint32_t g_snapshot_ack;
volatile platform_diag_t g_platform_snapshot;
volatile chassis_diag_t g_chassis_snapshot;

static void service_snapshot(void)
{
    uint32_t token = g_snapshot_request;
    if (token != g_snapshot_ack) {
        g_platform_snapshot = g_platform_diag;
        g_chassis_snapshot = chassis_diag;
        __asm volatile("dmb" ::: "memory");
        g_snapshot_ack = token;
    }
}

/* Returns true when a newly received frame/error requests immediate zero. */
static bool service_remote(void)
{
    bool force_zero = false;
    uint32_t errors = platform_rc_take_errors();
    if (errors != 0u) {
        chassis_remote_invalid(platform_millis(), UINT32_C(0x10000) | errors);
        force_zero = true;
    }
    uint8_t bytes[DR16_FRAME_BYTES];
    uint32_t received_ms;
    while (platform_rc_pop(bytes, &received_ms)) {
        dr16_frame_t frame;
        dr16_result_t result = dr16_decode(bytes, sizeof(bytes), received_ms, &frame);
        if (result == DR16_OK) {
            chassis_remote_update(&frame);
            if (frame.switch_left != DR16_SWITCH_UP) { force_zero = true; }
        } else {
            chassis_remote_invalid(platform_millis(), (uint32_t)result);
            force_zero = true;
        }
    }
    /* Catch an IRQ error which arrived while the preceding frame was decoded. */
    errors = platform_rc_take_errors();
    if (errors != 0u) {
        chassis_remote_invalid(platform_millis(), UINT32_C(0x10000) | errors);
        force_zero = true;
    }
    return force_zero;
}

int main(void)
{
    chassis_init();
    if (!platform_init()) { platform_panic(g_platform_diag.error_flags); }
    uint32_t last_step = platform_millis();
    bool was_enabled = false;
    bool was_unsafe = false;
    platform_emergency_zero();

    for (;;) {
        ++g_platform_diag.seq;
        __asm volatile("dmb" ::: "memory");
        ++g_platform_diag.main_loops;
        bool stop = platform_stop_requested();
        platform_service();
        bool rc_stop = service_remote();
        stop = platform_stop_requested() || stop;
        uint32_t now = platform_millis();
        uint32_t gap = (uint32_t)(now - last_step);
        if (gap > g_platform_diag.max_step_gap_ms) {
            g_platform_diag.max_step_gap_ms = gap;
        }
        if (gap > 5u) { platform_latch_error(PLATFORM_ERR_STEP_DELAY); }
        /* Clearing a recovered CAN latch never arms the chassis. The platform
         * requires sustained successful zero-current traffic and an explicit
         * safe DOWN request; the controller then restarts the normal dwell. */
        if (platform_try_can_recovery(now,
                !stop && chassis_can_recovery_requested(now))) {
            (void)chassis_clear_can_fault(now);
        }
        bool healthy = platform_healthy();
        bool unsafe = !healthy || stop;
        bool control_stopped = was_enabled && !chassis_output_enabled();
        bool step_due = now != last_step;
        if (step_due || rc_stop || control_stopped || (unsafe && !was_unsafe)) {
            chassis_step(now, healthy, stop);
            bool enabled = chassis_output_enabled();
            int16_t currents[8];
            chassis_currents(currents);
            if ((was_enabled && (rc_stop || unsafe || !enabled)) ||
                (unsafe && !was_unsafe)) {
                platform_emergency_zero();
            } else if (step_due) {
                platform_send_currents(currents, enabled);
            }
            was_enabled = enabled;
            last_step = now;
        }
        was_unsafe = unsafe;
        g_platform_diag.state = chassis_diag.state;
        g_platform_diag.tick_ms = platform_millis();
        __asm volatile("dmb" ::: "memory");
        ++g_platform_diag.seq;
        service_snapshot();
        platform_watchdog_feed();
    }
}
