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

static bool service_remote(void)
{
    bool switch_changed = false;
    uint32_t errors = platform_rc_take_errors();
    if (errors != 0u)
        chassis_remote_invalid(platform_millis(), UINT32_C(0x10000) | errors);
    uint8_t bytes[DR16_FRAME_BYTES];
    uint32_t received_ms;
    while (platform_rc_pop(bytes, &received_ms)) {
        dr16_frame_t frame;
        dr16_result_t result = dr16_decode(bytes, sizeof(bytes), received_ms, &frame);
        if (result == DR16_OK) {
            bool before = chassis_output_enabled();
            chassis_remote_update(&frame);
            switch_changed |= before != chassis_output_enabled();
        } else
            chassis_remote_invalid(platform_millis(), (uint32_t)result);
    }
    /* When packets accumulated, the last legal command wins. Recompute that
     * final command immediately even if DOWN then UP returned to the old state. */
    return switch_changed;
}

int main(void)
{
    chassis_init();
    if (!platform_init()) {
        /* Initialization did not produce a usable clock/peripheral setup.
         * Keep diagnostics readable; no runtime protection state is installed. */
        for (;;) service_snapshot();
    }
    uint32_t last_step = platform_millis();
    uint32_t last_send = last_step - 5u;
    bool was_enabled = false;
    for (;;) {
        ++g_platform_diag.seq;
        __asm volatile("dmb" ::: "memory");
        ++g_platform_diag.main_loops;
        platform_service();
        bool switch_changed = service_remote();
        uint32_t now = platform_millis();
        uint32_t gap = now - last_step;
        if (gap > g_platform_diag.max_step_gap_ms)
            g_platform_diag.max_step_gap_ms = gap;
        bool enabled = chassis_output_enabled();
        if (now != last_step || switch_changed || enabled != was_enabled) {
            chassis_step(now, true, false);
            last_step = now;
        }
        enabled = chassis_output_enabled();
        if (now - last_send >= 5u || switch_changed || enabled != was_enabled) {
            int16_t currents[8];
            chassis_currents(currents);
            platform_send_currents(currents, enabled);
            last_send = now;
        }
        was_enabled = enabled;
        g_platform_diag.state = chassis_diag.state;
        g_platform_diag.tick_ms = now;
        __asm volatile("dmb" ::: "memory");
        ++g_platform_diag.seq;
        service_snapshot();
    }
}
