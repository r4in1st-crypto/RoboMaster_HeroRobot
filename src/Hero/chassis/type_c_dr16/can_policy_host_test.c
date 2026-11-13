#include "can_policy.h"
#include <stdio.h>
#include <stdlib.h>

static unsigned checks;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #condition); exit(1); \
} } while (0)

static can_policy_sample_t safe_sample(uint32_t now)
{
    can_policy_sample_t sample = {
        .now_ms = now, .fault_flags = CAN_POLICY_RECOVERABLE_ERRORS,
        .esr = 0u, .zero_tx_count = 50u, .last_zero_tx_ms = now,
        .error_generation = 7u, .safe_off = true, .output_zero = true,
        .can_ready = true, .tx_ready = true, .have_zero_tx = true,
        .pending_empty = true
    };
    return sample;
}

static void tick(can_policy_sample_t *sample, uint32_t start, uint32_t elapsed,
                 uint32_t initial_tx)
{
    sample->now_ms = start + elapsed;
    sample->last_zero_tx_ms = start + elapsed - elapsed % 5u;
    sample->zero_tx_count = initial_tx + (elapsed / 5u) * 2u;
}

static void test_error_and_completion_policy(void)
{
    /* Actual incident: REC=1 and LEC=stuff, without warning/passive/bus-off. */
    CHECK(!can_policy_esr_fatal(UINT32_C(0x01000010)));
    CHECK(!can_policy_tx_failed(8u, false));  /* Failed attempt, still pending. */
    CHECK(!can_policy_tx_expired(101u, 100u));
    CHECK(!can_policy_tx_failed(3u, false));  /* Successful retry at 1 ms. */
    for (uint32_t warning_passive = 1u; warning_passive <= 3u; ++warning_passive) {
        CHECK(!can_policy_esr_fatal((128u << 16) | warning_passive));
        CHECK(!can_policy_tx_failed(3u, false)); /* Real completed TX is valid. */
        CHECK(!can_policy_tx_expired(101u, 100u));
        CHECK(can_policy_tx_expired(103u, 100u)); /* Same hard limit in passive. */
    }
    for (uint32_t bus_off = 4u; bus_off < 8u; ++bus_off) {
        CHECK(can_policy_esr_fatal(bus_off));    /* BOFF alone or any combination. */
    }
    /* Recorded sustained ACK-loss incident: flags alone no longer stop, but
     * its uncompleted request still hits the unchanged 3 ms deadline. */
    CHECK(!can_policy_esr_fatal(UINT32_C(0x01800033)));
    CHECK(can_policy_tx_expired(103u, 100u));
    for (uint32_t count = 0u; count < 96u; ++count) {
        for (uint32_t lec = 0u; lec < 8u; ++lec) {
            for (uint32_t severe = 0u; severe < 8u; ++severe) {
                uint32_t esr = (count << 24) | (count << 16) | (lec << 4) | severe;
                CHECK(can_policy_esr_fatal(esr) == ((severe & 4u) != 0u));
            }
        }
    }
    for (uint32_t status = 0u; status < 256u; ++status) {
        CHECK(can_policy_tx_failed(status, false) ==
              ((status & 1u) != 0u && (status & 2u) == 0u));
        CHECK(!can_policy_tx_failed(status, true));
    }
    CHECK(can_policy_tx_failed(9u, false));  /* RQCP + TERR. */
    CHECK(can_policy_tx_failed(5u, false));  /* Unexpected completed ALST. */
    CHECK(can_policy_tx_failed(1u, false));  /* Unexpected completion, no result. */
    CHECK(!can_policy_tx_failed(11u, false));/* TXOK takes priority over old TERR. */
    CHECK(!can_policy_tx_failed(9u, true));  /* We intentionally aborted a retry. */
    const uint32_t starts[] = {0u, 1234u, UINT32_MAX - 1u, UINT32_MAX};
    for (unsigned i = 0; i < sizeof(starts) / sizeof(starts[0]); ++i) {
        for (uint32_t age = 0u; age < 100u; ++age) {
            CHECK(can_policy_tx_expired(starts[i] + age, starts[i]) == (age >= 3u));
        }
    }
}

static void test_recovery_dwell_and_wrap(void)
{
    const uint32_t starts[] = {100u, UINT32_MAX - 500u};
    const uint32_t initial[] = {50u, UINT32_MAX - 150u};
    for (unsigned k = 0u; k < 2u; ++k) {
        can_policy_recovery_t state = {0};
        can_policy_sample_t sample = safe_sample(starts[k]);
        for (uint32_t elapsed = 0u; elapsed <= 1000u; ++elapsed) {
            tick(&sample, starts[k], elapsed, initial[k]);
            CHECK(can_policy_recovery_step(&state, &sample) == (elapsed == 1000u));
        }
        CHECK(!state.qualifying);             /* Release was consumed once. */
        sample.fault_flags = 0u;
        CHECK(!can_policy_recovery_step(&state, &sample));
    }
}

static void test_recovery_rejects_unsafe(void)
{
    /* Every fault outside the explicit CAN mask remains non-recoverable. */
    for (uint32_t bit = 0u; bit < 32u; ++bit) {
        if (((1u << bit) & CAN_POLICY_RECOVERABLE_ERRORS) != 0u) { continue; }
        can_policy_recovery_t state = {0};
        can_policy_sample_t sample = safe_sample(0u);
        sample.fault_flags |= 1u << bit;
        for (uint32_t t = 0u; t <= 1100u; ++t) {
            tick(&sample, 0u, t, 50u);
            CHECK(!can_policy_recovery_step(&state, &sample));
        }
    }
    for (uint32_t case_id = 0u; case_id < 10u; ++case_id) {
        can_policy_recovery_t state = {0};
        can_policy_sample_t sample = safe_sample(0u);
        for (uint32_t t = 0u; t <= 1100u; ++t) {
            tick(&sample, 0u, t, 50u);
            switch (case_id) {
            case 0u: sample.safe_off = false; break; /* S1 UP, or unsafe sticks/wheels. */
            case 1u: sample.output_zero = false; break;
            case 2u: sample.can_ready = false; break;
            case 3u: sample.tx_ready = false; break;
            case 4u: sample.have_zero_tx = false; break;
            case 5u: sample.esr = 4u; break;       /* Bus-off still present. */
            case 6u: sample.esr = 1u; break;       /* Warning still present. */
            case 7u: sample.esr = 2u; break;       /* Passive still present. */
            case 8u: sample.esr = 0x01000010u; break; /* Even a soft error prevents release. */
            default: sample.fault_flags = 0u; break;
            }
            CHECK(!can_policy_recovery_step(&state, &sample));
        }
    }
}

static void test_recovery_needs_progress_and_empty_mailbox(void)
{
    can_policy_recovery_t state = {0};
    can_policy_sample_t sample = safe_sample(0u);
    CHECK(!can_policy_recovery_step(&state, &sample));
    for (uint32_t t = 1u; t <= 1100u; ++t) {
        sample.now_ms = t;                    /* No further real successful TX. */
        CHECK(!can_policy_recovery_step(&state, &sample));
    }
    CHECK(!state.qualifying);                 /* Last zero TX became stale. */

    sample = safe_sample(2000u);
    CHECK(!can_policy_recovery_step(&state, &sample));
    sample.now_ms += 1000u;
    sample.last_zero_tx_ms = sample.now_ms;
    sample.zero_tx_count += 19u;
    CHECK(!can_policy_recovery_step(&state, &sample));
    ++sample.zero_tx_count;
    sample.pending_empty = false;
    CHECK(!can_policy_recovery_step(&state, &sample));
    sample.pending_empty = true;
    CHECK(can_policy_recovery_step(&state, &sample));
}

static void test_new_error_and_off_interrupt_restart_dwell(void)
{
    for (uint32_t event = 0u; event < 3u; ++event) {
        can_policy_recovery_t state = {0};
        can_policy_sample_t sample = safe_sample(0u);
        for (uint32_t t = 0u; t <= 1501u; ++t) {
            tick(&sample, 0u, t, 50u);
            if (event == 0u && t == 500u) { ++sample.error_generation; }
            if (event == 1u) { sample.safe_off = t != 500u; }
            if (event == 2u) { sample.esr = t == 500u ? 0x01000010u : 0u; }
            uint32_t release = event == 0u ? 1500u : 1501u;
            bool recovered = can_policy_recovery_step(&state, &sample);
            CHECK(recovered == (t == release));
            if (recovered) { sample.fault_flags = 0u; }
        }
    }
}

int main(void)
{
    test_error_and_completion_policy();
    test_recovery_dwell_and_wrap();
    test_recovery_rejects_unsafe();
    test_recovery_needs_progress_and_empty_mailbox();
    test_new_error_and_off_interrupt_restart_dwell();
    printf("CAN policy: %u assertions passed\n", checks);
    return 0;
}
