#include "can_policy.h"

/* ST RM0090, bxCAN transmission handling and CAN_TSR/CAN_ESR:
 * NART=0 retries failed attempts; RQCP is completed transmit OR abort.
 * TXOK is the final successful result, even if an earlier TERR was observed.
 * https://www.st.com/resource/en/reference_manual/rm0090-stm32f407-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
 * ST HAL_CAN_IRQHandler also tests TXOK before ALST/TERR/abort.
 */
bool can_policy_esr_fatal(uint32_t esr)
{
    /* BOFF means this controller cannot participate in communication. Warning
     * and error-passive alone do not prove that valid RX/TX has stopped, so
     * they remain diagnostic while CAN retries. The independent 3 ms pending
     * deadline and controller's 30 ms wheel-feedback deadline still stop any
     * sustained loss of valid communication; neither deadline is relaxed. */
    return (esr & 4u) != 0u;
}

bool can_policy_tx_expired(uint32_t now_ms, uint32_t queued_ms)
{
    return (uint32_t)(now_ms - queued_ms) >= CAN_POLICY_TX_DEADLINE_MS;
}

bool can_policy_tx_failed(uint32_t mailbox_status, bool intentional_abort)
{
    /* With NART=0, ordinary arbitration/error retries remain pending. A final
     * completion without success or our ABRQ is unexpected and must stop,
     * including ALST-only or missing result flags, not just a TERR completion. */
    return (mailbox_status & 1u) != 0u && (mailbox_status & 2u) == 0u &&
           !intentional_abort;
}

void can_policy_recovery_reset(can_policy_recovery_t *state)
{
    state->qualifying = false;
    state->start_ms = 0u;
    state->start_zero_tx_count = 0u;
    state->start_error_generation = 0u;
}

bool can_policy_recovery_step(can_policy_recovery_t *state,
                              const can_policy_sample_t *sample)
{
    if (sample->fault_flags == 0u ||
        (sample->fault_flags & ~CAN_POLICY_RECOVERABLE_ERRORS) != 0u ||
        !sample->safe_off || !sample->output_zero || !sample->can_ready ||
        !sample->tx_ready || sample->esr != 0u || !sample->have_zero_tx ||
        (uint32_t)(sample->now_ms - sample->last_zero_tx_ms) >
            CAN_POLICY_ZERO_TX_FRESH_MS) {
        can_policy_recovery_reset(state);
        return false;
    }
    if (!state->qualifying ||
        sample->error_generation != state->start_error_generation) {
        state->qualifying = true;
        state->start_ms = sample->now_ms;
        state->start_zero_tx_count = sample->zero_tx_count;
        state->start_error_generation = sample->error_generation;
        return false;
    }
    if ((uint32_t)(sample->now_ms - state->start_ms) < CAN_POLICY_RECOVERY_MS ||
        (uint32_t)(sample->zero_tx_count - state->start_zero_tx_count) <
            CAN_POLICY_RECOVERY_TX_MIN || !sample->pending_empty) {
        return false;
    }
    can_policy_recovery_reset(state);
    return true;
}
