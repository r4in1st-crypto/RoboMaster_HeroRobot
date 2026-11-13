#ifndef TYPE_C_CAN_POLICY_H
#define TYPE_C_CAN_POLICY_H

#include <stdbool.h>
#include <stdint.h>

#define CAN_POLICY_RECOVERABLE_ERRORS UINT32_C(0x000000c8)
#define CAN_POLICY_TX_DEADLINE_MS 3u
#define CAN_POLICY_RECOVERY_MS 1000u
#define CAN_POLICY_RECOVERY_TX_MIN 20u
#define CAN_POLICY_ZERO_TX_FRESH_MS 20u

/* Pure policy shared by firmware and host simulation; no peripheral access. */
/* Only BOFF is intrinsically fatal here. Warning/error-passive are diagnostic;
 * callers must retain TX-deadline and valid wheel-feedback freshness checks. */
bool can_policy_esr_fatal(uint32_t esr);
bool can_policy_tx_expired(uint32_t now_ms, uint32_t queued_ms);
/* One mailbox's TSR status, shifted so RQCP/TXOK/ALST/TERR are bits 0..3.
 * Pending retries and intentional software aborts are not final TX failures. */
bool can_policy_tx_failed(uint32_t mailbox_status, bool intentional_abort);

typedef struct {
    bool qualifying;
    uint32_t start_ms;
    uint32_t start_zero_tx_count;
    uint32_t start_error_generation;
} can_policy_recovery_t;

typedef struct {
    uint32_t now_ms;
    uint32_t fault_flags;
    uint32_t esr;
    uint32_t zero_tx_count;
    uint32_t last_zero_tx_ms;
    uint32_t error_generation;
    bool safe_off;       /* Controller's private OFF/neutral/still/valid state. */
    bool output_zero;
    bool can_ready;
    bool tx_ready;
    bool have_zero_tx;
    bool pending_empty;  /* Required at release; normal zero TX may be pending
                         * during the preceding qualification interval. */
} can_policy_sample_t;

void can_policy_recovery_reset(can_policy_recovery_t *state);
/* Caller samples continuously. Invalid conditions or a new error restart the
 * full dwell. A true return consumes the qualification but NEVER enables any
 * output: caller may clear only RECOVERABLE_ERRORS and must remain disarmed. */
bool can_policy_recovery_step(can_policy_recovery_t *state,
                              const can_policy_sample_t *sample);

#endif
