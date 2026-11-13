#ifndef TYPE_C_DR16_DIRECT_PLATFORM_H
#define TYPE_C_DR16_DIRECT_PLATFORM_H
#include <stdbool.h>
#include <stdint.h>

#define PLATFORM_DIAG_MAGIC UINT32_C(0x44523150)
#define PLATFORM_DIAG_VERSION 2u

/* Historical diagnostic categories, never motion-enable conditions. */
#define PLATFORM_ERR_CLOCK       (1u << 0)
#define PLATFORM_ERR_DEVICE      (1u << 1)
#define PLATFORM_ERR_CAN_INIT    (1u << 2)
#define PLATFORM_ERR_CAN_ESR     (1u << 3)
#define PLATFORM_ERR_RX_OVERRUN  (1u << 4)
#define PLATFORM_ERR_RX_INVALID  (1u << 5)
#define PLATFORM_ERR_TX          (1u << 6)
#define PLATFORM_ERR_OTHER_HOST  (1u << 10)
#define PLATFORM_ERR_EXCEPTION   (1u << 12)
#define PLATFORM_ERR_UART_HW     (1u << 14)
#define PLATFORM_RC_ERR_UART     (1u << 0)
#define PLATFORM_RC_ERR_DMA      (1u << 1)
#define PLATFORM_RC_ERR_LENGTH   (1u << 2)
#define PLATFORM_RC_ERR_OVERFLOW (1u << 3)

/* Same 224-byte, all-word layout as the earlier platform, version 2 semantics.
 * error_flags is event history only. stop_active, tx_timeout_count, and
 * abort_failed_count remain zero. No diagnostic field gates current output.
 * tx_count counts actual TXOK completions; tx_abort_count counts asynchronous
 * requests to replace an obsolete pending payload, not emergency stops. */
typedef struct {
    uint32_t magic;
    uint32_t version;
    uint32_t size_bytes;
    uint32_t seq;
    uint32_t tick_ms;
    uint32_t state;
    uint32_t error_flags;
    uint32_t first_error_ms;
    uint32_t esr;
    uint32_t first_bad_esr;
    uint32_t rx_count;
    uint32_t rx_known_count;
    uint32_t rx_unknown_count;
    uint32_t rx_invalid_count;
    uint32_t rx_overrun_count;
    uint32_t tx_queued_count;
    uint32_t tx_count;
    uint32_t tx_error_count;
    uint32_t tx_abort_count;
    uint32_t tx_timeout_count;
    uint32_t conflict_count;
    uint32_t last_rx_id;
    uint32_t last_rx_ms;
    uint32_t last_unknown_id;
    uint32_t reserved0; /* Version 2: latest payload groups waiting for a mailbox. */
    uint32_t reserved1; /* Version 2: asynchronous UART DMA restart count. */
    uint32_t stop_active;
    uint32_t device_id;
    uint32_t flash_kib;
    uint32_t hse_hz;
    uint32_t sysclk_hz;
    uint32_t can_btr;
    uint32_t can_mcr;
    uint32_t tx_pending_mask;
    uint32_t reset_flags;
    uint32_t main_loops;
    uint32_t max_step_gap_ms;
    uint32_t abort_failed_count;
    uint32_t last_tx_status;
    uint32_t output_enabled;
    uint32_t uart_sr;
    uint32_t uart_cr1;
    uint32_t uart_brr;
    uint32_t uart_dma_ndtr;
    uint32_t uart_idle_count;
    uint32_t uart_frame_count;
    uint32_t uart_error_count;
    uint32_t uart_dma_error_count;
    uint32_t uart_bad_length_count;
    uint32_t uart_queue_overflow_count;
    uint32_t uart_last_frame_ms;
    uint32_t uart_last_length;
    uint32_t uart_error_events;
    uint32_t uart_queue_depth;
    uint32_t uart_dma_cr;
    uint32_t uart_burst_invalid;
} platform_diag_t;

extern volatile platform_diag_t g_platform_diag;

bool platform_init(void);
uint32_t platform_millis(void);
void platform_service(void);
bool platform_rc_pop(uint8_t frame[18], uint32_t *received_ms, uint32_t *errors);
/* Caller controls pacing. Records the latest payload and services mailboxes
 * without waiting. enabled=false sends zero solely as the user's S1 command. */
void platform_send_currents(const int16_t currents[8], bool output_enabled);
void platform_panic(uint32_t reason) __attribute__((noreturn));

#endif
