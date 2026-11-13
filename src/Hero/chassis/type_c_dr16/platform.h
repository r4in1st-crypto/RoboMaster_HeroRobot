#ifndef DR16_PLATFORM_H
#define DR16_PLATFORM_H

#include <stdbool.h>
#include <stdint.h>

#define PLATFORM_DIAG_MAGIC UINT32_C(0x44523150)
#define PLATFORM_DIAG_VERSION 1u

#define PLATFORM_ERR_CLOCK       (1u << 0)
#define PLATFORM_ERR_DEVICE      (1u << 1)
#define PLATFORM_ERR_CAN_INIT    (1u << 2)
#define PLATFORM_ERR_CAN_ESR     (1u << 3)
#define PLATFORM_ERR_RX_OVERRUN  (1u << 4)
#define PLATFORM_ERR_RX_INVALID  (1u << 5)
#define PLATFORM_ERR_TX          (1u << 6)
#define PLATFORM_ERR_TX_TIMEOUT  (1u << 7)
#define PLATFORM_ERR_USER_STOP   (1u << 8)
#define PLATFORM_ERR_STEP_DELAY  (1u << 9)
#define PLATFORM_ERR_OTHER_HOST  (1u << 10)
#define PLATFORM_ERR_TX_ABORT    (1u << 11)
#define PLATFORM_ERR_EXCEPTION   (1u << 12)
#define PLATFORM_ERR_WATCHDOG    (1u << 13)
#define PLATFORM_ERR_UART_HW     (1u << 14)

#define PLATFORM_RC_ERR_UART     (1u << 0)
#define PLATFORM_RC_ERR_DMA      (1u << 1)
#define PLATFORM_RC_ERR_LENGTH   (1u << 2)
#define PLATFORM_RC_ERR_OVERFLOW (1u << 3)

/* All fields are fixed-width words; no pointers, floats, or bitfields.
 * seq is odd during one main-loop transaction. An exception leaves it odd.
 * state mirrors chassis_diag.state. CAN-only faults may be explicitly cleared
 * after private safe-OFF qualification; every other hardware error stays latched.
 * tx_count means hardware TXOK completions, not merely queued frames.
 */
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
    uint32_t reserved0; /* ABI-preserved name: explicit CAN recovery count. */
    uint32_t reserved1; /* ABI-preserved name: last CAN recovery time (ms). */
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
extern volatile uint32_t g_stop_request;

bool platform_init(void);
uint32_t platform_millis(void);
void platform_watchdog_feed(void);
void platform_service(void);
bool platform_healthy(void);
bool platform_stop_requested(void);
void platform_latch_error(uint32_t bits);
void platform_send_currents(const int16_t currents[8], bool output_enabled);
void platform_emergency_zero(void);
/* Clears CAN-only errors after 1 s safe OFF with real, fresh successful zero TX.
 * True never arms the controller; caller must clear its CAN fault to DISABLED. */
bool platform_try_can_recovery(uint32_t now_ms, bool safe_off);
/* RX ISR only queues. The main loop serializes decoding/control/snapshots. */
bool platform_rc_pop(uint8_t frame[18], uint32_t *received_ms);
uint32_t platform_rc_take_errors(void);
bool platform_rc_has_errors(void);
void platform_panic(uint32_t reason) __attribute__((noreturn));

#endif
