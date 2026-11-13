/* Standalone STM32F407IG / RoboMaster Type C platform.
 * Register map: ST RM0090. Pin and clock choices: RoboMaster official Type C
 * 14.CAN example. This file does not initialize CAN2, PWM, laser, or motors.
 */
#include "platform.h"
#include "chassis_control.h"
#include "can_policy.h"

#include <stddef.h>

#define REG32(a) (*(volatile uint32_t *)(uintptr_t)(a))
#define RCC_BASE UINT32_C(0x40023800)
#define RCC_CR REG32(RCC_BASE + 0x00u)
#define RCC_PLLCFGR REG32(RCC_BASE + 0x04u)
#define RCC_CFGR REG32(RCC_BASE + 0x08u)
#define RCC_APB1RSTR REG32(RCC_BASE + 0x20u)
#define RCC_AHB1ENR REG32(RCC_BASE + 0x30u)
#define RCC_APB1ENR REG32(RCC_BASE + 0x40u)
#define RCC_CSR REG32(RCC_BASE + 0x74u)
#define GPIOA_BASE UINT32_C(0x40020000)
#define GPIOC_BASE UINT32_C(0x40020800)
#define GPIOD_BASE UINT32_C(0x40020c00)
#define GPIO_MODER(b) REG32((b) + 0x00u)
#define GPIO_OTYPER(b) REG32((b) + 0x04u)
#define GPIO_OSPEEDR(b) REG32((b) + 0x08u)
#define GPIO_PUPDR(b) REG32((b) + 0x0cu)
#define GPIO_IDR(b) REG32((b) + 0x10u)
#define GPIO_AFRL(b) REG32((b) + 0x20u)
#define GPIO_AFRH(b) REG32((b) + 0x24u)
#define FLASH_ACR REG32(UINT32_C(0x40023c00))
#define PWR_CR REG32(UINT32_C(0x40007000))
#define SYST_CSR REG32(UINT32_C(0xe000e010))
#define SYST_RVR REG32(UINT32_C(0xe000e014))
#define SYST_CVR REG32(UINT32_C(0xe000e018))
#define IWDG_KR REG32(UINT32_C(0x40003000))
#define IWDG_PR REG32(UINT32_C(0x40003004))
#define IWDG_RLR REG32(UINT32_C(0x40003008))
#define IWDG_SR REG32(UINT32_C(0x4000300c))
#define CAN_BASE UINT32_C(0x40006400)
#define CAN_MCR REG32(CAN_BASE + 0x00u)
#define CAN_MSR REG32(CAN_BASE + 0x04u)
#define CAN_TSR REG32(CAN_BASE + 0x08u)
#define CAN_RF0R REG32(CAN_BASE + 0x0cu)
#define CAN_RF1R REG32(CAN_BASE + 0x10u)
#define CAN_IER REG32(CAN_BASE + 0x14u)
#define CAN_ESR REG32(CAN_BASE + 0x18u)
#define CAN_BTR REG32(CAN_BASE + 0x1cu)
#define CAN_TX(m, o) REG32(CAN_BASE + 0x180u + (m) * 16u + (o))
#define CAN_RX(o) REG32(CAN_BASE + 0x1b0u + (o))
#define CAN_FMR REG32(CAN_BASE + 0x200u)
#define CAN_FM1R REG32(CAN_BASE + 0x204u)
#define CAN_FS1R REG32(CAN_BASE + 0x20cu)
#define CAN_FFA1R REG32(CAN_BASE + 0x214u)
#define CAN_FA1R REG32(CAN_BASE + 0x21cu)
#define CAN_F0R1 REG32(CAN_BASE + 0x240u)
#define CAN_F0R2 REG32(CAN_BASE + 0x244u)

#define CAN_TME_ALL UINT32_C(0x1c000000)
#define CAN_ABORT_ALL UINT32_C(0x00808080)
#define CAN_COMPLETED_ALL UINT32_C(0x00010101)
#define CAN_ESR_BAD_MASK UINT32_C(0xffff0077)
#define CAN_PAIR_INTERVAL_MS 5u

#define UART_BASE UINT32_C(0x40004800)
#define UART_SR REG32(UART_BASE + 0x00u)
#define UART_DR REG32(UART_BASE + 0x04u)
#define UART_BRR REG32(UART_BASE + 0x08u)
#define UART_CR1 REG32(UART_BASE + 0x0cu)
#define UART_CR2 REG32(UART_BASE + 0x10u)
#define UART_CR3 REG32(UART_BASE + 0x14u)
#define DMA1_BASE UINT32_C(0x40026000)
#define DMA_LISR REG32(DMA1_BASE)
#define DMA_LIFCR REG32(DMA1_BASE + 0x08u)
#define DMA_S1CR REG32(DMA1_BASE + 0x28u)
#define DMA_S1NDTR REG32(DMA1_BASE + 0x2cu)
#define DMA_S1PAR REG32(DMA1_BASE + 0x30u)
#define DMA_S1M0AR REG32(DMA1_BASE + 0x34u)
#define DMA_S1FCR REG32(DMA1_BASE + 0x3cu)
#define DMA_S1_ALL_FLAGS UINT32_C(0x00000f40)
#define DMA_S1_ERROR_FLAGS UINT32_C(0x00000340)
#define UART_ERROR_FLAGS 15u
#define UART_IDLE_FLAG 16u
#define UART_RXNE_FLAG 32u
#define RC_DMA_BYTES 36u
#define RC_QUEUE_SIZE 4u

volatile platform_diag_t g_platform_diag;
volatile uint32_t g_stop_request;

static volatile uint32_t uptime_ms;
/* Diagnostics are writable/readable over SWD, but never authorize clearing
 * a real fault. Keep the latch private and publish only a mirror. */
static uint32_t latched_error_flags;
static bool can_ready;
/* Suppress application TX until an ESC is awake. Normal CAN receive/ACK stays
 * enabled; once opened, this gate never hides runtime faults or emergency zero. */
static bool can_tx_ready;
static bool watchdog_ready;
static uint32_t pending_mask;
/* One diagnostic timeout event per mailbox transaction, not per polling pass. */
static uint32_t pending_timeout_mask;
static uint32_t pending_zero_mask;
static uint32_t intentional_abort_mask;
static uint32_t pending_since[3];
static uint32_t last_pair_ms;
static bool has_pair;
static uint32_t zero_tx_count;
static uint32_t last_zero_tx_ms;
static bool have_zero_tx;
static uint32_t can_error_generation;
static uint32_t previous_error_observation;
static can_policy_recovery_t can_recovery;

typedef struct { uint8_t bytes[18]; uint32_t received_ms; } rc_packet_t;
static volatile uint8_t rc_dma_buffer[RC_DMA_BYTES];
static volatile rc_packet_t rc_queue[RC_QUEUE_SIZE];
static volatile uint32_t rc_head, rc_tail, rc_error_events;
static volatile uint32_t rc_idle_count, rc_frame_count, rc_uart_error_count;
static volatile uint32_t rc_dma_error_count, rc_bad_length_count, rc_overflow_count;
static volatile uint32_t rc_last_frame_ms, rc_last_length, rc_last_sr;
static volatile bool rc_burst_invalid, rc_hardware_fault;

_Static_assert(sizeof(platform_diag_t) == 224u, "platform diagnostic ABI");
_Static_assert(CAN_POLICY_RECOVERABLE_ERRORS ==
               (PLATFORM_ERR_CAN_ESR | PLATFORM_ERR_TX | PLATFORM_ERR_TX_TIMEOUT),
               "CAN recovery mask must match platform error ABI");

/* Freestanding implementations; GCC may use these for aggregate assignments. */
void *memset(void *dest, int value, size_t count)
{
    unsigned char *p = dest;
    for (size_t i = 0; i < count; ++i) { p[i] = (unsigned char)value; }
    return dest;
}

void *memcpy(void *dest, const void *source, size_t count)
{
    unsigned char *d = dest;
    const unsigned char *s = source;
    for (size_t i = 0; i < count; ++i) { d[i] = s[i]; }
    return dest;
}

void SysTick_Handler(void) { ++uptime_ms; }

uint32_t platform_millis(void) { return uptime_ms; }

void platform_latch_error(uint32_t bits)
{
    if (latched_error_flags == 0u) {
        g_platform_diag.first_error_ms = uptime_ms;
    }
    latched_error_flags |= bits;
    g_platform_diag.error_flags = latched_error_flags;
}

static bool wait_bits(volatile uint32_t *reg, uint32_t mask,
                      uint32_t expected)
{
    /* Bounded even before SysTick exists or if the clock fails. */
    for (uint32_t n = 0; n < 2000000u; ++n) {
        if ((*reg & mask) == expected) { return true; }
    }
    return false;
}

static bool clock_init(void)
{
    RCC_CR |= 1u;                       /* HSI on for initial clock transition. */
    if (!wait_bits(&RCC_CR, 2u, 2u)) { return false; }
    RCC_CFGR &= ~3u;
    if (!wait_bits(&RCC_CFGR, 12u, 0u)) { return false; }
    RCC_CR &= ~(1u << 24);
    if (!wait_bits(&RCC_CR, 1u << 25, 0u)) { return false; }
    RCC_APB1ENR |= 1u << 28;            /* PWR clock. F407 has no OverDrive. */
    (void)RCC_APB1ENR;
    PWR_CR |= 1u << 14;                 /* Voltage scale 1. */
    FLASH_ACR = (1u << 10) | (1u << 9) | (1u << 8) | 5u;
    RCC_CR |= 1u << 16;                 /* Board HSE is 12 MHz. */
    if (!wait_bits(&RCC_CR, 1u << 17, 1u << 17)) { return false; }
    /* 12 MHz / 12 * 336 / 2 = 168 MHz; PLLQ=7 => 48 MHz. */
    RCC_PLLCFGR = 12u | (336u << 6) | (1u << 22) | (7u << 24);
    RCC_CFGR = (RCC_CFGR & ~UINT32_C(0x0000fcf3)) |
               (5u << 10) | (4u << 13); /* AHB /1, APB1 /4, APB2 /2. */
    RCC_CR |= 1u << 24;
    if (!wait_bits(&RCC_CR, 1u << 25, 1u << 25)) { return false; }
    RCC_CFGR = (RCC_CFGR & ~3u) | 2u;
    if (!wait_bits(&RCC_CFGR, 12u, 8u)) { return false; }
    SYST_RVR = 168000u - 1u;
    SYST_CVR = 0u;
    SYST_CSR = 7u;
    g_platform_diag.hse_hz = 12000000u;
    g_platform_diag.sysclk_hz = 168000000u;
    return true;
}

static bool can_init(void)
{
    RCC_AHB1ENR |= (1u << 0) | (1u << 3); /* KEY and CAN1 pins only. */
    (void)RCC_AHB1ENR;
    GPIO_MODER(GPIOA_BASE) &= ~3u;        /* PA0 input, KEY active low. */
    GPIO_PUPDR(GPIOA_BASE) = (GPIO_PUPDR(GPIOA_BASE) & ~3u) | 1u;
    GPIO_MODER(GPIOD_BASE) = (GPIO_MODER(GPIOD_BASE) & ~15u) | 10u;
    GPIO_OTYPER(GPIOD_BASE) &= ~3u;
    GPIO_OSPEEDR(GPIOD_BASE) |= 15u;
    GPIO_PUPDR(GPIOD_BASE) &= ~15u;
    GPIO_AFRL(GPIOD_BASE) = (GPIO_AFRL(GPIOD_BASE) & ~255u) | 0x99u;

    RCC_APB1ENR |= 1u << 25;             /* CAN1; CAN2 remains disabled. */
    (void)RCC_APB1ENR;
    RCC_APB1RSTR |= 1u << 25;
    RCC_APB1RSTR &= ~(1u << 25);
    /* Init, automatic retransmission, locked receive FIFO. A pending request
     * retains its original 3 ms software deadline across hardware retries.
     * No automatic bus-off recovery: bus-off requires an explicit reset. */
    CAN_MCR = (1u << 3) | 1u;           /* NART=0, RFLM=1, ABOM=0. */
    if (!wait_bits(&CAN_MSR, 1u, 1u)) { return false; }
    CAN_IER = 0u;                       /* Main loop is the only FIFO owner. */
    /* APB1=42 MHz / [3 * (1+10+3)] = 1 Mbit/s, SJW=1. */
    CAN_BTR = 2u | (9u << 16) | (2u << 20);
    CAN_FMR = (14u << 8) | 1u;
    CAN_FA1R = 0u;
    CAN_FM1R = 0u;
    CAN_FS1R = 1u;
    CAN_FFA1R = 0u;
    CAN_F0R1 = 0u;
    CAN_F0R2 = 6u;                      /* Standard data only: IDE=RTR=0. */
    CAN_FA1R = 1u;
    CAN_FMR &= ~1u;
    CAN_MCR &= ~1u;
    if (!wait_bits(&CAN_MSR, 1u, 0u)) { return false; }
    can_ready = true;
    g_platform_diag.can_btr = CAN_BTR;
    g_platform_diag.can_mcr = CAN_MCR;
    return true;
}

static uint32_t interrupt_lock(void)
{
    uint32_t old;
    __asm volatile("mrs %0, primask\ncpsid i" : "=r"(old) :: "memory");
    return old;
}

static void interrupt_restore(uint32_t old)
{
    __asm volatile("msr primask, %0" :: "r"(old) : "memory");
}

static bool rc_dma_stop(void)
{
    DMA_S1CR &= ~1u;
    for (uint32_t n = 0; n < 1024u; ++n) {
        if ((DMA_S1CR & 1u) == 0u) {
            UART_CR3 &= ~(1u << 6);
            return true;
        }
    }
    UART_CR3 &= ~(1u << 6);
    rc_hardware_fault = true;
    rc_error_events |= PLATFORM_RC_ERR_DMA;
    return false;
}

static void rc_dma_restart(void)
{
    DMA_LIFCR = DMA_S1_ALL_FLAGS;
    DMA_S1M0AR = (uint32_t)(uintptr_t)rc_dma_buffer;
    DMA_S1NDTR = RC_DMA_BYTES;
    __asm volatile("dmb" ::: "memory");
    DMA_S1CR |= 1u;
    UART_CR3 |= 1u << 6;
}

static bool rc_uart_init(void)
{
    RCC_AHB1ENR |= (1u << 2) | (1u << 21); /* GPIOC and DMA1. */
    RCC_APB1ENR |= 1u << 18;               /* USART3, receive only. */
    (void)RCC_AHB1ENR;
    (void)RCC_APB1ENR;
    GPIO_MODER(GPIOC_BASE) = (GPIO_MODER(GPIOC_BASE) & ~(3u << 22)) |
                            (2u << 22);
    GPIO_PUPDR(GPIOC_BASE) = (GPIO_PUPDR(GPIOC_BASE) & ~(3u << 22)) |
                            (1u << 22);
    GPIO_OSPEEDR(GPIOC_BASE) |= 3u << 22;
    GPIO_AFRH(GPIOC_BASE) = (GPIO_AFRH(GPIOC_BASE) & ~(15u << 12)) |
                           (7u << 12);   /* PC11 AF7; PC10 is untouched. */
    RCC_APB1RSTR |= 1u << 18;
    RCC_APB1RSTR &= ~(1u << 18);
    UART_CR1 = 0u;
    UART_CR2 = 0u;                        /* One stop bit. */
    UART_CR3 = 1u;                        /* UART FE/NE/ORE error interrupt. */
    UART_BRR = 420u;                      /* APB1 42 MHz / 100000 baud. */
    DMA_S1CR = 0u;
    if (!wait_bits(&DMA_S1CR, 1u, 0u)) { return false; }
    DMA_LIFCR = DMA_S1_ALL_FLAGS;
    DMA_S1PAR = UART_BASE + 4u;
    DMA_S1CR = (4u << 25) | (3u << 16) | (1u << 10) |
               (1u << 4) | (1u << 2) | (1u << 1);
    DMA_S1FCR = 1u << 7;                  /* Direct mode, FIFO error interrupt. */
    /* M=1 is REQUIRED for 8 data bits plus parity on STM32F4.
     * M=0 with PCE=1 gives seven effective data bits, despite older examples. */
    UART_CR1 = (1u << 13) | (1u << 12) | (1u << 10) |
               (1u << 8) | (1u << 4) | (1u << 2);
    (void)UART_SR;
    (void)UART_DR;
    rc_dma_restart();
    *(volatile uint8_t *)(uintptr_t)(0xe000e400u + 12u) = 0x40u;
    *(volatile uint8_t *)(uintptr_t)(0xe000e400u + 39u) = 0x40u;
    REG32(UINT32_C(0xe000e280)) = 1u << 12; /* Clear pending DMA1_Stream1. */
    REG32(UINT32_C(0xe000e284)) = 1u << 7;  /* Clear pending USART3. */
    REG32(UINT32_C(0xe000e100)) = 1u << 12;
    REG32(UINT32_C(0xe000e104)) = 1u << 7;
    return true;
}

void DMA1_Stream1_IRQHandler(void)
{
    uint32_t flags = DMA_LISR & DMA_S1_ALL_FLAGS;
    DMA_LIFCR = DMA_S1_ALL_FLAGS;
    if ((flags & DMA_S1_ERROR_FLAGS) != 0u) {
        ++rc_dma_error_count;
        rc_error_events |= PLATFORM_RC_ERR_DMA;
        rc_burst_invalid = true;
    }
    if ((flags & (1u << 11)) != 0u && DMA_S1NDTR == 0u) {
        /* 36 bytes without IDLE is not a valid single 18-byte DR16 frame.
         * Do not restart mid-burst: discard through the next IDLE boundary. */
        ++rc_bad_length_count;
        rc_error_events |= PLATFORM_RC_ERR_LENGTH;
        rc_burst_invalid = true;
    }
}

void USART3_IRQHandler(void)
{
    uint32_t sr = UART_SR;
    rc_last_sr = sr;
    if ((sr & UART_ERROR_FLAGS) != 0u) {
        ++rc_uart_error_count;
        rc_error_events |= PLATFORM_RC_ERR_UART;
        rc_burst_invalid = true;
    }
    if ((sr & UART_IDLE_FLAG) != 0u) {
        ++rc_idle_count;
        uint32_t flags_before_stop = DMA_LISR & DMA_S1_ALL_FLAGS;
        if (!rc_dma_stop()) {
            (void)UART_DR;
            UART_CR1 &= ~((1u << 4) | (1u << 8));
            UART_CR3 &= ~1u;
            return;
        }
        uint32_t flags = (DMA_LISR | flags_before_stop) & DMA_S1_ALL_FLAGS;
        uint32_t remaining = DMA_S1NDTR;
        uint32_t length = remaining <= RC_DMA_BYTES ? RC_DMA_BYTES - remaining : 0u;
        rc_last_length = length;
        /* SR then DR clears IDLE/PE/FE/NE/ORE. If a byte remains unconsumed,
         * the packet is ambiguous and is discarded, never patched together. */
        (void)UART_DR;
        if ((sr & UART_RXNE_FLAG) != 0u) {
            ++rc_uart_error_count;
            rc_burst_invalid = true;
            rc_error_events |= PLATFORM_RC_ERR_UART;
        }
        if ((flags & DMA_S1_ERROR_FLAGS) != 0u) {
            ++rc_dma_error_count;
            rc_burst_invalid = true;
            rc_error_events |= PLATFORM_RC_ERR_DMA;
        }
        /* ST AN4031 / RM0090: clearing EN also raises TCIF. That artificial
         * completion is expected at every IDLE and is NOT a full buffer.
         * NDTR is authoritative after EN=0; retain true pre-stop completion. */
        if ((flags_before_stop & (1u << 11)) != 0u || remaining == 0u ||
            remaining > RC_DMA_BYTES) {
            rc_burst_invalid = true;
            rc_error_events |= PLATFORM_RC_ERR_LENGTH;
        }
        if (length == 18u && !rc_burst_invalid) {
            uint32_t head = rc_head;
            if ((uint32_t)(head - rc_tail) >= RC_QUEUE_SIZE) {
                ++rc_overflow_count;
                rc_error_events |= PLATFORM_RC_ERR_OVERFLOW;
            } else {
                uint32_t slot = head % RC_QUEUE_SIZE;
                for (uint32_t i = 0; i < 18u; ++i) {
                    rc_queue[slot].bytes[i] = rc_dma_buffer[i];
                }
                rc_queue[slot].received_ms = uptime_ms;
                __asm volatile("dmb" ::: "memory");
                rc_head = head + 1u;
                ++rc_frame_count;
                rc_last_frame_ms = uptime_ms;
            }
        } else if (length != 0u || rc_burst_invalid) {
            ++rc_bad_length_count;
            rc_error_events |= PLATFORM_RC_ERR_LENGTH;
        }
        rc_burst_invalid = false;
        rc_dma_restart();
    } else if ((sr & UART_ERROR_FLAGS) != 0u) {
        (void)UART_DR;                  /* Clear error; drop whole burst at IDLE. */
    }
}

bool platform_rc_has_errors(void) { return rc_error_events != 0u || rc_hardware_fault; }

uint32_t platform_rc_take_errors(void)
{
    uint32_t old = interrupt_lock();
    uint32_t errors = rc_error_events;
    rc_error_events = 0u;
    if (errors != 0u) { rc_tail = rc_head; } /* Discard queued pre-error commands. */
    interrupt_restore(old);
    return errors;
}

bool platform_rc_pop(uint8_t frame[18], uint32_t *received_ms)
{
    uint32_t old = interrupt_lock();
    uint32_t tail = rc_tail;
    bool available = tail != rc_head && rc_error_events == 0u;
    if (available) {
        uint32_t slot = tail % RC_QUEUE_SIZE;
        for (uint32_t i = 0; i < 18u; ++i) { frame[i] = rc_queue[slot].bytes[i]; }
        *received_ms = rc_queue[slot].received_ms;
        rc_tail = tail + 1u;
    }
    interrupt_restore(old);
    return available;
}

static bool watchdog_init(void)
{
    /* LSI / 32, reload 79: approximately 80 ms. It is NOT debug-frozen.
     * A stalled/halted controller resets into zero-output WAITING. */
    /* Debug settings can survive a software reset or be left by a programmer. */
    REG32(UINT32_C(0xe0042008)) &= ~(1u << 12); /* DBG_IWDG_STOP = 0. */
    RCC_CSR |= 1u;                     /* Configuration crosses the LSI domain. */
    if (!wait_bits(&RCC_CSR, 2u, 2u)) { return false; }
    /* ST HAL_IWDG_Init starts the peripheral BEFORE configuring PR/RLR.
     * An enabled LSI alone does not guarantee these update flags will clear
     * while the IWDG itself is stopped. Start exactly once, then configure. */
    IWDG_KR = 0xccccu;
    IWDG_KR = 0x5555u;
    IWDG_PR = 3u;
    IWDG_RLR = 79u;
    uint32_t start_ms = uptime_ms;
    uint32_t spins = 0u;
    while ((IWDG_SR & 3u) != 0u) {
        /* 30 ms exceeds the update latency at /32 even at slow LSI corners.
         * The iteration bound still terminates if SysTick stops advancing.
         * Failure is not masked by refreshing: panic leaves output disabled
         * and the already-started watchdog eventually resets to DISARMED. */
        if ((uint32_t)(uptime_ms - start_ms) >= 30u || ++spins >= 2000000u) {
            return false;
        }
    }
    IWDG_KR = 0xaaaau;
    watchdog_ready = true;
    return true;
}

void platform_watchdog_feed(void)
{
    if (watchdog_ready) { IWDG_KR = 0xaaaau; }
}

bool platform_init(void)
{
    g_stop_request = 0u;
    g_platform_diag.magic = PLATFORM_DIAG_MAGIC;
    g_platform_diag.version = PLATFORM_DIAG_VERSION;
    g_platform_diag.size_bytes = (uint32_t)sizeof(g_platform_diag);
    g_platform_diag.device_id = REG32(UINT32_C(0xe0042000));
    g_platform_diag.flash_kib = *(volatile uint16_t *)(uintptr_t)0x1fff7a22u;
    g_platform_diag.reset_flags = RCC_CSR;
    RCC_CSR |= 1u << 24;                 /* Clear reset-cause flags. */
    if ((g_platform_diag.device_id & 0xfffu) != 0x413u ||
        g_platform_diag.flash_kib != 1024u) {
        platform_latch_error(PLATFORM_ERR_DEVICE);
        return false;
    }
    if (!clock_init()) {
        platform_latch_error(PLATFORM_ERR_CLOCK);
        return false;
    }
    /* Prepare slow clock-domain peripherals before accepting CAN traffic.
     * Five ESCs can fill the three-deep FIFO during watchdog synchronization. */
    if (!watchdog_init()) {
        platform_latch_error(PLATFORM_ERR_WATCHDOG);
        return false;
    }
    platform_watchdog_feed();
    if (!rc_uart_init()) {
        platform_latch_error(PLATFORM_ERR_UART_HW);
        return false;
    }
    platform_watchdog_feed();
    if (!can_init()) {
        platform_latch_error(PLATFORM_ERR_CAN_INIT);
        return false;
    }
    platform_watchdog_feed();
    return true;
}

bool platform_stop_requested(void)
{
    bool pressed = (GPIO_IDR(GPIOA_BASE) & 1u) == 0u;
    bool stop = pressed || g_stop_request != 0u;
    g_platform_diag.stop_active = stop ? 1u : 0u;
    if (stop) { platform_latch_error(PLATFORM_ERR_USER_STOP); }
    return stop;
}

bool platform_healthy(void)
{
    return can_ready && latched_error_flags == 0u;
}

static void observe_can_esr(uint32_t esr)
{
    uint32_t observation = esr & CAN_ESR_BAD_MASK;
    g_platform_diag.esr = esr;
    if (observation != 0u) {
        if (g_platform_diag.first_bad_esr == 0u) {
            g_platform_diag.first_bad_esr = esr;
        }
        if (observation != previous_error_observation) {
            ++can_error_generation;
        }
    }
    previous_error_observation = observation;
    /* LEC, warning and error-passive are diagnostic while valid RX and timely
     * TX continue. Bus-off stops immediately; TX and feedback deadlines also
     * stop a link that is no longer delivering commands or measurements. */
    if (can_policy_esr_fatal(esr)) {
        platform_latch_error(PLATFORM_ERR_CAN_ESR);
    }
}

static void tx_service(void)
{
    uint32_t tsr = CAN_TSR;
    uint32_t clear = 0u;
    g_platform_diag.last_tx_status = tsr;
    for (uint32_t mb = 0; mb < 3u; ++mb) {
        uint32_t shift = mb * 8u;
        uint32_t bit = 1u << mb;
        if ((tsr & (1u << shift)) != 0u) {
            if ((tsr & (2u << shift)) != 0u) {
                ++g_platform_diag.tx_count;
                if ((pending_zero_mask & bit) != 0u) {
                    ++zero_tx_count;
                    last_zero_tx_ms = uptime_ms;
                    have_zero_tx = true;
                }
            } else if (can_policy_tx_failed(tsr >> shift,
                                            (intentional_abort_mask & bit) != 0u)) {
                ++g_platform_diag.tx_error_count;
                ++can_error_generation;
                platform_latch_error(PLATFORM_ERR_TX);
            }
            clear |= 1u << shift;
            pending_mask &= ~bit;
            pending_timeout_mask &= ~bit;
            pending_zero_mask &= ~bit;
            intentional_abort_mask &= ~bit;
        }
        if ((pending_mask & (1u << mb)) != 0u &&
            can_policy_tx_expired(uptime_ms, pending_since[mb])) {
            if ((pending_timeout_mask & (1u << mb)) == 0u) {
                pending_timeout_mask |= 1u << mb;
                ++g_platform_diag.tx_timeout_count;
                ++can_error_generation;
            }
            /* First expiry still latches immediately; repeated polls only
             * preserve that fault until completion or a successful abort. */
            platform_latch_error(PLATFORM_ERR_TX_TIMEOUT);
        }
    }
    if (clear != 0u) { CAN_TSR = clear; }
    g_platform_diag.tx_pending_mask = pending_mask;
}

void platform_service(void)
{
    g_platform_diag.error_flags = latched_error_flags;
    g_platform_diag.uart_sr = rc_last_sr;
    g_platform_diag.uart_cr1 = UART_CR1;
    g_platform_diag.uart_brr = UART_BRR;
    g_platform_diag.uart_dma_ndtr = DMA_S1NDTR;
    g_platform_diag.uart_idle_count = rc_idle_count;
    g_platform_diag.uart_frame_count = rc_frame_count;
    g_platform_diag.uart_error_count = rc_uart_error_count;
    g_platform_diag.uart_dma_error_count = rc_dma_error_count;
    g_platform_diag.uart_bad_length_count = rc_bad_length_count;
    g_platform_diag.uart_queue_overflow_count = rc_overflow_count;
    g_platform_diag.uart_last_frame_ms = rc_last_frame_ms;
    g_platform_diag.uart_last_length = rc_last_length;
    g_platform_diag.uart_error_events = rc_error_events;
    g_platform_diag.uart_queue_depth = (uint32_t)(rc_head - rc_tail);
    g_platform_diag.uart_dma_cr = DMA_S1CR;
    g_platform_diag.uart_burst_invalid = rc_burst_invalid ? 1u : 0u;
    if (rc_hardware_fault) { platform_latch_error(PLATFORM_ERR_UART_HW); }
    if (!can_ready) { return; }
    observe_can_esr(CAN_ESR);
    if (((CAN_RF0R | CAN_RF1R) & (1u << 4)) != 0u) {
        ++g_platform_diag.rx_overrun_count;
        platform_latch_error(PLATFORM_ERR_RX_OVERRUN);
        CAN_RF0R = 1u << 4;
        CAN_RF1R = 1u << 4;
    }
    tx_service();

    /* Only this loop releases RFOM. Hardware retains the output mailbox until
     * release, so the four words below belong to one complete received frame. */
    while ((CAN_RF0R & 3u) != 0u) {
        uint32_t rir = CAN_RX(0u);
        uint32_t rdtr = CAN_RX(4u);
        uint32_t low = CAN_RX(8u);
        uint32_t high = CAN_RX(12u);
        CAN_RF0R = 1u << 5;
        ++g_platform_diag.rx_count;
        uint16_t id = (uint16_t)((rir >> 21) & 0x7ffu);
        g_platform_diag.last_rx_id = id;
        g_platform_diag.last_rx_ms = uptime_ms;
        if ((rir & 6u) != 0u || (rdtr & 15u) != 8u) {
            ++g_platform_diag.rx_invalid_count;
            /* Discard a malformed application frame without refreshing any
             * wheel's valid-feedback timestamp. A missing valid stream still
             * stops the controller through its normal feedback deadline. */
        } else if (id == 0x200u || id == 0x1ffu) {
            /* Normal mode does not receive its own TX. A second current
             * controller can contend for identifiers and must inhibit motion. */
            ++g_platform_diag.conflict_count;
            platform_latch_error(PLATFORM_ERR_OTHER_HOST);
        } else if ((id >= 0x201u && id <= 0x204u) || id == 0x207u) {
            uint8_t data[8];
            for (uint32_t j = 0; j < 4u; ++j) {
                data[j] = (uint8_t)(low >> (j * 8u));
                data[j + 4u] = (uint8_t)(high >> (j * 8u));
            }
            ++g_platform_diag.rx_known_count;
            if ((((uint32_t)data[0] << 8) | data[1]) >= 8192u) {
                ++g_platform_diag.rx_invalid_count;
            } else {
                chassis_feedback(id, data, uptime_ms);
                can_tx_ready = true;
            }
        } else {
            ++g_platform_diag.rx_unknown_count;
            g_platform_diag.last_unknown_id = id;
        }
        if (g_stop_request != 0u || (GPIO_IDR(GPIOA_BASE) & 1u) == 0u) {
            break;                      /* Stop is handled immediately by main. */
        }
        if (((CAN_RF0R | CAN_RF1R) & (1u << 4)) != 0u) {
            ++g_platform_diag.rx_overrun_count;
            platform_latch_error(PLATFORM_ERR_RX_OVERRUN);
            CAN_RF0R = 1u << 4;
            CAN_RF1R = 1u << 4;
        }
    }
}

static bool abort_pending(void)
{
    uint32_t tsr = CAN_TSR;
    if ((tsr & CAN_TME_ALL) != CAN_TME_ALL) {
        ++g_platform_diag.tx_abort_count;
        /* RQCP also means a software-requested abort completed. Do not turn
         * cancellation of an old command into a new independent TX failure. */
        intentional_abort_mask |= (~tsr >> 26) & 7u;
        CAN_TSR = CAN_ABORT_ALL;
        if (!wait_bits(&CAN_TSR, CAN_TME_ALL, CAN_TME_ALL)) {
            ++g_platform_diag.abort_failed_count;
            platform_latch_error(PLATFORM_ERR_TX_ABORT);
            CAN_MCR |= 1u;              /* Stop controller if abort cannot finish. */
            can_ready = false;
            return false;
        }
    }
    tx_service();
    CAN_TSR = CAN_COMPLETED_ALL;
    pending_mask = 0u;
    pending_timeout_mask = 0u;
    pending_zero_mask = 0u;
    intentional_abort_mask = 0u;
    g_platform_diag.tx_pending_mask = 0u;
    return true;
}

static bool queue_frame(uint16_t id, const int16_t values[4])
{
    if (!can_ready || !can_tx_ready) { return false; }
    uint32_t tsr = CAN_TSR;
    uint32_t mb;
    for (mb = 0u; mb < 3u; ++mb) {
        if ((tsr & (1u << (26u + mb))) != 0u) { break; }
    }
    if (mb == 3u) { return false; }
    uint8_t b[8];
    bool all_zero = true;
    for (uint32_t j = 0u; j < 4u; ++j) {
        if (values[j] != 0) { all_zero = false; }
        uint16_t v = (uint16_t)values[j];
        b[2u * j] = (uint8_t)(v >> 8);
        b[2u * j + 1u] = (uint8_t)v;
    }
    uint32_t lo = (uint32_t)b[0] | ((uint32_t)b[1] << 8) |
                  ((uint32_t)b[2] << 16) | ((uint32_t)b[3] << 24);
    uint32_t hi = (uint32_t)b[4] | ((uint32_t)b[5] << 8) |
                  ((uint32_t)b[6] << 16) | ((uint32_t)b[7] << 24);
    CAN_TX(mb, 0u) = (uint32_t)id << 21;
    CAN_TX(mb, 4u) = 8u;
    CAN_TX(mb, 8u) = lo;
    CAN_TX(mb, 12u) = hi;
    pending_since[mb] = uptime_ms;
    pending_timeout_mask &= ~(1u << mb);
    intentional_abort_mask &= ~(1u << mb);
    if (all_zero) { pending_zero_mask |= 1u << mb; }
    else { pending_zero_mask &= ~(1u << mb); }
    pending_mask |= 1u << mb;
    __asm volatile("dmb" ::: "memory");
    CAN_TX(mb, 0u) |= 1u;
    ++g_platform_diag.tx_queued_count;
    g_platform_diag.tx_pending_mask = pending_mask;
    return true;
}

void platform_send_currents(const int16_t currents[8], bool output_enabled)
{
    if (!can_ready || !can_tx_ready) { return; }
    tx_service();
    (void)platform_stop_requested();
    /* Cut an existing enabled output immediately. Once zero is established,
     * a permanent UART fault must not repeatedly abort zero frames or bypass
     * the normal 200 Hz transmit rate. Main also handles the first hard fault. */
    if (platform_rc_has_errors() && g_platform_diag.output_enabled != 0u) {
        platform_emergency_zero();
        return;
    }
    /* Control computes at 1 kHz, but two CAN current groups transmit at 200 Hz.
     * This leaves bandwidth for five 1 kHz ESC feedback streams. */
    if (has_pair && (uint32_t)(uptime_ms - last_pair_ms) < CAN_PAIR_INTERVAL_MS) {
        return;
    }
    /* On errors only zeros are permitted, even if the caller passes nonzero. */
    int16_t out[8] = {0};
    bool enabled = platform_healthy() && output_enabled && !platform_rc_has_errors();
    if (enabled) {
        for (uint32_t i = 0; i < 8u; ++i) { out[i] = currents[i]; }
        out[4] = 0;
        out[5] = 0;
        out[6] = 0;                     /* Feeder ID7 is never controlled here. */
        out[7] = 0;
    }
    g_platform_diag.output_enabled = enabled ? 1u : 0u;
    if ((CAN_TSR & CAN_TME_ALL) != CAN_TME_ALL) {
        /* Previous 5 ms command pair should have finished. Never accumulate
         * stale setpoints. An incomplete pair is a transport fault.
         * tx_service already counted each expired mailbox transaction. */
        platform_latch_error(PLATFORM_ERR_TX_TIMEOUT);
        platform_emergency_zero();
        return;
    }
    if (!queue_frame(0x200u, &out[0]) || !queue_frame(0x1ffu, &out[4])) {
        platform_latch_error(PLATFORM_ERR_TX);
        platform_emergency_zero();
    } else {
        last_pair_ms = uptime_ms;
        has_pair = true;
    }
}

void platform_emergency_zero(void)
{
    static const int16_t zero[4] = {0, 0, 0, 0};
    g_platform_diag.output_enabled = 0u;
    if (!can_ready || !can_tx_ready || !abort_pending()) { return; }
    (void)queue_frame(0x200u, zero);
    (void)queue_frame(0x1ffu, zero);
    last_pair_ms = uptime_ms;
    has_pair = true;
}

bool platform_try_can_recovery(uint32_t now_ms, bool safe_off)
{
    bool stop = platform_stop_requested();
    uint32_t sampled_esr = 0u;
    if (can_ready) {
        sampled_esr = CAN_ESR;
        observe_can_esr(sampled_esr);
        tx_service();
    }
    /* A completion above may have crossed a SysTick after the caller sampled
     * now_ms. Use a fresh time so last_zero_tx_ms cannot appear in the future. */
    now_ms = platform_millis();
    can_policy_sample_t sample = {
        .now_ms = now_ms,
        .fault_flags = latched_error_flags,
        .esr = sampled_esr,
        .zero_tx_count = zero_tx_count,
        .last_zero_tx_ms = last_zero_tx_ms,
        .error_generation = can_error_generation,
        .safe_off = safe_off && !stop && !platform_rc_has_errors(),
        .output_zero = !chassis_output_enabled() &&
                       (pending_mask & ~pending_zero_mask) == 0u,
        .can_ready = can_ready,
        .tx_ready = can_tx_ready,
        .have_zero_tx = have_zero_tx,
        .pending_empty = pending_mask == 0u &&
                         (CAN_TSR & CAN_TME_ALL) == CAN_TME_ALL
    };
    if (!can_policy_recovery_step(&can_recovery, &sample)) { return false; }
    /* Re-read the live link at release; never clear unrelated fault bits,
     * overwrite a pending command, reset counters, or enable an output. */
    uint32_t esr = CAN_ESR;
    observe_can_esr(esr);
    if (esr != 0u || pending_mask != 0u ||
        (CAN_TSR & CAN_TME_ALL) != CAN_TME_ALL ||
        (latched_error_flags & ~CAN_POLICY_RECOVERABLE_ERRORS) != 0u ||
        platform_stop_requested() || platform_rc_has_errors()) {
        return false;
    }
    latched_error_flags &= ~CAN_POLICY_RECOVERABLE_ERRORS;
    g_platform_diag.error_flags = latched_error_flags;
    ++g_platform_diag.reserved0;        /* Successful explicit CAN recoveries. */
    g_platform_diag.reserved1 = now_ms; /* Last recovery time; history retained. */
    return true;
}

void platform_panic(uint32_t reason)
{
    g_platform_diag.seq |= 1u;
    platform_latch_error(reason);
    g_stop_request = 1u;
    platform_emergency_zero();
    /* IWDG is deliberately not fed: reset then wait for another explicit start.
     * Before watchdog initialization the fail-stop loop leaves CAN inactive. */
    for (;;) { __asm volatile("nop"); }
}

void Default_Handler(void) { platform_panic(PLATFORM_ERR_EXCEPTION); }
