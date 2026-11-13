/* Direct tests of dr16.c with an independent bit-field frame constructor.
 * No firmware, transmitter, receiver, or physical motor validation is implied.
 */
#include "dr16.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;

#define CHECK(c) do { ++checks; if (!(c)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #c); exit(1); \
} } while (0)

/* Generic bit writer is deliberately different from the production decoder. */
static void field(uint8_t data[18], unsigned bit, unsigned width, uint32_t value) {
    unsigned n;
    for (n = 0; n < width; ++n) {
        unsigned at = bit + n;
        uint8_t mask = (uint8_t)(1u << (at % 8u));
        if ((value >> n) & 1u) data[at / 8u] |= mask;
        else data[at / 8u] &= (uint8_t)~mask;
    }
}

static void neutral(uint8_t data[18]) {
    unsigned i;
    memset(data, 0, 18);
    for (i = 0; i < 4; ++i) field(data, i * 11, 11, 1024);
    field(data, 44, 2, DR16_SWITCH_MIDDLE);
    field(data, 46, 2, DR16_SWITCH_DOWN);
    field(data, 128, 16, 1024);
}

static void test_golden_center_and_extremes(void) {
    const uint8_t golden[18] = {0x00,0x04,0x20,0x00,0x01,0xb8,
                                0,0,0,0,0,0,0,0,0,0,0x00,0x04};
    uint8_t data[18];
    dr16_frame_t out;
    unsigned i;
    neutral(data);
    CHECK(memcmp(data, golden, 18) == 0);
    CHECK(dr16_decode(golden, 18, 123, &out) == DR16_OK);
    CHECK(sizeof(out) == 100);
    CHECK(out.switch_left == DR16_SWITCH_MIDDLE);
    CHECK(out.switch_right == DR16_SWITCH_DOWN);
    CHECK(out.last_frame_ms == 123 && out.wheel_valid == 1);
    for (i = 0; i < 5; ++i) {
        CHECK(out.raw[i] == 1024);
        CHECK(out.centered[i] == 0 && out.normalized[i] == 0);
    }
    for (i = 0; i < 5; ++i) {
        unsigned offset = i < 4 ? i * 11 : 128;
        unsigned width = i < 4 ? 11 : 16;
        neutral(data); field(data, offset, width, 364);
        CHECK(dr16_decode(data, 18, 0, &out) == DR16_OK);
        CHECK(out.raw[i] == 364 && out.centered[i] == -660 && out.normalized[i] == -1000);
        field(data, offset, width, 1684);
        CHECK(dr16_decode(data, 18, 0, &out) == DR16_OK);
        CHECK(out.raw[i] == 1684 && out.centered[i] == 660 && out.normalized[i] == 1000);
    }
    puts("PASS golden center packet, physical channel indices, all five extrema");
}

static void test_every_channel_code(void) {
    uint8_t data[18];
    dr16_frame_t out, before;
    unsigned channel, raw, other;
    for (channel = 0; channel < 4; ++channel) {
        for (raw = 0; raw < 2048; ++raw) {
            neutral(data);
            field(data, channel * 11, 11, raw);
            memset(&out, 0xa5, sizeof(out)); before = out;
            if (raw >= 364 && raw <= 1684) {
                CHECK(dr16_decode(data, 18, raw, &out) == DR16_OK);
                CHECK(out.raw[channel] == raw);
                for (other = 0; other < 4; ++other)
                    if (other != channel) CHECK(out.raw[other] == 1024);
                CHECK(out.switch_left == 3 && out.switch_right == 2);
            } else {
                CHECK(dr16_decode(data, 18, raw, &out) == DR16_ERROR_CHANNEL);
                CHECK(memcmp(&out, &before, sizeof(out)) == 0);
            }
        }
    }
    puts("PASS all 2048 wire codes for each main channel and cross-field isolation");
}

static void test_switches_and_length(void) {
    uint8_t data[64];
    dr16_frame_t out, before;
    unsigned left, right, length;
    memset(&out, 0x3c, sizeof(out)); before = out;
    for (left = 0; left < 4; ++left) for (right = 0; right < 4; ++right) {
        neutral(data); field(data, 44, 2, left); field(data, 46, 2, right);
        if (left && right) {
            CHECK(dr16_decode(data, 18, 7, &out) == DR16_OK);
            CHECK(out.switch_left == left && out.switch_right == right);
        } else {
            out = before;
            CHECK(dr16_decode(data, 18, 7, &out) == DR16_ERROR_SWITCH);
            CHECK(memcmp(&out, &before, sizeof(out)) == 0);
        }
    }
    neutral(data);
    for (length = 0; length < sizeof(data); ++length) if (length != 18) {
        out = before;
        CHECK(dr16_decode(data, length, 8, &out) == DR16_ERROR_LENGTH);
        CHECK(memcmp(&out, &before, sizeof(out)) == 0);
    }
    CHECK(dr16_decode(data, (size_t)-1, 8, &out) == DR16_ERROR_LENGTH);
    CHECK(dr16_decode(NULL, 18, 8, &out) == DR16_ERROR_ARGUMENT);
    CHECK(dr16_decode(data, 18, 8, NULL) == DR16_ERROR_ARGUMENT);
    CHECK(memcmp(&out, &before, sizeof(out)) == 0);
    puts("PASS all switch combinations, short/long/null inputs, failure atomicity");
}

static void test_deadzone_and_monotonicity(void) {
    uint8_t data[18];
    dr16_frame_t out;
    int32_t previous = -1001, positive, negative;
    unsigned raw, delta;
    for (raw = 364; raw <= 1684; ++raw) {
        neutral(data); field(data, 0, 11, raw);
        CHECK(dr16_decode(data, 18, 0, &out) == DR16_OK);
        CHECK(out.normalized[0] >= previous);
        CHECK(out.normalized[0] >= -1000 && out.normalized[0] <= 1000);
        CHECK(out.centered[0] == (int32_t)raw - 1024);
        if (raw >= 1004 && raw <= 1044) CHECK(out.normalized[0] == 0);
        else CHECK(out.normalized[0] != 0);
        previous = out.normalized[0];
    }
    for (delta = 0; delta <= 660; ++delta) {
        neutral(data); field(data, 0, 11, 1024 + delta);
        CHECK(dr16_decode(data, 18, 0, &out) == DR16_OK);
        positive = out.normalized[0];
        field(data, 0, 11, 1024 - delta);
        CHECK(dr16_decode(data, 18, 0, &out) == DR16_OK);
        negative = out.normalized[0];
        CHECK(positive == -negative);
    }
    puts("PASS inclusive deadzone, monotonic full-scale and symmetric normalization");
}

static void test_wheel_legacy_and_invalid_codes(void) {
    uint8_t data[18];
    dr16_frame_t out, before;
    uint32_t raw;
    memset(&before, 0x69, sizeof(before));
    for (raw = 0; raw < 65536u; ++raw) {
        neutral(data); field(data, 128, 16, raw); out = before;
        if (raw == 0u || (raw >= 364u && raw <= 1684u)) {
            CHECK(dr16_decode(data, 18, 9, &out) == DR16_OK);
            CHECK(out.raw[4] == raw);
            CHECK(out.wheel_valid == (raw != 0u));
            if (raw == 0u) CHECK(out.centered[4] == 0 && out.normalized[4] == 0);
        } else {
            CHECK(dr16_decode(data, 18, 9, &out) == DR16_ERROR_WHEEL);
            CHECK(memcmp(&out, &before, sizeof(out)) == 0);
        }
    }
    puts("PASS all 65536 wheel encodings, legacy reserved-zero compatibility");
}

static void test_mouse_keyboard_and_timestamp(void) {
    uint8_t data[18];
    dr16_frame_t out, before;
    unsigned value;
    neutral(data);
    field(data, 48, 16, 0x8000); field(data, 64, 16, 0x7fff);
    field(data, 80, 16, 0xffff); field(data, 112, 16, 0xa55a);
    data[12] = 1; data[13] = 0;
    CHECK(dr16_decode(data, 18, UINT32_MAX - 5u, &out) == DR16_OK);
    CHECK(out.mouse_x == -32768 && out.mouse_y == 32767 && out.mouse_z == -1);
    CHECK(out.keyboard == 0xa55a && out.mouse_left == 1 && out.mouse_right == 0);
    CHECK(out.last_frame_ms == UINT32_MAX - 5u);
    before = out;
    CHECK(dr16_decode(data, 17, 8, &out) == DR16_ERROR_LENGTH);
    CHECK(out.last_frame_ms == before.last_frame_ms);
    CHECK((uint32_t)(8u - out.last_frame_ms) == 14u);
    CHECK(dr16_decode(data, 18, 8, &out) == DR16_OK);
    CHECK(out.last_frame_ms == 8u);
    /* Pure decoding does not count, reorder, or authorize frames. */
    CHECK(dr16_decode(data, 18, 8, &out) == DR16_OK);
    CHECK(out.last_frame_ms == 8u);
    CHECK(dr16_decode(data, 18, 7, &out) == DR16_OK);
    CHECK(out.last_frame_ms == 7u);
    for (value = 2; value < 256; ++value) {
        before = out; data[12] = (uint8_t)value;
        CHECK(dr16_decode(data, 18, 10, &out) == DR16_ERROR_MOUSE_BUTTON);
        CHECK(memcmp(&out, &before, sizeof(out)) == 0);
        data[12] = 0; data[13] = (uint8_t)value;
        CHECK(dr16_decode(data, 18, 10, &out) == DR16_ERROR_MOUSE_BUTTON);
        CHECK(memcmp(&out, &before, sizeof(out)) == 0);
        data[13] = 0;
    }
    puts("PASS signed mouse, keyboard bits, timestamp/wrap and invalid-frame freshness");
}

int main(void) {
    test_golden_center_and_extremes();
    test_every_channel_code();
    test_switches_and_length();
    test_deadzone_and_monotonicity();
    test_wheel_legacy_and_invalid_codes();
    test_mouse_keyboard_and_timestamp();
    printf("ALL PASS: %u assertions; C parser tests only\n", checks);
    return 0;
}
