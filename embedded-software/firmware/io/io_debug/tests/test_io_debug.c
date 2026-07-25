/**
 * @file    test_io_debug.c
 * @brief   Host unit tests for the portable io_debug formatter.
 *
 * The formatter (io_debug_printf / io_debug_puts) is the only host-testable
 * part of the debug console — the transport (io_debug_write) is hardware
 * (HAL UART on LPUART1) and is faked here so we can assert exactly what bytes
 * the formatter would put on the wire.
 *
 * UBC Rocket, 2026
 */
#include "unity.h"
#include "io_sys/io_debug.h"

#include <string.h>

/* ---- Fake transport: capture whatever the formatter writes ------------- */

static uint8_t  g_cap[1024];
static size_t   g_cap_len;
static io_status_t g_write_ret;     /* what the fake io_debug_write returns */
static int      g_write_calls;

io_status_t io_debug_write(const uint8_t *data, size_t len) {
    g_write_calls++;
    if (data == NULL || len == 0U) return IO_ERR_PARAM;
    if (g_write_ret != IO_OK) return g_write_ret;
    if (g_cap_len + len <= sizeof(g_cap)) {
        memcpy(g_cap + g_cap_len, data, len);
        g_cap_len += len;
    }
    return IO_OK;
}

void setUp(void) {
    memset(g_cap, 0, sizeof(g_cap));
    g_cap_len    = 0;
    g_write_ret  = IO_OK;
    g_write_calls = 0;
}

void tearDown(void) {}

/* ---- Tests ------------------------------------------------------------- */

/* A plain string passes through byte-for-byte and reports its length. */
static void test_plain_string_passthrough(void) {
    int n = io_debug_printf("hello world\r\n");
    TEST_ASSERT_EQUAL_INT(13, n);
    TEST_ASSERT_EQUAL_size_t(13U, g_cap_len);
    TEST_ASSERT_EQUAL_MEMORY("hello world\r\n", g_cap, 13);
}

/* Integer conversions are formatted correctly. */
static void test_integer_format(void) {
    int n = io_debug_printf("tick=%d ok=%u", 42, 7U);
    TEST_ASSERT_EQUAL_INT(12, n);
    TEST_ASSERT_EQUAL_MEMORY("tick=42 ok=7", g_cap, 12);
}

/* A NULL format string is rejected without touching the transport. */
static void test_null_format_rejected(void) {
    int n = io_debug_printf(NULL);
    TEST_ASSERT_EQUAL_INT(-1, n);
    TEST_ASSERT_EQUAL_INT(0, g_write_calls);
}

/* Output longer than the line buffer is truncated, never overflows, and the
 * returned/written length is clamped to IO_DEBUG_LINE_MAX-1. */
static void test_truncation_clamped(void) {
    char big[IO_DEBUG_LINE_MAX * 2];
    memset(big, 'A', sizeof(big) - 1U);
    big[sizeof(big) - 1U] = '\0';

    int n = io_debug_printf("%s", big);
    TEST_ASSERT_EQUAL_INT((int)(IO_DEBUG_LINE_MAX - 1U), n);
    TEST_ASSERT_EQUAL_size_t(IO_DEBUG_LINE_MAX - 1U, g_cap_len);
}

/* A transport failure is surfaced to the caller as -1. */
static void test_write_failure_propagates(void) {
    g_write_ret = IO_ERR_TIMEOUT;
    int n = io_debug_printf("x");
    TEST_ASSERT_EQUAL_INT(-1, n);
}

/* An empty formatted result writes nothing and reports zero. */
static void test_empty_output(void) {
    int n = io_debug_printf("%s", "");
    TEST_ASSERT_EQUAL_INT(0, n);
    TEST_ASSERT_EQUAL_INT(0, g_write_calls);
}

/* io_debug_puts sends the string verbatim (no NUL, no formatting). */
static void test_puts_verbatim(void) {
    io_status_t st = io_debug_puts("abc");
    TEST_ASSERT_EQUAL_INT(IO_OK, st);
    TEST_ASSERT_EQUAL_size_t(3U, g_cap_len);
    TEST_ASSERT_EQUAL_MEMORY("abc", g_cap, 3);
}

/* io_debug_puts rejects NULL. */
static void test_puts_null_rejected(void) {
    TEST_ASSERT_EQUAL_INT(IO_ERR_PARAM, io_debug_puts(NULL));
    TEST_ASSERT_EQUAL_INT(0, g_write_calls);
}

int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_plain_string_passthrough);
    RUN_TEST(test_integer_format);
    RUN_TEST(test_null_format_rejected);
    RUN_TEST(test_truncation_clamped);
    RUN_TEST(test_write_failure_propagates);
    RUN_TEST(test_empty_output);
    RUN_TEST(test_puts_verbatim);
    RUN_TEST(test_puts_null_rejected);
    return UNITY_END();
}
