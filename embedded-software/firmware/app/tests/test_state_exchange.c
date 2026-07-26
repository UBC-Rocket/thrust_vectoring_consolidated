/*
 * Host-side unit tests for app/shared/state_exchange.c — specifically the
 * throttle-command and motor-RPM slots, whose returned change counter must be
 * meaningful ACROSS cores: CM4 publishes throttle, CM7 polls and compares
 * counters to decide whether to apply; CM7 publishes RPM, CM4 telemetry reads.
 *
 * The cross-core split is simulated in-process: each core's image has its own
 * zero-initialised .bss, so "the other core" is emulated by resetting the
 * module's per-core seq statics to 0 via IO_TEST hooks after publishing.
 *
 * These two slots also carry a magic word: the .shared region is NOLOAD, so
 * power-on garbage can sit behind an even ("stable") seqlock counter. A
 * garbage float must never become a throttle command.
 */
#include "unity.h"

#include "app/shared_memory.h"
#include "app/state_exchange.h"
#include "io_sys/io_intercore.h"
#include "io_sys/io_test_hooks.h"

#include <stdbool.h>
#include <string.h>

/* Host-side backing for the shared region; on target the linker places this
 * in SRAM4 (NOLOAD, so tests fill it with garbage to mimic power-on state). */
uint8_t __shared_region_start__[APP_SHARED_REGION_SIZE] __attribute__((aligned(8)));

/* state_exchange.c signals the other core on state/control/flight publishes;
 * there is no other core here. */
void io_intercore_signal(io_ic_event_t evt) { (void)evt; }

IO_TEST_HOOK_DECL_RW(uint32_t, state_exchange_throttle_seq)
IO_TEST_HOOK_DECL_RW(uint32_t, state_exchange_motor_rpm_seq)
IO_TEST_HOOK_DECL_RW(bool,     state_exchange_initialised)

void setUp(void) {}
void tearDown(void) {}

/* Reset the module as if this image had just booted, over a shared region
 * filled with @p fill (power-on garbage pattern; 0x00 for a benign region). */
static void reset_module(uint8_t fill)
{
    memset(__shared_region_start__, fill, sizeof(__shared_region_start__));
    IO_TEST_set_state_exchange_initialised(false);
    IO_TEST_set_state_exchange_throttle_seq(0);
    IO_TEST_set_state_exchange_motor_rpm_seq(0);
    state_exchange_init();
}

/* Emulate the reading core's freshly booted image: its .bss statics are 0
 * while the shared region keeps whatever the writing core published. */
static void fake_reader_boot(void)
{
    IO_TEST_set_state_exchange_throttle_seq(0);
    IO_TEST_set_state_exchange_motor_rpm_seq(0);
}

static uint32_t *slot_seq_ptr(size_t offset)
{
    /* io_intercore_slot_hdr_t.seq is the first word of every slot. */
    return (uint32_t *)(void *)(__shared_region_start__ + offset);
}

/* ── A publish must be visible to the OTHER core's poll ─────────────────── */

void test_throttle_and_motor_rpm_slots_cross_core(void)
{
    reset_module(0x00);

    /* CM4 → CM7: operator throttle command, one value per motor. */
    app_throttle_cmd_t thr = { .throttle_lower = 0.42f, .throttle_upper = 0.55f };
    (void)state_exchange_publish_throttle_cmd(&thr);
    /* CM7 → CM4: RPM readback. */
    app_motor_rpm_t rpm = { .rpm_lower = 8342.0f, .rpm_upper = 8127.5f, .valid = true };
    (void)state_exchange_publish_motor_rpm(&rpm);

    fake_reader_boot();

    app_throttle_cmd_t thr_got;
    memset(&thr_got, 0, sizeof(thr_got));
    uint32_t seq = state_exchange_get_throttle_cmd(&thr_got);
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_FLOAT(0.42f, thr_got.throttle_lower);
    TEST_ASSERT_EQUAL_FLOAT(0.55f, thr_got.throttle_upper);

    app_motor_rpm_t rpm_got;
    memset(&rpm_got, 0, sizeof(rpm_got));
    seq = state_exchange_get_motor_rpm(&rpm_got);
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_FLOAT(8342.0f, rpm_got.rpm_lower);
    TEST_ASSERT_EQUAL_FLOAT(8127.5f, rpm_got.rpm_upper);
    TEST_ASSERT_TRUE(rpm_got.valid);
}

/* Meaningful zero: upper=0 with lower>0 must survive the trip (a zeroed
 * upper motor during coaxial bench characterization is a real command). */
void test_throttle_meaningful_zero_upper(void)
{
    reset_module(0x00);

    app_throttle_cmd_t thr = { .throttle_lower = 0.30f, .throttle_upper = 0.0f };
    (void)state_exchange_publish_throttle_cmd(&thr);

    fake_reader_boot();
    app_throttle_cmd_t got;
    memset(&got, 0x5A, sizeof(got));
    uint32_t seq = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_FLOAT(0.30f, got.throttle_lower);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,  got.throttle_upper);
}

/* ── poll_tunables()-style gate: fires once per publish, then stays shut ── */

void test_poll_gate_fires_once_per_publish(void)
{
    reset_module(0x00);

    app_throttle_cmd_t thr = { .throttle_lower = 0.10f, .throttle_upper = 0.10f };
    (void)state_exchange_publish_throttle_cmd(&thr);

    fake_reader_boot();
    uint32_t last = 0;

    app_throttle_cmd_t got;
    uint32_t seq = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_TRUE(seq != last);          /* apply fires */
    last = seq;

    seq = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_TRUE(seq == last);          /* no new publish → no re-apply */

    thr.throttle_lower = 0.20f;
    (void)state_exchange_publish_throttle_cmd(&thr);
    seq = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_TRUE(seq != last);          /* second publish detected */
    TEST_ASSERT_EQUAL_FLOAT(0.20f, got.throttle_lower);
}

/* ── Power-on garbage must never masquerade as a publish ────────────────── */

void test_even_garbage_rejected_by_magic(void)
{
    /* 0xA6A6A6A6 is an even "stable" seqlock counter but the magic word
     * cannot match — the getter must ignore the slot entirely. */
    reset_module(0xA6);

    app_throttle_cmd_t thr_got;
    memset(&thr_got, 0x5A, sizeof(thr_got));
    app_throttle_cmd_t sentinel = thr_got;
    uint32_t seq = state_exchange_get_throttle_cmd(&thr_got);
    TEST_ASSERT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &thr_got, sizeof(thr_got));

    app_motor_rpm_t rpm_got;
    memset(&rpm_got, 0, sizeof(rpm_got));
    rpm_got.valid = false;
    seq = state_exchange_get_motor_rpm(&rpm_got);
    TEST_ASSERT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_FALSE(rpm_got.valid);
}

void test_odd_garbage_rejected_as_write_in_progress(void)
{
    /* 0xA7A7A7A7 is odd → seqlock read retries then fails. */
    reset_module(0xA7);

    app_throttle_cmd_t got;
    memset(&got, 0x5A, sizeof(got));
    app_throttle_cmd_t sentinel = got;

    uint32_t seq = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &got, sizeof(got));
}

/* ── Mid-update writer: keep last-good data and counter ─────────────────── */

void test_write_in_progress_returns_last_good(void)
{
    reset_module(0x00);

    app_throttle_cmd_t thr = { .throttle_lower = 0.40f, .throttle_upper = 0.40f };
    (void)state_exchange_publish_throttle_cmd(&thr);

    fake_reader_boot();
    app_throttle_cmd_t got;
    uint32_t seq_good = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq_good);

    /* Freeze the slot mid-write (odd counter). */
    uint32_t *hdr_seq = slot_seq_ptr(APP_SLOT_THROTTLE_CMD_OFFSET);
    uint32_t saved = *hdr_seq;
    *hdr_seq = saved | 1U;

    memset(&got, 0x5A, sizeof(got));
    app_throttle_cmd_t sentinel = got;
    uint32_t seq = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_EQUAL_UINT32(seq_good, seq);   /* unchanged → caller won't apply */
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &got, sizeof(got));

    *hdr_seq = saved;
    seq = state_exchange_get_throttle_cmd(&got);
    TEST_ASSERT_EQUAL_UINT32(seq_good, seq);
    TEST_ASSERT_EQUAL_FLOAT(0.40f, got.throttle_lower);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_throttle_and_motor_rpm_slots_cross_core);
    RUN_TEST(test_throttle_meaningful_zero_upper);
    RUN_TEST(test_poll_gate_fires_once_per_publish);
    RUN_TEST(test_even_garbage_rejected_by_magic);
    RUN_TEST(test_odd_garbage_rejected_as_write_in_progress);
    RUN_TEST(test_write_in_progress_returns_last_good);
    return UNITY_END();
}
