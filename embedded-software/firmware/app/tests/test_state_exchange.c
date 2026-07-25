/*
 * Host-side unit tests for app/shared/state_exchange.c — specifically the
 * tunables slots (pid_gains / reference / vehicle_config), whose returned
 * change counter must be meaningful ACROSS cores: CM4 publishes, CM7 polls
 * and compares counters to decide whether to merge.
 *
 * The cross-core split is simulated in-process: each core's image has its own
 * zero-initialised .bss, so "the other core" is emulated by resetting the
 * module's per-core seq statics to 0 via IO_TEST hooks after publishing.
 * Regression context: the getters used to return a per-core publish counter,
 * which on the reading core's image stayed 0 forever — poll_tunables() never
 * saw a change and uplinked PID gains / z_ref never reached the controller.
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

IO_TEST_HOOK_DECL_RW(uint32_t, state_exchange_pid_gains_seq)
IO_TEST_HOOK_DECL_RW(uint32_t, state_exchange_reference_seq)
IO_TEST_HOOK_DECL_RW(uint32_t, state_exchange_vehicle_cfg_seq)
IO_TEST_HOOK_DECL_RW(uint32_t, state_exchange_readiness_seq)
IO_TEST_HOOK_DECL_RW(bool,     state_exchange_initialised)

void setUp(void) {}
void tearDown(void) {}

/* Reset the module as if this image had just booted, over a shared region
 * filled with @p fill (power-on garbage pattern; 0x00 for a benign region). */
static void reset_module(uint8_t fill)
{
    memset(__shared_region_start__, fill, sizeof(__shared_region_start__));
    IO_TEST_set_state_exchange_initialised(false);
    IO_TEST_set_state_exchange_pid_gains_seq(0);
    IO_TEST_set_state_exchange_reference_seq(0);
    IO_TEST_set_state_exchange_vehicle_cfg_seq(0);
    IO_TEST_set_state_exchange_readiness_seq(0);
    state_exchange_init();
}

/* Emulate the reading core's freshly booted image: its .bss statics are 0
 * while the shared region keeps whatever the writing core published. */
static void fake_reader_boot(void)
{
    IO_TEST_set_state_exchange_pid_gains_seq(0);
    IO_TEST_set_state_exchange_reference_seq(0);
    IO_TEST_set_state_exchange_vehicle_cfg_seq(0);
    IO_TEST_set_state_exchange_readiness_seq(0);
}

static uint32_t *slot_seq_ptr(size_t offset)
{
    /* io_intercore_slot_hdr_t.seq is the first word of every slot. */
    return (uint32_t *)(void *)(__shared_region_start__ + offset);
}

/* ── The regression: a publish must be visible to the OTHER core's poll ──── */

void test_pid_gains_publish_visible_to_fresh_reader(void)
{
    reset_module(0x00);

    app_pid_gains_t gains;
    memset(&gains, 0, sizeof(gains));
    gains.z_kp             = 2.5f;
    gains.z_ki             = 0.75f;
    gains.z_integral_limit = 1.5f;
    (void)state_exchange_publish_pid_gains(&gains);

    fake_reader_boot();

    app_pid_gains_t got;
    memset(&got, 0, sizeof(got));
    uint32_t seq = state_exchange_get_pid_gains(&got);

    /* Old behaviour: seq == 0 (reader-local counter), merge gate never
     * opened. New behaviour: the shared counter comes back. */
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_FLOAT(2.5f,  got.z_kp);
    TEST_ASSERT_EQUAL_FLOAT(0.75f, got.z_ki);
    TEST_ASSERT_EQUAL_FLOAT(1.5f,  got.z_integral_limit);
}

void test_reference_publish_visible_to_fresh_reader(void)
{
    reset_module(0x00);

    app_reference_t ref;
    memset(&ref, 0, sizeof(ref));
    ref.z_ref  = 12.0f;
    ref.vz_ref = -0.5f;
    (void)state_exchange_publish_reference(&ref);

    fake_reader_boot();

    app_reference_t got;
    memset(&got, 0, sizeof(got));
    uint32_t seq = state_exchange_get_reference(&got);

    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_FLOAT(12.0f, got.z_ref);
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, got.vz_ref);
}

void test_vehicle_config_publish_visible_to_fresh_reader(void)
{
    reset_module(0x00);

    app_vehicle_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mass_kg = 3.2f;
    cfg.T_max   = 40.0f;
    (void)state_exchange_publish_vehicle_config(&cfg);

    fake_reader_boot();

    app_vehicle_config_t got;
    memset(&got, 0, sizeof(got));
    uint32_t seq = state_exchange_get_vehicle_config(&got);

    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_FLOAT(3.2f,  got.mass_kg);
    TEST_ASSERT_EQUAL_FLOAT(40.0f, got.T_max);
}

void test_readiness_publish_visible_and_garbage_safe(void)
{
    reset_module(0x00);

    vehicle_readiness_t rdy;
    memset(&rdy, 0, sizeof(rdy));
    rdy.imu     = SENSOR_CAL_READY;
    rdy.armable = true;
    (void)state_exchange_publish_readiness(&rdy);

    fake_reader_boot();
    vehicle_readiness_t got;
    memset(&got, 0, sizeof(got));
    uint32_t seq = state_exchange_get_readiness(&got);
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_INT(SENSOR_CAL_READY, got.imu);
    TEST_ASSERT_TRUE(got.armable);

    /* Power-on garbage must never fake armable=true — this slot gates ARM. */
    reset_module(0xA6);
    memset(&got, 0, sizeof(got));
    got.armable = false;
    seq = state_exchange_get_readiness(&got);
    TEST_ASSERT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_FALSE(got.armable);
}

/* ── poll_tunables()-style gate: fires once per publish, then stays shut ── */

void test_poll_gate_fires_once_per_publish(void)
{
    reset_module(0x00);

    app_pid_gains_t gains;
    memset(&gains, 0, sizeof(gains));
    gains.z_ki = 0.1f;
    (void)state_exchange_publish_pid_gains(&gains);

    fake_reader_boot();
    uint32_t last = 0;

    app_pid_gains_t got;
    uint32_t seq = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_TRUE(seq != last);          /* merge fires */
    last = seq;

    seq = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_TRUE(seq == last);          /* no new publish → no merge */

    gains.z_ki = 0.2f;
    (void)state_exchange_publish_pid_gains(&gains);
    seq = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_TRUE(seq != last);          /* second publish detected */
    TEST_ASSERT_EQUAL_FLOAT(0.2f, got.z_ki);
}

/* ── Power-on garbage must never masquerade as a publish ────────────────── */

void test_even_garbage_rejected_by_magic(void)
{
    /* 0xA6A6A6A6 is an even "stable" seqlock counter but the magic word
     * cannot match — the getter must ignore the slot entirely. */
    reset_module(0xA6);

    app_pid_gains_t got;
    memset(&got, 0x5A, sizeof(got));
    app_pid_gains_t sentinel = got;

    uint32_t seq = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &got, sizeof(got));

    app_reference_t ref_got;
    memset(&ref_got, 0x5A, sizeof(ref_got));
    app_reference_t ref_sentinel = ref_got;
    seq = state_exchange_get_reference(&ref_got);
    TEST_ASSERT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_MEMORY(&ref_sentinel, &ref_got, sizeof(ref_got));
}

void test_odd_garbage_rejected_as_write_in_progress(void)
{
    /* 0xA7A7A7A7 is odd → seqlock read retries then fails. */
    reset_module(0xA7);

    app_pid_gains_t got;
    memset(&got, 0x5A, sizeof(got));
    app_pid_gains_t sentinel = got;

    uint32_t seq = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_EQUAL_UINT32(0U, seq);
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &got, sizeof(got));
}

/* ── Mid-update writer: keep last-good data and counter ─────────────────── */

void test_write_in_progress_returns_last_good(void)
{
    reset_module(0x00);

    app_pid_gains_t gains;
    memset(&gains, 0, sizeof(gains));
    gains.z_kp = 4.0f;
    (void)state_exchange_publish_pid_gains(&gains);

    fake_reader_boot();
    app_pid_gains_t got;
    uint32_t seq_good = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_NOT_EQUAL_UINT32(0U, seq_good);

    /* Freeze the slot mid-write (odd counter). */
    uint32_t *hdr_seq = slot_seq_ptr(APP_SLOT_PID_GAINS_OFFSET);
    uint32_t saved = *hdr_seq;
    *hdr_seq = saved | 1U;

    memset(&got, 0x5A, sizeof(got));
    app_pid_gains_t sentinel = got;
    uint32_t seq = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_EQUAL_UINT32(seq_good, seq);   /* unchanged → caller won't merge */
    TEST_ASSERT_EQUAL_MEMORY(&sentinel, &got, sizeof(got));

    *hdr_seq = saved;
    seq = state_exchange_get_pid_gains(&got);
    TEST_ASSERT_EQUAL_UINT32(seq_good, seq);
    TEST_ASSERT_EQUAL_FLOAT(4.0f, got.z_kp);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_pid_gains_publish_visible_to_fresh_reader);
    RUN_TEST(test_reference_publish_visible_to_fresh_reader);
    RUN_TEST(test_vehicle_config_publish_visible_to_fresh_reader);
    RUN_TEST(test_readiness_publish_visible_and_garbage_safe);
    RUN_TEST(test_poll_gate_fires_once_per_publish);
    RUN_TEST(test_even_garbage_rejected_by_magic);
    RUN_TEST(test_odd_garbage_rejected_as_write_in_progress);
    RUN_TEST(test_write_in_progress_returns_last_good);
    return UNITY_END();
}
