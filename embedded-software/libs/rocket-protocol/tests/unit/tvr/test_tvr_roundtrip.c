/*
 * Wire round-trip tests over the real tvr messages (committed generated
 * sources) and rp_packet framing — the exact encode path the ground station
 * runs and the exact decode path the flight controller runs, so a pass here
 * verifies GCS↔FC transmission compatibility for these fields end to end
 * (everything except the radio itself).
 */
#include "rp/codec.h"
#include "unity.h"

#include "command.pb.h"
#include "downlink.pb.h"

#include <string.h>

void setUp(void) {}
void tearDown(void) {}

/* Frame + unframe helper: GCS-side encode, FC-side decode (or vice versa). */
static void roundtrip(const pb_msgdesc_t *fields, const void *in, void *out)
{
    uint8_t packet[300];
    rp_packet_encode_result_t enc =
        rp_packet_encode(packet, sizeof(packet), fields, in);
    TEST_ASSERT_EQUAL_INT(RP_CODEC_OK, enc.status);

    rp_packet_decode_result_t dec =
        rp_packet_decode(packet, enc.written, fields, out);
    TEST_ASSERT_EQUAL_INT(RP_CODEC_OK, dec.status);
}

void test_set_throttle_roundtrip(void)
{
    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_throttle_tag;
    cmd.payload.set_throttle.throttle = 0.42f;

    tvr_FlightCommand got = tvr_FlightCommand_init_zero;
    roundtrip(tvr_FlightCommand_fields, &cmd, &got);

    TEST_ASSERT_EQUAL_UINT(tvr_FlightCommand_set_throttle_tag, got.which_payload);
    TEST_ASSERT_EQUAL_FLOAT(0.42f, got.payload.set_throttle.throttle);
}

void test_set_throttle_per_motor_roundtrip(void)
{
    /* Split lower/upper throttle: values and the has-flag must survive,
     * including a meaningful upper of 0 (presence-tracked, not zero-elided). */
    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_throttle_tag;
    cmd.payload.set_throttle.throttle           = 0.40f;
    cmd.payload.set_throttle.has_throttle_upper = true;
    cmd.payload.set_throttle.throttle_upper     = 0.0f;

    tvr_FlightCommand got = tvr_FlightCommand_init_zero;
    got.payload.set_throttle.throttle_upper = 0.99f;  /* stale sentinel */
    roundtrip(tvr_FlightCommand_fields, &cmd, &got);

    TEST_ASSERT_EQUAL_UINT(tvr_FlightCommand_set_throttle_tag, got.which_payload);
    TEST_ASSERT_EQUAL_FLOAT(0.40f, got.payload.set_throttle.throttle);
    TEST_ASSERT_TRUE(got.payload.set_throttle.has_throttle_upper);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, got.payload.set_throttle.throttle_upper);
}

void test_set_throttle_legacy_single_roundtrip(void)
{
    /* A legacy sender (no throttle_upper) must decode with the has-flag
     * false so the FC mirrors the single value to both motors. */
    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_throttle_tag;
    cmd.payload.set_throttle.throttle = 0.42f;

    tvr_FlightCommand got = tvr_FlightCommand_init_zero;
    got.payload.set_throttle.has_throttle_upper = true;  /* stale sentinel */
    roundtrip(tvr_FlightCommand_fields, &cmd, &got);

    TEST_ASSERT_EQUAL_FLOAT(0.42f, got.payload.set_throttle.throttle);
    TEST_ASSERT_FALSE(got.payload.set_throttle.has_throttle_upper);
}

void test_set_pid_gains_integral_roundtrip(void)
{
    /* The originally reported symptom: z_ki / z_integral_limit "not sent".
     * Prove the wire carries them intact. */
    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_pid_gains_tag;
    cmd.payload.set_pid_gains.z_kp             = 2.0f;
    cmd.payload.set_pid_gains.z_ki             = 0.75f;
    cmd.payload.set_pid_gains.z_kd             = 0.1f;
    cmd.payload.set_pid_gains.z_integral_limit = 1.5f;

    tvr_FlightCommand got = tvr_FlightCommand_init_zero;
    roundtrip(tvr_FlightCommand_fields, &cmd, &got);

    TEST_ASSERT_EQUAL_UINT(tvr_FlightCommand_set_pid_gains_tag, got.which_payload);
    TEST_ASSERT_EQUAL_FLOAT(0.75f, got.payload.set_pid_gains.z_ki);
    TEST_ASSERT_EQUAL_FLOAT(1.5f,  got.payload.set_pid_gains.z_integral_limit);
}

void test_set_pid_gains_attitude_ki_roundtrip(void)
{
    /* attitude_ki is the newest SetPidGains field (tag 7) — prove both the
     * values and the has-flag survive the wire. */
    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_pid_gains_tag;
    cmd.payload.set_pid_gains.has_attitude_ki = true;
    cmd.payload.set_pid_gains.attitude_ki.x   = 0.02f;
    cmd.payload.set_pid_gains.attitude_ki.y   = 0.03f;
    cmd.payload.set_pid_gains.attitude_ki.z   = 0.0f;

    tvr_FlightCommand got = tvr_FlightCommand_init_zero;
    roundtrip(tvr_FlightCommand_fields, &cmd, &got);

    TEST_ASSERT_EQUAL_UINT(tvr_FlightCommand_set_pid_gains_tag, got.which_payload);
    TEST_ASSERT_TRUE(got.payload.set_pid_gains.has_attitude_ki);
    TEST_ASSERT_EQUAL_FLOAT(0.02f, got.payload.set_pid_gains.attitude_ki.x);
    TEST_ASSERT_EQUAL_FLOAT(0.03f, got.payload.set_pid_gains.attitude_ki.y);
    TEST_ASSERT_EQUAL_FLOAT(0.0f,  got.payload.set_pid_gains.attitude_ki.z);
}

void test_set_pid_gains_attitude_ki_absent_roundtrip(void)
{
    /* A sender built before attitude_ki existed omits the field entirely —
     * the decoder must report has_attitude_ki=false so the FC holds its
     * current tilt integral gains. */
    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_pid_gains_tag;
    cmd.payload.set_pid_gains.z_kp = 2.0f;   /* keep the frame non-empty */

    tvr_FlightCommand got = tvr_FlightCommand_init_zero;
    got.payload.set_pid_gains.has_attitude_ki = true;  /* stale sentinel */
    roundtrip(tvr_FlightCommand_fields, &cmd, &got);

    TEST_ASSERT_FALSE(got.payload.set_pid_gains.has_attitude_ki);
}

void test_set_reference_zref_roundtrip(void)
{
    tvr_FlightCommand cmd = tvr_FlightCommand_init_zero;
    cmd.which_payload = tvr_FlightCommand_set_reference_tag;
    cmd.payload.set_reference.z_ref  = 12.5f;
    cmd.payload.set_reference.vz_ref = -0.25f;

    tvr_FlightCommand got = tvr_FlightCommand_init_zero;
    roundtrip(tvr_FlightCommand_fields, &cmd, &got);

    TEST_ASSERT_EQUAL_UINT(tvr_FlightCommand_set_reference_tag, got.which_payload);
    TEST_ASSERT_EQUAL_FLOAT(12.5f,  got.payload.set_reference.z_ref);
    TEST_ASSERT_EQUAL_FLOAT(-0.25f, got.payload.set_reference.vz_ref);
}

void test_telemetry_motor_rpm_roundtrip(void)
{
    tvr_Downlink dl = tvr_Downlink_init_zero;
    dl.which_payload = tvr_Downlink_telemetry_tag;
    dl.payload.telemetry.timestamp_ms    = 1234u;
    dl.payload.telemetry.motor_rpm_lower = 8342.0f;
    dl.payload.telemetry.motor_rpm_upper = 8127.5f;
    dl.payload.telemetry.motor_rpm_valid = true;

    tvr_Downlink got = tvr_Downlink_init_zero;
    roundtrip(tvr_Downlink_fields, &dl, &got);

    TEST_ASSERT_EQUAL_UINT(tvr_Downlink_telemetry_tag, got.which_payload);
    TEST_ASSERT_EQUAL_FLOAT(8342.0f, got.payload.telemetry.motor_rpm_lower);
    TEST_ASSERT_EQUAL_FLOAT(8127.5f, got.payload.telemetry.motor_rpm_upper);
    TEST_ASSERT_TRUE(got.payload.telemetry.motor_rpm_valid);
}

void test_telemetry_motor_rpm_invalid_roundtrip(void)
{
    /* No DShot backend: zeros + valid=false must decode as exactly that
     * (proto3 skips zero scalars on the wire; decode re-materialises them). */
    tvr_Downlink dl = tvr_Downlink_init_zero;
    dl.which_payload = tvr_Downlink_telemetry_tag;
    dl.payload.telemetry.thrust_cmd = 3.0f;   /* keep the frame non-empty */

    tvr_Downlink got = tvr_Downlink_init_zero;
    got.payload.telemetry.motor_rpm_lower = 999.0f;  /* stale sentinel */
    got.payload.telemetry.motor_rpm_valid = true;
    roundtrip(tvr_Downlink_fields, &dl, &got);

    TEST_ASSERT_EQUAL_FLOAT(0.0f, got.payload.telemetry.motor_rpm_lower);
    TEST_ASSERT_FALSE(got.payload.telemetry.motor_rpm_valid);
}

int main(void)
{
    UNITY_BEGIN();
    RUN_TEST(test_set_throttle_roundtrip);
    RUN_TEST(test_set_throttle_per_motor_roundtrip);
    RUN_TEST(test_set_throttle_legacy_single_roundtrip);
    RUN_TEST(test_set_pid_gains_integral_roundtrip);
    RUN_TEST(test_set_pid_gains_attitude_ki_roundtrip);
    RUN_TEST(test_set_pid_gains_attitude_ki_absent_roundtrip);
    RUN_TEST(test_set_reference_zref_roundtrip);
    RUN_TEST(test_telemetry_motor_rpm_roundtrip);
    RUN_TEST(test_telemetry_motor_rpm_invalid_roundtrip);
    return UNITY_END();
}
