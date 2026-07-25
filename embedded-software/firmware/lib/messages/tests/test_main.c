#include "unity.h"

void setUp(void)    {}
void tearDown(void) {}

/* test_crc.c */
void test_crc_ccitt_false_check_vector(void);
void test_crc_empty_buffer(void);
void test_crc_resumable(void);

/* test_envelope.c */
void test_envelope_byte_for_byte(void);
void test_envelope_length_field(void);
void test_envelope_crc_matches_independent(void);
void test_envelope_unknown_message_drops(void);
void test_envelope_wrong_payload_size_drops(void);

/* test_token_bucket.c */
void test_token_bucket_10hz_burst_drops_90(void);
void test_token_bucket_recovers_after_idle(void);
void test_token_bucket_zero_rate_blocks(void);

/* test_drop_counter.c */
void test_drop_counter_struct_offsets(void);
void test_drop_counter_publish_emits_envelope(void);
void test_vcp_only_publish_bumps_vcp_drops(void);
void test_no_sink_bumps_sd_drops(void);
void test_publish_b_is_stub(void);

int main(void)
{
    UNITY_BEGIN();

    /* CRC */
    RUN_TEST(test_crc_ccitt_false_check_vector);
    RUN_TEST(test_crc_empty_buffer);
    RUN_TEST(test_crc_resumable);

    /* Envelope */
    RUN_TEST(test_envelope_byte_for_byte);
    RUN_TEST(test_envelope_length_field);
    RUN_TEST(test_envelope_crc_matches_independent);
    RUN_TEST(test_envelope_unknown_message_drops);
    RUN_TEST(test_envelope_wrong_payload_size_drops);

    /* Token bucket */
    RUN_TEST(test_token_bucket_10hz_burst_drops_90);
    RUN_TEST(test_token_bucket_recovers_after_idle);
    RUN_TEST(test_token_bucket_zero_rate_blocks);

    /* Drop counters / channel stubs / Class B stub */
    RUN_TEST(test_drop_counter_struct_offsets);
    RUN_TEST(test_drop_counter_publish_emits_envelope);
    RUN_TEST(test_vcp_only_publish_bumps_vcp_drops);
    RUN_TEST(test_no_sink_bumps_sd_drops);
    RUN_TEST(test_publish_b_is_stub);

    return UNITY_END();
}
