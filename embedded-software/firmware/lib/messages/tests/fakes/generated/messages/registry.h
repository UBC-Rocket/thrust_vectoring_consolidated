/* Test fake mirroring the codegen contract for generated/messages/registry.h.
 * Mirrors the values documented in firmware/lib/messages spec. Real codegen
 * output replaces this at integration time.
 */
#ifndef FAKE_GENERATED_MESSAGES_REGISTRY_H
#define FAKE_GENERATED_MESSAGES_REGISTRY_H

#include <stdint.h>

#define REGISTRY_VERSION 1
#define REGISTRY_CRC32   0xDEADBEEFU

#define CH_SD   0
#define CH_VCP  1
#define CH_UDP  2
#define CH_COUNT 3

#define MSG_CLASS_A        0x41
#define MSG_CLASS_B        0x42
#define MSG_CLASS_CMD_REQ  0x43
#define MSG_CLASS_CMD_RESP 0x44

/* Module IDs */
#define MOD_SYSTEM 0
#define MOD_TEST   7

/* Message IDs */
#define MSG_SYSTEM_DROP_COUNTER 2
#define MSG_TEST_PING           1
#define MSG_TEST_TINY           3
#define MSG_TEST_LIMITED        4
#define MSG_TEST_VCP_ONLY       5

#endif /* FAKE_GENERATED_MESSAGES_REGISTRY_H */
