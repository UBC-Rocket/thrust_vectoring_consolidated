/**
 * @file gnss_radio_master.c
 * @brief GNSS Radio push mode driver implementation
 *
 * Implements two-phase push mode reading from the GNSS Radio slave board:
 * - Phase 1: Read TYPE byte (1 byte) to determine message type
 * - Phase 2: Read PAYLOAD based on type (GPS: 48 bytes, Radio: 256 bytes)
 *
 * Also supports master-to-slave radio TX via raw 256-byte payloads.
 *
 * UBC Rocket, Feb 2026
 */

#include "gnss_radio_master.h"
#include "main.h"
#include "timestamp.h"
#include "sync.h"
#include "FreeRTOS.h"
#include "task.h"
#include "app_freertos.h"
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Global Context                                                             */
/* -------------------------------------------------------------------------- */

gnss_radio_context_t gnss_radio_ctx;

/* -------------------------------------------------------------------------- */
/* Forward Declarations                                                       */
/* -------------------------------------------------------------------------- */

static void submit_type_read(void);
static void submit_payload_read(uint8_t type);

/* -------------------------------------------------------------------------- */
/* Initialization                                                             */
/* -------------------------------------------------------------------------- */

void gnss_radio_init(void)
{
    /* Clear entire context */
    memset(&gnss_radio_ctx, 0, sizeof(gnss_radio_ctx));

    /* Set hardware configuration */
    gnss_radio_ctx.cs_port = GNSS_RADIO_CS_PORT;
    gnss_radio_ctx.cs_pin = GNSS_RADIO_CS_PIN;
    gnss_radio_ctx.irq_port = GNSS_RADIO_IRQ_PORT;
    gnss_radio_ctx.irq_pin = GNSS_RADIO_IRQ_PIN;

    /* Initialize state machine */
    gnss_radio_ctx.state = GNSS_STATE_IDLE;
    gnss_radio_ctx.irq_pending = false;

    /* Queues are zeroed by memset above (head = tail = 0 → empty) */

    /* Initialize TX dummy buffer (zeros for read operations) */
    memset(gnss_radio_ctx.tx_dummy, 0x00, sizeof(gnss_radio_ctx.tx_dummy));

    gnss_radio_ctx.initialized = true;
}

/* -------------------------------------------------------------------------- */
/* IRQ Handler                                                                */
/* -------------------------------------------------------------------------- */

void gnss_radio_irq_handler(void)
{
    if (!gnss_radio_ctx.initialized) {
        return;
    }

    gnss_radio_ctx.irq_count++;

    /* Only process if we're idle */
    if (gnss_radio_ctx.state != GNSS_STATE_IDLE) {
        /* Already processing a push transaction - IRQ might be stale */
        gnss_radio_ctx.irq_pending = true;
        return;
    }

    /* Two-phase mode: read TYPE first */
    submit_type_read();
}

/* -------------------------------------------------------------------------- */
/* Phase 1: TYPE Read                                                         */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit a job to read just the TYPE byte
 */
static void submit_type_read(void)
{
    spi1_job_t job = {0};

    job.cs_port = gnss_radio_ctx.cs_port;
    job.cs_pin = gnss_radio_ctx.cs_pin;
    job.len = GNSS_PHASE1_TYPE_READ_SIZE;
    job.type = SPI_XFER_TXRX;
    job.done = gnss_radio_phase1_done;
    job.done_arg = NULL;
    job.task_notification_flag = 0;

    /* TX buffer: dummy bytes (slave ignores during push) */
    memset(job.tx, 0x00, job.len);

    gnss_radio_ctx.state = GNSS_STATE_TYPE_PENDING;

    spi1_submit_job(&job);

    gnss_radio_ctx.state = GNSS_STATE_TYPE_READING;
}

/**
 * @brief Phase 1 completion callback
 *
 * Called when TYPE byte has been read. Examines the type and submits
 * phase 2 job.
 */
void gnss_radio_phase1_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg)
{
    (void)job;
    (void)arg;

    /* Extract TYPE byte */
    uint8_t type = rx_buf[0];
    gnss_radio_ctx.pending_type = type;

    /* Validate type and submit phase 2 */
    if (type == GNSS_PUSH_TYPE_GPS || type == GNSS_PUSH_TYPE_RADIO) {
        submit_payload_read(type);
    } else {
        /* Unknown type - return to idle */
        gnss_radio_ctx.transaction_errors++;
        gnss_radio_ctx.state = GNSS_STATE_IDLE;

        /* Check for pending IRQ */
        if (gnss_radio_ctx.irq_pending) {
            gnss_radio_ctx.irq_pending = false;
            gnss_radio_irq_handler();
        }
    }
}

/* -------------------------------------------------------------------------- */
/* Phase 2: PAYLOAD Read                                                      */
/* -------------------------------------------------------------------------- */

/**
 * @brief Submit a job to read the PAYLOAD based on type
 *
 * @param type The TYPE byte from phase 1
 */
static void submit_payload_read(uint8_t type)
{
    spi1_job_t job = {0};

    job.cs_port = gnss_radio_ctx.cs_port;
    job.cs_pin = gnss_radio_ctx.cs_pin;
    job.type = SPI_XFER_TXRX;
    job.done = gnss_radio_phase2_done;
    job.done_arg = (void *)(uintptr_t)type;
    job.task_notification_flag = 0;

    /* Set length based on type */
    switch (type) {
    case GNSS_PUSH_TYPE_GPS:
        job.len = GNSS_PUSH_GPS_PAYLOAD;
        break;

    case GNSS_PUSH_TYPE_RADIO:
        job.len = GNSS_PUSH_RADIO_PAYLOAD;
        break;

    default:
        /* Should not reach here */
        gnss_radio_ctx.state = GNSS_STATE_IDLE;
        return;
    }

    /* TX buffer: dummy bytes */
    memset(job.tx, 0x00, job.len);

    gnss_radio_ctx.state = GNSS_STATE_PAYLOAD_PENDING;

    spi1_submit_job(&job);

    gnss_radio_ctx.state = GNSS_STATE_PAYLOAD_READING;
}

/**
 * @brief Phase 2 completion callback
 *
 * Called when PAYLOAD has been read. Parses the data and enqueues
 * to the appropriate queue.
 */
void gnss_radio_phase2_done(spi1_job_t *job, const uint8_t *rx_buf, void *arg)
{
    (void)job;
    uint8_t type = (uint8_t)(uintptr_t)arg;

    BaseType_t xWoken = pdFALSE;

    switch (type) {
    case GNSS_PUSH_TYPE_GPS:
        /* Parsed GPS fix from rx_buf */
        if (gnss_gps_fix_q_push(&gnss_radio_ctx.gps_queue,
                                 (const gnss_gps_fix_t *)rx_buf)) {
            gnss_radio_ctx.gps_fixes_received++;
            xTaskNotifyFromISR(StateEstimationHandle,
                               GNSS_GPS_FIX_READY_FLAG,
                               eSetBits, &xWoken);
        } else {
            gnss_radio_ctx.queue_overflows++;
        }
        break;

    case GNSS_PUSH_TYPE_RADIO:
        if (gnss_radio_msg_q_push(&gnss_radio_ctx.radio_queue,
                                   (const gnss_radio_msg_t *)rx_buf)) {
            gnss_radio_ctx.radio_msgs_received++;
            xTaskNotifyFromISR(MissionManagerHandle,
                               GNSS_RADIO_MSG_READY_FLAG,
                               eSetBits, &xWoken);
        } else {
            gnss_radio_ctx.queue_overflows++;
        }
        break;

    default:
        gnss_radio_ctx.transaction_errors++;
        break;
    }

    /* Return to idle state */
    gnss_radio_ctx.state = GNSS_STATE_IDLE;

    /* Check for pending IRQ */
    if (gnss_radio_ctx.irq_pending) {
        gnss_radio_ctx.irq_pending = false;
        /* Re-trigger IRQ handler to process next message */
        gnss_radio_irq_handler();
    }

    portYIELD_FROM_ISR(xWoken);
}

/* -------------------------------------------------------------------------- */
/* Queue operations are now provided by SPSC_RING_DECLARE macros:             */
/*   gnss_gps_fix_q_push/pop/peek/empty/full/count                           */
/*   gnss_radio_msg_q_push/pop/peek/empty/full/count                         */
/* -------------------------------------------------------------------------- */

/* -------------------------------------------------------------------------- */
/* Radio TX (Master to Slave)                                                 */
/* -------------------------------------------------------------------------- */

bool gnss_radio_send(const uint8_t *msg, uint16_t len)
{
    if (!gnss_radio_ctx.initialized) {
        return false;
    }

    if (len > GNSS_RADIO_MESSAGE_MAX_LEN) {
        return false;
    }

    spi1_job_t job = {0};

    job.cs_port = gnss_radio_ctx.cs_port;
    job.cs_pin = gnss_radio_ctx.cs_pin;
    job.len = GNSS_RADIO_MESSAGE_MAX_LEN;  /* Always 256 bytes raw */
    job.type = SPI_XFER_TXRX;
    job.done = NULL;
    job.done_arg = NULL;
    job.task_notification_flag = 0;

    /* Raw payload — slave detects via spi_slave_rx_has_valid_data() */
    memcpy(job.tx, msg, len);
    /* Remaining bytes already zero from {0} initializer */

    return spi1_submit_job(&job);
}

/* -------------------------------------------------------------------------- */
/* Statistics                                                                 */
/* -------------------------------------------------------------------------- */

void gnss_radio_get_stats(uint32_t *gps_count,
                          uint32_t *radio_count,
                          uint32_t *errors)
{
    if (gps_count) {
        *gps_count = gnss_radio_ctx.gps_fixes_received;
    }
    if (radio_count) {
        *radio_count = gnss_radio_ctx.radio_msgs_received;
    }
    if (errors) {
        *errors = gnss_radio_ctx.transaction_errors + gnss_radio_ctx.queue_overflows;
    }
}

void gnss_radio_reset_stats(void)
{
    uint32_t primask;
    SYNC_ENTER_CRITICAL_RAW(primask);

    gnss_radio_ctx.gps_fixes_received = 0;
    gnss_radio_ctx.radio_msgs_received = 0;
    gnss_radio_ctx.irq_count = 0;
    gnss_radio_ctx.transaction_errors = 0;
    gnss_radio_ctx.queue_overflows = 0;

    SYNC_EXIT_CRITICAL_RAW(primask);
}

gnss_push_state_t gnss_radio_get_state(void)
{
    return gnss_radio_ctx.state;
}
