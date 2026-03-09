#include "SPI_device_interactions.h"

/* Sample ring buffers */
bmi088_accel_sample_queue_t bmi088_acc_sample_ring;
bmi088_gyro_sample_queue_t bmi088_gyro_sample_ring;
ms5611_sample_queue_t ms5611_sample_ring;
ms5607_sample_queue_t ms5607_sample_ring;

/* Device configuration structs */
bmi088_accel_t accel;
bmi088_gyro_t gyro;

/* SPI job queues - one per SPI bus */
spi_job_queue_t jobq_spi_1;  /* SPI1 bus - external/payload sensors */
spi_job_queue_t jobq_spi_2;  /* SPI2 bus - IMU and barometer 1 (MS5611) */
spi_job_queue_t jobq_spi_4;  /* SPI4 bus - barometer 2 (MS5607) */

/* Device ready flags */
volatile bool bmi088_accel_ready = false;
volatile bool bmi088_gyro_ready = false;
