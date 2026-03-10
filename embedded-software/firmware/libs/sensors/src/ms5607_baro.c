/**
 * @file ms5607_baro.c
 * @brief MS5607-02BA03 barometer driver implementation.
 *
 * Command builders, parsers, CRC validation, and compensation algorithm.
 *
 * Note: MS5607 uses different compensation formulas than MS5611!
 * The bit shifts and second-order coefficients differ.
 *
 * @ UBC Rocket, Jan 2026
 */

#include "sensors/ms5607_baro.h"
#include <string.h>
#include <stdint.h>

/* -------------------------------------------------------------------------- */
/* Command builders                                                           */
/* -------------------------------------------------------------------------- */

size_t ms5607_build_reset(uint8_t *tx_buf)
{
    /* 0x1E; caller must wait >= 2.8 ms for PROM reload after this. */
    tx_buf[0] = MS5607_CMD_RESET;
    return 1;
}

size_t ms5607_build_convert_d1(ms5607_osr_t osr, uint8_t *tx_buf)
{
    /* One-byte command: 0x40 | osr_bits */
    tx_buf[0] = (uint8_t)(MS5607_CMD_CONVERT_D1 | (uint8_t)osr);
    return 1;
}

size_t ms5607_build_convert_d2(ms5607_osr_t osr, uint8_t *tx_buf)
{
    /* One-byte command: 0x50 | osr_bits */
    tx_buf[0] = (uint8_t)(MS5607_CMD_CONVERT_D2 | (uint8_t)osr);
    return 1;
}

size_t ms5607_build_adc_read(uint8_t *tx_buf)
{
    /*
     * After a conversion completes, ADC read is:
     * - send 0x00, then clock out 3 bytes MSB-first.
     * We return 4 bytes total so the caller can TXRX(4) and then pass
     * &rx[1] to the parse function (which expects the first data byte).
     */
    tx_buf[0] = MS5607_CMD_ADC_READ;
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    tx_buf[3] = 0x00;
    return 4;
}

size_t ms5607_build_prom_read(uint8_t index, uint8_t *tx_buf)
{
    /*
     * PROM read is:
     * - send 0xA0 | (addr<<1), then clock out 2 bytes MSB-first.
     * We return 3 bytes so the caller can TXRX(3) and then pass &rx[1]
     * to the parse function (which expects the first data byte).
     */
    index &= 0x07;
    tx_buf[0] = (uint8_t)(MS5607_CMD_PROM_READ | (index << 1));
    tx_buf[1] = 0x00;
    tx_buf[2] = 0x00;
    return 3;
}

/* -------------------------------------------------------------------------- */
/* Parsing helpers                                                            */
/* -------------------------------------------------------------------------- */

bool ms5607_parse_prom_word(const uint8_t *rx_buf, uint16_t *out_word)
{
    if (!rx_buf || !out_word) return false;
    /* Expect rx_buf points to MSB of the 16-bit word */
    *out_word = (uint16_t)((((uint16_t)rx_buf[0]) << 8) | (uint16_t)rx_buf[1]);
    return true;
}

bool ms5607_parse_adc_result(const uint8_t *rx_buf, uint32_t *out_val)
{
    if (!rx_buf || !out_val) return false;
    /* Expect rx_buf points to MSB of the 24-bit value */
    *out_val = (uint32_t)((((uint32_t)rx_buf[0]) << 16) |
                          (((uint32_t)rx_buf[1]) << 8)  |
                           ((uint32_t)rx_buf[2]));
    return true;
}

/* -------------------------------------------------------------------------- */
/* PROM CRC-4 check                                                           */
/* -------------------------------------------------------------------------- */
/*
 * Standard MS5607 CRC algorithm (CRC-4, polynomial 0x3000) over 7*16+4 bits.
 * Same algorithm as MS5611.
 */
bool ms5607_check_crc(const uint16_t *C)
{
    if (!C) return false;

    uint16_t prom[8];
    memcpy(prom, C, sizeof(prom));

    const uint8_t crc_read = (uint8_t)(prom[7] & 0x000F); /* lower 4 bits */
    prom[7] &= 0xFFF0; /* clear CRC nibble */

    uint16_t n_rem = 0x0000;

    /* Process 16 bytes (MSB first then LSB) from prom[0..7], but CRC nibble cleared */
    for (int cnt = 0; cnt < 16; ++cnt) {
        /* Select byte */
        uint8_t byte;
        if (cnt % 2 == 0) {
            byte = (uint8_t)(prom[cnt / 2] >> 8);   /* MSB */
        } else {
            byte = (uint8_t)(prom[cnt / 2] & 0xFF); /* LSB */
        }

        n_rem ^= ((uint16_t)byte) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (n_rem & 0x8000) n_rem = (n_rem << 1) ^ 0x3000;
            else                n_rem = (n_rem << 1);
        }
    }

    uint8_t crc_calc = (uint8_t)((n_rem >> 12) & 0x0F);
    return (crc_calc == crc_read);
}

/* -------------------------------------------------------------------------- */
/* Compensation (first + second order) - MS5607 SPECIFIC                      */
/* -------------------------------------------------------------------------- */
/*
 * MS5607 formulas are DIFFERENT from MS5611!
 *
 * First-order (datasheet page 8):
 *   dT   = D2 - C5 × 2^8
 *   TEMP = 2000 + dT × C6 / 2^23
 *   OFF  = C2 × 2^17 + (C4 × dT) / 2^6    ← Different from MS5611!
 *   SENS = C1 × 2^16 + (C3 × dT) / 2^7    ← Different from MS5611!
 *   P    = (D1 × SENS / 2^21 - OFF) / 2^15
 *
 * Second-order (datasheet page 9):
 *   If TEMP < 2000:
 *     T2    = dT^2 / 2^31
 *     OFF2  = 61 × (TEMP - 2000)^2 / 2^4  ← Different from MS5611!
 *     SENS2 = 2 × (TEMP - 2000)^2         ← Different from MS5611!
 *     If TEMP < -1500:
 *       OFF2  += 15 × (TEMP + 1500)^2     ← Different from MS5611!
 *       SENS2 += 8 × (TEMP + 1500)^2      ← Different from MS5611!
 */

static inline int64_t sll_mul(int64_t a, int64_t b) { return a * b; }

bool ms5607_compute(ms5607_t *dev)
{
    if (!dev) return false;

    /* Coefficients (C1..C6 used in computation) */
    const int64_t C1 = dev->C[1];
    const int64_t C2 = dev->C[2];
    const int64_t C3 = dev->C[3];
    const int64_t C4 = dev->C[4];
    const int64_t C5 = dev->C[5];
    const int64_t C6 = dev->C[6];

    const int64_t D1 = dev->D1_raw;
    const int64_t D2 = dev->D2_raw;

    /* First-order calculations (use 64-bit to avoid overflow) */
    /* dT = D2 - C5 * 2^8 */
    int64_t dT = D2 - (C5 << 8);

    /* TEMP = 2000 + dT * C6 / 2^23  (units: 0.01 °C) */
    int64_t TEMP = 2000 + ((sll_mul(dT, C6)) >> 23);

    /* MS5607-specific bit shifts! */
    /* OFF  = C2 * 2^17 + (C4 * dT) / 2^6 */
    int64_t OFF = (C2 << 17) + ((sll_mul(C4, dT)) >> 6);

    /* SENS = C1 * 2^16 + (C3 * dT) / 2^7 */
    int64_t SENS = (C1 << 16) + ((sll_mul(C3, dT)) >> 7);

    /* Second-order compensation */
    int64_t T2 = 0, OFF2 = 0, SENS2 = 0;

    if (TEMP < 2000) { /* low temperature */
        /* T2 = (dT^2) / 2^31 */
        T2 = (sll_mul(dT, dT)) >> 31;

        int64_t t_low = TEMP - 2000;         /* (in 0.01 °C) */
        int64_t t_low2 = sll_mul(t_low, t_low);

        /* MS5607-specific coefficients! */
        /* OFF2 = 61 * (TEMP - 2000)^2 / 2^4 */
        OFF2 = (61 * t_low2) >> 4;

        /* SENS2 = 2 * (TEMP - 2000)^2 */
        SENS2 = 2 * t_low2;

        if (TEMP < -1500) { /* very low temperature */
            int64_t t_very = TEMP + 1500;
            int64_t t_very2 = sll_mul(t_very, t_very);

            /* MS5607-specific coefficients! */
            /* OFF2  += 15 * (TEMP + 1500)^2 */
            OFF2 += 15 * t_very2;

            /* SENS2 += 8 * (TEMP + 1500)^2 */
            SENS2 += 8 * t_very2;
        }

        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    /* Pressure: P = (D1 * SENS / 2^21 - OFF) / 2^15 */
    int64_t P = ((sll_mul(D1, SENS) >> 21) - OFF) >> 15;

    /* Store results */
    dev->TEMP_centi   = (int32_t)TEMP; /* 0.01 °C */
    dev->P_centi_mbar = (int32_t)P;    /* 0.01 mbar (hPa) */

    return true;
}
