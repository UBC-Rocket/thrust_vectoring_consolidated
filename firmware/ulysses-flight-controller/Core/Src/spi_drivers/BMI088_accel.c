#include "BMI088_accel.h"

/* ---- Internal SPI command helpers ---- */
#define SPI_RD(a)   ((uint8_t)((a) | 0x80))
#define SPI_WR(a)   ((uint8_t)((a) & 0x7F))

/* -------------------------------------------------------------------------- */
/* Soft reset                                                                 */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_softreset(uint8_t *tx_buf)
{
    /* Datasheet: write 0xB6 to ACC_SOFTRESET (0x7E) */
    tx_buf[0] = SPI_WR(BMI088_ACC_SOFTRESET);
    tx_buf[1] = 0xB6;
    return 2;  // 2-byte write
}

/* -------------------------------------------------------------------------- */
/* Power configuration                                                        */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_power_conf(bmi088_accel_power_t pwr,
                                     uint8_t *tx_buf,
                                     bmi088_accel_t *dev)
{
    /* Two registers matter:
       - ACC_PWR_CONF (0x7C): 0x00 active, 0x03 suspend
       - ACC_PWR_CTRL (0x7D): 0x04 accel enable, 0x00 disable
       Sequence per datasheet: write PWR_CONF first, then PWR_CTRL.
    */
    uint8_t pwr_conf = (pwr == BMI088_ACC_PWR_ACTIVE) ? 0x00 : 0x03;
    uint8_t pwr_ctrl = (pwr == BMI088_ACC_PWR_ACTIVE) ? 0x04 : 0x00;

    tx_buf[0] = SPI_WR(BMI088_ACC_PWR_CONF);
    tx_buf[1] = pwr_conf;
    tx_buf[2] = SPI_WR(BMI088_ACC_PWR_CTRL);
    tx_buf[3] = pwr_ctrl;

    if (dev) {
        dev->pwr_conf_reg = pwr_conf;
        dev->pwr_ctrl_reg = pwr_ctrl;
    }

    return 4;  // 2 writes × 2 bytes each
}

/* -------------------------------------------------------------------------- */
/* Range configuration                                                        */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_range(bmi088_accel_range_t range,
                                uint8_t *tx_buf,
                                bmi088_accel_t *dev)
{
    tx_buf[0] = SPI_WR(BMI088_ACC_RANGE);
    tx_buf[1] = (uint8_t)(range & 0x03);

    if (dev) {
        dev->range_reg    = tx_buf[1];
        // convert to mg/LSB from datasheet
        switch (range) {
            case BMI088_ACC_RANGE_3G:  dev->scale_mg_lsb = 1000.0f / 10920.0f; break;
            case BMI088_ACC_RANGE_6G:  dev->scale_mg_lsb = 1000.0f / 5460.0f;  break;
            case BMI088_ACC_RANGE_12G: dev->scale_mg_lsb = 1000.0f / 2730.0f;  break;
            case BMI088_ACC_RANGE_24G: dev->scale_mg_lsb = 1000.0f / 1365.0f;  break;
        }
    }
    return 2;
}

/* -------------------------------------------------------------------------- */
/* ODR + bandwidth configuration                                              */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_odr_bw(bmi088_accel_odr_t odr,
                                 bmi088_accel_bwp_t bwp,
                                 uint8_t *tx_buf,
                                 bmi088_accel_t *dev)
{
    /* ACC_CONF: [7:4]=acc_bwp, [3:0]=acc_odr */
    uint8_t val = (uint8_t)(((bwp & 0x0F) << 4) | (odr & 0x0F));

    tx_buf[0] = SPI_WR(BMI088_ACC_CONF);
    tx_buf[1] = val;

    if (dev) {
        dev->conf_reg = val;
        switch (odr) {
            case BMI088_ACC_ODR_12_5_HZ: dev->odr_hz = 12;   break;
            case BMI088_ACC_ODR_25_HZ:   dev->odr_hz = 25;   break;
            case BMI088_ACC_ODR_50_HZ:   dev->odr_hz = 50;   break;
            case BMI088_ACC_ODR_100_HZ:  dev->odr_hz = 100;  break;
            case BMI088_ACC_ODR_200_HZ:  dev->odr_hz = 200;  break;
            case BMI088_ACC_ODR_400_HZ:  dev->odr_hz = 400;  break;
            case BMI088_ACC_ODR_800_HZ:  dev->odr_hz = 800;  break;
            case BMI088_ACC_ODR_1600_HZ: dev->odr_hz = 1600; break;
            default: dev->odr_hz = 0; break;
        }
    }
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Interrupt pin electrical configuration                                     */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_int_pin_conf(bmi088_accel_int_pin_t pin,
                                       bmi088_accel_int_drive_t drive,
                                       bmi088_accel_int_polarity_t polarity,
                                       bool latch,
                                       uint8_t *tx_buf)
{
    /* Bits in INTx_IO_CONF:
       bit3: out enable (must be 1 for interrupt output)
       bit2: open drain (1=open drain, 0=push-pull)
       bit1: level (1=active high, 0=active low)
       bit0: latch (1=latching until cleared, 0=pulsed)
    */
    uint8_t v = 0;
    v |= (1u << 3); // enable output
    if (drive == BMI088_ACC_INT_OPEN_DRAIN) v |= (1u << 2);
    if (polarity == BMI088_ACC_INT_ACTIVE_HIGH) v |= (1u << 1);
    if (latch) v |= (1u << 0);

    tx_buf[0] = SPI_WR((pin == BMI088_ACC_INT1) ? BMI088_ACC_INT1_IO_CONF
                                                : BMI088_ACC_INT2_IO_CONF);
    tx_buf[1] = v;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* Interrupt event mapping                                                     */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_int_event_map(bmi088_accel_int_pin_t pin,
                                        uint8_t events,
                                        uint8_t *tx_buf)
{
    /* INT1_INT2_MAP_DATA @ 0x58:
       bits 2:0 map events to INT1
       bits 6:4 map events to INT2
       DRDY, FIFO watermark, FIFO full
    */
    uint8_t v = 0;

    if (pin == BMI088_ACC_INT1) {
        if (events & BMI088_ACC_INT_EVENT_DATA_READY) v |= (1u << 2);
        if (events & BMI088_ACC_INT_EVENT_FIFO_WTM)   v |= (1u << 1);
        if (events & BMI088_ACC_INT_EVENT_FIFO_FULL)  v |= (1u << 0);
    } else {
        if (events & BMI088_ACC_INT_EVENT_DATA_READY) v |= (1u << 6);
        if (events & BMI088_ACC_INT_EVENT_FIFO_WTM)   v |= (1u << 5);
        if (events & BMI088_ACC_INT_EVENT_FIFO_FULL)  v |= (1u << 4);
    }

    tx_buf[0] = SPI_WR(BMI088_ACC_INT1_INT2_MAP);
    tx_buf[1] = v;
    return 2;
}

/* -------------------------------------------------------------------------- */
/* FIFO configuration                                                         */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_fifo_conf(bmi088_accel_fifo_mode_t mode,
                                    bmi088_accel_fifo_source_t src,
                                    uint16_t watermark,
                                    uint8_t *tx_buf)
{
    /* Build the sequence of register writes:
       - FIFO_CONFIG_0
       - FIFO_CONFIG_1
       - FIFO_DOWNS
       - FIFO_WTM_0 / FIFO_WTM_1
    */

    uint8_t cfg0 = 0x02; // bit1 must be 1
    if (mode == BMI088_ACC_FIFO_FIFO)   cfg0 |= 0x01; // stop-on-full
    if (mode == BMI088_ACC_FIFO_STREAM) cfg0 |= 0x00; // stream

    uint8_t cfg1 = 0x10; // bit4 must be 1
    if (src & BMI088_ACC_FIFO_XYZ)        cfg1 |= 0x40; // accel enable
    if (src & BMI088_ACC_FIFO_SENSORTIME) cfg1 |= 0x80; // sensortime enable

    uint8_t downs = 0x80; // bit7 must be 1; no downsampling

    // Watermark in bytes (caller should multiply by 7 if thinking in samples)
    uint16_t wtm = watermark & 0x1FFF;

    uint8_t *p = tx_buf;
    *p++ = SPI_WR(BMI088_ACC_FIFO_CONFIG_0);
    *p++ = cfg0;
    *p++ = SPI_WR(BMI088_ACC_FIFO_CONFIG_1);
    *p++ = cfg1;
    *p++ = SPI_WR(BMI088_ACC_FIFO_DOWNS);
    *p++ = downs;
    *p++ = SPI_WR(BMI088_ACC_FIFO_WTM_0);
    *p++ = (uint8_t)(wtm & 0xFF);
    *p++ = SPI_WR(BMI088_ACC_FIFO_WTM_1);
    *p++ = (uint8_t)((wtm >> 8) & 0x1F);

    return (size_t)(p - tx_buf);
}

/* -------------------------------------------------------------------------- */
/* Self-test configuration                                                    */
/* -------------------------------------------------------------------------- */
size_t bmi088_accel_build_selftest(uint8_t axis_mask, bool enable, uint8_t *tx_buf)
{
    /* ACC_SELF_TEST (0x6D):
       bit0 = X enable
       bit1 = Y enable
       bit2 = Z enable
       bit7 = 1 to enable self-test mode
    */
    uint8_t val = 0;
    if (enable) val |= 0x80;
    val |= (axis_mask & 0x07);

    tx_buf[0] = SPI_WR(BMI088_ACC_SELF_TEST);
    tx_buf[1] = val;

    return 2;
}

size_t bmi088_accel_build_read(bmi088_accel_read_t what, uint8_t *tx_buf)
{
    uint8_t addr;
    size_t len;  // number of *payload* bytes we want (1,2,3,6,...)

    switch (what) {
    case BMI088_ACC_READ_DATA_XYZ:    addr = BMI088_ACC_DATA_START;      len = 6; break;
    case BMI088_ACC_READ_TEMP:        addr = BMI088_ACC_TEMP_MSB;        len = 2; break;
    case BMI088_ACC_READ_SENSORTIME:  addr = BMI088_ACC_SENSORTIME_0;    len = 3; break;
    case BMI088_ACC_READ_FIFO_STATUS: addr = BMI088_ACC_FIFO_LENGTH_0;   len = 2; break;
    case BMI088_ACC_READ_FIFO_DATA:   addr = BMI088_ACC_FIFO_DATA;       len = 6; break; // caller may override
    default: return 0;
    }

    // Compose SPI read frame for the *accelerometer* (requires a dummy response byte)
    tx_buf[0] = addr | 0x80;   // read flag in MSB
    tx_buf[1] = 0x00;          // clock the accel's dummy response
    for (size_t i = 0; i < len; ++i)
        tx_buf[2 + i] = 0x00;  // clock out the real data bytes

    return len + 2;            // address + dummy + data
}

size_t bmi088_accel_build_read_reg(uint8_t reg, uint8_t *tx_buf){
    tx_buf[0] = 0x80 | reg; // read bit set
    tx_buf[1] = 0x00;       // dummy byte
    tx_buf[2] = 0x00;
    
    return 3;
}


bool bmi088_accel_parse_data_xyz(const uint8_t *rx_buf,
                                 bmi088_accel_sample_t *sample,
                                 const bmi088_accel_t *dev)
{
    if (!rx_buf || !sample || !dev) return false;

    // Convert from little endian to signed 16-bit
    int16_t x = (int16_t)((rx_buf[1] << 8) | rx_buf[0]);
    int16_t y = (int16_t)((rx_buf[3] << 8) | rx_buf[2]);
    int16_t z = (int16_t)((rx_buf[5] << 8) | rx_buf[4]);

    sample->x = x;
    sample->y = y;
    sample->z = z;

    // Convert to m/s^2
    const float mps2_per_lsb = (dev->scale_mg_lsb * 1e-3f) * 9.80665f;
    sample->ax = (float)x * mps2_per_lsb;
    sample->ay = (float)y * mps2_per_lsb;
    sample->az = (float)z * mps2_per_lsb;

    return true;
}

bool bmi088_accel_parse_temp(const uint8_t *rx_buf, float *temp_c)
{
    if (!rx_buf || !temp_c) return false;

    // 11-bit signed temperature, see datasheet
    uint16_t raw = (uint16_t)((rx_buf[0] << 3) | (rx_buf[1] >> 5));
    if (raw > 1023) raw -= 2048; // two's complement
    *temp_c = ((float)raw * 0.125f) + 23.0f;

    return true;
}

bool bmi088_accel_parse_sensortime(const uint8_t *rx_buf, uint32_t *sensortime_us)
{
    if (!rx_buf || !sensortime_us) return false;

    uint32_t val = ((uint32_t)rx_buf[2] << 16) |
                   ((uint32_t)rx_buf[1] << 8)  |
                   ((uint32_t)rx_buf[0]);

    *sensortime_us = (uint32_t)((float)val * 39.0625f);
    return true;
}

bool bmi088_accel_parse_fifo_length(const uint8_t *rx_buf, uint16_t *fifo_bytes)
{
    if (!rx_buf || !fifo_bytes) return false;

    uint16_t len = ((rx_buf[1] & 0x3F) << 8) | rx_buf[0];
    *fifo_bytes = len;

    return true;
}

bool bmi088_accel_parse_fifo(const uint8_t *rx_buf, size_t len,
                             bmi088_accel_sample_t *out_samples,
                             size_t *max_samples,
                             const bmi088_accel_t *dev)
{
    if (!rx_buf || !out_samples || !max_samples || !dev) return false;

    size_t capacity = *max_samples;
    size_t count = 0;
    size_t i = 0;

    while (i < len && count < capacity) {
        uint8_t hdr = rx_buf[i++];

        if ((hdr & 0xFCu) == 0x84u) { // accelerometer frame
            if (i + 6 > len) break;
            bmi088_accel_parse_data_xyz(&rx_buf[i], &out_samples[count], dev);
            count++;
            i += 6;
        } else if ((hdr & 0xFCu) == 0x44u) { // sensor time frame
            i += 3;
        } else if ((hdr & 0xFCu) == 0x40u) { // skip frame
            i += 1;
        } else if ((hdr & 0xFCu) == 0x48u) { // input config frame
            i += 1;
        } else if ((hdr & 0xFCu) == 0x50u) { // sample drop frame
            i += 1;
        } else {
            // Unknown frame → stop parsing
            break;
        }
    }

    *max_samples = count;
    return (count > 0);
}

