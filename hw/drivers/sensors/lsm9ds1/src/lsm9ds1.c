#include <string.h>
#include <errno.h>
#include <assert.h>

#include "os/mynewt.h"
#include "hal/hal_i2c.h"
#include "sensor/sensor.h"
#include "sensor/accel.h"
#include "sensor/mag.h"
#include "sensor/temperature.h"

#include "lsm9ds1/lsm9ds1.h"
#include "lsm9ds1_priv.h"

#include "log/log.h"
#include "stats/stats.h"

/* Define the stats section and records */
STATS_SECT_START(lsm9ds1_stat_section)
    STATS_SECT_ENTRY(samples_acc_2g)
    STATS_SECT_ENTRY(samples_acc_4g)
    STATS_SECT_ENTRY(samples_acc_8g)
    STATS_SECT_ENTRY(samples_acc_16g)
    STATS_SECT_ENTRY(samples_mag_4g)
    STATS_SECT_ENTRY(samples_mag_8g)
    STATS_SECT_ENTRY(samples_mag_12g)
    STATS_SECT_ENTRY(samples_mag_16g)
    STATS_SECT_ENTRY(samples_gyro_245dps)
    STATS_SECT_ENTRY(samples_gyro_500dps)
    STATS_SECT_ENTRY(samples_gyro_2000dps)
    STATS_SECT_ENTRY(errors)
STATS_SECT_END

/* Define stat names for querying */
STATS_NAME_START(lsm9ds1_stat_section)
    STATS_NAME(lsm9ds1_stat_section, samples_acc_2g)
    STATS_NAME(lsm9ds1_stat_section, samples_acc_4g)
    STATS_NAME(lsm9ds1_stat_section, samples_acc_8g)
    STATS_NAME(lsm9ds1_stat_section, samples_acc_16g)
    STATS_NAME(lsm9ds1_stat_section, samples_mag_4g)
    STATS_NAME(lsm9ds1_stat_section, samples_mag_8g)
    STATS_NAME(lsm9ds1_stat_section, samples_mag_12g)
    STATS_NAME(lsm9ds1_stat_section, samples_mag_16g)
    STATS_NAME(lsm9ds1_stat_section, samples_gyro_245dps)
    STATS_NAME(lsm9ds1_stat_section, samples_gyro_500dps)
    STATS_NAME(lsm9ds1_stat_section, samples_gyro_2000dps)
    STATS_NAME(lsm9ds1_stat_section, errors)
STATS_NAME_END(lsm9ds1_stat_section)

/* Global variable used to hold stats data */
STATS_SECT_DECL(lsm9ds1_stat_section) g_lsm9ds1_stats;

#define LOG_MODULE_LSM9DS1 (303)
#define LSM9DS1_INFO(...)  LOG_INFO(&_log, LOG_MODULE_LSM9DS1, __VA_ARGS__)
#define LSM9DS1_ERR(...)   LOG_ERROR(&_log, LOG_MODULE_LSM9DS1, __VA_ARGS__)
static struct log _log;

/* Exports for the sensor API */
static int lsm9ds1_sensor_read(struct sensor *, sensor_type_t,
        sensor_data_func_t, void *, uint32_t);
static int lsm9ds1_sensor_get_config(struct sensor *, sensor_type_t,
        struct sensor_cfg *);

static const struct sensor_driver g_lsm9ds1_sensor_driver = {
    lsm9ds1_sensor_read,
    lsm9ds1_sensor_get_config
};

#if MYNEWT_VAL(I2C_0)
struct sensor_itf g_lsm9ds1_i2c_0_itf = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 0,
    .si_addr = 0
};
#endif


/**
 * Writes a single byte to the specified register
 *
 * @param The sensor interface
 * @param The I2C address to use
 * @param The register address to write to
 * @param The value to write
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm9ds1_write8(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
                  uint32_t value)
{
    int rc;
    uint8_t payload[2] = { reg, value & 0xFF };

    struct hal_i2c_master_data data_struct = {
        .address = addr,
        .len = 2,
        .buffer = payload
    };

    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        LSM9DS1_ERR("Failed to write to 0x%02X:0x%02X with value 0x%02lX\n",
                       addr, reg, value);
        STATS_INC(g_lsm9ds1_stats, errors);
    }

    return rc;
}

/**
 * Reads a single byte from the specified register
 *
 * @param The sensor interface
 * @param The I2C address to use
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm9ds1_read8(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
              uint8_t *value)
{
    int rc;
    uint8_t payload;

    struct hal_i2c_master_data data_struct = {
        .address = addr,
        .len = 1,
        .buffer = &payload
    };

    /* Register write */
    payload = reg;
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        LSM9DS1_ERR("I2C access failed at address 0x%02X\n", addr);
        STATS_INC(g_lsm9ds1_stats, errors);
        goto err;
    }

    /* Read one byte back */
    payload = 0;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);
    *value = payload;
    if (rc) {
        LSM9DS1_ERR("Failed to read from 0x%02X:0x%02X\n", addr, reg);
        STATS_INC(g_lsm9ds1_stats, errors);
    }

err:
    return rc;
}

/**
 * Reads a six bytes from the specified register
 *
 * @param The sensor interface
 * @param The I2C address to use
 * @param The register address to read from
 * @param Pointer to where the register value should be written
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm9ds1_read48(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
               uint8_t *buffer)
{
    int rc;
    uint8_t payload[7] = { reg, 0, 0, 0, 0, 0, 0 };

    struct hal_i2c_master_data data_struct = {
        .address = addr,
        .len = 1,
        .buffer = payload
    };

    /* Clear the supplied buffer */
    memset(buffer, 0, 6);

    /* Register write */
    rc = hal_i2c_master_write(itf->si_num, &data_struct,
                              OS_TICKS_PER_SEC / 10, 1);
    if (rc) {
        LSM9DS1_ERR("I2C access failed at address 0x%02X\n", addr);
        STATS_INC(g_lsm9ds1_stats, errors);
        goto err;
    }

    /* Read six bytes back */
    memset(payload, 0, sizeof(payload));
    data_struct.len = 6;
    rc = hal_i2c_master_read(itf->si_num, &data_struct,
                             OS_TICKS_PER_SEC / 10, 1);

    if (rc) {
        LSM9DS1_ERR("Failed to read from 0x%02X:0x%02X\n", addr, reg);
        STATS_INC(g_lsm9ds1_stats, errors);
        goto err;
    }

    /* Copy the I2C results into the supplied buffer */
    memcpy(buffer, payload, 6);

err:
    return rc;
}

/**
 * Expects to be called back through os_dev_create().
 *
 * @param The device object associated with this accelerometer
 * @param Argument passed to OS device init, unused
 *
 * @return 0 on success, non-zero error on failure.
 */
int
lsm9ds1_init(struct os_dev *dev, void *arg)
{
    lsm9ds1_t *lsm;
    struct sensor *sensor;
    int rc;

    if (!arg || !dev) {
        rc = SYS_ENODEV;
        goto err;
    }

    lsm = (lsm9ds1_t *)dev;

    lsm->cfg.mask = SENSOR_TYPE_LINEAR_ACCEL | SENSOR_TYPE_GYROSCOPE |
                    SENSOR_TYPE_MAGNETIC_FIELD | SENSOR_TYPE_TEMPERATURE;

    log_register(dev->od_name, &_log, &log_console_handler, NULL, LOG_SYSLEVEL);

    sensor = &lsm->sensor;

    /* Initialise the stats entry */
    rc = stats_init(
        STATS_HDR(g_lsm9ds1_stats),
        STATS_SIZE_INIT_PARMS(g_lsm9ds1_stats, STATS_SIZE_32),
        STATS_NAME_INIT_PARMS(lsm9ds1_stat_section));
    SYSINIT_PANIC_ASSERT(rc == 0);
    /* Register the entry with the stats registry */
    rc = stats_register(dev->od_name, STATS_HDR(g_lsm9ds1_stats));
    SYSINIT_PANIC_ASSERT(rc == 0);

    rc = sensor_init(sensor, dev);
    if (rc != 0) {
        goto err;
    }

    /* Add the accelerometer/magnetometer driver */
    rc = sensor_set_driver(sensor, lsm->cfg.mask,
            (struct sensor_driver *) &g_lsm9ds1_sensor_driver);
    if (rc != 0) {
        goto err;
    }

    /* Set the interface */
    rc = sensor_set_interface(sensor, arg);
    if (rc) {
        goto err;
    }

    rc = sensor_mgr_register(sensor);
    if (rc != 0) {
        goto err;
    }

    return (0);
err:
    return (rc);
}

int
lsm9ds1_config(lsm9ds1_t *lsm, lsm9ds1_cfg_t *cfg)
{
    int rc;
    struct sensor_itf *itf;
    uint8_t reg;

    itf = SENSOR_GET_ITF(&(lsm->sensor));

    /* Most sensor chips have a single address and just use different
     * registers to get data for different sensors
     */
    if (!cfg->accel_addr || !cfg->mag_addr) {
        rc = SYS_EINVAL;
        goto err;
    }

    /* Set accel/gyro data rate (or power down) and gyro scale */
    reg = cfg->accel_rate | cfg->gyro_scale;
    rc = lsm9ds1_write8(itf, cfg->accel_addr,
                        LSM9DS1_REGISTER_CTRL_REG1_G,
                        reg);
    if (rc) {
        goto err;
    }

    lsm->cfg.accel_rate = cfg->accel_rate;

    /* Set accel scale */
    reg = cfg->accel_rate | cfg->accel_range;
    rc = lsm9ds1_write8(itf, cfg->accel_addr,
                        LSM9DS1_REGISTER_CTRL_REG6_XL,
                        reg);
    if (rc) {
        goto err;
    }

    lsm->cfg.accel_range = cfg->accel_range;

    /* Enable the magnetomer (set to continuous conversion mode) */
    rc = lsm9ds1_write8(itf, cfg->mag_addr,
                        LSM9DS1_REGISTER_CTRL_REG3_M,
                        0x00);
    if (rc) {
        goto err;
    }

    /* Set mag rate */
    reg = (1 << 7);  // enable temperature compensation
    reg |= cfg->mag_rate;
    rc = lsm9ds1_write8(itf, cfg->mag_addr,
                        LSM9DS1_REGISTER_CTRL_REG1_M,
                        reg);
    if (rc) {
        goto err;
    }

    lsm->cfg.mag_rate = cfg->mag_rate;

    /* Set mag gain */
    rc = lsm9ds1_write8(itf, cfg->mag_addr,
                        LSM9DS1_REGISTER_CTRL_REG2_M,
                        cfg->mag_gain);
    if (rc) {
        goto err;
    }

    lsm->cfg.mag_gain = cfg->mag_gain;

    rc = sensor_set_type_mask(&(lsm->sensor), cfg->mask);
    if (rc) {
        goto err;
    }

    lsm->cfg.mask = cfg->mask;
    lsm->cfg.mag_addr = cfg->mag_addr;
    lsm->cfg.accel_addr = cfg->accel_addr;

    return 0;
err:
    return (rc);
}

static int
lsm9ds1_sensor_read(struct sensor *sensor, sensor_type_t type,
        sensor_data_func_t data_func, void *data_arg, uint32_t timeout)
{
    int rc;
    int16_t x, y, z;
    uint8_t temp_h, temp_l;
    float mg_lsb;
    int16_t gauss_lsb_xy;
    int16_t gauss_lsb_z;
    uint8_t payload[6];
    struct sensor_itf *itf;
    lsm9ds1_t *lsm;
    union {
        struct sensor_accel_data sad;
        struct sensor_mag_data smd;
        struct sensor_temp_data std;
    } databuf;

    /* If the read isn't looking for accel or mag data, don't do anything. */
    if ((!(type & SENSOR_TYPE_LINEAR_ACCEL)) &&
        (!(type & SENSOR_TYPE_GYROSCOPE)) &&
        (!(type & SENSOR_TYPE_MAGNETIC_FIELD)) &&
        (!(type & SENSOR_TYPE_TEMPERATURE))) {
        rc = SYS_EINVAL;
        goto err;
    }

    itf = SENSOR_GET_ITF(sensor);
    lsm = (lsm9ds1_t *) SENSOR_GET_DEVICE(sensor);

    /* Get a new accelerometer sample */
    if (type & SENSOR_TYPE_LINEAR_ACCEL) {
        x = y = z = 0;
        rc = lsm9ds1_read48(itf, lsm->cfg.accel_addr,
                               LSM9DS1_REGISTER_OUT_X_L_XL,
                               payload);
        if (rc != 0) {
            goto err;
        }

        /* Shift accel values into 16-bit int */
        x = (int16_t)(payload[0] | (payload[1] << 8));
        y = (int16_t)(payload[2] | (payload[3] << 8));
        z = (int16_t)(payload[4] | (payload[5] << 8));

        /* Determine mg per lsb based on range */
        switch(lsm->cfg.accel_range) {
            case LSM9DS1_ACCELRANGE_2G:
                STATS_INC(g_lsm9ds1_stats, samples_acc_2g);
                mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
                break;
            case LSM9DS1_ACCELRANGE_4G:
                STATS_INC(g_lsm9ds1_stats, samples_acc_4g);
                mg_lsb = LSM9DS1_ACCEL_MG_LSB_4G;
                break;
            case LSM9DS1_ACCELRANGE_8G:
                STATS_INC(g_lsm9ds1_stats, samples_acc_8g);
                mg_lsb = LSM9DS1_ACCEL_MG_LSB_8G;
                break;
            case LSM9DS1_ACCELRANGE_16G:
                STATS_INC(g_lsm9ds1_stats, samples_acc_16g);
                mg_lsb = LSM9DS1_ACCEL_MG_LSB_16G;
                break;
            default:
                LSM9DS1_ERR("Unknown accel range: 0x%02X. Assuming +/-2G.\n",
                    lsm->cfg.accel_range);
                mg_lsb = LSM9DS1_ACCEL_MG_LSB_2G;
                break;
        }

        /* Convert from mg to Earth gravity in m/s^2 */
        databuf.sad.sad_x = (float)x * mg_lsb * 9.80665F;
        databuf.sad.sad_y = (float)y * mg_lsb * 9.80665F;
        databuf.sad.sad_z = (float)z * mg_lsb * 9.80665F;

        databuf.sad.sad_x_is_valid = 1;
        databuf.sad.sad_y_is_valid = 1;
        databuf.sad.sad_z_is_valid = 1;

        /* Call data function */
        rc = data_func(sensor, data_arg, &databuf.sad, SENSOR_TYPE_LINEAR_ACCEL);
        if (rc != 0) {
            goto err;
        }
    }

    /* Get a new magnetometer sample */
    if (type & SENSOR_TYPE_MAGNETIC_FIELD) {
        x = y = z = 0;
        rc = lsm9ds1_read48(itf, lsm->cfg.mag_addr,
                               LSM9DS1_REGISTER_OUT_X_L_M,
                               payload);
        if (rc != 0) {
            goto err;
        }

        /* Shift mag values into 16-bit int */
        x = (int16_t)(payload[0] | ((int16_t)payload[1] << 8));
        y = (int16_t)(payload[2] | ((int16_t)payload[3] << 8));
        z = (int16_t)(payload[4] | ((int16_t)payload[5] << 8));

        /* Determine gauss per lsb based on gain */
        // FIXME: these guass_lsb_* values are wrong
        switch (lsm->cfg.mag_gain) {
            case LSM9DS1_MAGGAIN_4GAUSS:
                STATS_INC(g_lsm9ds1_stats, samples_mag_4g);
                gauss_lsb_xy = 1100;
                gauss_lsb_z = 980;
                break;
            case LSM9DS1_MAGGAIN_8GAUSS:
                STATS_INC(g_lsm9ds1_stats, samples_mag_8g);
                gauss_lsb_xy = 855;
                gauss_lsb_z = 760;
                break;
            case LSM9DS1_MAGGAIN_12GAUSS:
                STATS_INC(g_lsm9ds1_stats, samples_mag_12g);
                gauss_lsb_xy = 670;
                gauss_lsb_z = 600;
                break;
            case LSM9DS1_MAGGAIN_16GAUSS:
                STATS_INC(g_lsm9ds1_stats, samples_mag_16g);
                gauss_lsb_xy = 450;
                gauss_lsb_z = 400;
                break;
            default:
                LSM9DS1_ERR("Unknown mag gain: 0x%02X. Assuming +/-4g.\n",
                    lsm->cfg.mag_gain);
                gauss_lsb_xy = 1100;
                gauss_lsb_z = 980;
                break;
        }

        /* Convert from gauss to micro Tesla */
        databuf.smd.smd_x = (float)x / gauss_lsb_xy * 100.0F;
        databuf.smd.smd_y = (float)y / gauss_lsb_xy * 100.0F;
        databuf.smd.smd_z = (float)z / gauss_lsb_z * 100.0F;

        databuf.smd.smd_x_is_valid = 1;
        databuf.smd.smd_y_is_valid = 1;
        databuf.smd.smd_z_is_valid = 1;

        /* Call data function */
        rc = data_func(sensor, data_arg, &databuf.smd, SENSOR_TYPE_MAGNETIC_FIELD);
        if (rc != 0) {
            goto err;
        }
    }

    if (type & SENSOR_TYPE_TEMPERATURE) {
        temp_h = temp_l = 0;
        rc = lsm9ds1_read8(itf, lsm->cfg.accel_addr, LSM9DS1_REGISTER_TEMP_OUT_H, &temp_h);
        if (rc != 0) {
            goto err;
        }
        rc = lsm9ds1_read8(itf, lsm->cfg.accel_addr, LSM9DS1_REGISTER_TEMP_OUT_L, &temp_l);
        if (rc != 0) {
            goto err;
        }

        x = (int16_t)(temp_l | ((int16_t)(temp_h << 8)));
        databuf.std.std_temp = x;   // convert to degrees C
        databuf.std.std_temp_is_valid = 1;

        rc = data_func(sensor, data_arg, &databuf.std, SENSOR_TYPE_TEMPERATURE);
        if (rc != 0) {
            goto err;
        }
    }

    // TODO: implement SENSOR_TYPE_GYROSCOPE

    return (0);
err:
    return (rc);
}

static int
lsm9ds1_sensor_get_config(struct sensor *sensor, sensor_type_t type,
        struct sensor_cfg *cfg)
{
    int rc;

    if ((type != SENSOR_TYPE_LINEAR_ACCEL) &&
        (type != SENSOR_TYPE_GYROSCOPE) &&
        (type != SENSOR_TYPE_MAGNETIC_FIELD)) {
        rc = SYS_EINVAL;
        goto err;
    }

    cfg->sc_valtype = SENSOR_VALUE_TYPE_FLOAT_TRIPLET;

    return (0);
err:
    return (rc);
}

static int
lsm9ds1_checked_read8(struct sensor_itf *itf, uint8_t addr, uint8_t reg,
                      uint8_t *value)
{
    int rc;
    uint8_t tmp;

    rc = lsm9ds1_read8(itf, addr, reg, &tmp);
    if (rc) {
        goto err;
    }

    *value = tmp;

    return 0;
err:
    return rc;
}

int
lsm9ds1_get_chip_id(struct sensor_itf *itf, uint8_t *id)
{
    return lsm9ds1_checked_read8(itf, LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_WHO_AM_I_XG, id);
}

int
lsm9ds1_get_accel_status(struct sensor_itf *itf, uint8_t *status)
{
    return lsm9ds1_checked_read8(itf, LSM9DS1_ADDRESS_ACCELGYRO, LSM9DS1_REGISTER_STATUS_REG, status);
}

int
lsm9ds1_get_mag_status(struct sensor_itf *itf, uint8_t *status)
{
    return lsm9ds1_checked_read8(itf, LSM9DS1_ADDRESS_MAG, LSM9DS1_REGISTER_STATUS_REG_M, status);
}
