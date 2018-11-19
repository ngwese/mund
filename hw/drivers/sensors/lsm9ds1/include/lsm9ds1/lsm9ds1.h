//
// -- lsm9ds1.h --------------------------------------------------------
//

// This is a library for the LSM9DS1 Accelerometer and magnentometer/compass
//
// Designed specifically to work with the Adafruit LSM9DS1 Breakouts
//
// These sensors use I2C to communicate, 2 pins are required to interface.
//
// Adafruit invests time and resources providing this open source code,
// please support Adafruit andopen-source hardware by purchasing products
// from Adafruit!
//
// Written by Kevin Townsend for Adafruit Industries.
// BSD license, all text above must be included in any redistribution

#ifndef __SENSOR_LSM9DS1_H__
#define __SENSOR_LSM9DS1_H__

#include "os/mynewt.h"
#include "sensor/sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LSM9DS1_ADDRESS_ACCELGYRO          (0x6B)
#define LSM9DS1_ADDRESS_MAG                (0x1E)
#define LSM9DS1_XG_ID                      (0b01101000)
#define LSM9DS1_MAG_ID                     (0b00111101)

// Linear Acceleration: mg per LSB
#define LSM9DS1_ACCEL_MG_LSB_2G            (0.061F)
#define LSM9DS1_ACCEL_MG_LSB_4G            (0.122F)
#define LSM9DS1_ACCEL_MG_LSB_8G            (0.244F)
#define LSM9DS1_ACCEL_MG_LSB_16G           (0.732F)

// Magnetic Field Strength: gauss range
#define LSM9DS1_MAG_MGAUSS_4GAUSS          (0.14F)
#define LSM9DS1_MAG_MGAUSS_8GAUSS          (0.29F)
#define LSM9DS1_MAG_MGAUSS_12GAUSS         (0.43F)
#define LSM9DS1_MAG_MGAUSS_16GAUSS         (0.58F)

// Angular Rate: dps per LSB
#define LSM9DS1_GYRO_DPS_DIGIT_245DPS      (0.00875F)
#define LSM9DS1_GYRO_DPS_DIGIT_500DPS      (0.01750F)
#define LSM9DS1_GYRO_DPS_DIGIT_2000DPS     (0.07000F)

// Temperature: LSB per degree celsius
#define LSM9DS1_TEMP_LSB_DEGREE_CELSIUS    (8)  // 1°C = 8, 25° = 200, etc.

#define MAGTYPE                            (true)
#define XGTYPE                             (false)

typedef enum
{
    LSM9DS1_ACCELRANGE_2G                = (0b00 << 3),
    LSM9DS1_ACCELRANGE_16G               = (0b01 << 3),
    LSM9DS1_ACCELRANGE_4G                = (0b10 << 3),
    LSM9DS1_ACCELRANGE_8G                = (0b11 << 3),
} lsm9ds1_accel_range_t;

typedef enum
{
    LSM9DS1_ACCELDATARATE_POWERDOWN      = (0b000 << 5),
    LSM9DS1_ACCELDATARATE_14_9HZ         = (0b001 << 5),
    LSM9DS1_ACCELDATARATE_59_5HZ         = (0b010 << 5),
    LSM9DS1_ACCELDATARATE_119HZ          = (0b011 << 5),
    LSM9DS1_ACCELDATARATE_238HZ          = (0b100 << 5),
    LSM9DS1_ACCELDATARATE_476HZ          = (0b101 << 5),
    LSM9DS1_ACCELDATARATE_952HZ          = (0b110 << 5)
} lsm9ds1_accel_data_rate_t;

typedef enum
{
    LSM9DS1_MAGGAIN_4GAUSS               = (0b00 << 5),  // +/- 4 gauss
    LSM9DS1_MAGGAIN_8GAUSS               = (0b01 << 5),  // +/- 8 gauss
    LSM9DS1_MAGGAIN_12GAUSS              = (0b10 << 5),  // +/- 12 gauss
    LSM9DS1_MAGGAIN_16GAUSS              = (0b11 << 5)   // +/- 16 gauss
} lsm9ds1_mag_gain_t;

typedef enum
{
    LSM9DS1_MAGDATARATE__625HZ           = (0b000 << 2),
    LSM9DS1_MAGDATARATE_1_25HZ           = (0b001 << 2),
    LSM9DS1_MAGDATARATE_2_5HZ            = (0b010 << 2),
    LSM9DS1_MAGDATARATE_5HZ              = (0b011 << 2),
    LSM9DS1_MAGDATARATE_10HZ             = (0b100 << 2),
    LSM9DS1_MAGDATARATE_20HZ             = (0b101 << 2),
    LSM9DS1_MAGDATARATE_40HZ             = (0b110 << 2),
    LSM9DS1_MAGDATARATE_80HZ             = (0b111 << 2)
} lsm9ds1_mag_data_rate_t;

typedef enum
{
    LSM9DS1_GYROSCALE_245DPS             = (0b00 << 3),  // +/- 245 degrees per second rotation
    LSM9DS1_GYROSCALE_500DPS             = (0b01 << 3),  // +/- 500 degrees per second rotation
    LSM9DS1_GYROSCALE_2000DPS            = (0b11 << 3)   // +/- 2000 degrees per second rotation
} lsm9ds1_gyro_scale_t;

typedef struct
{
    float x;
    float y;
    float z;
} lsm9ds1_vector_t;

typedef struct lsm9ds1_cfg {
    lsm9ds1_accel_range_t accel_range;
    lsm9ds1_accel_data_rate_t accel_rate;
    lsm9ds1_mag_gain_t mag_gain;
    lsm9ds1_mag_data_rate_t mag_rate;
    lsm9ds1_gyro_scale_t gyro_scale;
    uint8_t accel_addr;
    uint8_t mag_addr;
    sensor_type_t mask;
} lsm9ds1_cfg_t;

typedef struct lsm9ds1 {
    struct os_dev dev;
    struct sensor sensor;
    lsm9ds1_cfg_t cfg;
    os_time_t last_read_time;
} lsm9ds1_t;

int lsm9ds1_init(struct os_dev *, void *);
int lsm9ds1_config(lsm9ds1_t *, lsm9ds1_cfg_t *);

#if MYNEWT_VAL(LSM9DS1_CLI)
int lsm9ds1_shell_init(void);
#endif

int lsm9ds1_get_chip_id(struct sensor_itf *itf, uint8_t *id);

#ifdef __cplusplus
}
#endif

#endif  // __SENSOR_LSM9DS1_H__
