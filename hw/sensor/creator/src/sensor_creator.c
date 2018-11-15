/*
 * Licensed to the Apache Software Foundation (ASF) under one
 * or more contributor license agreements.  See the NOTICE file
 * distributed with this work for additional information
 * regarding copyright ownership.  The ASF licenses this file
 * to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance
 * with the License.  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
 * KIND, either express or implied.  See the License for the
 * specific language governing permissions and limitations
 * under the License.
 */

#include <assert.h>
#include <string.h>
#include "os/mynewt.h"

#if MYNEWT_VAL(LSM9DS1_OFB)
#include <lsm9ds1/lsm9ds1.h>
#endif

/* Driver definitions */
#if MYNEWT_VAL(LSM9DS1_OFB)
static struct lsm9ds1 lsm9ds1;
#endif

/**
 * If a UART sensor needs to be created, interface is defined in
 * the following way
 *
 * #if MYNEWT_VAL(UART_0)
 * static const struct sensor_itf uart_0_itf = {
 *   .si_type = SENSOR_ITF_UART,
 *   .si_num = 0,
 * };
 * #endif
 *
 * #if MYNEWT_VAL(UART_1)
 * static struct sensor_itf uart_1_itf = {
 *    .si_type = SENSOR_ITF_UART,
 *    .si_num = 1,
 *};
 *#endif
 */


#if MYNEWT_VAL(I2C_0) && MYNEWT_VAL(LSM9DS1_OFB)
static struct sensor_itf i2c_0_itf_lsm = {
    .si_type = SENSOR_ITF_I2C,
    .si_num  = 0,
    .si_addr = 0
};
#endif


/**
 * lsm9ds1 Sensor default configuration used by the creator package
 *
 * @return 0 on success, non-zero on failure
 */
#if MYNEWT_VAL(LSM9DS1_OFB)
static int
config_lsm9ds1_sensor(void)
{
    int rc;
    struct os_dev *dev;
    struct lsm9ds1_cfg lsmcfg;

    dev = (struct os_dev *) os_dev_open("lsm9ds1_0", OS_TIMEOUT_NEVER, NULL);
    assert(dev != NULL);

    /* read once per sec.  API should take this value in ms. */
    lsmcfg.accel_rate = LSM9DS1_ACCELDATARATE_14_9HZ;
    lsmcfg.accel_range = LSM9DS1_ACCELRANGE_2G;
    lsmcfg.mag_gain = LSM9DS1_MAG_MGAUSS_8GAUSS;
    lsmcfg.mag_rate = LSM9DS1_MAGDATARATE_1_25HZ;
    lsmcfg.gyro_scale = LSM9DS1_GYROSCALE_245DPS;

    // FIXME: configure accel_addr, mag_addr or remove
    lsmcfg.accel_addr = LSM9DS1_ADDRESS_ACCELGYRO;
    lsmcfg.mag_addr = LSM9DS1_ADDRESS_MAG;

    lsmcfg.mask = SENSOR_TYPE_LINEAR_ACCEL |
                  SENSOR_TYPE_GYROSCOPE |
                  SENSOR_TYPE_MAGNETIC_FIELD;

    rc = lsm9ds1_config((struct lsm9ds1 *) dev, &lsmcfg);

    os_dev_close(dev);
    return rc;
}
#endif


/* Sensor device creation */
void
ext_sensor_dev_create(void)
{
    int rc;

    (void)rc;

#if MYNEWT_VAL(LSM9DS1_OFB)
    /* Since this sensor has multiple I2C addreses,
     * 0x1E for accelerometer and 0x19 for magnetometer,
     * they are made part of the config. Not setting the address in the sensor
     * interface makes it take the address either from the driver or
     * from the config, however teh develeoper would like to deal with it.
     */
    rc = os_dev_create((struct os_dev *) &lsm9ds1, "lsm9ds1_0",
      OS_DEV_INIT_PRIMARY, 0, lsm9ds1_init, (void *)&i2c_0_itf_lsm);
    assert(rc == 0);

    rc = config_lsm9ds1_sensor();
    assert(rc == 0);
#endif


}
