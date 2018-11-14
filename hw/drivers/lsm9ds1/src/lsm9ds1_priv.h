#ifndef __SENSOR_LSM9DS1_PRIV_H__
#define __SENSOR_LSM9DS1_PRIV_H__

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    LSM9DS1_REGISTER_ACT_THS             = 0x04,
    LSM9DS1_REGISTER_ACT_DUR             = 0x05,
    LSM9DS1_REGISTER_INT_GEN_CFG_XL      = 0x06,
    LSM9DS1_REGISTER_INT_GEN_THS_X_XL    = 0x07,
    LSM9DS1_REGISTER_INT_GEN_THS_Y_XL    = 0x08,
    LSM9DS1_REGISTER_INT_GEN_THS_Z_XL    = 0x09,
    LSM9DS1_REGISTER_INT_GEN_DUR_XL      = 0x0A,
    LSM9DS1_REGISTER_REFERENCE_G         = 0x0B,
    LSM9DS1_REGISTER_INT1_CTRL           = 0x0C,
    LSM9DS1_REGISTER_INT2_CTRL           = 0x0D,
    
    LSM9DS1_REGISTER_WHO_AM_I_XG         = 0x0F,
    LSM9DS1_REGISTER_CTRL_REG1_G         = 0x10,
    LSM9DS1_REGISTER_CTRL_REG2_G         = 0x11,
    LSM9DS1_REGISTER_CTRL_REG3_G         = 0x12,
    LSM9DS1_REGISTER_TEMP_OUT_L          = 0x15,
    LSM9DS1_REGISTER_TEMP_OUT_H          = 0x16,
    LSM9DS1_REGISTER_STATUS_REG          = 0x17,
    LSM9DS1_REGISTER_OUT_X_L_G           = 0x18,
    LSM9DS1_REGISTER_OUT_X_H_G           = 0x19,
    LSM9DS1_REGISTER_OUT_Y_L_G           = 0x1A,
    LSM9DS1_REGISTER_OUT_Y_H_G           = 0x1B,
    LSM9DS1_REGISTER_OUT_Z_L_G           = 0x1C,
    LSM9DS1_REGISTER_OUT_Z_H_G           = 0x1D,
    LSM9DS1_REGISTER_CTRL_REG4           = 0x1E,
    LSM9DS1_REGISTER_CTRL_REG5_XL        = 0x1F,
    LSM9DS1_REGISTER_CTRL_REG6_XL        = 0x20,
    LSM9DS1_REGISTER_CTRL_REG7_XL        = 0x21,
    LSM9DS1_REGISTER_CTRL_REG8           = 0x22,
    LSM9DS1_REGISTER_CTRL_REG9           = 0x23,
    LSM9DS1_REGISTER_CTRL_REG10          = 0x24,

    LSM9DS1_REGISTER_INT_GEN_SRC_XL      = 0x26,
    
    LSM9DS1_REGISTER_OUT_X_L_XL          = 0x28,
    LSM9DS1_REGISTER_OUT_X_H_XL          = 0x29,
    LSM9DS1_REGISTER_OUT_Y_L_XL          = 0x2A,
    LSM9DS1_REGISTER_OUT_Y_H_XL          = 0x2B,
    LSM9DS1_REGISTER_OUT_Z_L_XL          = 0x2C,
    LSM9DS1_REGISTER_OUT_Z_H_XL          = 0x2D,
    LSM9DS1_REGISTER_FIFO_CTRL           = 0x2E,
    LSM9DS1_REGISTER_FIFO_SRC            = 0x2F,
    LSM9DS1_REGISTER_INT_GEN_CFG_G       = 0x30,
    LSM9DS1_REGISTER_INT_GEN_THS_XH_G    = 0x31,
    LSM9DS1_REGISTER_INT_GEN_THS_XL_G    = 0x32,
    LSM9DS1_REGISTER_INT_GEN_THS_YH_G    = 0x33,
    LSM9DS1_REGISTER_INT_GEN_THS_YL_G    = 0x34,
    LSM9DS1_REGISTER_INT_GEN_THS_ZH_G    = 0x35,
    LSM9DS1_REGISTER_INT_GEN_THS_ZL_G    = 0x36,
    LSM9DS1_REGISTER_INT_DUR_G           = 0x37
} lsm9ds1_acc_gyro_registers_t;
  
typedef enum
{
    LSM9DS1_REGISTER_OFFSET_X_REG_L_M   = 0x05,
    LSM9DS1_REGISTER_OFFSET_X_REG_H_M   = 0x06,
    LSM9DS1_REGISTER_OFFSET_Y_REG_L_M   = 0x07,
    LSM9DS1_REGISTER_OFFSET_Y_REG_H_M   = 0x08,
    LSM9DS1_REGISTER_OFFSET_Z_REG_L_M   = 0x09,
    LSM9DS1_REGISTER_OFFSET_Z_REG_H_M   = 0x0A,

    LSM9DS1_REGISTER_WHO_AM_I_M         = 0x0F,
    
    LSM9DS1_REGISTER_CTRL_REG1_M        = 0x20,
    LSM9DS1_REGISTER_CTRL_REG2_M        = 0x21,
    LSM9DS1_REGISTER_CTRL_REG3_M        = 0x22,
    LSM9DS1_REGISTER_CTRL_REG4_M        = 0x23,
    LSM9DS1_REGISTER_CTRL_REG5_M        = 0x24,
    
    LSM9DS1_REGISTER_STATUS_REG_M       = 0x27,
    LSM9DS1_REGISTER_OUT_X_L_M          = 0x28,
    LSM9DS1_REGISTER_OUT_X_H_M          = 0x29,
    LSM9DS1_REGISTER_OUT_Y_L_M          = 0x2A,
    LSM9DS1_REGISTER_OUT_Y_H_M          = 0x2B,
    LSM9DS1_REGISTER_OUT_Z_L_M          = 0x2C,
    LSM9DS1_REGISTER_OUT_Z_H_M          = 0x2D,
    
    LSM9DS1_REGISTER_CFG_M              = 0x30,
    LSM9DS1_REGISTER_INT_SRC_M          = 0x31,
    LSM9DS1_REGISTER_INT_THS_L_M        = 0x32,
    LSM9DS1_REGISTER_INT_THS_H_M        = 0x33,
} lsm9ds1_mag_registers_t;

int lsm9ds1_write8(struct sensor_itf *itf, uint8_t addr, uint8_t reg, uint32_t value);
int lsm9ds1_read8(struct sensor_itf *itf, uint8_t addr, uint8_t reg, uint8_t *value);
int lsm9ds1_read48(struct sensor_itf *itf, uint8_t addr, uint8_t reg, uint8_t *buffer);

#ifdef __cplusplus
}
#endif

#endif  // __SENSOR_LSM9DS1_PRIV_H__