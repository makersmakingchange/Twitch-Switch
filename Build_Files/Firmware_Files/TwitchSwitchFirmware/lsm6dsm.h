#ifndef __LSM6DSM_H__
#define __LSM6DSM_H__

#define LSM6DSM_ADDRESS                            0x6B

#define LSM6DSM_REG_FUNC_CFG_ACCESS                0x01

#define LSM6DSM_REG_SENSOR_SYNC_TIME_FRAME         0x04
#define LSM6DSM_REG_SENSOR_SYNC_RES_RATIO          0x05
#define LSM6DSM_REG_FIFO_CTRL1                     0x06
#define LSM6DSM_REG_FIFO_CTRL2                     0x07
#define LSM6DSM_REG_FIFO_CTRL3                     0x08
#define LSM6DSM_REG_FIFO_CTRL4                     0x09
#define LSM6DSM_REG_FIFO_CTRL5                     0x0A
#define LSM6DSM_REG_DRDY_PULSE_CFG                 0x0B
#define LSM6DSM_REG_INT1_CTRL                      0x0D
#define LSM6DSM_REG_INT2_CTRL                      0x0E
#define LSM6DSM_REG_WHO_AM_I                       0x0F
#define LSM6DSM_REG_CTRL1_XL                       0x10
#define LSM6DSM_REG_CTRL2_G                        0x11
#define LSM6DSM_REG_CTRL3_C                        0x12
#define LSM6DSM_REG_CTRL4_C                        0x13
#define LSM6DSM_REG_CTRL5_C                        0x14
#define LSM6DSM_REG_CTRL6_C                        0x15
#define LSM6DSM_REG_CTRL7_G                        0x16
#define LSM6DSM_REG_CTRL8_XL                       0x17
#define LSM6DSM_REG_CTRL9_XL                       0x18
#define LSM6DSM_REG_CTRL10_C                       0x19
#define LSM6DSM_REG_MASTER_CONFIG                  0x1A
#define LSM6DSM_REG_WAKE_UP_SRC                    0x1B
#define LSM6DSM_REG_TAP_SRC                        0x1C
#define LSM6DSM_REG_D6D_SRC                        0x1D
#define LSM6DSM_REG_STATUS_REG                     0x1E
#define LSM6DSM_REG_OUT_TEMP_L                     0x20
#define LSM6DSM_REG_OUT_TEMP_H                     0x21
#define LSM6DSM_REG_OUTX_L_G                       0x22
#define LSM6DSM_REG_OUTX_H_G                       0x23
#define LSM6DSM_REG_OUTY_L_G                       0x24
#define LSM6DSM_REG_OUTY_H_G                       0x25
#define LSM6DSM_REG_OUTZ_L_G                       0x26
#define LSM6DSM_REG_OUTZ_H_G                       0x27
#define LSM6DSM_REG_OUTX_L_XL                      0x28
#define LSM6DSM_REG_OUTX_H_XL                      0x29
#define LSM6DSM_REG_OUTY_L_XL                      0x2A
#define LSM6DSM_REG_OUTY_H_XL                      0x2B
#define LSM6DSM_REG_OUTZ_L_XL                      0x2C
#define LSM6DSM_REG_OUTZ_H_XL                      0x2D
#define LSM6DSM_REG_SENSORHUB1_REG                 0x2E
#define LSM6DSM_REG_SENSORHUB2_REG                 0x2F
#define LSM6DSM_REG_SENSORHUB3_REG                 0x30
#define LSM6DSM_REG_SENSORHUB4_REG                 0x31
#define LSM6DSM_REG_SENSORHUB5_REG                 0x32
#define LSM6DSM_REG_SENSORHUB6_REG                 0x33
#define LSM6DSM_REG_SENSORHUB7_REG                 0x34
#define LSM6DSM_REG_SENSORHUB8_REG                 0x35
#define LSM6DSM_REG_SENSORHUB9_REG                 0x36
#define LSM6DSM_REG_SENSORHUB10_REG                0x37
#define LSM6DSM_REG_SENSORHUB11_REG                0x38
#define LSM6DSM_REG_SENSORHUB12_REG                0x39
#define LSM6DSM_REG_FIFO_STATUS1                   0x3A
#define LSM6DSM_REG_FIFO_STATUS2                   0x3B
#define LSM6DSM_REG_FIFO_STATUS3                   0x3C
#define LSM6DSM_REG_FIFO_STATUS4                   0x3D
#define LSM6DSM_REG_FIFO_DATA_OUT_L                0x3E
#define LSM6DSM_REG_FIFO_DATA_OUT_H                0x3F
#define LSM6DSM_REG_TIMESTAMP0_REG                 0x40
#define LSM6DSM_REG_TIMESTAMP1_REG                 0x41
#define LSM6DSM_REG_TIMESTAMP2_REG                 0x42

#define LSM6DSM_REG_STEP_TIMESTAMP_L               0x49
#define LSM6DSM_REG_STEP_TIMESTAMP_H               0x4A
#define LSM6DSM_REG_STEP_COUNTER_L                 0x4B
#define LSM6DSM_REG_STEP_COUNTER_H                 0x4C
#define LSM6DSM_REG_SENSORHUB13_REG                0x4D
#define LSM6DSM_REG_SENSORHUB14_REG                0x4E
#define LSM6DSM_REG_SENSORHUB15_REG                0x4F
#define LSM6DSM_REG_SENSORHUB16_REG                0x50
#define LSM6DSM_REG_SENSORHUB17_REG                0x51
#define LSM6DSM_REG_SENSORHUB18_REG                0x52
#define LSM6DSM_REG_FUNC_SRC1                      0x53
#define LSM6DSM_REG_FUNC_SRC2                      0x54
#define LSM6DSM_REG_WRIST_TILT_IA                  0x55

#define LSM6DSM_REG_TAP_CFG                        0x58
#define LSM6DSM_REG_TAP_THS_6D                     0x59
#define LSM6DSM_REG_INT_DUR2                       0x5A
#define LSM6DSM_REG_WAKE_UP_THS                    0x5B
#define LSM6DSM_REG_WAKE_UP_DUR                    0x5C
#define LSM6DSM_REG_FREE_FALL                      0x5D
#define LSM6DSM_REG_MD1_CFG                        0x5E
#define LSM6DSM_REG_MD2_CFG                        0x5F
#define LSM6DSM_REG_MASTER_CMD_CODE                0x60
#define LSM6DSM_REG_SENS_SYNC_SPI_ERROR_CODE       0x61

#define LSM6DSM_REG_OUT_MAG_RAW_X_L                0x66
#define LSM6DSM_REG_OUT_MAG_RAW_X_H                0x67
#define LSM6DSM_REG_OUT_MAG_RAW_Y_L                0x68
#define LSM6DSM_REG_OUT_MAG_RAW_Y_H                0x69
#define LSM6DSM_REG_OUT_MAG_RAW_Z_L                0x6A
#define LSM6DSM_REG_OUT_MAG_RAW_Z_H                0x6B

#define LSM6DSM_REG_INT_OIS                        0x6F
#define LSM6DSM_REG_CTRL1_OIS                      0x70
#define LSM6DSM_REG_CTRL2_OIS                      0x71
#define LSM6DSM_REG_CTRL3_OIS                      0x72
#define LSM6DSM_REG_X_OFS_USR                      0x73
#define LSM6DSM_REG_Y_OFS_USR                      0x74
#define LSM6DSM_REG_Z_OFS_USR                      0x75

// Bank A embedded function registers
#define LSM6DSM_EFREGA_SLV0_ADD                    0x02
#define LSM6DSM_EFREGA_SLV0_SUBADD                 0x03
#define LSM6DSM_EFREGA_SLAVE0_CONFIG               0x04
#define LSM6DSM_EFREGA_SLV1_ADD                    0x05
#define LSM6DSM_EFREGA_SLV1_SUBADD                 0x06
#define LSM6DSM_EFREGA_SLAVE1_CONFIG               0x07
#define LSM6DSM_EFREGA_SLV2_ADD                    0x08
#define LSM6DSM_EFREGA_SLV2_SUBADD                 0x09
#define LSM6DSM_EFREGA_SLAVE2_CONFIG               0x0A
#define LSM6DSM_EFREGA_SLV3_ADD                    0x0B
#define LSM6DSM_EFREGA_SLV3_SUBADD                 0x0C
#define LSM6DSM_EFREGA_SLAVE3_CONFIG               0x0D
#define LSM6DSM_EFREGA_DATAWRITE_SRC_MODE_SUB_SLV0 0x0E
#define LSM6DSM_EFREGA_CONFIG_PEDO_THS_MIN         0x0F

#define LSM6DSM_EFREGA_SM_THS                      0x13
#define LSM6DSM_EFREGA_PEDO_DEB_REG                0x14
#define LSM6DSM_EFREGA_STEP_COUNT_DELTA            0x15

#define LSM6DSM_EFREGA_MAG_SI_XX                   0x24
#define LSM6DSM_EFREGA_MAG_SI_XY                   0x25
#define LSM6DSM_EFREGA_MAG_SI_XZ                   0x26
#define LSM6DSM_EFREGA_MAG_SI_YX                   0x27
#define LSM6DSM_EFREGA_MAG_SI_YY                   0x28
#define LSM6DSM_EFREGA_MAG_SI_YZ                   0x29
#define LSM6DSM_EFREGA_MAG_SI_ZX                   0x2A
#define LSM6DSM_EFREGA_MAG_SI_ZY                   0x2B
#define LSM6DSM_EFREGA_MAG_SI_ZZ                   0x2C
#define LSM6DSM_EFREGA_MAG_OFFX_L                  0x2D
#define LSM6DSM_EFREGA_MAG_OFFX_H                  0x2E
#define LSM6DSM_EFREGA_MAG_OFFY_L                  0x2F
#define LSM6DSM_EFREGA_MAG_OFFY_H                  0x30
#define LSM6DSM_EFREGA_MAG_OFFZ_L                  0x31
#define LSM6DSM_EFREGA_MAG_OFFZ_H                  0x32

// Bank B embedded function registers
#define LSM6DSM_EFREGB_A_WRIST_TILT_LAT            0x50

#define LSM6DSM_EFREGB_A_WRIST_TILT_THS            0x54

#define LSM6DSM_EFREGB_A_WRIST_TILT_MASK           0x59

class lsm6dsm
{
  public:
    lsm6dsm();
    int reset();
    int reboot();
    int begin(uint8_t arange, uint8_t grange, uint8_t odr);
    int begin();
    int end();
    int available(uint16_t* diff_fifo, uint8_t* wm_flag);
    int getPattern(uint16_t* fifo_pattern);
    int readFIFO(uint8_t* buffer, uint8_t length);
    int readRegister(uint8_t address, uint8_t* destination);
    int readRegisters(uint8_t startAddress, uint8_t* values, uint8_t length);
    int writeRegister(uint8_t address, uint8_t value);
    int writeRegisters(uint8_t startAddress, uint8_t* values, uint8_t length);
    int setWUThreshold(uint8_t WUThreshold);
    int setWUDuration(uint8_t WUDuration);
    int setFIFOwatermark(uint16_t watermark);
    int getWakeUpSource(uint8_t* source);
    int getTapSource(uint8_t* source);
    bool isTap();
    int enableWUInterrupt();
    int enableInactivityInterrupt();
    int disableWUInterrupt();
    int enableWMInterrupt();
    int disableWMInterrupt();
  private:
    bool initialized;
};

#endif