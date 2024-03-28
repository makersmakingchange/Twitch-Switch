#include <Arduino.h>
#include <Wire.h>
#include "lsm6dsm.h"

// SEE: https://www.st.com/resource/en/application_note/an4987-lsm6dsm-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf

/*
int main() {
  uint8_t tx_buffer[1000];
  int16_t data_raw[3];
stmdev_ctx_t dev_ctx;
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t whoamI;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  dev_ctx.handle = &SENSOR_BUS;
  platform_init();
  platform_delay(BOOT_TIME);
  lsm6dsm_device_id_get(&dev_ctx, &whoamI);

  if (whoamI != LSM6DSM_ID)
    while (1)
      ;

  lsm6dsm_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    lsm6dsm_reset_get(&dev_ctx, &rst);
  } while (rst);

  lsm6dsm_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_52Hz);
  lsm6dsm_xl_full_scale_set(&dev_ctx, LSM6DSM_4g);
  platform_delay(WAIT_TIME_A);
  do {
    lsm6dsm_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);
  lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw);
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    do {
      lsm6dsm_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsm_from_fs4g_to_mg(data_raw[j]);
    }
  }

  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  lsm6dsm_xl_self_test_set(&dev_ctx, LSM6DSM_XL_ST_NEGATIVE);
  //lsm6dsm_xl_self_test_set(&dev_ctx, LSM6DSM_XL_ST_POSITIVE);
  platform_delay(WAIT_TIME_A);

  do {
    lsm6dsm_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw);
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    do {
      lsm6dsm_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    lsm6dsm_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsm_from_fs4g_to_mg(data_raw[j]);
    }
  }

  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if ((MIN_ST_LIMIT_mg > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  lsm6dsm_xl_self_test_set(&dev_ctx, LSM6DSM_XL_ST_DISABLE);
  lsm6dsm_xl_data_rate_set(&dev_ctx, LSM6DSM_XL_ODR_OFF);
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_208Hz);
  lsm6dsm_gy_full_scale_set(&dev_ctx, LSM6DSM_2000dps);
  platform_delay(WAIT_TIME_G_01);

  do {
    lsm6dsm_gy_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw);
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    do {
      lsm6dsm_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += lsm6dsm_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  lsm6dsm_gy_self_test_set(&dev_ctx, LSM6DSM_GY_ST_POSITIVE);
  //lsm6dsm_gy_self_test_set(&dev_ctx, LIS2DH12_GY_ST_NEGATIVE);
  platform_delay(WAIT_TIME_G_02);
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    do {
      lsm6dsm_gy_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    lsm6dsm_angular_rate_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += lsm6dsm_from_fs2000dps_to_mdps(data_raw[j]);
    }
  }

  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  for (i = 0; i < 3; i++) {
    if ((MIN_ST_LIMIT_mdps > test_val[i]) || (test_val[i] > MAX_ST_LIMIT_mdps)) {
      st_result = ST_FAIL;
    }
  }

  lsm6dsm_gy_self_test_set(&dev_ctx, LSM6DSM_GY_ST_DISABLE);
  lsm6dsm_gy_data_rate_set(&dev_ctx, LSM6DSM_GY_ODR_OFF);

  if (st_result == ST_PASS) {
    sprintf((char*)tx_buffer, "Self Test - PASS\r\n");
  }

  else {
    sprintf((char*)tx_buffer, "Self Test - FAIL\r\n");
  }

  tx_com(tx_buffer, strlen((char const*)tx_buffer));
}
*/

int lsm6dsm::reset() {
  uint8_t readval;
  int ret;

  ret = writeRegister(LSM6DSM_REG_CTRL2_G, 0);  // Set gyro to power down

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_CTRL1_XL, 0xA0);  // Set accelerometer to high-performance
  }

  if (ret == 0) {
    ret = readRegister(LSM6DSM_REG_CTRL3_C, &readval);
  }

  if (ret == 0) {
    readval |= 0x01;                                    // Set SW_RESET
    ret = writeRegister(LSM6DSM_REG_CTRL3_C, readval);  // Reset device
  }

  if (ret == 0) {
    do {
      ret = readRegister(LSM6DSM_REG_CTRL3_C, &readval);  // Wait for reset to end
    } while (readval & 0x01);
  }

  return ret;
}

int lsm6dsm::reboot() {
  int ret;

  ret = writeRegister(LSM6DSM_REG_CTRL2_G, 0);  // Set gyro to power down

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_CTRL1_XL, 0xA0);  // Set accelerometer to high-performance
  }

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_CTRL3_C, 0x80);  // Reboot device
  }

  delay(25);

  return ret;
}

int lsm6dsm::begin() {
  return begin(0, 1, 5);  // 2g, 125dps, 208 Hz
}

int lsm6dsm::begin(uint8_t arange, uint8_t grange, uint8_t odr) {
  uint8_t readval;
  int ret;

  arange &= 0b11;
  grange &= 0b111;

  if (initialized == true) {
    return 0;
  }

  ret = readRegister(LSM6DSM_REG_WHO_AM_I, &readval);

  if (ret == 0 && readval != 0x6A) {
    ret = -1;  // Fail if who am I is incorrect
  }

  if (ret == 0) {
    ret = reset();
  }

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_CTRL3_C, 0x44);  // Auto-increment burst access and use BDU
    //0x44 = 01000100
    //CTRL3_C (12h) = BOOT BDU H_LACTIVE PP_OD SIM IF_INC BLE SW_RESET
    //                  0   1      0        0    0   1     0     0
  }

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_CTRL1_XL, (odr << 4) | (arange << 2));  // Set full-scale accelerometer range to +/- ???g
  }

  delay(100);

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_CTRL8_XL, (1 << 2));  // Set accelerometer to high-passed mode
  }

  if (ret == 0) {                                                          // grange: anything odd == 125, 0 == 250, 2 == 500, 4 == 1000, 6 == 2000
    ret = writeRegister(LSM6DSM_REG_CTRL2_G, (odr << 4) | (grange << 1));  // Set full-scale gyroscope range to +/- ??? dpsR
  }

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_CTRL7_G, (1 << 6) | (3 << 4));
  }

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_FIFO_CTRL5, (odr << 3) | 0x06);  // 208Hz ODR, continuous FIFO
  }

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_FIFO_CTRL3, 0x09);  // No gyro or accelerometer FIFO decimation
  }

  if (ret == 0) {
    //SEE Page 36/124 of https://www.st.com/resource/en/application_note/an4987-lsm6dsm-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf
    // 0x91 = 0b10010001 Enable wake-up interrupt generation, use digital HPF selection, latch interrupts
    // 0x90 = 0b10010000 Enable wake-up interrupt generation, use digital HPF selection, NO latch interrupts
    //                   - I thought this would cause the FIFO from filling when the tap 'stops', but doesn't seem to work...
    // 0x80 = 0b10000000 Enable wake-up interrupt generation, no filtering, no latch.

    // TAP_CFG 58h = INTERRUPTS ENABLE | INACT_EN1 | INACT_EN0 | SLOPE_FDS | TAP_X_EN | TAP_Y_EN | TAP_Z_EN | LIR
    // FOR LIR, 0 is LATCH DISABLED.

    // Inactivity configuration: acc to 12.5 LP, gyro to Power-Down Enable slope filter
    //E0h = 11100000
    
    // 0x0D = 0b11010000
    //ret = writeRegister(LSM6DSM_REG_TAP_CFG, 0x90);  // Enable wake-up interrupt generation, use digital HPF selection, no latch interrupts
    
    ret = writeRegister(LSM6DSM_REG_TAP_CFG, 0xE0);  // Enable wake-up interrupt generation, use digital HPF selection, no latch interrupts
    

    //Jan 26 - 0x80 did nothing... because we didn't set it to detect x/y/z?
    // Try 10001110 = 0x8E
    // Try 10001111 = 0x8F
    //ret = writeRegister(LSM6DSM_REG_TAP_CFG, 0b10010000); 
  }

  if (ret == 0) {
    initialized = true;
  }

  return ret;
}

// Read the Tap Source register and see if SINGLE_TAP (bit 5) is true
bool lsm6dsm::isTap() {
  //SINGLE_TAP = Bit5 true

  uint8_t temp;
  int ret = readRegister(LSM6DSM_REG_TAP_SRC, &temp);

  if (temp & 0b10000)
    return true;
  else
    return false;
}

int lsm6dsm::getTapSource(uint8_t* source) {
  int ret;

  ret = readRegister(LSM6DSM_REG_TAP_SRC, source);

  return ret;  
}

int lsm6dsm::getWakeUpSource(uint8_t* source) {
  int ret;

  //If latched mode is disabled (LIR bit of TAP_CFG is set to 0), the interrupt signal is automatically reset when the
  //  free-fall condition is no longer verified. I
  ret = readRegister(LSM6DSM_REG_WAKE_UP_SRC, source);

  return ret;
}

int lsm6dsm::setFIFOwatermark(uint16_t watermark) {
  int ret;
  uint8_t temp;

  ret = writeRegister(LSM6DSM_REG_FIFO_CTRL1, watermark & 0xFF);

  if (ret == 0) {
    ret = readRegister(LSM6DSM_REG_FIFO_CTRL2, &temp);
  }

  if (ret == 0) {
    temp &= 0xF8;
    temp |= (watermark >> 8) & 0x07;
    ret = writeRegister(LSM6DSM_REG_FIFO_CTRL2, temp);
  }

  return ret;
}

// Args:
//     buffer: the place you want the FIFO data to end up
//     length: how much data to MOVE to the buffer (in bytes). SHOULD / MUST be an even number so we're reading full bytes. Ideally 24.
int lsm6dsm::readFIFO(uint8_t* buffer, uint8_t length) {
  int ret;

  ret = readRegisters(LSM6DSM_REG_FIFO_DATA_OUT_L, buffer, length);

  return ret;
}

int lsm6dsm::end() {
  int ret;

  initialized = false;

  ret = reset();

  return ret;
}

int lsm6dsm::setWUDuration(uint8_t WUDuration) {
  int ret;

  ret = writeRegister(LSM6DSM_REG_WAKE_UP_DUR, (WUDuration & 0x03) << 5);

  return ret;
}

int lsm6dsm::setWUThreshold(uint8_t WUThreshold) {
  int ret;

  //0x3F = 0b00111111
  ret = writeRegister(LSM6DSM_REG_WAKE_UP_THS, WUThreshold & 0x3F);
  //The SINGLE_DOUBLE_TAP bit of WAKE_UP_THS has to be set to 0 in order to enable single-tap recognition only
  
  return ret;
}

// Return: success code. 0=success
// Args:
//      diff_fifo: pointer to how much data is in the IMU FIFO in number of words
//      wm_flag: boolean. True if watermark is surpassed
int lsm6dsm::available(uint16_t* diff_fifo, uint8_t* wm_flag) {
  int ret;
  uint8_t temp;

  ret = readRegister(LSM6DSM_REG_FIFO_STATUS2, &temp);

  if (ret == 0) {
    if (temp & 0x10) {  // If empty
      *diff_fifo = 0;
      *wm_flag = 0;
    } else if (temp & 0x60) {  // If full
      *diff_fifo = 2048;
      *wm_flag = 1;
    } else {  // Otherwise...
      *wm_flag = (temp & 0x80 ? 1 : 0);
      *diff_fifo = ((uint16_t)temp & 0x0007) << 8;
      ret = readRegister(LSM6DSM_REG_FIFO_STATUS1, &temp);
      *diff_fifo += (uint16_t)temp;
    }
  }
  return ret;
}

// Returns where we are in the pattern. We've set it up to have a pattern length to be 6. AXYZ GXYZ.
// This number should be < 6
int lsm6dsm::getPattern(uint16_t* fifo_pattern) {
  int ret;
  uint8_t temp;

  ret = readRegister(LSM6DSM_REG_FIFO_STATUS3, &temp);

  if (ret == 0) {
    *fifo_pattern = temp;
    ret = readRegister(LSM6DSM_REG_FIFO_STATUS4, &temp);
  }

  if (ret == 0) {
    *fifo_pattern |= ((uint16_t)temp) << 8;
  }

  return ret;
}

//Activity/Inactivity recognition - https://www.st.com/resource/en/application_note/an4987-lsm6dsm-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf Section 5.6
//Figure 20. Activity/Inactivity recognition (using the slope filter)
int lsm6dsm::enableInactivityInterrupt() {
//INT1_INACT_STATE: Inactivity interrupt on INT1
  int ret;
  uint8_t readval;

  ret = readRegister(LSM6DSM_REG_MD1_CFG, &readval);

  if (ret == 0) {
    //0b10000000. Bit7. 80h INT1_INACT_STATE
    ret = writeRegister(LSM6DSM_REG_MD1_CFG, readval | 0b10000000);
  }

  return ret;
}


int lsm6dsm::enableWUInterrupt() {
  int ret;
  uint8_t readval;

  ret = readRegister(LSM6DSM_REG_MD1_CFG, &readval);

  if (ret == 0) {
    //0x20 = 0b100000. Bit5. INT1_WU
    ret = writeRegister(LSM6DSM_REG_MD1_CFG, readval | 0x20);
  }

  return ret;
}

int lsm6dsm::disableWUInterrupt() {
  int ret;
  uint8_t readval;

  ret = readRegister(LSM6DSM_REG_MD1_CFG, &readval);

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_MD1_CFG, readval & ~0x20);
  }

  return ret;
}

int lsm6dsm::enableWMInterrupt() {
  int ret;
  uint8_t readval;

  ret = readRegister(LSM6DSM_REG_INT1_CTRL, &readval);

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_INT1_CTRL, readval | 0x08);
  }

  return ret;
}

int lsm6dsm::disableWMInterrupt() {
  int ret;
  uint8_t readval;

  ret = readRegister(LSM6DSM_REG_INT1_CTRL, &readval);

  if (ret == 0) {
    ret = writeRegister(LSM6DSM_REG_INT1_CTRL, readval & ~0x08);
  }

  return ret;
}


int lsm6dsm::readRegister(uint8_t address, uint8_t* destination) {
  int ret;
  Wire.beginTransmission(LSM6DSM_ADDRESS);
  Wire.write(address);

  ret = Wire.endTransmission(false);
  if (ret != 0) {
    return ret;
  }

  ret = Wire.requestFrom(LSM6DSM_ADDRESS, 1);
  if (ret != 1) {
    return 6;
  }

  *destination = Wire.read();
  return 0;
}

int lsm6dsm::readRegisters(uint8_t startAddress, uint8_t* values, uint8_t length) {
  int ret;
  Wire.beginTransmission(LSM6DSM_ADDRESS);
  Wire.write(startAddress);

  ret = Wire.endTransmission(false);
  if (ret != 0) {
    return ret;
  }

  if (Wire.requestFrom(LSM6DSM_ADDRESS, length) != length) {
    return 6;
  }

  for (uint8_t i = 0; i < length; i++) {
    values[i] = Wire.read();
  }
  return 0;
}

int lsm6dsm::writeRegister(uint8_t address, uint8_t value) {
  Wire.beginTransmission(LSM6DSM_ADDRESS);
  Wire.write(address);
  Wire.write(value);
  return Wire.endTransmission();
}

int lsm6dsm::writeRegisters(uint8_t startAddress, uint8_t* values, uint8_t length) {
  Wire.beginTransmission(LSM6DSM_ADDRESS);
  Wire.write(startAddress);
  for (uint8_t i = 0; i < length; i++) {
    Wire.write(values[i]);
  }
  return Wire.endTransmission();
}

lsm6dsm::lsm6dsm() {
}