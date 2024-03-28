#ifndef __PINS_H__
#define __PINS_H__

// All pins not explicitly defined here are
// taken care of already. This includes I2C and SPI,
// including chip-select.

// Output
#define PIN_STATUS_LED PIN_PA5
#define PIN_RF_CE PIN_PA7

// Input with interrupt on change
#define PIN_PWR_DOWN PIN_PA6

// Input with interrupt on change
#define PIN_IMU_INT1 PIN_PB2
#define PIN_BAT_PG PIN_PB3
#define PIN_BAT_CHG PIN_PB4

// Analog input
#define PIN_VBAT_SENSE PIN_PB5

// Input with interrupt on change
#define PIN_RF_INT PIN_PC2

#endif  /*PINS_H*/