#ifndef __CONFIG_CHECK_H__
#define __CONFIG_CHECK_H__

#ifndef MEGATINYCORE_MCU
#error "Must use MegaTinyCore"
#else

#if (MEGATINYCORE_MCU != 1616)
#error "Must select ATtiny3226/3216/1626/1616/1606/826/816/806/426/416/406"
#else

// I want to put a brown-out voltage warning here as well
// but it appears as though there is no easy define to check
// because it is a fuse setting. Very frustrating. If you are
// reading this - DON'T SET BOD GREATER THAN 3 VOLTS. This will
// brick the chip.

#if (CLOCK_SOURCE != 0)
#error "Clock source must be set to internal"
#endif

#if (F_CPU != 5000000)
#error "CPU frequency must be set to 5MHz"
#endif

#ifndef MILLIS_USE_TIMERRTC
#error "RTC must be set as millis source"
#endif

#endif /*(MEGATINYCORE_MCU == 1606)*/
#endif /*defined(MEGATINYCORE_MCU)*/

#endif  /*CONFIG_CHECK_H*/