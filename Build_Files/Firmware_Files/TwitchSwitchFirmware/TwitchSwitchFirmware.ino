/*
                   TWITCH SWITCH (SWTICH) Mk-IV

            Stan Cassidy Center for Rehabilitation
                      Oct 25, 2023

            Arduino Platform: 
              Base: Adafruit Feather nRF52840 Express
              SWITCH: CPU - Adafruit ATtiny 1616
                      IMU - https://www.st.com/en/mems-and-sensors/lsm6dsm.html https://www.st.com/resource/en/application_note/an4987-lsm6dsm-alwayson-3d-accelerometer-and-3d-gyroscope-stmicroelectronics.pdf
                      Charger - https://www.ti.com/lit/ds/symlink/bq24045.pdf
            Firmware version number#: TODO: 20231004-01
            For updates, see: https://github.com/rmccaskill/TwitchSwitch2023
            For instructions on building the programmer for the ATTiny, see: https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md

            (C)2023 STAN CASSIDY CENTER FOR REHABILITATION (SCCR)
                              ALL RIGHT SRESERVED

                           ===== DISCLAMER =====
  THIS SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF, OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.

  ----------------------------------------------------------------------------------------------
TODO: UPDATE

  Special features:
     #Adafruit nRF52840 Feather Express

  Single button operation:
     # Base unit connects to 3 swtiches via radio. These switches stream their data back to the base. The base reads the values 
       and 'decides' when a 'twitch' happens and triggers the appropriate action (move move / button press)
     # Three profiles - indivudually tuned for different scenarios. For example: sitting in a chair vs laying in bed.

  LED messages:
    NOTE - CHARGE INDICATORS AREN'T WORKING YET
  //Battery State indicator (TODO)
  // Low battery = Power ON:  Blink LED every few seconds
  // Charging = POWER ON or OFF: Breath LED
  // Charged = POWER ON or OFF: LED ON SOLID

  NOTES
  - For programming the firmware - see https://github.com/SpenceKonde/AVR-Guidance/blob/master/UPDI/jtag2updi.md
*/

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>

#include "pins.h"
#include <avr/sleep.h>

#include <stdio.h>

#include <Wire.h>
#include "lsm6dsm.h"

#define WM_LEVEL 192
#define WU_THRESHOLD 2

RF24 radio(PIN_RF_CE, PIN_SPI_SS);
lsm6dsm imu;

uint8_t readbuf[32];
uint8_t buffer[24];

uint8_t ourAddress[5] = { 0x8d, 0x97, 0xb0, 0xe7, 0x47 };

// Semiphore to track when power switch has changed state
volatile uint8_t handlePowerSwitch = 1; //On first - we want the LED to blink if it's plugged in and turned on.

volatile uint8_t handleIMUInterrupt = 0;
volatile uint8_t handleRadioInterrupt = 0;


//Charger details: https://www.ti.com/lit/ds/symlink/bq24045.pdf?ts=1706249391058
volatile uint8_t handleBatPG = 0; //Low (FET on) indicates the input voltage is above UVLO and the OUT (battery) voltage.
volatile uint8_t handleBatCharge = 0; //Low (FET on) indicates charging and Open Drain (FET off) indicates no Charging or Charge complete
volatile uint8_t handleBatSense = 0; // What does this do?

//Power states: ON, Charging, Charged, OFF

enum {
  MODE_OFF = 0,
  MODE_WAITING = 1,
  MODE_SENDING = 2,
};

uint8_t mode = MODE_OFF;

ISR(PORTC_PORT_vect, ISR_ALIASOF(PORTA_PORT_vect));
ISR(PORTB_PORT_vect, ISR_ALIASOF(PORTA_PORT_vect));
ISR(PORTA_PORT_vect) {
  // 1 - why are we here. Who fired?
  // 2 - Respond to the cause
  // 3 - clear interrupt flag
  byte flags_a = VPORTA.INTFLAGS;
  byte flags_b = VPORTB.INTFLAGS;
  byte flags_c = VPORTC.INTFLAGS;

  // 1st - check if it was the power switch
  // Power is on PORTA PA6, so check there
  if (flags_a & (1 << 6)) {
    handlePowerSwitch = 1;
    VPORTA.INTFLAGS |= (1 << 6);
  }

  // 1st - check if it was the IMU that interrupted
  // IMU_INT1 is on PB2, so check there
  if (flags_b & (1 << 2)) {
    handleIMUInterrupt = 1;
    VPORTB.INTFLAGS |= (1 << 2);
  }

  // BAT_PG is on PB3
  if (flags_b & (1 << 3)) {
    handleBatPG = 1;
    VPORTB.INTFLAGS |= (1 << 3);
  }

  // BAT_CHG is on PB4
  if (flags_b & (1 << 4)) {
    handleBatCharge = 1;
    VPORTB.INTFLAGS |= (1 << 4);
  }

  // PIN_VBAT_SENSE is on PB5
  if (flags_b & (1 << 5)) {
    handleBatSense = 1;
    VPORTB.INTFLAGS |= (1 << 5);
  }
  

  // RADIO_INT is on PC2
  if (flags_c & (1 << 2)) {
    handleRadioInterrupt = 1;
    VPORTC.INTFLAGS |= (1 << 2);
  }
}

/*
ISR(TCA0_CMP0_vect) {
  // LED alarm tripped
  TCA0.SINGLE.INTFLAGS = TCA_SINGLE_CMP0_bm;
  // Do the stuff to handle the LED

  // If we want to turn LED on in 5.76 seconds:
  // Multiply seconds by clock rate - 5.76 * 5e6 / 64 = 450000
  // Take cycle number and divide by 2^16 for number of overflows required: 450000 / 2^16 = 6
  // Take remainder of previous division and multiply by 2^16 for the alarm compare: 0.86645508*2^16 = 56784

  TCA0.SINGLE.CMP0 = TCA0.SINGLE.CNT + 39062;
  digitalWriteFast(PIN_STATUS_LED, !digitalReadFast(PIN_STATUS_LED));
}
*/

void handleWDR() {
  // Copy all important bug-detection info somewhere safe. We want to look at it to diagnose the problem.
}

/*
void init_TCA0() {
  TCA0.SINGLE.PER = 0xFFFF;  // This can be changed. Currently as high as possible
  TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
  TCA0.SINGLE.INTCTRL = TCA_SINGLE_CMP0_bm;  // Enable LED interrupt
  TCA0.SINGLE.CTRLA = (TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm);
}
*/

void setup() {
  // If WDT caused reset, run handleWDR.
  uint8_t resetcause = GPIOR0;
  if (resetcause & 0x08) {
    handleWDR();
  }

  set_sleep_mode(SLEEP_MODE_IDLE); // Use standby for 100x less power, but only if you don't need TCA0
  sleep_enable();

  pinConfigure(PIN_STATUS_LED, OUTPUT);
  //For more details about configuring pins on the ATtiny - sell https://github.com/SpenceKonde/megaTinyCore/blob/master/megaavr/extras/Ref_Digital.md

  //Configure the pins so that we can read which switch this is (1,2,3)
  pinConfigure(PIN_PC0, PIN_PULLUP | PIN_DIR_INPUT);
  pinConfigure(PIN_PC1, PIN_PULLUP | PIN_DIR_INPUT);
  pinConfigure(PIN_PC3, PIN_PULLUP | PIN_DIR_INPUT);

  pinConfigure(PIN_IMU_INT1, PIN_DIR_INPUT | PIN_ISC_RISE);
  pinConfigure(PIN_PWR_DOWN, PIN_DIR_INPUT | PIN_ISC_CHANGE);


  pinConfigure(PIN_BAT_PG, PIN_DIR_INPUT | PIN_ISC_CHANGE);
  pinConfigure(PIN_BAT_CHG, PIN_DIR_INPUT | PIN_ISC_CHANGE);
  pinConfigure(PIN_VBAT_SENSE, PIN_DIR_INPUT | PIN_ISC_CHANGE);

  delay(15);

  Wire.begin();
  Wire.setClock(400000);

  //Read in port C. Mask off the blobs that we're interested in. These are the solder blobs on the board. A0, A1, A2 on the board
  uint8_t configPins = VPORTC.IN & 0b00001011;
  ourAddress[0] += configPins;  //Added to 0x8d

  // We can save some power by turning off the pads we read as grounded (soldered)
  if (!(configPins & (1 << 3))) {
    // PC3 IS soldered
    pinConfigure(PIN_PC3, PIN_DIR_INPUT);
  }
  if (!(configPins & (1 << 1))) {
    // PC1 IS soldered
    pinConfigure(PIN_PC1, PIN_DIR_INPUT);
  }
  if (!(configPins & (1 << 0))) {
    // PC0 IS soldered
    pinConfigure(PIN_PC0, PIN_DIR_INPUT);
  }
  //1010, 1001, 0011

  //Initial mode (on / off) based on the power switch
  if (digitalReadFast(PIN_PWR_DOWN) == 1) {
    //ON
    mode = MODE_WAITING; 
  }
  else
  {
    mode = MODE_OFF; 
  }
  

  while (!radio.begin())
    ;

  radio.setDataRate(RF24_2MBPS);
  radio.enableDynamicPayloads();
  //radio.enableAckPayload();
  
  //Nov 18
  radio.setAutoAck(false);

  radio.setPALevel(0);
  radio.setChannel(57);
  radio.openWritingPipe(ourAddress);
  radio.stopListening();
}

bool isCharging = false; //digitalReadFast(PIN_BAT_CHG);

void startCharging()
{
  digitalWriteFast(PIN_STATUS_LED, HIGH);
  isCharging = true;
}

void stopCharging()
{
  digitalWriteFast(PIN_STATUS_LED, LOW);
  isCharging = false;
}

void blinkLED(long msDelay)
{
  for (uint8_t i = 0; i < 10; i++) {
    digitalWriteFast(PIN_STATUS_LED, !digitalReadFast(PIN_STATUS_LED));
    delay(msDelay);
  }
  digitalWriteFast(PIN_STATUS_LED, LOW);
}

// Note - this is blocking. Don't use in production. Debugging only.
void breathLED()
{
  for (uint8_t i = 0; i < 255; i++) {
    analogWrite(PIN_STATUS_LED, i);
    delay(5);
  }

  for (uint8_t i = 255; i > 0; i--) {
    analogWrite(PIN_STATUS_LED, i);
    delay(5);
  }
  digitalWriteFast(PIN_STATUS_LED, LOW);

}

void loop()
{

/*
  // Note Feb 2, 2024.PIN_BAT_CHG NOT high when charging. PIN_BAT_CHG is seemingly NEVER high...
  // From DOCS: CHG 8 8 O Low (FET on) indicates charging and Open Drain (FET off) indicates no Charging or Charge complete
  if (digitalReadFast(PIN_BAT_PG) == LOW)
  {
    digitalWriteFast(PIN_STATUS_LED, HIGH);
    //startCharging();
  }
  else
  {
    digitalWriteFast(PIN_STATUS_LED, LOW);
    //stopCharging();

  }
  //return; 
*/
  static uint8_t packetnum = 0;


  //Read vbat somehow? See https://forum.arduino.cc/t/voltage-reading/57530/2
  //r9 = 215k ohm
  //r10 = 113k ohm
  //const float R9 = 215000.0; // Put exact value here
  //const float R10 = 113000.0; // Put exact value here
  //const float Vref = 3.7;   // Put exact value here
  //int adcReading = analogRead(PIN_VBAT_SENSE);
  //float voltage = (R9+R10)/R9 * adcReading * Vref / 1024.0;

/*
  if (handleBatCharge)
  {
    cli();  // Disable interrupts
    handleBatCharge = 0;
    sei();  // Enable interrupts

    if (digitalReadFast(PIN_BAT_CHG))
      startCharging();
    else
      stopCharging();
  }
*/

  //If IMU Interrupt fired. This is configured to fire when the IMU goes idle. Thatis, when it's not being moved any more.
  // When we detect this, we can stop send IMU data over the radio to the base unit
  if (handleIMUInterrupt)
  {
    cli();  // Disable interrupts
    handleIMUInterrupt = 0;
    sei();  // Enable interrupts

    transitionToWaiting();
  }

  if (handlePowerSwitch) {
    cli();  // Disable interrupts
    handlePowerSwitch = 0;
    sei();  // Enable interrupts

    if (digitalReadFast(PIN_PWR_DOWN) == 1)
    {
      //Power is now ON, blink 5 times to indicate 'power on' change)
      blinkLED(100);

      transitionToWaiting();
    }
    else
    {
        //Turn off everything
        transitionToOff();
    }
  }

  //Not doing anything here for now
  if (handleRadioInterrupt)
  {
    cli();  // Disable interrupts
    handleRadioInterrupt = 0;
    sei();  // Enable interrupts
  }

  switch (mode) {
    case MODE_OFF:
      {
        //We're OFF

        //The power switch was turned on
        if (digitalReadFast(PIN_PWR_DOWN) == 1) {
          transitionToWaiting();
        } 
        else
        {
          //Turn off LED UNLESS we're charging...
          if (!isCharging)
            digitalWriteFast(PIN_STATUS_LED, LOW);

          sleep_cpu();
        }
      }
      break;
    case MODE_WAITING:
      {
        //The power switch was turned off
        if (digitalReadFast(PIN_PWR_DOWN) == 0) {
          transitionToOff();
          break;
        }

        uint8_t source;
        imu.getWakeUpSource(&source);
        if (source) {
          transitionToSending();
        } else
          sleep_cpu();
      }
      break;
    case MODE_SENDING:
      {
        //The power switch was turned off
        if (digitalReadFast(PIN_PWR_DOWN) == 0) {
          transitionToOff();
          break;
        }

        uint16_t fifo_level;  //this is in WORDS
        uint8_t wm_flag;      //watermark full?

        uint16_t vbatReading = analogRead(PIN_VBAT_SENSE); //Needs two bytes to send this over the radio since our array is (uint8)
        uint8_t imu_data[29]; //Grew from 27 so that we can send battery info over as well

        if (imu.available(&fifo_level, &wm_flag)) {
          //Something went wrong
          for (uint8_t i = 0; i < 10; i++) {
            digitalWriteFast(PIN_STATUS_LED, !digitalReadFast(PIN_STATUS_LED));
            delay(2000);
          }
        } 
        else if (wm_flag || fifo_level >= WM_LEVEL)
        {
          // Pattern
          uint16_t patternOffset;
          imu.getPattern(&patternOffset);

          while ((patternOffset != 0) && (fifo_level != 0))
          {
            //Discard a sample into a garbage location (could be anywhere, but we'll use patternOffset to save a location)
            imu.readFIFO((uint8_t*)&patternOffset, 2);

            // update the variables we're checking in this while loop
            imu.available(&fifo_level, &wm_flag);
            imu.getPattern(&patternOffset);
          }

          uint8_t _failCounter = 0;
          while (fifo_level >= 12)
          {
digitalWriteFast(PIN_STATUS_LED, HIGH);

            //we have 24 bytes
            //move the bytes from the fifo to a buffer then to the radio
            // 24 byte buffer.
            imu.readFIFO(imu_data, 24);

            //fifo_level is 16 bits. Need to split over 2 array indexes 
            imu_data[24] = fifo_level & 0xFF;
            imu_data[25] = (fifo_level >> 8) & 0xFF;

            imu_data[26] = packetnum++;

            //vbatReading is 16 bits. Need to split over 2 array indexes 
            imu_data[27] = (vbatReading >> 8) & 0xFF;
            imu_data[28] = vbatReading & 0xFF;

            if (radio.isAckPayloadAvailable()) {

              radio.read(readbuf, radio.getDynamicPayloadSize());
              // We recieved ack data in readbuf. Handle it here
              if (readbuf[0] == 'S')
              {
                transitionToWaiting();
                packetnum = 0;
                break;
              }
            }

            if (!radio.write(imu_data, 29))
            {
              _failCounter++;
            }

            if (_failCounter >= 20)
            {
              //we failed a lot.... the base unit is gone maybe?
              // break out of this loop so we can start fresh again

              //Fast blink LED so that we can 'see' the error
              for (uint8_t i = 0; i < 6; i++) {
                digitalWriteFast(PIN_STATUS_LED, !digitalReadFast(PIN_STATUS_LED));
                delay(100);
              }
              //Make sure we leave the LED on
              digitalWriteFast(PIN_STATUS_LED, HIGH);

              transitionToWaiting();
              packetnum = 0;
              break;
            }
            else
            {
              imu.available(&fifo_level, &wm_flag);
            }
          }

 digitalWriteFast(PIN_STATUS_LED, LOW);
        }
        
        //If we're sending and PIN_IMU_INT1 PIN_PB2 is LOW, go to sleep?
        if (mode == MODE_SENDING && digitalReadFast(PIN_IMU_INT1) != 1) sleep_cpu();
      }
      break;
    
    default:
      break;
  }
}

void transitionToOff() {
  mode = MODE_OFF;

  //Send an 'I'm turning off' message.'
  //TODO radio.write(imu_data, 27);

  radio.powerDown();
  imu.end();
}

void transitionToWaiting() {
  mode = MODE_WAITING;
  radio.powerUp();

  while (imu.begin())
    ;

  digitalWriteFast(PIN_STATUS_LED, LOW);

  imu.setWUDuration(2); //2 what?
  imu.setWUThreshold(WU_THRESHOLD);

  imu.setFIFOwatermark(WM_LEVEL);
  imu.disableWMInterrupt();
  imu.enableWUInterrupt();

  imu.enableInactivityInterrupt();
  // Turn everything on, only WU interrupt
}

void transitionToSending() {
  
  mode = MODE_SENDING;
  imu.disableWUInterrupt();
  imu.enableWMInterrupt();
  // Everything is already on, swap WU for WM interrupt
  
  //digitalWriteFast(PIN_STATUS_LED, HIGH);
}
