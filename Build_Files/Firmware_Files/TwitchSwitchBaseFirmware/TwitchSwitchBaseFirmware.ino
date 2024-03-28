/*
                   TWITCH SWITCH (BASE UNIT) Mk-IV

            Stan Cassidy Center for Rehabilitation
                      Feb 23, 2023

            Arduino Platform: 
              Base: Adafruit Feather nRF52840 Express

            Firmware version number#: 20231220 v1.1

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

  Visual of the board for referance

       |
  -O  [‾]      KNOB 3, SWITCH [2], SENSOR 1
  -O  [ ]      KNOB 2, SWITCH [1], SENSOR 2
  -O  [_]      KNOB 1, SWITCH [0], SENSOR 3

  LED messages:
    - Need to decide on 'charging indicators'
*/

#include <SPI.h>
#include <RF24.h>
#include <nRF24L01.h>
#include <Adafruit_NeoPixel.h>

#include <Adafruit_TinyUSB.h>

#include <bluefruit.h>

// Set up file system for storage
#include <Adafruit_LittleFS.h>  //https://github.com/adafruit/Adafruit_nRF52_Arduino/tree/master/libraries/Adafruit_LittleFS
#include <InternalFileSystem.h>
using namespace Adafruit_LittleFS_Namespace;
#define FILENAME "/twitchswitch.txt"
File file(InternalFS);

#include <SwRotaryEncoder.h>

#include <AceButton.h>  // See https://github.com/bxparks/AceButton#Features
using namespace ace_button;

#include "pins.h"

BLEDis bledis;          //Bluetooth Device Information Service  https://learn.adafruit.com/introducing-the-adafruit-nrf52840-feather/bledis
BLEHidAdafruit blehid;  //BLEHidAdafruit allows you to simulate a mouse or keyboard using the HID (Human Interface Device) profile that is part of the Bluetooth Low Energy standard.

SwRotaryEncoder knobs[3];

RF24 radio(PIN_RADIO_CE, PIN_RADIO_CS, 10000000UL);

Adafruit_NeoPixel pixel(1, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

//Radio pipe Config
uint8_t switch1_pipe[5] = { 0x97, 0x97, 0xb0, 0xe7, 0x47 };
uint8_t switch2_pipe[5] = { 0x96, 0x97, 0xb0, 0xe7, 0x47 };
uint8_t switch3_pipe[5] = { 0x90, 0x97, 0xb0, 0xe7, 0x47 };

uint8_t readbuf[32];

#define PULSE_LEN 250     // Output pulse duration in milliseconds

#define BAUD_RATE 115200  // USB communications baud rate
#define VERSION_TWITCHSWITCH "1.2" //1.2 added battery improvements and keyboard mode

// Constants for OPERATION MODES
const uint8_t MODE_MOUSE = 0; //GREEN
const uint8_t MODE_TABLET = 1; //BLUE
const uint8_t MODE_KEYBOARD = 2; //YELLOW
const uint8_t MODE_AIRMOUSE = 3; //ORANGE?


// Constants for HOW THE MOUSE POINTER IS MOVING
const uint8_t MOUSE_STOPPED = 0;
const uint8_t MOUSE_MOVING_UP = 1;
const uint8_t MOUSE_MOVING_DOWN = 2;
const uint8_t MOUSE_MOVING_RIGHT = 3;
const uint8_t MOUSE_MOVING_LEFT = 4;

// Constants for WHICH SENSOR WAS TRIGGERED (same as the numbers on the sensor case)
const uint8_t SENSOR_CLICK = 1;
const uint8_t SENSOR_LEFTRIGHT = 2;
const uint8_t SENSOR_UPDOWN = 3;

uint8_t currentMouseMode = MOUSE_STOPPED;

// Accelleration settings
uint32_t mouseMoveStartTime = 0;  //When the mouse STARTED moving (so we can try some accelleration of some kind)
uint8_t mouseBaseSpeed = 1;
uint8_t maxAccelleration = 4; // This will cause the mouse speed to start at 1, then change to 2, then change to 4 each time it starts moving

uint32_t lastWatermarkupdate = 0;  //When the last time a rotary encoder was changed

//************ BUTTON CONFIG ************
// Create 2 ButtonConfigs. The System ButtonConfig is not used.
ButtonConfig modeConfig;
ButtonConfig profileConfig;
ButtonConfig knobButtonsConfig;

AceButton modeButton(&modeConfig);          //Side button
AceButton profileButton(&profileConfig);    //Side button
AceButton knobButton1(&knobButtonsConfig);  //Top three encoder buttons
AceButton knobButton2(&knobButtonsConfig);  //Top three encoder buttons
AceButton knobButton3(&knobButtonsConfig);  //Top three encoder buttons

// Forward reference to prevent Arduino compiler becoming confused.
void handleModeButtonEvent(AceButton*, uint8_t, uint8_t);
void handleProfileButtonEvent(AceButton*, uint8_t, uint8_t);
void handleKnobButtonsEvent(AceButton*, uint8_t, uint8_t);
//************ END BUTTON CONFIG ************


// State Tracking
uint8_t mouseLastLR = 2;  // Last known direction of mouse travel on X axis; 1 = right, 2 = left
uint8_t mouseLastUD = 2;  // Last known direction of mouse travel on Y axis; 1 = up, 2 = down

uint8_t autoCalibrationSensor = 0;  //If this is NON 0, we're in autocalibration mode for a sensor (SENSOR_CLICK, SENSOR_LEFTRIGHT, SENSOR_UPDOWN)

//************ Structures to hold data ************

// One reading from the IMU. Could be gyro or accel
typedef struct {
  int16_t x;
  int16_t y;
  int16_t z;
} reading_t;

// Holds one gyro and one accelerometer reading
typedef struct {
  reading_t gyro;
  reading_t accel;
} sample_t;

// The full payload coming from a switch.
typedef struct {
  sample_t samples[2];
  uint16_t fifo_level;
  //uint8_t myARC;
  uint8_t packetnum;
  uint16_t vbatReading;
} payload_t;

typedef struct {
  char ack_type;
} ack_t;

//Example:
//pnum 2:198,fl:288,XLx:-1232,XLy:262,XLz:3619,GYx:-31835,GYy:4000,GYz:1222

//Structure of the configuration of the switch. This is what will be persisted to the memory and reloaded on boot.
typedef struct {
  uint8_t modeId;

  uint16_t sensorClickWatermark;
  uint16_t sensorLeftRightWatermark;
  uint16_t sensorUpDownWatermark;
} ProfileSettings;

typedef struct {
  uint8_t currentProfileId;
  ProfileSettings profiles[3];  //There are 3 profiles

} TwitchSettings;
//************ END Structures to hold data ************
TwitchSettings defaultTwitchSettings;
TwitchSettings twitchSettings;  // populated later via readSettings()

//Config Mouse for TinyUSB
// HID report descriptor using TinyUSB's template - Single Report (no ID) descriptor
uint8_t const desc_hid_report[] = {
  TUD_HID_REPORT_DESC_KEYBOARD( HID_REPORT_ID(1) ),
  TUD_HID_REPORT_DESC_MOUSE( HID_REPORT_ID(2) )
};
// USB HID object. For ESP32 these values cannot be changed after this declaration
// desc report, desc len, protocol, interval, use out endpoint
//Adafruit_USBD_HID usb_hid(desc_hid_report, sizeof(desc_hid_report), HID_ITF_PROTOCOL_MOUSE, 2, false);
Adafruit_USBD_HID usb_hid;

void encoder_callback(int step) {
  //Serial.printf("Encoders: %i, %i, %i\n", knobs[0].readAbs(), knobs[1].readAbs(), knobs[2].readAbs());
  //Serial.printf("Encoders: %i, %i, %i\n", knobs[0].read(), knobs[1].read(), knobs[2].read());

  /***
  Visual of the board for referance
           |
  -O  [-]  KNOB 3, ENCODER [2], SENSOR 1 (CLICK) (visually, on the board)
  -O  [ ]  KNOB 2, ENCODER [1], SENSOR 2 (L/R)
  -O  [_]  KNOB 1, ENCODER [0], SENSOR 3 (U/D)
  ***/

  uint16_t _tuningMultiplier = 30;  // When you turn the dial, we'll multiply the read value by this so that tuning is faster.

  //Update watermarks. This is 'manual tuning'
  twitchSettings.profiles[twitchSettings.currentProfileId].sensorClickWatermark += ((uint16_t)knobs[2].read() * _tuningMultiplier);
  twitchSettings.profiles[twitchSettings.currentProfileId].sensorLeftRightWatermark += ((uint16_t)knobs[1].read() * _tuningMultiplier);
  twitchSettings.profiles[twitchSettings.currentProfileId].sensorUpDownWatermark += ((uint16_t)knobs[0].read() * _tuningMultiplier);

  // we need to check a limit here. The Watermark limits are unsigned, so they shouldn't ever go 'below' zero or they'll wrap around.
  if (twitchSettings.profiles[twitchSettings.currentProfileId].sensorClickWatermark < 500)
    twitchSettings.profiles[twitchSettings.currentProfileId].sensorClickWatermark = 500;

  if (twitchSettings.profiles[twitchSettings.currentProfileId].sensorUpDownWatermark < 500)
    twitchSettings.profiles[twitchSettings.currentProfileId].sensorUpDownWatermark = 500;

  if (twitchSettings.profiles[twitchSettings.currentProfileId].sensorLeftRightWatermark < 500)
    twitchSettings.profiles[twitchSettings.currentProfileId].sensorLeftRightWatermark = 500;

  Serial.printf("  Tuning Profile %i: C %u, LR %u, UD %u\n", twitchSettings.currentProfileId, twitchSettings.profiles[twitchSettings.currentProfileId].sensorClickWatermark, twitchSettings.profiles[twitchSettings.currentProfileId].sensorLeftRightWatermark, twitchSettings.profiles[twitchSettings.currentProfileId].sensorUpDownWatermark);

  //We need to persist the tunings, BUT! We don't want to do it what every turn on an encoder.
  // This would lead to too many writes to FLASH as wear it out prematurely.
  // Instead, we'll set a timer when the last change happens and check it to see if a few seconds have passed until the last change, THEN save the setting
  lastWatermarkupdate = millis();
}

void setup() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Look at PR #551. These two lines satisfy dwt_enabled();
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  pinMode(PIN_RADIO_CS, OUTPUT);
  SPI.begin();

  digitalWrite(PIN_LED_CH1, LOW);
  digitalWrite(PIN_LED_CH2, LOW);
  digitalWrite(PIN_LED_CH3, LOW);
  digitalWrite(PIN_BUDDY_DISABLE, HIGH);  //This pin can be used to turn off the switch relays. 
                                          //They're connected to the same pins as the RED LEDs. If you want to light and LED, but NOT click, you need to turn this ON

  pinMode(PIN_LED_CH1, OUTPUT);
  pinMode(PIN_LED_CH2, OUTPUT);
  pinMode(PIN_LED_CH3, OUTPUT);
  pinMode(PIN_BUDDY_DISABLE, OUTPUT);

  pinMode(PIN_KNOB1_SW, INPUT_PULLUP);
  pinMode(PIN_KNOB2_SW, INPUT_PULLUP);
  pinMode(PIN_KNOB3_SW, INPUT_PULLUP);
  knobButton1.init(PIN_KNOB1_SW);
  knobButton2.init(PIN_KNOB2_SW);
  knobButton3.init(PIN_KNOB3_SW);
  knobButtonsConfig.setEventHandler(handleKnobButtonsEvent);
  knobButtonsConfig.setFeature(ButtonConfig::kFeatureClick);
  knobButtonsConfig.setFeature(ButtonConfig::kFeatureLongPress);

  pinMode(PIN_RADIO_IRQ, INPUT_PULLUP);

  pinMode(PIN_MODE, INPUT_PULLUP);
  modeButton.init(PIN_MODE);
  modeConfig.setEventHandler(handleModeButtonEvent);
  modeConfig.setFeature(ButtonConfig::kFeatureClick);
  modeConfig.setClickDelay(250);  // increase click delay from default 200 ms

  pinMode(PIN_PROFILE, INPUT_PULLUP);
  profileButton.init(PIN_PROFILE);
  profileConfig.setEventHandler(handleProfileButtonEvent);
  profileConfig.setFeature(ButtonConfig::kFeatureClick);
  profileConfig.setClickDelay(250);  // increase click delay from default 200 ms

  //Set up 'Friendly' names when plugged into a computer (Instead of just 'Feather nRF52840 Express' showing up)
  TinyUSBDevice.setManufacturerDescriptor("SCCR");
  TinyUSBDevice.setProductDescriptor("Twitch Switch");

  // Turn on neo pixel
  pixel.begin();


  // Set up HID (mouse & keyboard)
  usb_hid.setPollInterval(2);
  usb_hid.setReportDescriptor(desc_hid_report, sizeof(desc_hid_report));
  usb_hid.begin();

  defaultTwitchSettings.currentProfileId = 0;
  defaultTwitchSettings.profiles[0] = { (uint8_t)0, (uint16_t)10011, (uint16_t)10012, (uint16_t)10013 };
  defaultTwitchSettings.profiles[1] = { (uint8_t)0, (uint16_t)10021, (uint16_t)10022, (uint16_t)10023 };
  defaultTwitchSettings.profiles[2] = { (uint8_t)0, (uint16_t)10031, (uint16_t)10032, (uint16_t)10033 };

  twitchSettings = defaultTwitchSettings;

  //Wait for USB to time out if not connected
  Serial.begin(BAUD_RATE);
  unsigned long _start = millis();
  boolean _serialConnected = true;
  while (!Serial)  //Wait until the serial port is opened with a 5 second timeout
  {
    //Make the LED 'breath'
    for (int i = 0; i < 255; i += 2) {

      pixel.setPixelColor(0, pixel.Color(i, 0, i));  //Turn on VIOLET LED
      pixel.show();                                  //Show the color
      delay(3);
    }
    for (int i = 255; i > 0; i -= 2) {
      pixel.setPixelColor(0, pixel.Color(i, 0, i));  //Turn on VIOLET LED
      pixel.show();                                  //Show the color
      delay(3);
    }

    if (millis() - _start > 5000) {
      //NO Serial connected
      _serialConnected = false;
      break;
    }
  }

  // Print info so that when a switch is plugged in, we can see the firmware version / date it was last programmed
  Serial.println(F("START " __FILE__ " from " __DATE__ "\r\nUsing firmware version " VERSION_TWITCHSWITCH));

  // Initialize Internal File System
  if (!InternalFS.begin())
    Serial.printf("Error mounting file system. Starting with default settings");
  else
  {
    // Check to see if the user is trying to reset the switch to factory defaults.
    bool resetToFactory = modeButton.isPressedRaw() && profileButton.isPressedRaw();
  
    if (resetToFactory)
    {
      Serial.println(F("Resetting to factory defaults"));

      //Delete the stored settings file so that it can be reset to the hard coded defaults
      InternalFS.remove(FILENAME);

      //Blink LED RED to show reset
      for (int i = 0; i<3; i++)
      {
        pixel.setPixelColor(0, pixel.Color(128, 0, 0));  //Turn on RED
        pixel.show();                                  //Show the color
        delay(500);
        pixel.setPixelColor(0, pixel.Color(0, 0, 0));  //OFF
        pixel.show();                                  //Show the color
        delay(500);
      }
    }

    twitchSettings = readSettings();
  }
  
  setProfile(twitchSettings.currentProfileId);
  setMode(twitchSettings.profiles[twitchSettings.currentProfileId].modeId);  //Mode is part of the profile now

  pixel.show();

  if (_serialConnected) {
    while (!TinyUSBDevice.mounted()) delay(1);
  }

  // Configure the rotary encoders
  knobs[0].begin(PIN_KNOB1_A, PIN_KNOB1_B);
  knobs[1].begin(PIN_KNOB2_A, PIN_KNOB2_B);
  knobs[2].begin(PIN_KNOB3_A, PIN_KNOB3_B);
  for (int i = 0; i < 3; i++) knobs[i].setCallback(encoder_callback);

  // Configure radio
  while (!radio.begin())
    ;

//https://forum.arduino.cc/t/nrf24l01-how-does-writeackpayload-really-work/355140/9

  radio.setDataRate(RF24_2MBPS);
  radio.setPALevel(0);
  radio.setChannel(57);  // 2.457 GHz band
  radio.enableDynamicPayloads();
  radio.enableAckPayload();
  radio.openReadingPipe(1, switch1_pipe);
  radio.openReadingPipe(2, switch2_pipe);
  radio.openReadingPipe(3, switch3_pipe);
  radio.startListening();

  // Configure bluetooth connection
  Bluefruit.begin();
  Bluefruit.setName("Twitch Switch");
  Bluefruit.Periph.setConnInterval(9, 16);  // min = 9*1.25==11.25ms, max = 16*1.25=20ms
  Bluefruit.setTxPower(4);                  // Check bluefruit.h for supported values

  bledis.setManufacturer("SCCR");
  bledis.setModel("Twitch Switch");
  bledis.begin();

  blehid.begin();

  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_MOUSE);
  Bluefruit.Advertising.addService(blehid);
  Bluefruit.Advertising.addName();

  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);  // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);    // number of seconds in fast mode
  Bluefruit.Advertising.start(0);              // 0 = Don't stop advertising after n seconds
}


int8_t mouseMoveX = 0;
int8_t mouseMoveY = 0;

boolean _validMoveDetected = false;
uint16_t _lastvalueDetectionTime = 0;

void loop() {
  modeButton.check();
  profileButton.check();
  knobButton1.check();
  knobButton2.check();
  knobButton3.check();

  static uint8_t temp = 0;

  boolean clickSensorTriggered = false;      //!digitalRead(PIN_KNOB3_SW);  This is SENSOR 1
  boolean upDownSensorTriggered = false;     //!digitalRead(PIN_KNOB2_SW);   This is SENSOR 2
  boolean leftRightSensorTriggered = false;  //!digitalRead(PIN_KNOB1_SW);   This is SENSOR 3

  uint16_t _peakReading = 0;

  uint8_t pipenum;  // Pipenum will contain the pipe (switch) that has sent data to us (the base)

  while (radio.available(&pipenum))
  {
    uint8_t payloadSize = radio.getDynamicPayloadSize();
    if (payloadSize > 0) {
      radio.read(readbuf, payloadSize);

      // See struct def for info around payload data scructure
      payload_t* payload = (payload_t*)readbuf;

      ack_t ack_payload;
      ack_payload.ack_type = 'S';

      //if (payload->packetnum == 100)
      //{
      //if (payload->packetnum == 200) {
        //uint8_t ret = radio.writeAckPayload(pipenum, &ack_payload, sizeof(ack_t) / sizeof(uint8_t));
        //if (ret) {
          //Successfull wrote ack. Set LED to PURPLE
          //pixel.setPixelColor(0, pixel.Color(200, 0, 200)); //Commenting out. if we're in TABLET MODE, the light should be blue. Setting this makes it look like it's resetting to MOUSE mode 
        //} else {
          //Ack failed. Set LED to RED
          //pixel.setPixelColor(0, pixel.Color(200, 0, 0));
        //}

        //pixel.show();
     // }


      //Check if enough time has passed to check for a new 'twitch'
      uint16_t elapsedtime = millis() - _lastvalueDetectionTime;

      // if we already detected a valid 'move' on the piece of data, we don't need any more data now.
      //   We still need to read all the data (to get it off of the buffers) but we dont need it to trigger anything else for now!
      if (_validMoveDetected || (elapsedtime < 1000)) {
        //Serial.printf("We already detected a move or ... Not enough time passed. Throw out readings an no new move!\n");
        continue;
      }

      for (unsigned int i = 0; i < 2; i++) {
        //Example payload
        //pnum 2:198,fl:288,XLx:-1232,XLy:262,XLz:3619,GYx:-31835,GYy:4000,GYz:1222

        //Serial.printf("Accel %04hx:%04hx:%04hx",sample[i].accel.x, sample[i].accel.y, sample[i].accel.z);
        //Serial.printf("  ");
        //Serial.printf("Gyro %04hx:%04hx:%04hx",sample[i].gyro.x, sample[i].gyro.y, sample[i].gyro.z);
        //Serial.printf("\n");
        // if (micros() - last_micros < 10000) {
        //   Serial.print("x:"); Serial.print(micros()-last_micros); Serial.print(",");
        // } else {
        //  Serial.print("x:"); Serial.print(0); Serial.print(",");
        //}

        //Normalize
        uint16_t sensorNormalizedValue = sqrt(sq(payload->samples[i].gyro.x) + sq(payload->samples[i].gyro.y) + sq(payload->samples[i].gyro.z));  // Compute compound received value for this reading
        if (sensorNormalizedValue > _peakReading)
          _peakReading = sensorNormalizedValue;

        //Serial.printf("S: %i", pipenum);
        //Serial.print(", ");

        uint16_t _watermark = getWatermark(pipenum);
        if (sensorNormalizedValue > _watermark) 
        {
Serial.printf("sensorNormalizedValue > _watermark?: %ld > %ld \n", sensorNormalizedValue, _watermark);
          _validMoveDetected = true;
          _lastvalueDetectionTime = millis();

          if (pipenum == SENSOR_CLICK)
          {
            Serial.printf("Sensor 1 (A0) Triggered!\n");
            clickSensorTriggered = true;
          }
          else if (pipenum == SENSOR_LEFTRIGHT) {
            Serial.printf("Sensor 2 (A1) Triggered!\n");
            leftRightSensorTriggered = true;
          } 
          else if (pipenum == SENSOR_UPDOWN) {
            Serial.printf("Sensor 3 (A2) Triggered!\n");
            upDownSensorTriggered = true;
          }

          i = 100;  //break; // Break out of FOR loop
        }

        uint16_t vBat = payload->vbatReading;
        // Might need some sanity checking here in case the vbat wasn't sent (from older sensors?)
        
        //uint8_t voltage =
        //https://www.schindlerelectronics.com/voltage-sensor 

        boolean _printData = false;
        if (_printData) {
          Serial.printf("i: %i", i);
          Serial.print(", ");
          Serial.printf("pnum %i:", pipenum);
          Serial.print(payload->packetnum);
          Serial.print(", ");
          Serial.print("vbat:");
          Serial.print(vBat);
          Serial.print(", ");
          Serial.print("fl:");
          Serial.print(payload->fifo_level);
          Serial.print(",  ");
          Serial.print("XLx:");
          Serial.print(payload->samples[i].accel.x);
          Serial.print(", ");
          Serial.print("XLy:");
          Serial.print(payload->samples[i].accel.y);
          Serial.print(", ");
          Serial.print("XLz:");
          Serial.print(payload->samples[i].accel.z);
          Serial.print(", ");
          Serial.print("GYx:");
          Serial.print(payload->samples[i].gyro.x);
          Serial.print(", ");
          Serial.print("GYy:");
          Serial.print(payload->samples[i].gyro.y);
          Serial.print(", ");
          Serial.print("GYz:");
          Serial.print(payload->samples[i].gyro.z);
          Serial.println("");
        }
      }
    }
  } // END WHILE

  if (autoCalibrationSensor != 0) {
    //We need to auto calibrate!
    Serial.println("AutoCalibrate RUNNING!!");
    Serial.println("AutoCalibrate RUNNING!!");
    Serial.println("AutoCalibrate RUNNING!!");
    Serial.println("AutoCalibrate COMPLETE!!");

    autoCalibrationSensor = 0;
  }

  //Only 'click' if the mouse is stopped

  if (clickSensorTriggered) // Sensor 1 - CLICK 
  {
    if (currentMouseMode == MOUSE_STOPPED)
    {
      
      if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_KEYBOARD)
      {
        //Keyboard MODE
        Serial.println("Keyboard Mode: Sending F1 keypress");
        sendKeyPress(HID_KEY_F1);
      }
      else if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_MOUSE) {
        Serial.println("Mouse Mode: Sending Mouse Click");
        //TurnOnLED()
        mouseClick();
        //TurnOffLED();
      }
      else if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_TABLET) {
        Serial.println("Tablet Mode: Sending Mouse Click");
        //TurnOnLED()
        mouseClick();
        //TurnOffLED();
      }

      sendButtonClick(PIN_LED_CH3);
    }

    //Reset Mouse Movements
    mouseMoveX = 0;
    mouseMoveY = 0;
    currentMouseMode = MOUSE_STOPPED;
    
    _validMoveDetected = false;
  } 
  else if (upDownSensorTriggered)  // Sensor 3 - UP DOWN
  {
    // Mouse: UP - STOP - DOWN
    // Tablet: SCROLL DOWN
    
    //Reset horiztonal mouse movement
    mouseMoveX = 0;
    
    if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_KEYBOARD)
    {
      //Keyboard MODE
      Serial.println("Keyboard Mode: Sending F3 keypress");
      sendKeyPress(HID_KEY_F3);
    }
    else if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_TABLET)
    {
      //Tablet MODE
      Serial.println("Scrolling mouse DOWN");
      mouseScrollDown();
    }
    else
    {
      // If the mouse WAS moving, stop it.
      if ((currentMouseMode == MOUSE_MOVING_UP) || (currentMouseMode == MOUSE_MOVING_DOWN) || (currentMouseMode == MOUSE_MOVING_RIGHT) || (currentMouseMode == MOUSE_MOVING_LEFT)) {
        currentMouseMode = MOUSE_STOPPED;
      }
      // If the mouse WAS stopped, Move the opposite direction from the last time we 'twitched'
      else if (currentMouseMode == MOUSE_STOPPED) {

        //Grab the last direction we were moving, so we can unvert it later
        currentMouseMode = mouseLastUD;

        mouseMoveStartTime = millis();

        // Toggle up / down for nex time!
        if (mouseLastUD == MOUSE_MOVING_UP) {
          Serial.printf("Setting mouseLastUD to MOUSE_MOVING_DOWN \n");
          // Last move was up, so move down next time
          mouseLastUD = MOUSE_MOVING_DOWN;  // Record motion direction
        } else {
          Serial.printf("Setting mouseLastUD to MOUSE_MOVING_UP \n");

          // Last move was down, so move up next time
          mouseLastUD = MOUSE_MOVING_UP;  // Record motion direction
        }
      }
      //Serial.printf("2 currentMouseMode: %d \n", currentMouseMode);

      if (currentMouseMode != MOUSE_STOPPED) {
        int8_t mouseTicksToMove = mouseBaseSpeed;
        if (currentMouseMode == MOUSE_MOVING_UP)
          mouseTicksToMove = -mouseBaseSpeed;

        Serial.printf("UP / DOWN mouseTicksToMove: %d \n", mouseTicksToMove);
        mouseMoveY = mouseTicksToMove;
      }

    //Reset Mouse Movements
      Serial.printf("mouseLastUD: %d \n", mouseLastUD);
      Serial.println("");
    }

    sendButtonClick(PIN_LED_CH1);

    _validMoveDetected = false;
  } 
  else if (leftRightSensorTriggered)  // Sensor 2 - LEFT / RIGHT
  {
    // MOUSE MODE = RIGHT - STOP - LEFT
    // TABLET MODE = SCROLL UP

    //Reset vertical movement
    mouseMoveY = 0;

    // If the mouse WAS moving, stop it.
    //Serial.printf("3 currentMouseMode: %d \n", currentMouseMode);

    if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_KEYBOARD)
    {
      //Keyboard MODE
      Serial.println("Keyboard Mode: Sending F2 keypress");
      sendKeyPress(HID_KEY_F2);
    }
    else if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_TABLET)
    {
      //Tablet MODE
      Serial.println("Scrolling mouse UP");
      mouseScrollUp();
    }
    else
    {
      if ((currentMouseMode == MOUSE_MOVING_UP) || (currentMouseMode == MOUSE_MOVING_DOWN) || (currentMouseMode == MOUSE_MOVING_RIGHT) || (currentMouseMode == MOUSE_MOVING_LEFT)) {
        Serial.printf("Setting mouseLastLR to MOUSE_STOPPED \n");
        currentMouseMode = MOUSE_STOPPED;
      }
      // If the mouse WAS stopped, Move the opposite direction from the last time we 'twitched'
      else if (currentMouseMode == MOUSE_STOPPED) {

        //Grab the last direction we were moving, so we can unvert it later
        currentMouseMode = mouseLastLR;

        mouseMoveStartTime = millis();

        // Toggle up / down for nex time!
        if (mouseLastLR == MOUSE_MOVING_LEFT) {
          Serial.printf("Setting mouseLastLR to MOUSE_MOVING_RIGHT \n");
          // Last move was left, so move right next time
          mouseLastLR = MOUSE_MOVING_RIGHT;  // Record motion direction
        } else {
          Serial.printf("Setting mouseLastLR to MOUSE_MOVING_LEFT \n");

          // Last move was down, so move up next time
          mouseLastLR = MOUSE_MOVING_LEFT;  // Record motion direction
        }
      }
      //Serial.printf("4 currentMouseMode: %d \n", currentMouseMode);

      if (currentMouseMode != MOUSE_STOPPED) {
        int8_t mouseTicksToMove = mouseBaseSpeed;

        if (currentMouseMode == MOUSE_MOVING_LEFT)
          mouseTicksToMove = -mouseBaseSpeed;

        mouseMoveX = mouseTicksToMove;
      }
    }

    //Send a button click regardless of the mode
    sendButtonClick(PIN_LED_CH2);

    _validMoveDetected = false;  // We processed this move, set the value back so we can detect again in the next loop
  } // END Valid MOVEMENT CHECKS

  if (currentMouseMode == MOUSE_STOPPED) {
    mouseMoveX = 0;
    mouseMoveY = 0;

    mouseMoveStartTime = 0;
  }
  else
  {
    //Start accellerating after 2 seconds of movement
    if ((millis() - mouseMoveStartTime) > 2000)
    {
      //Serial.print("Accellerating Mouse \n");

      mouseMoveX = mouseMoveX * 2;
      mouseMoveY = mouseMoveY * 2;

      //Cap cursor speed
      if (mouseMoveX < -maxAccelleration)
        mouseMoveX = -maxAccelleration;
      if (mouseMoveX > maxAccelleration)
        mouseMoveX = maxAccelleration;

      if (mouseMoveY < -maxAccelleration)
        mouseMoveY = -maxAccelleration;
      if (mouseMoveY > maxAccelleration)
        mouseMoveY = maxAccelleration;

      mouseMoveStartTime = millis();
    }
  }

  //Only move the mouse every few loops to slow it down and make it easier to hit a target
  // AND ONLY if we're in MOUSE MODE
  if (--temp == 0) {
    temp = 3;

    if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId == MODE_MOUSE)
    {
      //Actually move the mouse
      mouseMove(mouseMoveX, mouseMoveY);
    }
  }

  if (lastWatermarkupdate > 0) {
    //Check if we need to persist the tuning watermarks. That is, 3+ seconds have passed since the last tuning.
    if ((millis() - lastWatermarkupdate) > 5000) {
      Serial.printf("5s ellapsed. Saving watermark changes \n");
      writeSettings(twitchSettings);

      lastWatermarkupdate = 0;  //reset the watermark to start tracking the 5 seconds again.
    }
  }

  // This was test code to set the LED to a color based on the knob 1 dail.
  //pixel.setPixelColor(0, pixel.ColorHSV(knobs[1].readAbs()*182, 255, 25));
  //pixel.show();

  /*
  if (!digitalRead(PIN_KNOB1_SW))
  {
    //Serial.println("Click Mouse!");
    //mouseClick();
    setProfile(2);
    writeSettings(twitchSettings);
  }
  else if (!digitalRead(PIN_KNOB2_SW))
  {
    setProfile(1);
    writeSettings(twitchSettings);
  }
  else if (!digitalRead(PIN_KNOB3_SW))
  {
    setProfile(0);
    writeSettings(twitchSettings);
  }
  */
}


/**
 * Moves the mouse by the specified x / y increment. It takes care of the move for both USB and bluetooth
 * 
 * @param x the amount to move the mouse on the X (horizontal) plane. Left<0  Right>0
 * @param y the amount to move the mouse on the Y (vertical) plane. Up<0  Down>0
 */
void mouseMove(int8_t x, int8_t y) {

  if ((abs(x) > 0) || (abs(y) > 0))
  {
    //Serial.printf("Move Mouse: %d, %d \n", x, y);
  }
  else
  {
    //No movement to send
    return;
  }
  if (blehid.mouseMove(x, y)) {
    //Move
  } else {
    //https://forum.arduino.cc/t/serial-print-f-hello-world/411623
    //Serial.println(F("mouseMove failed. Please make sure device is paired and try again\n"));
  }

  if (usb_hid.ready()) {
    usb_hid.mouseMove(2, x, y);
    delay(10);  // Unknown if this is needed
  }
}

void mouseClick() {
  //Serial.printf("Mouse Click\n");
  mousePress();
  delay(10);
  mouseRelease();
  delay(PULSE_LEN);
}

void mousePress() {
  if (blehid.mouseButtonPress(MOUSE_BUTTON_LEFT)) {
    // Press (and hold) the Left mouse's button
    //Serial.printf("  Mouse Press - Bluetooth\n");
  } else {
    //Serial.println(F("Please make sure device is paired and try again\n"));
  }

  if (usb_hid.ready()) {
    usb_hid.mouseButtonPress(2, 1);
    //Serial.printf("  Mouse Press - Serial\n");
  }
}

void mouseRelease() {
  if (blehid.mouseButtonRelease()) {
    // Release all buttons
    //Serial.printf("  Mouse Release - Bluetooth\n");
  } else {
    //Serial.println(F("Please make sure device is paired and try again\n"));
  }

  if (usb_hid.ready()) {
    usb_hid.mouseButtonRelease(2);
    //Serial.printf("  Mouse Release - Serial\n");
  }
}

void mouseScrollUp()
{
  int8_t _scrollAmount = 5;

  if (blehid.mouseScroll(_scrollAmount))
  {
    //Serial.printf("  Mouse Scroll Up - Bluetooth\n");
  }
  else if (usb_hid.ready())
  {
    usb_hid.mouseScroll(2, _scrollAmount, 0);
  }
  
  delay(10);  // Unknown if this is needed
}

void mouseScrollDown()
{
  int8_t _scrollAmount = -5;

  if (blehid.mouseScroll(_scrollAmount))
  {
    //Serial.printf("  Mouse Scroll Up - Bluetooth\n");
  }  
  else if (usb_hid.ready())
  {
    usb_hid.mouseScroll(2, _scrollAmount, 0);
  }
  
  delay(10);  // Unknown if this is needed
}

void sendKeyPress(uint8_t keycode)
{
  //blehid.keyPress() takes a char and hid_ascii_to_keycode conversion on it. We have a keycode here, not a CHAR, so this wont work for us.
  // See: https://github.com/adafruit/Adafruit_nRF52_Arduino/blob/master/libraries/Bluefruit52Lib/src/services/BLEHidAdafruit.cpp#L126

  // Solution: https://arduino.stackexchange.com/questions/65513/how-do-i-send-non-ascii-keys-over-the-ble-hid-connection-using-an-adafruit-nrf52
  uint8_t keycodes[6] = { keycode, HID_KEY_NONE , HID_KEY_NONE , HID_KEY_NONE , HID_KEY_NONE , HID_KEY_NONE };    
  if (blehid.keyboardReport(0, keycodes))
  {
     delay(10);
     blehid.keyRelease();
  }
  else if (usb_hid.ready())
  {
    uint8_t keycodearray[6] = { 0 };
    keycodearray[0] = keycode;

    //Press then release
    usb_hid.keyboardReport(1, 0, keycodearray);
    delay(10);
    usb_hid.keyboardRelease(1);
  }

  delay(10);  // Unknown if this is needed
}

bool processSensorData(sample_t reading) {
  Serial.printf("reading X: %d \n", reading.gyro.x);
  if (reading.gyro.x > 1000) {
    return true;
  }

  return false;
}

// Returns the proper watermark value for the given sensor and profile (Click, LR, UP)
// Note: This value will still be limited by the sensor firmware. They're only set to 'wake up' and stream data when they're moved enough!
uint16_t getWatermark(uint8_t pipenum) {
  if (pipenum == SENSOR_CLICK)
    return twitchSettings.profiles[twitchSettings.currentProfileId].sensorClickWatermark;
  else if (pipenum == SENSOR_LEFTRIGHT)
    return twitchSettings.profiles[twitchSettings.currentProfileId].sensorLeftRightWatermark;
  else if (pipenum == SENSOR_UPDOWN)
    return twitchSettings.profiles[twitchSettings.currentProfileId].sensorUpDownWatermark;

  Serial.printf("Unknown pipe number %d. Returning 3000.\n", pipenum);
  return 3000;
}



/**
 * Persist the setting. 
 * 
 * @param settings TwitchSettings which you want to persist
 * @returns {bool} true if the file was properly written
 */
bool writeSettings(TwitchSettings _twitchSettings) {
  //Need to delete the old file first?
  InternalFS.remove(FILENAME);

  if (file.open(FILENAME, FILE_O_WRITE)) {
    //Serial.println("Writting these settings:");
    displayTwitchSettings(_twitchSettings);

    size_t writeSize = file.write((byte*)&_twitchSettings, sizeof(TwitchSettings));
    //Serial.printf("Wrote Settings \n");

    file.close();

    return (writeSize > 0);  // Return true if bytes were written
  } else {
    Serial.printf("Writing failed \n");
    return false;
  }
}


/**
 * Stores current settings to disk 
 * Note: Shouldn't call this TOOOO often as there are limited writes to FLASH
 * Note: https://arduino.stackexchange.com/questions/90204/how-to-store-a-struct-in-a-file
 */
TwitchSettings readSettings() {
  file.open(FILENAME, FILE_O_READ);

  TwitchSettings _twitchSettings = defaultTwitchSettings;

  if (!file)
  {
    //No valid data. Return defaults
    Serial.printf("File not found %s. Returning defaults. \n", FILENAME);
  } 
  else
  {
    // We found valid data. Read it in.
    Serial.printf("Found file %s. Valid data retrived. \n", FILENAME);

    uint32_t readlen = file.read(&_twitchSettings, sizeof(TwitchSettings));
    //Serial.printf("Read %d bytes \n", readlen);

    // What happens if we read an old, incompatible data structure?
    //  From what I've read, this can't happen - Whent the sketch is uploaded, the flash files are removed.

    file.close();
  }

  displayTwitchSettings(_twitchSettings);

  return _twitchSettings;
}

void displayTwitchSettings(TwitchSettings settings)
{
  return; // cut down on noise for now!

  Serial.printf("Twitch Settings \n");
  Serial.printf("--------------------------------------- \n");
  Serial.printf("- Default Profile: %d\n", settings.currentProfileId);

  for (uint8_t i = 0; i < 3; i++) {
    ProfileSettings _profileSettings = settings.profiles[i];

    Serial.printf("- Profile: %u\n", i);
    Serial.printf("  - Mode: %u\n", _profileSettings.modeId);
    Serial.printf("  - Click Water: %u\n", _profileSettings.sensorClickWatermark);
    Serial.printf("  - LR Water: %u\n", _profileSettings.sensorLeftRightWatermark);
    Serial.printf("  - UD Water: %u\n", _profileSettings.sensorUpDownWatermark);
  }
  Serial.printf("--------------------------------------- \n");
}

/**
 * Send a pulse out of the specifed 'buddy button' ports to 'click' the attached device 
 * 
 * @param buttonToClick Pin of the button you want to click

 */
void sendButtonClick(uint8_t buttonToClick)
{
  //Serial.println("Buddy button CLICK! ");

  //Trigger the relay on by switch
  //Looking at the schematic, it looks like we need to
  // 1: REMEMBER the state of all LEDS. 
  // 2: Turn off all LEDs off. 
  // 3: ENABLE the buddy buttons
  // 4: Turn on the correct LED, which will turn on the switch
  // 5: delay
  // 6: Turn OFF the LED
  // 7: Turn OFF the buddy buttons
  // 8: deturn the LEDs to the original state.

  //1:
  bool initialLED1State = digitalRead(PIN_LED_CH1);
  bool initialLED2State = digitalRead(PIN_LED_CH2);
  bool initialLED3State = digitalRead(PIN_LED_CH3);

  //2:
  digitalWrite(PIN_LED_CH1, LOW);
  digitalWrite(PIN_LED_CH2, LOW);
  digitalWrite(PIN_LED_CH3, LOW);
  
  //3:
  digitalWrite(PIN_BUDDY_DISABLE, LOW); //Enable Buddy Button

  //4:
  digitalWrite(buttonToClick, HIGH);
  //5:
  delay(PULSE_LEN);
  //6:
  digitalWrite(buttonToClick, LOW);

  //7:
  digitalWrite(PIN_BUDDY_DISABLE, HIGH); //DISABLE the buddy button

  //8:
  digitalWrite(PIN_LED_CH1, initialLED1State);
  digitalWrite(PIN_LED_CH2, initialLED2State);
  digitalWrite(PIN_LED_CH3, initialLED3State);
}

void flashLED() {
  uint8_t ledToFlash = 0;

  //Turn off last mode LED
  digitalWrite(PIN_LED_CH1, LOW);
  digitalWrite(PIN_LED_CH2, LOW);
  digitalWrite(PIN_LED_CH3, LOW);

  if (twitchSettings.currentProfileId == 0)
    ledToFlash = PIN_LED_CH3;
  else if (twitchSettings.currentProfileId == 1)
    ledToFlash = PIN_LED_CH2;
  else if (twitchSettings.currentProfileId == 2)
    ledToFlash = PIN_LED_CH1;

  for (byte a = 0; a < 10; a++) {  // Indicate selected profile by flashing LED
    digitalWrite(ledToFlash, HIGH);
    delay(75);
    digitalWrite(ledToFlash, LOW);
    delay(75);
  }

  //Let's leave the LED on so it's easy to tell the mode you're in
  digitalWrite(ledToFlash, HIGH);
}




//*******************
// There can be three profiles
void advanceProfile() {
  twitchSettings.currentProfileId++;

  if (twitchSettings.currentProfileId > 2) twitchSettings.currentProfileId = 0;

  setProfile(twitchSettings.currentProfileId);
  writeSettings(twitchSettings);
}

/**
 * Set the current profile.
 * Each profile has independant settings for Mode and 'click' thresholds.  
 * 
 * @param newProfile Number of the profile you want to set the Twitch Switch to. Note - there are 3 settings (0 indexed) 
 */
void setProfile(uint8_t newProfile) {
  if ((newProfile >= 0) && (newProfile <= 2)) {
    twitchSettings.currentProfileId = newProfile;
    Serial.printf("Changing to profile %d\n", newProfile);

    //set to make sure the mode is set properly for this profile
    setMode(twitchSettings.profiles[twitchSettings.currentProfileId].modeId);

    flashLED();

    displayTwitchSettings(twitchSettings);
  } else {
    Serial.printf("Invalid profile set %d. Defaulting to 0", newProfile);
    setProfile(0);
  }
}
//*******************


//*******************
// Three modes
// 0 = MODE_MOUSE
// 1 = MODE_TABLET
// 2 = MODE_KEYBOARD
void advanceMode() {
  twitchSettings.profiles[twitchSettings.currentProfileId].modeId++;
  if (twitchSettings.profiles[twitchSettings.currentProfileId].modeId > 2) twitchSettings.profiles[twitchSettings.currentProfileId].modeId = 0;

  setMode(twitchSettings.profiles[twitchSettings.currentProfileId].modeId);
  writeSettings(twitchSettings);
}

void setMode(uint8_t newMode) {
  if ((newMode == 0) || (newMode == 1) || newMode == 2) {
    twitchSettings.profiles[twitchSettings.currentProfileId].modeId = newMode;

    if (newMode == MODE_MOUSE)
      pixel.setPixelColor(0, pixel.Color(0, 128, 0));  //Turn on GREEN
    else if (newMode == MODE_TABLET)
      pixel.setPixelColor(0, pixel.Color(0, 0, 128));  //Turn on BLUE
    else if (newMode == MODE_KEYBOARD)
      pixel.setPixelColor(0, pixel.Color(128, 128, 0));  //Turn on YELLOW

    pixel.show();  //Show the color
  } else {
    Serial.printf("Invalid mode set %d. Defaulting to MODE_MOUSE", newMode);
    setMode(MODE_MOUSE);
  }
}
//*******************


void handleModeButtonEvent(AceButton* /*button*/, uint8_t eventType, uint8_t /*buttonState*/) {
  switch (eventType) {
    case AceButton::kEventPressed:
      Serial.printf("Mode Button Pressed!\n");
      advanceMode();
      break;
    case AceButton::kEventReleased:
      //Serial.printf("Mode Button Released!\n");
      break;
  }
}

void handleProfileButtonEvent(AceButton* /*button*/, uint8_t eventType, uint8_t /*buttonState*/) {
  switch (eventType) {
    case AceButton::kEventPressed:
      Serial.printf("Profile Button Pressed!\n");
      advanceProfile();

      break;
    case AceButton::kEventReleased:
      //Serial.printf("Profile Button Released!\n");
      break;
  }
}

void handleKnobButtonsEvent(AceButton* button, uint8_t eventType, uint8_t /*buttonState*/) {
  switch (eventType) {
    case AceButton::kEventPressed:

      if (button->getPin() == PIN_KNOB1_SW) {
        Serial.printf("Profile Switch 1 Button Pressed!\n");

        if (twitchSettings.currentProfileId != 2) {
          setProfile(2);
          writeSettings(twitchSettings);
        }
      } else if (button->getPin() == PIN_KNOB2_SW) {
        Serial.printf("Profile Switch 2 Button Pressed!\n");

        if (twitchSettings.currentProfileId != 1) {
          setProfile(1);
          writeSettings(twitchSettings);
        }
      } else if (button->getPin() == PIN_KNOB3_SW) {
        Serial.printf("Profile Switch 3 Button Pressed!\n");
        if (twitchSettings.currentProfileId != 0) {
          setProfile(0);
          writeSettings(twitchSettings);
        }
      }

      break;
    case AceButton::kEventLongPressed:

      if (button->getPin() == PIN_KNOB1_SW) {
        Serial.printf("Starting auto calibration for Sensor 1, profile %d \n", twitchSettings.currentProfileId);
        autoCalibrationSensor = SENSOR_CLICK;

      } else if (button->getPin() == PIN_KNOB2_SW) {
        Serial.printf("Starting auto calibration for Sensor 2, profile %d \n", twitchSettings.currentProfileId);
        autoCalibrationSensor = SENSOR_LEFTRIGHT;
      } else if (button->getPin() == PIN_KNOB3_SW) {
        Serial.printf("Starting auto calibration for Sensor 3, profile %d \n", twitchSettings.currentProfileId);
        autoCalibrationSensor = SENSOR_UPDOWN;
      }

      break;
    case AceButton::kEventReleased:
      //Serial.printf("Profile Button Released!\n");
      break;
  }
}
