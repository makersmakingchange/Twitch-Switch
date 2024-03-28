/* ==========                   AUTOCALIBRATION                         ==========

   Autocal is run at User request only. All sensors must be positioned and activated
   prior to autocalibration start. USB "mouse output" must be disconnected as well.
   Autocal is initiated by pressing and holding both
   PROFILE and MODE buttons down for three seconds. Autocal function will be indicated
   by all three channel LEDs being lit sequentially.

   Autocalibration is performed for each detected sensor individually. If no sensors are detected,
   we automatically exit the function and indicate the error by rapidly flashing the 3 channel LEDs.

   Autocalibration computes average "background" movements that are not intentional or deliberate,
   average maximum value for deliberate movements, and the time between deliberate movements.
*/


/*
void autoCalibration(void) {

  // Local variables:
  unsigned long _peak1, _peak2, _deltaPeak, _avgTime;
  unsigned long _moveStart, _moveEnd;
  unsigned long _moveDur[4] = {};
  unsigned int _peakValue;                                  // Don't think we need _peakValue
  unsigned long _sum, _sum2, _sum3, _sumDur;                // Don't think we need _sum, _sum2, or _sum3
  int _readingCount, _peakCount, _twitchNum, _sensorLoop;   // Don't think we need _readingCount
  float _reading, _lastReading, _prevReading;
  byte _button;
  unsigned long _timeOut;
  unsigned long _blink;
  bool _LEDon = false;
  bool _noPeak = true;    // Indicate no peaks detected yet
  bool oneDone = false;
  bool twoDone = false;
  bool threeDone = false;
  bool _newMove = true;

  sensor_num = 0;     // Reset sensor number before reading
  
  calMode = true;     // Indicate device is in calibration mode

  allLEDoff();                                  // Turn off all 3 channel LEDs
  digitalWrite(SSR_EN, LOW);                    // Disable the output relays

  if (activeSensors == 0) goto AbortAutocal;    // No active sensors to calibrate

  // Loop to callibrate all active sensors, checks sensors in numerical order
  for (_sensorLoop = 1; _sensorLoop < 4; _sensorLoop++){
    
    //  # # # # Starting with Sensor 1 # # # #    //
    if (_sensorLoop == 1) {
      if ((activeSensors & 0x01) == 0) {           // True only if sensor 1 was previously detected
        continue;
      }
    }

    //  # # # # Sensor 2 # # # #    //
    if (_sensorLoop == 2) {
      if ((activeSensors & 0x02) == 0) {          // True only if sensor 2 was previously detected
        continue;
      }
    }

    //  # # # # Sensor 3 # # # #    //
    if (_sensorLoop == 3){
      if ((activeSensors & 0x04) == 0) {           // True only if sensor 3 was previously detected
        continue;
      }
    }
    
    ledOn(_sensorLoop);                         // Turns on LED of sensor to be calibrated
    _timeOut = millis();                        // Start the timeOut timer

    do {                                        // Wait for MODE button press to begin
      _button = checkSwitches();                // Read the panel buttons
      if ((millis() - _timeOut) > 300000) goto AbortAutocal;    // No user input for 5 minutes, exit autocalibration
    } while (_button != 1);

    ledOff(_sensorLoop);
    _LEDon = false;
    _blink = millis();                          // Start the slow blink timer - this indicates phase 1: no motion
    _timeOut = millis();                        // Start a 10 seconds timer
    _peak1 = 0;                                 // Clear peak value

    // ----- PHASE ONE: NO MOTION ----- //
    do {
      readSensors();
       if (sensor_num == _sensorLoop) {                   // Ignore non-selected sensors
        _reading = sensorsReceived[sensor_num-1];

        if (_reading > _peak1) _peak1 = _reading;
        
      }

      if ((millis() - _blink) > 500) {          // Perform slow blink of LED corresponding to selected sensor
        if (_LEDon) {
          ledOff(_sensorLoop);
          _LEDon = false;
        } else {
          ledOn(_sensorLoop);
          _LEDon = true;
        }
        _blink = millis();
      }

      delay(100);   // Short pause between readings
    } while ((millis() - _timeOut) < 10000);    // Take consecutive readings for 10 seconds while User stays still

    minThreshold = int(_peak1 * 1.1);         // Compute baseline for selected sensor

    if (minThreshold < THRESH_MIN) {          // Ensure threshold is between defined min and max
      minThreshold = THRESH_MIN;
    } else if (minThreshold > THRESH_MAX) {
      minThreshold = THRESH_MAX;
    }

    ledOn(_sensorLoop);                         // Turn on LED corresponding to selected sensor to indicate waiting for MODE press

    do {                                        // Wait for MODE button press to begin
      _button = checkSwitches();                // Read the panel buttons
      if ((millis() - _timeOut) > 300000) goto AbortAutocal;    // No user input for 5 minutes, exit autocalibration
    } while (_button != 1);

    ledOff(_sensorLoop);                        // Turn off sensor LED
    _LEDon = false;

    _sum = 0;                                   // Clear integrator

    _blink = millis();                          // Start the 5 second blink timer
    _peakValue = 0;                             // Clear the last recorded peak value
    _noPeak = true;                             // No peaks detected yet
    _peakCount = 0;                             // Reset counter
    _sum = 0;                                   // Clear integrators
    _sum2 = 0;
    _sum3 = 0;
    _sumDur = 0;
    _lastReading = 0;
    _reading = 0;
    _prevReading = 0;
    _twitchNum = 0;
    _readingCount = 0;
    _newMove = true;

    // ----- PHASE TWO: INTENTIONAL TWITCH EVERY 5 SECONDS ----- //
    do {
      readSensors();

      if (_newMove){                    // check if a new movement should be prompted
        delay(200);
        ledOn(_sensorLoop);             // flash the LED to prompt the user to do an intentional twitch / movement
        delay(PULSE_LEN);
        ledOff(_sensorLoop);
        _newMove = false;
      }
      
      if (sensor_num == _sensorLoop) {                    // Ignore non-selected sensors
        _prevReading = _reading;
        _reading = sensorsReceived[sensor_num-1];
        _readingCount ++;

        if ((abs(_reading-_prevReading)) < 0.001) { 
          _reading = 0;
        }

        if (_reading > minThreshold) {          // Reading is above threshold defined above
          _peakCount++;                 // ***** I dont think I need this
          Serial.print("peak: ");
          Serial.print(_peakCount);
          Serial.print("\t");
          Serial.println(_reading);
          //_sum += _reading;             // ***** I dont think I need this
          if (_noPeak) {
            _moveStart = millis();              // Time stamp occurance of the start of the movement
            _moveEnd = _moveStart;              // If there is only one reading during the twitch start = end
            _noPeak = false;
          } else {
            //_peak2 = millis();                // Time stamp occurence of second peak value *** DON'T NEED
            _moveEnd = millis();                // Time stamp occurance of the end of the movement
          }
          if (_reading > _peakValue) _peakValue = _reading;   // Record the highest peak measured
        }
      }
      if ((millis() - _blink) > 5000) {          // Every 5 seconds save values, reset variables, and indicate new movement
        if (_noPeak){
          _moveDur[_twitchNum]=1; // or other error handling
        }
        else {
          _moveDur[_twitchNum] = (_moveEnd - _moveStart);
        }
        Serial.println("new move");
        _noPeak = true;                         // reset variable
        _peakCount = 0;                         // reset variable
        _newMove = true;                        // indicate a new move should be prompted
        
        
        _twitchNum ++;                          // increase number of twitches counter
        _blink = millis();                      // Reset timer
        
      }  

      // Clear values
      x_received[_sensorLoop-1] = 0;
      y_received[_sensorLoop-1] = 0;
      z_received[_sensorLoop-1] = 0;
      _reading = 0;
      sen1received = 0;
      sen2received = 0;
      sen3received = 0;
      sensor_num = 0;                         //  Clear sensor number ahead of next reading
      tempByte = radio.flush_rx();

      delay(100);                               // Short pause between readings
      
    } while (_twitchNum < 4);    // Take 4 consecutive readings for 30 seconds while User makes movements

    for (int i=0; i<sizeof(_moveDur)/sizeof(_moveDur[0]); i++){ 
      if (_moveDur[i] == 1){            // Checks for missing value flag
      _twitchNum -=1;               // if measurement is missing, adjust total twitches and don't add to total
      } else {
        _sumDur += _moveDur[i];     // if valid data point, add to total
      }
    
      _avgTime = _sumDur/(_twitchNum+1);        // Computes the average time of a movement

      // Error handling, ensuring delay is within limits
      if (_avgTime < MIN_DELAY){
        _avgTime = MIN_DELAY;
      } else if (_avgTime > MAX_DELAY) {
        _avgTime = MAX_DELAY;
      }
  
      // Save autocalibration results for the selected sensor to dynamic memory
      threshold_data[activeProfile - 1][_sensorLoop-1] = minThreshold;    // Store new threshold value to dynamic RAM

      move_duration[activeProfile - 1][_sensorLoop - 1] = _avgTime;
      
      saveProfiles();                             // Commit new profile values to EEPROM
      
      if (_sensorLoop == 1) {
        oneDone = true;
      } else if (_sensorLoop == 2) {
        twoDone = true;
      } else if (_sensorLoop == 3) {
        threeDone = true;
      }
      
      allLEDoff();
    }    // end of sensor loop, check next sensor
  }   // end of sensor calibration


  autoCal = true;                               // Signal that autocalibration was performed
  calMode = false;                              // Device exiting calibration mode
  allLEDoff();
  for (char a = 0; a < 2; a++) {                // Flash all 3 channel LEDs twice to indicate completed
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, HIGH);
    delay(500);
    allLEDoff();
    delay(350);
  }
  if (mode == 0) digitalWrite(LED_M, HIGH);
  if (mode == 1) digitalWrite(LED_L, HIGH);

  return;                                       // Exit autocalibration function

AbortAutocal:                                   // Autocalibration abort - no sensors detected or left unattended too long
  allLEDoff();
  calMode = false;

  // Flash channel LEDs rapidly to indicate error
  for (char a = 0; a < 10; a++) {
    digitalWrite(LED_1, HIGH);
    digitalWrite(LED_2, HIGH);
    digitalWrite(LED_3, HIGH);
    delay(50);
    allLEDoff();
    delay(100);
  }

  digitalWrite(LED_M, LOW);
  digitalWrite(LED_L, LOW);

  // Restore previous MODE display
  if (mode == 0) digitalWrite(LED_M, HIGH);
  if (mode == 1) digitalWrite(LED_L, HIGH);

}
*/

/* ----------------------------------------------------------------------------------------
   ==========                         MANUAL ADJUSTMENT                          ==========

   If autocalibration values are inadequate for a particular activity, sensors can be manualy adjusted.
   The manual adjustment sets response times of individual sensors, making them more or less sensitive
   to long or slow movements. This allows to tailor the system's responsiveness to an individuals needs
   for a special task.

   Manual override is initiated by pressing and holding the PROFILE button down for 3 seconds or more,
   then is indicated by lighting the MOMENTARY and PULSE LEDs simultaneously for 2 seconds.
   To exit manual mode, the User simply selects another profile.
*/

/*
void manualCal(void) {

  char _button;
  digitalWrite(LED_M, HIGH);
  digitalWrite(LED_L, HIGH);
  allLEDoff();
  switch (activeProfile) { // Show current profile
    case 1:
      digitalWrite(LED_1, HIGH);
      break;
    case 2:
      digitalWrite(LED_2, HIGH);
      break;
    case 3:
      digitalWrite(LED_3, HIGH);
      break;
    default:
      break;
  }  // end of switch/case statement
  delay(500);
  allLEDoff();

  do {
    getPots();    // Read potentiometers
    readSensors();
    checkValidSensorMove();
    if (validSensor == 1) {   // Sensor 1 triggered
      digitalWrite(LED_1, HIGH);
      delay(PULSE_LEN);
      digitalWrite(LED_1, LOW);
    }

    if (validSensor == 2) {  // Sensor 2 triggered
      digitalWrite(LED_2, HIGH);
      delay(PULSE_LEN);
      digitalWrite(LED_2, LOW);
    }

    if (validSensor == 3) {  // Sensor 3 triggered
      digitalWrite(LED_3, HIGH);
      delay(PULSE_LEN);
      digitalWrite(LED_3, LOW);
    }

    _button = checkSwitches();

    clearSensors();
    delay(50);

  } while (_button != 2);

}
*/
/* -------------------------------------------------------------------------------- */

/*
void getPots(void) {
  // Manual mode initiated by pressing PROFILE button and holding it down for 3+ seconds
  //   Reads the three potentiometers and overrides autocalibration values.

  //  Local Variables:
  unsigned long _pot1reading, _pot2reading, _pot3reading;
  unsigned long _sum1, _sum2, _sum3;                                                // what are these??????????

  _pot1reading = analogRead(POT_1);
  _pot2reading = analogRead(POT_2);
  _pot3reading = analogRead(POT_3);

  // Manual mode adjusts the time between movements
  manualProfile_1 = _pot1reading * (MAX_DELAY-MIN_DELAY)/1023L;
  manualProfile_2 = _pot2reading * (MAX_DELAY-MIN_DELAY)/1023L;
  manualProfile_3 = _pot3reading * (MAX_DELAY-MIN_DELAY)/1023L;

}  // done manual function
*/