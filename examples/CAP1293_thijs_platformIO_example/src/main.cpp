/*

NOTE: this example sketch is NOT FINISHED!!!

for the TODO list of remaining features/functions, see CAP1293_thijs.h (bottom of comment at the top)

This is a test of the touch sensor IC: CAP1293


TODO:
- finish library (see TODO list there)
- finish missing examples here
- combo-mode specific example (seperate, as it's somewhat comprehensive)
*/

#include <Arduino.h>


#define CAP1293_useWireLib  // force the use of Wire.h (instead of platform-optimized code (if available))

#define CAP1293debugPrint(x)  Serial.println(x)    //you can undefine these printing functions no problem, they are only for printing I2C errors
//#define CAP1293debugPrint(x)  log_d(x)  // ESP32 style logging

#include <CAP1293_thijs.h>

CAP1293_thijs sensor;

#define INTpin PA1 // (A2/D16) aka PA1, should be PB5, can be PB2. NOTE: D16 does not work everywhere (like activating interrupts, i think), while PA1 does

#ifdef ARDUINO_ARCH_ESP32  // on the ESP32, almost any pin can become an I2C pin
  const uint8_t CAP1293_SDApin = 26; // 'defualt' is 21 (but this is just some random value Arduino decided.)
  const uint8_t CAP1293_SCLpin = 27; // 'defualt' is 22 
#endif
#ifdef ARDUINO_ARCH_STM32   // on the STM32, each I2C peripheral has several pin options
  const uint8_t CAP1293_SDApin = SDA; // default pin, on the STM32WB55 (nucleo_wb55rg_p) that's pin PB9
  const uint8_t CAP1293_SCLpin = SCL; // default pin, on the STM32WB55 (nucleo_wb55rg_p) that's pin PB8
  /* Here is a handy little table of I2C pins on the STM32WB55 (nucleo_wb55rg_p):
      I2C1: SDA: PA10, PB7, PB9
            SCL: PA9, PB6, PB8
      I2C3: SDA: PB4, PB11, PB14, PC1
            SCL: PA7, PB10, PB13, PC0      */
#endif

#ifdef CAP1293_useWireLib // (currently) only implemented with Wire.h
  bool checkI2Caddress(uint8_t address) {
    Wire.beginTransmission(address);
    return(Wire.endTransmission() == 0);
  }
  void I2CdebugScan() {
    Serial.println("I2C debug scan...");
    for(uint8_t address = 1; address<127; address++) {
      if(checkI2Caddress(address)) {
        Serial.print("got ACK at address: 0x"); Serial.println(address, HEX);
      }
      delay(1);
    }
    Serial.println("scanning done");
  }
#endif

#ifdef INTpin
  void CAP1293_ISR() {
    // Serial.print("ISR "); Serial.println(digitalRead(INTpin));
    // uint8_t touchFlags = sensor.getTouchFlags(); Serial.print("touch flags: "); Serial.println(touchFlags, BIN);
    // Serial.print("power button: "); Serial.println(sensor.getGenStatus(), BIN); // does not seem to work??????
    //// certain flags must read BEFORE clearing the INT bit, because writing the INT bit can affect other flags as well
    CAP1293_ERR_RETURN_TYPE err = sensor.setMainControl_INT(false); // clear INT bit (also deactivates INT pin)
    if(!sensor._errGood(err)) { Serial.println("failed to clear INT bit!"); }
    uint8_t deltaSense = sensor.getActiveSensitivity(); // assume active mode
    int16_t deltaCounts[3]; for(uint8_t i=0; i<3; i++) { deltaCounts[i] = sensor.calcDeltaCountFull(sensor.getDeltaCountRaw(i), deltaSense); }
    Serial.print("deltas: "); for(uint8_t i=0; i<3; i++) { Serial.print(deltaCounts[i]); Serial.print('\t'); } Serial.println();
  }
#endif

void setup() 
{
  Serial.begin(115200);  delay(50); Serial.println();

  delay(CAP1293_BOOT_DELAY_COMM_READY); // according to the datasheet, the IC needs 15ms (max) boot time before it's ready to communicate

  #ifdef CAP1293_useWireLib // the slow (but pretty universally compatible) way
    sensor.init(100000); // NOTE: it's up to the user to find a frequency that works well.
  #elif defined(__AVR_ATmega328P__) || defined(__AVR_ATmega328__) // TODO: test 328p processor defines! (also, this code may be functional on other AVR hw as well?)
    pinMode(SDA, INPUT_PULLUP); //A4    NOT SURE IF THIS INITIALIZATION IS BEST (external pullups are strongly recommended anyways)
    pinMode(SCL, INPUT_PULLUP); //A5
    sensor.init(100000); // your average atmega328p should do 800kHz
  #elif defined(ARDUINO_ARCH_ESP32)
//    pinMode(CAP1293_SDApin, INPUT_PULLUP); //not needed, as twoWireSetup() does the pullup stuff for you
//    pinMode(CAP1293_SCLpin, INPUT_PULLUP);
    esp_err_t initErr = sensor.init(100000, CAP1293_SDApin, CAP1293_SCLpin, 0); //on the ESP32 (almost) any pins can be I2C pins
    if(initErr != ESP_OK) { Serial.print("I2C init fail. error:"); Serial.println(esp_err_to_name(initErr)); Serial.println("while(1){}..."); while(1) {} }
    //note: on the ESP32 the actual I2C frequency is lower than the set frequency (by like 20~40% depending on pullup resistors, 1.5kOhm gets you about 800kHz)
  #elif defined(__MSP430FR2355__) //TBD: determine other MSP430 compatibility: || defined(ENERGIA_ARCH_MSP430) || defined(__MSP430__)
    // not sure if MSP430 needs pinMode setting for I2C, but it seems to work without just fine.
    sensor.init(100000); // TODO: test what the limit of this poor microcontroller are ;)
    delay(50);
  #elif defined(ARDUINO_ARCH_STM32)
    // not sure if STM32 needs pinMode setting for I2C
    sensor.init(100000, SDA, SCL, false); // TODO: test what the limits of this poor microcontroller are ;)
    /* NOTE: for initializing multiple devices, the code should look roughly like this:
      i2c_t* sharedBus = sensor.init(100000, SDA, SCL, false); // returns sensor._i2c (which is (currently) also just a public member, btw)
      secondSensor.init(sharedBus); // pass the i2c_t object (pointer) to the second device, to avoid re-initialization of the i2c peripheral
      //// repeated initialization of the same i2c peripheral will result in unexplained errors or silent crashes (during the first read/write action)!
    */
  #else
    #error("should never happen, CAP1293_useWireLib should have automatically been selected if your platform isn't one of the optimized ones")
  #endif

  #ifdef CAP1293_useWireLib
    I2CdebugScan();
    Serial.println(); // seperator
  #endif

  if(!sensor.connectionCheck()) { Serial.println("CAP1293 connection check failed!");    while(1);    } else { Serial.println("connection good"); }
  
  delay(CAP1293_BOOT_DELAY_DATA_READY - CAP1293_BOOT_DELAY_COMM_READY); // according to the datasheet, the IC needs 170~200 (typ~max) boot time before the first conversion is ready.
  
  //// at this point, it should also have completed the first calibration (for each sensor). Check whether it went as planned:
  uint8_t calibActive = sensor.getCalibOngoingAll(); // first of all, check whether calibration is still ongoing
  while((calibActive & CAP1293_channel_bit(0)) || (calibActive & CAP1293_channel_bit(1)) || (calibActive & CAP1293_channel_bit(2))) { // a needlessly explicit (but hopefully explanatory) check of each sensor input's bit
    Serial.print("calibration not finished: "); Serial.print(calibActive, BIN);
    Serial.print("\t noise flags: "); Serial.println(sensor.getNoiseFlagAll(), BIN); // if RF- or Low-Freq noise is detected, the rest of the sensor basically refuses to work.
    delay(250);
    calibActive = sensor.getCalibOngoingAll(); // update before looping again
  } // now that we're absolutely sure calibration is finished:
  Serial.println("calibration active bits are LOW (which is good)");
  if(sensor.getNoiseFlagAll()) { Serial.print("noise flags raised!: "); Serial.println(sensor.getNoiseFlagAll(), BIN); }
  if(sensor.getGenStatus_ACAL_FAIL()) { Serial.println("Analog Calibration Faillure flag raised!"); }
  // Serial.print("Analog Calibration result values (debug?): "); for(uint8_t i=0; i<3; i++) { Serial.print(sensor.getAnalogCalibResult(i)); Serial.print('\t'); } Serial.println();
  // Serial.print("Analog Sensitivities (CALSENx) (debug): "); for(uint8_t i=0; i<3; i++) { Serial.print(sensor.getCalibConf_CALSENbits(i)); Serial.print('\t'); } Serial.println();
  if(sensor.getGenStatus_BC_OUT()) { Serial.println("Base Count Out Of Limit (any) flag raised!"); }
  if(sensor.getBaseCountsOOL()) { Serial.print("Base Count Out Of Limit (individual) flags raised!:"); Serial.println(sensor.getBaseCountsOOL()); }
  // Serial.print("Base shift (debug): "); Serial.println(min(sensor.getBaseShift(), (uint8_t)8));
  Serial.print("Base Counts (full): "); for(uint8_t i=0; i<3; i++) { Serial.print(sensor.getBaseCountFull(i)); Serial.print('\t'); } Serial.println();
  Serial.print("ideal Base Count (full): "); Serial.println(sensor.calcIdealBaseCount(sensor.getActiveSampleTimeBits()));
  uint8_t activeDELTA_SENSE = sensor.getActiveSensitivity();   uint8_t standbyDELTA_SENSE = sensor.getStandbySensitivity();
  // Serial.print("DELTA_SENSE shift for Active and Standby (debug): "); Serial.print(7-activeDELTA_SENSE); Serial.print(' '); Serial.println(7-standbyDELTA_SENSE);
  Serial.print("touch thresholds (full): "); for(uint8_t i=0; i<3; i++) { Serial.print(sensor.getActiveTouchThreshFull(i)); Serial.print('\t'); } Serial.println();
  Serial.println();

  //// Noise:
  // sensor.setConf_1_DIS_LF_NOISE(true); // ignore all Low-Freq noise
  // sensor.setConf_2_DIS_RF_NOISE(true); // ignore all RF noise
  // sensor.setConf_2_DIS_LF_FLAG(true); // disables LF noise from setting the Noise flag (interrupt). Does NOT disable LF noise hardware, just silences it

  //// Hold (repeated interrupts) feature:
  // sensor.setHoldThresholdMillis(CAP1293_HOLD_THRSH_LIMITS[0]); // see CAP1293_HOLD_THRSH_LIMITS

  //// Power Button feature:
  // Serial.print("sensor input selected for power button: "); Serial.println(sensor.getPowerButtonSelect());
  // Serial.print("power button enabled in active- or standby mode: "); Serial.print(sensor.getPwrBtnConf_ACTV_EN()); Serial.print(' '); Serial.println(sensor.getPwrBtnConf_STBY_EN());
  // Serial.println();

  //// Multiple Touch feature:

  //// Multiple Touch Patter (MTP) feature:

  //// changing/tuning sensitivity:
  // sensor.setCalibConf_CALSENbits(0, CAP1293_CALSEN_5_50p); // set a higher than default CALSEN (capacity range), (can be used to make proximity sensors)
  // sensor.setCalibConf_CALSENbits(1, CAP1293_CALSEN_0_25p); // set a higher than default CALSEN (capacity range), (can be used to make proximity sensors)
  // sensor.setCalibConf_CALSENbits(2, CAP1293_CALSEN_0_12p5); // set a higher than default CALSEN (capacity range), (can be used to make proximity sensors)
  // delay(50); // shouldn't be needed, please remove
  // sensor.startCalibAll();
  // delay(5); // not sure if actually needed, but it can't hurt
  // calibActive = sensor.getCalibOngoingAll(); // first of all, check whether calibration is still ongoing
  // while(calibActive) {
  //   Serial.print("still recalibrating: "); Serial.println(calibActive, BIN);
  //   delay(25);
  //   calibActive = sensor.getCalibOngoingAll(); // update before looping again
  // } // wait for calibration to finish
  // Serial.println("done recalibrating!");
  // Serial.print("main control: "); Serial.println(sensor.getMainControl());
  // Serial.print("gen status: "); Serial.println(sensor.getGenStatus());
  // if(sensor.getNoiseFlagAll()) { Serial.print("noise flags raised!: "); Serial.println(sensor.getNoiseFlagAll(), BIN); }
  // if(sensor.getGenStatus_ACAL_FAIL()) { Serial.println("Analog Calibration Faillure flag raised!"); }
  // Serial.print("Analog Calibration result values (debug?): "); for(uint8_t i=0; i<3; i++) { Serial.print(sensor.getAnalogCalibResult(i)); Serial.print('\t'); } Serial.println();
  // Serial.print("Analog Sensitivities (CALSENx) (debug): "); for(uint8_t i=0; i<3; i++) { Serial.print(sensor.getCalibConf_CALSENbits(i)); Serial.print('\t'); } Serial.println();
  // if(sensor.getGenStatus_BC_OUT()) { Serial.println("Base Count Out Of Limit (any) flag raised!"); }
  // if(sensor.getBaseCountsOOL()) { Serial.print("Base Count Out Of Limit (individual) flags raised!:"); Serial.println(sensor.getBaseCountsOOL()); }
  // // Serial.print("Base shift (debug): "); Serial.println(min(sensor.getBaseShift(), (uint8_t)8));
  // Serial.print("Base Counts (full): "); for(uint8_t i=0; i<3; i++) { Serial.print(sensor.getBaseCountFull(i)); Serial.print('\t'); } Serial.println();

  //// Interrupts (disabling and enabling them):
  sensor.setInterruptEnable(0,true); // enable sensor input 0 to assert the ALERT pin (drive it LOW). This is enabled by default for all sensor, this is just demostrative
  // ... hold (HOLD_THRESH), setRptRateMillis
  // ... release
  //sensor.setConf_2_DIS_LF_FLAG(true); // disables LF noise from setting the Noise flag (interrupt). Does NOT disable LF noise hardware, just silences it

  #ifdef INTpin // if you've connected the INT pin (a.k.a. ALARM), which is optional (but recommended)
    pinMode(INTpin, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTpin), CAP1293_ISR, FALLING); // INT pin (a.k.a. ALERT) is active LOW
  #else
    Serial.println("non-interrupt example code is TBD!");
  #endif
  CAP1293_ERR_RETURN_TYPE err = sensor.setMainControl_INT(false); // clear INT bit (also deactivates INT pin)
  if(!sensor._errGood(err)) { Serial.println("failed to clear INT bit!"); }
}

void loop()
{
  // uint8_t deltaSense = sensor.getActiveSensitivity(); // assume active mode
  // int16_t deltaCounts[3]; for(uint8_t i=0; i<3; i++) { deltaCounts[i] = sensor.calcDeltaCountFull(sensor.getDeltaCountRaw(i), deltaSense); }
  // Serial.print("deltas: "); for(uint8_t i=0; i<3; i++) { Serial.print(deltaCounts[i]); Serial.print('\t'); } Serial.println();
  // delay(2000);
}