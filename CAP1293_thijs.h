
// datasheet link: https://ww1.microchip.com/downloads/aemDocuments/documents/OTH/ProductDocuments/DataSheets/00001566B.pdf
/*

The CAP1293 operators on I2C at 10~400 kHz (or SMBus, )
The address is 0b0101000x (where x is the R/W bit)
 in Arduino convention, this would be address 0b_0101000 => 0x28
It "supports" clock stretching, but will NOT do it itself

I2C data transfer is very comparable to my other libraries (see AS5600_thijs, TMP112_thijs or NT3H1x01_thijs)
You send it an register_address value to indicate what you would like to read/write
then you follow up with data. Much like the AS5600, this has an incrementing memory pointer,
 so writing/reading multiple bytes will affect bytes by their memory pointer sequentially (the datasheet sais 'contiguous', but i think they mean continous in this case)
Most of the time this is not extremely useful (there are very few multi-byte values),
 but I2C START/SLA_x/STOP overhead is not to be underestimated
This memory-pointer does rollover (0xFF to 0x00), so theoretically, you could read the whole memory in a never-ending cycle (of inefficiency)

Explained concepts:
- Basics (oddities)
- Power Modes (Active, Standby, Combo and Deep Sleep)
- Timings (sens cycle, sample time, etc.)
- Sensitivity (and/vs gain, base/delta counts and bitshifting)
- Calibration (auto-, re-, manual-)
- Noise (RF & low-freq)
- Guard pad
- Hold, Multi-Touch and Multi-Touch-Pattern

Basics (oddities):
- a touch is determined as (((measurement_count-base_count)>>DELTA_SENSE) >= touch_threshold_reg)
- you will never have access to raw measurements, only 'delta' counts, which are (measurement_count-base_count)
- there are several (sometimes redundant) sensitivity settings, please read through the Sensitivity explenation below (NOTE: DELTA_SENSE is seperate for Active and Standby mode)
- Active and Standby mode are not really about power and more about seperating the input settings (to get one input to be significantly more sensitive than others)
- input 2 can be used as a Signal-Guard, improving the performance of the pads it encompasses (basically a must-have for proximity sensors)
- the INT bit in the Main Control register (linked to ALARM pin), must be cleared for status registers (other flags like CAP1293_TOUCH_FLAG_REG) to be reset.
- if you touch 1 sensor input, touch flag 1 and INT bit go high, you clear the INT bit, and then you touch another input (while still holding the first), it does NOT update the touch flags correctly
   (but the Multiple-Touch flag does go high)

Power modes:
- Active is the regular(/highest) mode
  Timings and sensitivity settings are shared, but base counts are seperate
- Standby is not really about saving power, it's really just an alternate mode
  Instead of individual touch thresholds, Standby mode has a unified threshold for all sensors
  It's focussed on more sensitive application (you can even sum samples instead of averaging them, see CAP1293_STBY_CONF_AVG_SUM_bits)
- Combo is a freaky hybrid of Active and Standy
  The datasheet talks about this mode being intended for proximity detection (Standby-enabled-inputs??) and normal buttons (Active-enabled-inputs?) simultaniously
  In general, if you need some of your sensors to have different sampling
- Deep Sleep is exactly what it sounds like
  The datasheet claims ~5uA (typ) consumption in this mode
  Sensors inputs are inactive and communications will wake it back up

Timings:
- A 'sensing cycle' is one complete measurement on ALL sensor inputs.
  The sensing cycle time is set using CAP1293_SAMP_CONF_CYCL_TM_bits (and CAP1293_STBY_CONF_CYCL_TM_bits for Standby).
  If there is time left in the cycle after all inputs are read, it will conserve a little power (if CAP1293_CONF_2_BLK_PWR_CTRL_bits).
- Averaging can be set using CAP1293_SAMP_CONF_AVG_bits (and CAP1293_STBY_CONF_SAMP_CNT_bits for Standby).
- The sample-time can be set using CAP1293_SAMP_CONF_SAMP_TM_bits (and CAP1293_STBY_CONF_SAMP_TM_bits for Standby).
  The capacitive measurement hardware 'likes' to work at 10MHz, considering that the 'ideal sample count' is (10*sample_time_in_us).

Sensitivity:
- 'Base count' is used to compensate for the natural capacitance of the sensing pad
  the base count bytes can be found CAP1293_BASE_COUNT_x_REG and should be left-shifted by CAP1293_SENS_BASE_SHIFT_bits to produce the actual base count value
  By default, the BASE_SHIFT is set to the max 8 bits. It's not clearly stated whether BASE_SHIFT if automatically set, but considering the ideal
  At a sample time of 320us, the ideal base count is 3200. To maintain similar base count resolution, BASE_SHIFT = 5+(SAMPLE_TM_bits)
  Out Of Limit base count refers to a base count value that is more than +-12.5% off from the ideal value
- CALSEN is the capacitance range of the pad. This basically determines whether it's a touch or procimity input
  this range selection is done using CAP1293_CALIB_CONF_CALSEN bits
- Analog gain (in the Main Control register) affects counts per capacitance underwater.
  the analog gain can be set using CAP1293_MAIN_CTRL_GAIN_bits
  Except, when in Combo mode, which makes the Standby-enabled-sensors use CAP1293_MAIN_CTRL_COMBO_GAIN_bits instead
- DELTA_SENSE is a (mediocrely documented / strangely implemented) secondary gain.
  there are 2 DELTA_SENSE settings, one for Active mode and one for Standby, CAP1293_SENS_ACTV_SENSE_bits (in CAP1293_SENS_REG) and CAP1293_STBY_SENS_REG respectively
  This value will shift the delta count to the right before being stored in the register, so delta_count_reg = ((measurement_count-base_count)>>DELTA_SENSE)
  The touch threshold will be compared against the delta_count register value
- Touch (binary) sensitivity is determined by a threshold applied to delta (not to the measurement).
  The thresholds per sensor can be set using CAP1293_TOUCH_THRSH_x_REG (and CAP1293_STBY_THRSH_REG for Standby)

Calibration:
- Analog calibration attempts to balance the natural capacitance of the sensor pad and the ICs internal circuitry
  the results of this calibration (10bit value) can be found at CAP1293_CALIB_x_MSB_REG and CAP1293_CALIB_x_LSB_REG
  No unit is given for how to interpret this value, and it's not as important as base-count. But if it's near the limits that's probably bad pad design
- Digital calibration determines the base-count value (per sensor input in Active mode)
  the number of samples- and cycles used for (digital?) calibration is set using CAP1293_RCLB_CONF_CAL_CFG_bits
  some of the options in CAP1293_RCLB_CONF_CAL_CFG_ENUM allow for more cycles than samples, and thus some settling time (in case the act of sensing affects measurements?)
- (full calibration is: analog then digital).
- If calibration is ongoing, a bit will be set HIGH in CAP1293_CALIB_ACTIV_REG
  Manual calibration can also be triggered using CAP1293_CALIB_ACTIV_REG, by writing a 1 manually
- Automatic calibration can be triggered by the following things:
  - Power On Reset
  - enabling a sensor input (that was previously disabled within the current power mode)
  - calibration sensitivity changed
  - gain (sensor input gain?) changed
  - sample time changed
  - several negative delta counts (if pad changed, or if last calibration was bad). See CAP1293_RCLB_CONF_NEGDLT_CNT_bits
  - a very long touch (badly set thresholds?) ONLY if enabled at CAP1293_CONF_1_MAX_DUR_EN_bits, see also: CAP1293_TIMING_CONF_MAX_DUR_bits
  - a failed calibration triggers (by default, but can be turned off in some cases) another attempt at calibration
- Faillure of calibration can trigger (or be triggered by) some of the following effects:
  - if a Noise bit is set, analog calibration fails outright. Clear the noise flags and/or fix your RF/power-supply design
  - ACAL_FAIL bit indicates analog calibration faillure. Tune Gain bits or pad design to fix
  - BC_OUT bits indicates digital calibration faillure (becuase the base count is Out Of Limit, which is +-12.5% of ideal value). Tune Gain(?) and BASE_SHIFT (not sample-time???) to fix

Noise:
- the sensor includes detection hardware for RF- and low-freq noise corrupting the signal
- when noise is detected during a sampling, that sample will be discarded. I'm not sure what that means for the final result, the datasheet is rather vague
  DIS_LF_NOISE and DIS_RF_NOISE can prevent samples from being discarded (disable noise detection blocks). Use with caution
  SHOW_RF_NOISE (_CONF_2_DIS_LF_FLAG_bits) just prevents LF noise from raising the Noise flag (interrupt) bit. Samples will still be discarded, it's only the flag.
  I truly have no clue what the heck DIS_DIG_NOISE does, and the datasheet is increadably vague in the single instance it's mentioned. Avoid if possible
- to differentiate between a touch and general capacitance(/proximity), use the touch_threshold (seperate for active and standby)
  or just read out the delta_count frequently and do your own filtering

Guard (Signal-Guard pad/perimeter):
- input 2 can be used as a Signal-Guard pad
  which other sensor input it's guarding is set using CAP1293_GUARD_EN_FOR_x_bits
  Signal-Guard pad is strongly recommended for proximity sensors (which are subject to more noise) or otherwise-noisy designs (RF/buck-boost-power)

Hold (& repeat), Power-Button, Multi-Touch and Multi-Touch-Pattern:
- Hold (also referred to as 'press and hold' in the datasheet) is used to generate repeated interrupts, beside the touch- and release interrupts (only useful in very niche usecases)
  Once a touch is detected, a (first) timer is started, which counts up to M_PRESS (CAP1293_HOLD_THRSH_REG)
   then, for as long as the touch is held, interrupts (repeats) will be generated at an interval set by RPT_RATE (CAP1293_TIMING_CONF_RPT_RATE_bits)
  If the release-interrupt is enabled (it probably should be anyways), the whole press-and-hold feature becomes redundant. I'm not sure who thought this would be a worthwhile addition
- Power Button is really just adding a minimum-touch-time threshold to avoid waking devices from sleep when the sensor is not really touched all the way.
  only 1 sensor input can be used simultaniously
- Multi-Touch ...
- Multi-Touch-Pattern (MTP) ...

TODO:
- getActiveTouchThreshRaw() should probably return int8_t (signed) instead of uint8_t. Test before push though!
- make printConfig() function (mostly about the sensitivity settings)
- finish Multiple-Touch & MTP documentation up here
- function documentation (almost all)
- EASY version (child class), stores: DELTA_SENSE (both), 
- can the touch flag be manually cleared???
- is generalStatus Read-only?
- test exact effects of DELTA_SENSE and other sensitivity settings
- test which sensitivity settings trigger a recalibration when changed (ACTV/STBY DELTA_SENSE, gain, CALSEN)
- test contiguous vs continuous memory pointer movement (gaps in register addresses)
- test why power button feature doesnt work???
- write EXTENSIVE example code (active, standby, combo. calibration. ,,,)
- HW testing STM32
- fix inconsistant function names (like  getConf_2_BC_OUT_RECAL vs getMaxDur  and  getConf_2 vs getTimingConfReg)
- HW testing ESP32
- HW testing MSP430
- HW testing 328p
- check Wire.h function return values for I2C errors
- test if 'static' vars in the ESP32 functions actually are static (connect 2 sensors?)
- generalized memory map struct (also for other libraries). Could just be an enum, i just don't love #define.

To write:
- waitForCalibrationFinished()
- (auto-tune sensitivity/gain?)
- (easy-mode child-class?)

*/

#ifndef CAP1293_thijs_h
#define CAP1293_thijs_h

#include "Arduino.h" // always import Arduino.h

//#define CAP1293debugPrint(x)  Serial.println(x)
//#define CAP1293debugPrint(x)  log_d(x)   //using the ESP32 debug printing

#ifndef CAP1293debugPrint
  #define CAP1293debugPrint(x)  ;
#endif

#define CAP1293_channel_bit(sensorIndex)   (1<<sensorIndex) // many registers use 3LSBits for the 3 sensors respectively. This turns 0 into 0b_001, 1 into 0b_010 and 2 into 0b_100
#define CAP1293_constr_index(sensorIndex)  ((sensorIndex>2) ? 2 : sensorIndex) // there are only 3 sensor inputs, this prevents overflow mis-usage

//// CAP1293 constants:
#define CAP1293_BOOT_DELAY_COMM_READY  15  // (milliseconds) "Time to Communications Ready" in datasheet (page 8)
#define CAP1293_BOOT_DELAY_DATA_READY  200 // (milliseconds) "Time to Forst Conversion Ready" in datasheet (page 8)

//// register addresses:
#define CAP1293_INVALID_REG_ADDR  0x70 // NOT IN DATASHEET, this is just for my own code. There should be nothing in the memory at this location

// (status registers)
#define CAP1293_GEN_STATUS_REG    0x02  // (R/W?) General Status register, see defined _bits below   NOTE: datasheet has typo, this register is both defined as R/W and R only, I suspect R only is correct
#define CAP1293_NOISE_FLAG_REG    0x0A  // (R) Noise Flag Status register (per sensor), 3LSBits are 3 sensor channels respectively

// (sensor output registers)
#define CAP1293_TOUCH_FLAG_REG    0x03  // (R) Sensor Input (touch) Status register (per sensor), 3LSBits are 3 sensor channels respectively
#define CAP1293_DELTA_COUNT_REG_1 0x10  // (R) sensor 1 Delta Count register (SIGNED int8_t) interpret as(???): (measured_count - base_count)>>DELTA_SENSE
// #define CAP1293_DELTA_COUNT_1_REG 0x10  // (R) sensor 1 Delta Count register (SIGNED int8_t) interpret as(???): (measured_count - base_count)>>DELTA_SENSE
// #define CAP1293_DELTA_COUNT_2_REG 0x11  // (R) sensor 1 Delta Count register (SIGNED int8_t) interpret as(???): (measured_count - base_count)>>DELTA_SENSE
// #define CAP1293_DELTA_COUNT_3_REG 0x12  // (R) sensor 1 Delta Count register (SIGNED int8_t) interpret as(???): (measured_count - base_count)>>DELTA_SENSE

// Configuration registers
#define CAP1293_MAIN_CTRL_REG     0x00  // (R/W) Main Control register
#define CAP1293_CONF_1_REG        0x20  // (R/W) Configuration   register
#define CAP1293_CONF_2_REG        0x44  // (R/W) Configuration 2 register
#define CAP1293_SENS_REG          0x1F  // (R/W) Sensitivity Control register, holds input sensitivity/scaling (DELTA_SENSE) (for Active mode inputs!) and base-count scaling (BASE_SHIFT)
// (general sensor input enable registers):
#define CAP1293_ACTV_CHAN_EN_REG  0x21  // (R/W) Sensor Input Enable register, which sensor inputs are monitored (in Active mode), 3LSBits are 3 sensor channels respectively
#define CAP1293_ALERT_EN_REG      0x27  // (R/W) (Alert) Interrupt Enable register (per sensor), 3LSBits are 3 sensor channels respectively
#define CAP1293_REPEAT_EN_REG     0x28  // (R/W) Repeat Rate Enable register (per sensor), 3LSBits are 3 sensor channels respectively
#define CAP1293_GUARD_EN_REG      0x29  // (R/W) Signal Guard Enable register, see defined _bits below

// (some timing-related registers):
#define CAP1293_TIMING_CONF_REG   0x22  // (R/W) Sensor Input Configuration (1) register, holds 2 4bit values (nibbles), see defined _bits below
#define CAP1293_HOLD_THRSH_REG    0x23  // (R/W) Sensor Input Configuration 2 register (holds 4bit M_PRESS value), press-and-hold (instead of touch) time threshold = (35ms*(THRSH_bits+1)), default = 280ms
#define CAP1293_SAMP_CONF_REG     0x24  // (R/W) Averaging and Sampling Configuration register, see defined _bits below

// (some multi-touch-related registers):
#define CAP1293_MULT_CONF_REG     0x2A  // (R/W) Mutliple Touch Configuration register, see defined _bits below
#define CAP1293_PTRN_CONF_REG     0x2B  // (R/W) Mutliple Touch Pattern Configuration register, see defined _bits below
#define CAP1293_PTRN_SEL_REG      0x2D  // (R/W) Mutliple Touch Pattern (selection) register. Either it's which sensor inputs are- , or it's how MANY (aboslute) sensor inputs are  'part of the pattern'

// (some calibration-related registers):
#define CAP1293_CALIB_ACTIV_REG   0x26  // (R/W) Calibration Activate and Status register (per sensor), 3LSBits are 3 sensor channels respectively. Read for status/faillure, write 1 to trigger calibration
#define CAP1293_BASE_CNT_OOL_REG  0x2E  // (R) Base Count Out Of Limit register, 3 LSBits indicate whether the (auto-calibrated) base count is Out Of Limit (per sensor)
#define CAP1293_RCLB_CONF_REG     0x2F  // (R/W) Recalibration Configuration register, see defined _bits below
#define CAP1293_TOUCH_THRSH_REG_1 0x30  // (R/W) touch Threshold register for sensor 1, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
// #define CAP1293_TOUCH_THRSH_1_REG 0x30  // (R/W) touch Threshold register for sensor 1, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
// #define CAP1293_TOUCH_THRSH_2_REG 0x31  // (R/W) touch Threshold register for sensor 2, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
// #define CAP1293_TOUCH_THRSH_3_REG 0x32  // (R/W) touch Threshold register for sensor 3, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
#define CAP1293_NOISE_THRSH_REG   0x38  // (R/W) Noise Threshold register, 2bit value, see CAP1293_NOISE_THRSH_ENUM for 2bit contents

// Standby Configuration registers:
#define CAP1293_STBY_CHAN_EN_REG  0x40  // (R/W) Standby Channel (enabled) register, 3LSBits are 3 sensor channels respectively
#define CAP1293_STBY_CONF_REG     0x41  // (R/W) Standby Configuration register, see defined _bits below
#define CAP1293_STBY_SENS_REG     0x42  // (R/W) (DELTA_SENSE for Standby mode!) Standby Sensitivity register, 3bit value to determine (in Standby mode) sensor data right-shift, see DELTA_SENSE documentation
#define CAP1293_STBY_THRSH_REG    0x43  // (R/W) Standby Threshold register, count-delta threshold that constitutes a touch, 7bit value

// Base Count Regsiters:
#define CAP1293_BASE_COUNT_REG_1  0x50  // (R) sensor 1 Base Count (reference count value) register, default = 0xC8
// #define CAP1293_BASE_COUNT_1_REG  0x50  // (R) sensor 1 Base Count (reference count value) register, default = 0xC8
// #define CAP1293_BASE_COUNT_2_REG  0x51  // (R) sensor 2 Base Count (reference count value) register, default = 0xC8
// #define CAP1293_BASE_COUNT_3_REG  0x52  // (R) sensor 3 Base Count (reference count value) register, default = 0xC8

// Power Button Regsiters:
#define CAP1293_PWR_BTN_SEL_REG   0x60  // (R/W) Power Button (selection) register, 2bit(?) value, can be 0, 1 or 2. The datasheet does not mention any scenario where the 3rd bit is nonzero
#define CAP1293_PWR_BTN_CONF_REG  0x61  // (R/W) Power Button Configuration register, see defined _bits below

// Calibration Regsiters:
#define CAP1293_CALIB_CONF_REG    0x80  // (R/W) Calibration Sensitivity Configuration register, see defined _bits below
#define CAP1293_CALIB_MSB_REG_1   0xB1  // (R) sensor 1 Calibration MSBs register
// #define CAP1293_CALIB_1_MSB_REG   0xB1  // (R) sensor 1 Calibration MSBs register
// #define CAP1293_CALIB_2_MSB_REG   0xB2  // (R) sensor 2 Calibration MSBs register
// #define CAP1293_CALIB_3_MSB_REG   0xB3  // (R) sensor 3 Calibration MSBs register
#define CAP1293_CALIB_x_LSB_REG   0xB9  // (R) all sensor inputs Calibration LSBs register

// ID Regsiters: (Read only)
#define CAP1293_PROD_ID_REG       0xFD  // (R) Product ID register, should return 0x6F
#define CAP1293_MF_ID_REG         0xFE  // (R) Manufacturer ID register, should return 0x5D
#define CAP1293_REV_REG           0xFF  // (R) Revision register, should return 0x00 ??
static const uint8_t CAP1293_ID_REG_DEFAULTS[3] = {0x6F, 0x5D, 0x00}; // default values for product ID, manufacturer ID and revision number


// General Status register bits: (read only?)
#define CAP1293_GEN_STATUS_BC_OUT_bits    0b01000000 // BC_OUT indicates base count Out Of Limit for one or more sensor inputs
#define CAP1293_GEN_STATUS_ACAL_FAIL_bits 0b00100000 // ACAL_FAIL indicates analog calibration faillure for one or more sensor inputs
#define CAP1293_GEN_STATUS_PWR_bits       0b00010000 // PWR is the output flag for the Power-Button feature. Linked to the INT bit (main control register) in some way
#define CAP1293_GEN_STATUS_MULT_bits      0b00000100 // MULT indicates that Multiple Touch output ("blocking"), INT bit should not be affected
#define CAP1293_GEN_STATUS_MTP_bits       0b00000010 // MTP indicates that the Multiple Touch Pattern conditions were met (threshold crossed / pattern acceptable) for INT bit behaviour, see MPT_ALERT bit
#define CAP1293_GEN_STATUS_TOUCH_bits     0b00000001 // TOUCH indicates a touch is detected for one or more sensor inputs (but you should really just check CAP1293_TOUCH_FLAG_REG)

// Sensitivity Control register bits:
#define CAP1293_SENS_ACTV_SENSE_bits      0b01110000 // DELTA_SENSE is a 3bit value which determines the sensor data right-shift, so sensitivity multiplier = (128>>(STBY_SENS))
#define CAP1293_SENS_BASE_SHIFT_bits      0b00001111 // BASE_SHIFT is a 4bit value which determines the base count left-shift, so scaling factor = (1<<min(BASE_SHIFT, 8))

// Main Control register:
#define CAP1293_MAIN_CTRL_GAIN_bits       0b11000000 // GAIN is analog capacitance amplification for sensor inputs (in Active mode), 2bit value where gain = (1<<(GAIN))
#define CAP1293_MAIN_CTRL_STBY_bits       0b00100000 // STBY power mode enable (instead of Active mode)
#define CAP1293_MAIN_CTRL_DSLEEP_bits     0b00010000 // DSLEEP power mode enable (overrules STBY and COMBO)
#define CAP1293_MAIN_CTRL_COMBO_GAIN_bits 0b00001100 // C_GAIN is analog capacitance amplification ONLY IN COMBO MODE for sensor inputs enabled in Standby mode, 2bit value where gain = (1<<(GAIN))
#define CAP1293_MAIN_CTRL_COMBO_bits      0b00000010 // COMBO power mode enable (overrules STBY)
#define CAP1293_MAIN_CTRL_INT_bits        0b00000001 // INT interrupt flag, controls ALERT pin (used to indicate several things). Must be cleared in order to clear other status flags (like CAP1293_TOUCH_FLAG_REG)
const uint8_t CAP1293_MAIN_CTRL_DEFAULT = 0b01000000;

// Configuration (1) register bits:
#define CAP1293_CONF_1_TIMOUT_bits        0b10000000 // TIMOUT enabled SMBus timeout feature. SMBus interface is reset if CLK is low for 30ms or ... (defualt disabled)
#define CAP1293_CONF_1_DIS_DIG_NOISE_bits 0b00100000 // DIS_DIG_NOISE i have truly no idea what this does, the datasheet is jut too vague (default 1)
#define CAP1293_CONF_1_DIS_LF_NOISE_bits  0b00010000 // DIS_ANA_NOISE disables low-freq noise hardware (which wants to discard measurements if noise is detected) (default enabled (0))
#define CAP1293_CONF_1_MAX_DUR_EN_bits    0b00001000 // MAX_DUR_EN enables auto-recalibration if a touch is detected for too long (see CAP1293_TIMING_CONF_MAX_DUR_bits) (default disabled)
const uint8_t CAP1293_CONF_1_DEFAULT =    0b00100000; // x_xxx___
// Configuration 2 register bits:
#define CAP1293_CONF_2_BC_OUT_RECAL_bits  0b01000000 // BC_OUT_RECAL enabled retying analog-calib. (instead of falling back to OOL-base-count) when BC_OUTx bit is set (default enabled)
#define CAP1293_CONF_2_BLK_PWR_CTRL_bits  0b00100000 // BLK_PWR_CTRL determines whether to reduce power consumption near end of sensing cycle (default enabled)
#define CAP1293_CONF_2_BC_OUT_INT_bits    0b00010000 // BC_OUT_INT enables sending an interrupt if base count is Out Of Limit for one or more sensor inputs (default disabled)
#define CAP1293_CONF_2_DIS_LF_FLAG_bits   0b00001000 // SHOW_RF_NOISE prevent Low-Freq noise hardware from setting the noise interrupt bit (default disabled, both noise kinds trigger noise bit)
#define CAP1293_CONF_2_DIS_RF_NOISE_bits  0b00000100 // DIS_RF_NOISE disables RF noise hardware (which wants to discard measurements if noise is detected) (default enabled (0))
#define CAP1293_CONF_2_ACAL_FAIL_INT_bits 0b00000010 // ACAL_FAIL_INT enables sending an interrupt if analog calibration fails for one or more sensor inputs (default disabled)
#define CAP1293_CONF_2_REL_INT_bits       0b00000001 // INT_REL_n disables sending an interrupt when a touch is released (default enabled)
const uint8_t CAP1293_CONF_2_DEFAULT =    0b01000000; // _xxxxxxx

// Signal Guard Enable register bits:
#define CAP1293_GUARD_EN_FOR_3_bits       0b00000100 // CS3_SG_EN enables signal guard feature around sensor input 3 (input 2 is connected to guard pad)
#define CAP1293_GUARD_EN_FOR_1_bits       0b00000001 // CS1_SG_EN enables signal guard feature around sensor input 1 (input 2 is connected to guard pad)

// Sensor Input Configuration (1) register bits:
#define CAP1293_TIMING_CONF_MAX_DUR_bits  0b11110000 // MAX_DUR determines the how long a sensor needs to detect a touch before it start auto-recalibration, see CAP1293_MAX_DUR_RECAL_ENUM for 4bit contents 
#define CAP1293_TIMING_CONF_RPT_RATE_bits 0b00001111 // RPT_RATE determines time between repeat-touch (interrupt) events, repeat rate = (35ms*(RPT_RATE_bits+1)), default = 175ms

// Averaging and Sampling Configuration register bits:
#define CAP1293_SAMP_CONF_AVG_bits        0b01110000 // AVG determines number of samples to be averaged (in Active mode) (to form one measurement), 3bit value where number_of_samples = (1<<(STBY_AVG))
#define CAP1293_SAMP_CONF_SAMP_TM_bits    0b00001100 // SAMP_TIME determines the (single-)sample time (in Active mode), see CAP1293_SAMP_TM_ENUM for 2bit contents 
#define CAP1293_SAMP_CONF_CYCL_TM_bits    0b00000011 // CYCLE_TIME determines the desired sensing cycle time (in Active mode), cycle time = (35ms*(TM_bits+1))

// Mutliple Touch Configuration register bits:
#define CAP1293_MULT_CONF_EN_bits         0b10000000 // MULT_BLK_EN enables the Multiple Touch circuitry
#define CAP1293_MULT_CONF_TH_bits         0b00001100 // B_MULT_T determines how many sensor inputs must be (simultaniously) touched to trigger an MT event. 2bit value, threshold = min(_TH_bits+1,3)
// Mutliple Touch Pattern Configuration register bits:
#define CAP1293_PTRN_CONF_MTP_EN_bits     0b10000000 // MTP_EN enables the Multiple Touch Pattern
#define CAP1293_PTRN_CONF_MTP_TH_bits     0b00001100 // MTP_TH sets the threshold (% of (Active/Standby) sensor input threshold) for MTP touches(???), see CAP1293_MTP_THRSH_ENUM for 2bit contents
#define CAP1293_PTRN_CONF_SOME_ANY_bits   0b00000010 // COMP_PTRN determines (?) whether specific sensor inputs-, or whether some number of (any) sensor inputs must be touched to trigger MTP event
#define CAP1293_PTRN_CONF_INT_EN_bits     0b00000001 // MTP_ALERT enables sending an interrupt upon MTP event trigger

// Recalibration Configuration register bits:
#define CAP1293_RCLB_CONF_WR_ALL_THR_bits 0b10000000 // BUT_LD_TH bit, if 1 (default) then updating _ACTV_THRSH_1 will update them all, if 0 it only updates the threshold for sensor 1
// #define CAP1293_RCLB_CONF_CLR_INTD_bits   0b01000000 // NO_CLR_INTD bit, if 0 (default) the noise status bit being set also clears 'accumulation of intermediate data'   ????
// #define CAP1293_RCLB_CONF_CLR_NEG_bits    0b00100000 // NO_CLR_NEG bit, if 0 (default) the noise status bit being set also clears 'e consecutive negative delta counts counter'  ????
#define CAP1293_RCLB_CONF_INTD_NEG_bits   0b01100000 // NO_CLR_INTD and NO_CLR_NEG bits (default=00), determine some internal response to noise flag being set. See datasheet page 40
#define CAP1293_RCLB_CONF_NEGDLT_CNT_bits 0b00011000 // NEG_DELTA_CNT determines how many consecutive negative delta counts triggers (digital) recalibration, see CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM for 2bit contents
#define CAP1293_RCLB_CONF_CAL_CFG_bits    0b00000111 // CAL_CFG determines the number of samples and cycles for calibration, see CAP1293_RCLB_CONF_CAL_CFG_ENUM for 3bit contents

// Standby Configuration register bits:
#define CAP1293_STBY_CONF_AVG_SUM_bits    0b10000000 // AVG_SUM determines Standby-enabled-sensor-input values to be based on Average value OR Summation(/total-count) value (default == average)
#define CAP1293_STBY_CONF_SAMP_CNT_bits   0b01110000 // STBY_AVG determines samples taken per channel (consequitive, not paralel), 3bit value where number_of_samples = (1<<(STBY_AVG))
#define CAP1293_STBY_CONF_SAMP_TM_bits    0b00001100 // STB_SAMP_TIME determines the (single-)sample time (in Standby mode), see CAP1293_SAMP_TM_ENUM for 2bit contents 
#define CAP1293_STBY_CONF_CYCL_TM_bits    0b00000011 // STB_CY_TIME determines the desired sensing cycle time (in Standby mode), cycle time = (35ms*(TM_bits+1)) 

// Power Button Configuration register bits:
#define CAP1293_PWR_BTN_CONF_STBY_EN_bits 0b01000000 // STBY_PWR_EN enables the power button feature in Standby state
#define CAP1293_PWR_BTN_CONF_STBY_TM_bits 0b00110000 // STBY_PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents
#define CAP1293_PWR_BTN_CONF_ACTV_EN_bits 0b00000100 // PWR_EN enables the power button feature in Active state
#define CAP1293_PWR_BTN_CONF_ACTV_TM_bits 0b00000011 // PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents

// Calibration Sensitivity Configuration register bits:
#define CAP1293_CALIB_CONF_CALSEN_3_bits  0b00110000 // CALSEN3 is the calibration gain for sensor 3, see CAP1293_CALSEN_ENUM for 2bit contents
#define CAP1293_CALIB_CONF_CALSEN_2_bits  0b00001100 // CALSEN2 is the calibration gain for sensor 2, see CAP1293_CALSEN_ENUM for 2bit contents
#define CAP1293_CALIB_CONF_CALSEN_1_bits  0b00000011 // CALSEN1 is the calibration gain for sensor 1, see CAP1293_CALSEN_ENUM for 2bit contents

static const uint8_t CAP1293_CYCL_TM_LIMITS[2] = {35, 140}; // (milliseconds) sensing cycle time (in millis) limits (both Active and Standby). 2bit values are used, time = 35*(1+bits)
static const uint16_t CAP1293_RPT_RATE_LIMITS[2] = {35, 560}; // (milliseconds) Repeat Rate (RPT_RATE in _TIMING_CONF) can be set to between 35ms and 560ms
static const uint16_t CAP1293_HOLD_THRSH_LIMITS[2] = {35, 560}; // (milliseconds) press-and-hold time threshold can be set to between 35ms and 560ms

// these are basically just aliases of the _bits versions. Just use those directly...
// enum CAP1293_mode_ENUM : uint8_t { // 
//   CAP1293_mode_Active = 0b00000000, // all mode bits set to 0 means active mode
//   CAP1293_mode_Standby = CAP1293_MAIN_CTRL_STBY_bits, // only Standby bit
//   CAP1293_mode_Combo = CAP1293_MAIN_CTRL_STBY_bits, // Combo bit overrules Standby- and Active modes
//   CAP1293_mode_DeepSleep = CAP1293_MAIN_CTRL_DSLEEP_bits // Deep-Skeep overrules all other modes
// }
// static const uint8_t CAP1293_mode_MASK = CAP1293_MAIN_CTRL_STBY_bits | CAP1293_MAIN_CTRL_STBY_bits | CAP1293_MAIN_CTRL_DSLEEP_bits; // only these bits should be affected

enum CAP1293_MAX_DUR_RECAL_ENUM : uint8_t { // 4bit value to determine how long a touch can be detected before it triggers an auto-recalibration
  CAP1293_MAX_DUR_RECAL_560, // 560ms
  CAP1293_MAX_DUR_RECAL_840, // 840ms
  CAP1293_MAX_DUR_RECAL_1120, // 1120ms
  CAP1293_MAX_DUR_RECAL_1400, // 1400ms
  CAP1293_MAX_DUR_RECAL_1680, // 1680ms
  CAP1293_MAX_DUR_RECAL_2240, // 2240ms
  CAP1293_MAX_DUR_RECAL_2800, // 2800ms
  CAP1293_MAX_DUR_RECAL_3360, // 3360ms
  CAP1293_MAX_DUR_RECAL_3920, // 3920ms
  CAP1293_MAX_DUR_RECAL_4480, // 4480ms
  CAP1293_MAX_DUR_RECAL_5600, // 5600ms (default)
  CAP1293_MAX_DUR_RECAL_6720, // 6720ms
  CAP1293_MAX_DUR_RECAL_7840, // 7840ms
  CAP1293_MAX_DUR_RECAL_8960, // 8960ms(?) typo in datasheet??? it sais 8906, but 8960 makes much more sense in the sequence of options
  CAP1293_MAX_DUR_RECAL_10080, // 10080ms
  CAP1293_MAX_DUR_RECAL_11200, // 11200ms
}; // the general formula for this 4bit value is rather ugly: time_in_ms = (16 + min(x, 4)*8 + min(max(x-4, 0), 5)*16 + min(max(x-9, 0), 6)*32) * 35ms  (you can express the multiplications with 8,16,32 as bitshifts)

enum CAP1293_SAMP_TM_ENUM : uint8_t { // 2bit value to determine the (single-)sample time, used for SAMP_TM bits in both STBY_CONF and SAMP_CONF. Basically 320<<(2bit_value)
  CAP1293_SAMP_TM_320  = 0, // 320us
  CAP1293_SAMP_TM_640  = 1, // 640us
  CAP1293_SAMP_TM_1280 = 2, // 1.28ms (default)
  CAP1293_SAMP_TM_2560 = 3  // 2.56ms
};

enum CAP1293_MTP_THRSH_ENUM : uint8_t { // 2bit value to determine the Multiple Touch Pattern (MTP) threshold
  CAP1293_MTP_THRSH_125  = 0, // 12.5% (default)
  CAP1293_MTP_THRSH_250  = 1, // 25.0%
  CAP1293_MTP_THRSH_375  = 2, // 37.5%
  CAP1293_MTP_THRSH_1000 = 3  // 100%   (effectively making it the same as non-Pattern Multiple Touch code(????))
};

enum CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM : uint8_t { // 2bit value to determine how many consecutive negative delta counts triggers (digital) recalibration
  CAP1293_RCLB_CONF_NEGDLT_CNT_8   = 0, // 8 consecutive negative delta counts
  CAP1293_RCLB_CONF_NEGDLT_CNT_16  = 1, // 16 consecutive negative delta counts (default)
  CAP1293_RCLB_CONF_NEGDLT_CNT_32  = 2, // 32 consecutive negative delta counts
  CAP1293_RCLB_CONF_NEGDLT_CNT_NONE= 3  // disable NEG_DELTA_CNT (don't trigger automatic (digital) recalibration)
};
enum CAP1293_RCLB_CONF_CAL_CFG_ENUM : uint8_t { // 3bit value to determine both the number of samples used for (digital?) calibration, and how long it should (ideally) go on for
  CAP1293_RCLB_CONF_CAL_CFG_16_16    = 0, // 16 samples, 16 sens-cycle-periods
  CAP1293_RCLB_CONF_CAL_CFG_32_32    = 1, // 32 samples, 32 sens-cycle-periods
  CAP1293_RCLB_CONF_CAL_CFG_64_64    = 2, // 64 samples, 64 sens-cycle-periods (default)
  CAP1293_RCLB_CONF_CAL_CFG_128_128  = 3, // 128 samples, 128 sens-cycle-periods
  CAP1293_RCLB_CONF_CAL_CFG_256_256  = 4, // 256 samples, 256 sens-cycle-periods
  CAP1293_RCLB_CONF_CAL_CFG_256_1024 = 5, // 256 samples, 1024 sens-cycle-periods (768 cycles settling time(?))
  CAP1293_RCLB_CONF_CAL_CFG_256_2048 = 6, // 256 samples, 2048 sens-cycle-periods (1792 cycles settling time(?))
  CAP1293_RCLB_CONF_CAL_CFG_256_4096 = 7  // 256 samples, 4096 sens-cycle-periods (3840 cycles settling time(?))
};

enum CAP1293_NOISE_THRSH_ENUM : uint8_t { // 2bit value to determine the Noise Threshold (percentage). Basically 25+12.5*(2bit_value)
  CAP1293_NOISE_THRSH_250 = 0, // 25.0%
  CAP1293_NOISE_THRSH_375 = 1, // 37.5% (default)
  CAP1293_NOISE_THRSH_500 = 2, // 50.0%
  CAP1293_NOISE_THRSH_625 = 3  // 62.5%
};

enum CAP1293_PWR_BTW_TM_ENUM : uint8_t { // 2bit value to determine the time the power button must be held for the power button bit & interrupt to trigger. Basically 280<<(2bit_value)
  CAP1293_PWR_BTW_TM_280  = 0, // 280ms hold time
  CAP1293_PWR_BTW_TM_560  = 1, // 560ms hold time
  CAP1293_PWR_BTW_TM_1120 = 2, // 1.12sec hold time (default)
  CAP1293_PWR_BTW_TM_2240 = 3  // 2.24sec hold time
};

enum CAP1293_CALSEN_ENUM : uint8_t { // 2bit value to determine the Calibration Sensitivity (gain)
  CAP1293_CALSEN_5_50p  = 0, // 5~50 pF (default)
  CAP1293_CALSEN_0_25p  = 1, // 0~25 pF
  CAP1293_CALSEN_0_12p5 = 2, // 0~12.5 pF
};


#include "_CAP1293_thijs_base.h" // this file holds all the nitty-gritty low-level stuff (I2C implementations (platform optimizations))
/**
 * An I2C interfacing library for the CAP1293 touch sensor
 * 
 * features the option to use the Wire.h library, or optimized code for the following platforms:
 * atmega328p (direct register manipulation)
 * ESP32 (below HAL layer, but not lowest level)
 * MSP430 (through Energia(?) middle layer)
 * STM32 (through twi->HAL layers)
 */
class CAP1293_thijs : public _CAP1293_thijs_base
{
  public:
  //private:
  uint8_t _readBuff; // used for user-friendly functions (saves a tiny by of CPU overhead for initialization OR a significant degree of IO overhead if used as cache (perilous))
  uint8_t _readBuffAddress = CAP1293_INVALID_REG_ADDR; // indicates the memory address the _readBuff stores. Use with great caution, and only if speed is an absolute necessity!
  public:
  using _CAP1293_thijs_base::_CAP1293_thijs_base;
  /*
  This class only contains the higher level functions.
   for the base functions, please refer to _CAP1293_thijs_base.h
  here is a brief list of all the lower-level functions:
  - init()
  - requestReadBytes()
  - onlyReadBytes()
  - writeBytes()
  */
  //// the following functions are abstract enough that they'll work for either architecture

  public:

  /**
   * (just a macro) check whether an CAP1293_ERR_RETURN_TYPE (which may be one of several different types) is fine or not 
   * @param err (bool or esp_err_t or i2c_status_e, see on defines at top)
   * @return whether the error is fine
   */
  bool _errGood(CAP1293_ERR_RETURN_TYPE err) { return(err == CAP1293_ERR_RETURN_TYPE_OK); }
  //   #if defined(CAP1293_return_esp_err_t)     // these few commented lines were replaced with the one above, but still serve to show how the error system works:
  //     return(err == ESP_OK);
  //   #elif defined(CAP1293_return_i2c_status_e)
  //     return(err == I2C_OK);
  //   #else
  //     return(err);
  //   #endif
  // }

  /**
   * (private) overwrite a portion (mask) of a register without affecting the rest of it
   * @param regAddress register byte (see list of defines at top)
   * @param newVal value (partial byte) to write into the block
   * @param mask which bits to affect
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE _setRegBits(uint8_t regAddress, uint8_t newVal, uint8_t mask, bool useCache=false) {
    CAP1293_ERR_RETURN_TYPE err = CAP1293_ERR_RETURN_TYPE_OK;
    if( ! (useCache && (regAddress == _readBuffAddress))) { // if cache can't be used (normally true)
      err = requestReadBytes(regAddress, &_readBuff, 1); _readBuffAddress = regAddress; // read into cache
      if(!_errGood(err)) { CAP1293debugPrint("_setRegBits() read/write error!"); _readBuffAddress = CAP1293_INVALID_REG_ADDR; return(err); }
    }
    _readBuff &= ~mask;            // excise old data
    _readBuff |= (newVal & mask);  // insert new data
    err = writeBytes(regAddress, &_readBuff, 1);
    if(!_errGood(err)) { CAP1293debugPrint("_setRegBits() write error!"); }
    return(err);
  }
  /**
   * (private) overwrite a portion (mask) of a register without affecting the rest of it
   * @param regAddress register byte (see list of defines at top)
   * @param newVal boolean to write into the block
   * @param mask which bits to affect
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE _setRegOneBit(uint8_t regAddress, bool newVal, uint8_t mask, bool useCache=false) { return(_setRegBits(regAddress, newVal ? mask : 0, mask, useCache)); } // (just a macro)
  /**
   * (private) overwrite a sensor-input-channel bit specifically
   * @param regAddress register byte (see list of defines at top)
   * @param newVal boolean to write into the block
   * @param mask which bits to affect
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE _setChanBit(uint8_t regAddress, bool newVal, uint8_t sensorIndex, bool useCache=false) {
    return(_setRegOneBit(regAddress, newVal, CAP1293_channel_bit(CAP1293_constr_index(sensorIndex)), useCache)); } // (just a macro)

  /**
   * (private) request a specific register and read 1 byte into _readBuff (optional cache)
   * @param regAddress register byte (see list of defines at top)
   * @param readBuff byte pointer to put the read data in (note: will be a copy of _readBuff). If NULL, use _readBuff
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE _getRegRetErr(uint8_t regAddress, uint8_t* readBuff=NULL, bool useCache=false) {
    if( ! (useCache && (regAddress == _readBuffAddress))) { // if cache can't be used (normally true)
      CAP1293_ERR_RETURN_TYPE err = requestReadBytes(regAddress, &_readBuff, 1); _readBuffAddress = regAddress; // read into cache
      if(!_errGood(err)) { CAP1293debugPrint("_getRegRetErr read/write error: "); _readBuffAddress = CAP1293_INVALID_REG_ADDR; return(err); }
    } // else, the cache already has what you need (and you specified the cache could be used), so skip reading entirely
    if(readBuff) { *readBuff = _readBuff; } // copy results to the user-specified location
    return(CAP1293_ERR_RETURN_TYPE_OK);
  }
  /**
   * (private) request a specific register and read 1 byte (optional cache)
   * @param regAddress register byte (see list of defines at top)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the byte it read (unless the read failed!)
   */
  uint8_t _getRegRetVal(uint8_t regAddress, bool useCache=false) { // (mostly a macro)
    if( ! (useCache && (regAddress == _readBuffAddress))) { // if cache can't be used (normally true)
      CAP1293_ERR_RETURN_TYPE err = requestReadBytes(regAddress, &_readBuff, 1); _readBuffAddress = regAddress; // read into cache
      if(!_errGood(err)) { CAP1293debugPrint("_getRegRetVal read/write error: "); _readBuffAddress = CAP1293_INVALID_REG_ADDR; return(err); }
    }
    return(_readBuff);
  }
  /**
   * (private) retrieve one bit from a register
   * @param mask which bit to retrieve
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the bit (as a boolean)
   */
  bool _getRegOneBit(uint8_t regAddress, uint8_t mask, bool useCache=false) { return((_getRegRetVal(regAddress, useCache) & CAP1293_MAIN_CTRL_STBY_bits) != 0); } // (just a macro)
  /**
   * (private) overwrite a sensor-input-channel bit specifically
   * @param regAddress register byte (see list of defines at top)
   * @param newVal boolean to write into the block
   * @param mask which bits to affect
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  bool _getChanBit(uint8_t regAddress, uint8_t sensorIndex, bool useCache=false) {
    return(_getRegOneBit(regAddress, CAP1293_channel_bit(CAP1293_constr_index(sensorIndex)), useCache)); } // (just a macro)

  // uint16_t _factor35(uint8_t mult) { return(35*(1+mult)); } // lots of timing values in this sensor use multiples of 35 (but even i cannot bring myself to make it a dedicated function)
  // uint8_t _divide35(uint16_t bigVal, const uint16_t limits[]) { return((constrain(bigVal, limits[0], limits[1]) / 35) - 1); }
  // uint8_t findMSBit(uint8_t powOfTwo) { if(!powOfTwo) {return(0);} for(uint8_t i=1; i<8; i++) { if((powOfTwo>>i)==0) {return(i-1);} } return(7); } // (hopefully efficient) find largest bit set to 1

/////////////////////////////////////////////////////////////////////////////////////// get functions: //////////////////////////////////////////////////////////
  
  /**
   * retrieve Main Control register (this version of the function lets you check for I2C errors)
   * @param readBuff byte pointer to put the result in (if NULL, use class._readBuff). See CAP1293_MAIN_CTRL_x_bits for contents
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getMainControl(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_MAIN_CTRL_REG, readBuff, useCache)); } // (just a macro)
  /**
   * retrieve Main Control register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the Main Control byte. See CAP1293_MAIN_CTRL_x_bits for contents
   */
  uint8_t getMainControl(bool useCache=false) { return(_getRegRetVal(CAP1293_MAIN_CTRL_REG, useCache)); } // (just a macro)
  /**
   * retrieve GAIN bits from the Main Control register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the GAIN bits, amplification = (1<<(GAIN))
   */
  uint8_t getMainControl_GAIN(bool useCache=false) { return((_getRegRetVal(CAP1293_MAIN_CTRL_REG, useCache) & CAP1293_MAIN_CTRL_GAIN_bits) >> 6); } // (just a macro)
  /**
   * retrieve STBY bit from the Main Control register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the STBY bit, 1 means Standby power mode is active (unless overruled by Combo or Deep Sleep)
   */
  bool getMainControl_STBY(bool useCache=false) { return(_getRegOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_STBY_bits, useCache)); } // (just a macro)
  /**
   * retrieve DSLEEP bit from the Main Control register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the STBY bit, 1 means Deep Sleep power mode is active
   */
  bool getMainControl_DSLEEP(bool useCache=false) { return(_getRegOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_DSLEEP_bits, useCache)); } // (just a macro)
  /**
   * retrieve C_GAIN (ONLY IN COMBO MODE, used for Standby-enabled-sensors) bits from the Main Control register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the C_GAIN bits, amplification = (1<<(GAIN))
   */
  uint8_t getMainControl_COMBO_GAIN(bool useCache=false) { return((_getRegRetVal(CAP1293_MAIN_CTRL_REG, useCache) & CAP1293_MAIN_CTRL_COMBO_GAIN_bits) >> 2); } // (just a macro)
  /**
   * retrieve DSLEEP bit from the Main Control register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the STBY bit, 1 means Combo power mode is active (unless overruled by Deep Sleep)
   */
  bool getMainControl_COMBO(bool useCache=false) { return(_getRegOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_COMBO_bits, useCache)); } // (just a macro)
  /**
   * retrieve INT bit from the Main Control register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the INT bit, directly relates to ALERT pin
   */
  bool getMainControl_INT(bool useCache=false) { return(_getRegOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_INT_bits, useCache)); } // (just a macro)

  // CAP1293_mode_ENUM getMode(bool useCache=false) { // probably does work, but i'm on the fence about whether it's actually useful
  //   uint8_t mainControl = getMainControl(useCache);
  //   if((mainControl & CAP1293_mode_DeepSleep) != 0) { return(CAP1293_mode_DeepSleep); }
  //   else if((mainControl & CAP1293_mode_Combo) != 0) { return(CAP1293_mode_Combo); }
  //   else if((mainControl & CAP1293_mode_Standby) != 0) { return(CAP1293_mode_Standby); }
  //   /* else */ return(CAP1293_mode_Active);
  // }

  /**
   * retrieve General Status register (this version of the function lets you check for I2C errors)
   * @param readBuff byte pointer to put the result in (if NULL, use class._readBuff). See CAP1293_GEN_STATUS_x_bits for contents
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getGenStatus(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_GEN_STATUS_REG, readBuff, useCache)); } // (just a macro)
  /**
   * retrieve General Status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the General Status byte. See CAP1293_GEN_STATUS_x_bits for contents
   */
  uint8_t getGenStatus(bool useCache=false) { return(_getRegRetVal(CAP1293_GEN_STATUS_REG, useCache)); } // (just a macro)
  /**
   * retrieve BC_OUT bit from the General Status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the BC_OUT bit, indicating Base Count is Out Of Limit
   */
  bool getGenStatus_BC_OUT(bool useCache=false) { return(_getRegOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_BC_OUT_bits, useCache)); } // (just a macro)
  /**
   * retrieve ACAL_FAIL bit from the General Status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the ACAL_FAIL bit, indicating Analog Calibration faillure
   */
  bool getGenStatus_ACAL_FAIL(bool useCache=false) { return(_getRegOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_ACAL_FAIL_bits, useCache)); } // (just a macro)
  /**
   * retrieve PWR bit from the General Status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the PWR bit, indicating Power-Button was detected (conditions were met)
   */
  bool getGenStatus_PWR(bool useCache=false) { return(_getRegOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_PWR_bits, useCache)); } // (just a macro)
  /**
   * retrieve MULT bit from the General Status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the MULT bit, indicating Multiple-Touch was detected (conditions were met)
   */
  bool getGenStatus_MULT(bool useCache=false) { return(_getRegOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_MULT_bits, useCache)); } // (just a macro)
  /**
   * retrieve MTP bit from the General Status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the MTP bit, indicating Multiple-Touch-Pattern was detected (conditions were met)
   */
  bool getGenStatus_MTP(bool useCache=false) { return(_getRegOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_MTP_bits, useCache)); } // (just a macro)
  /**
   * retrieve TOUCH bit from the General Status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the TOUCH bit, indicating a touch was detected (see touch flags register for which one)
   */
  bool getGenStatus_TOUCH(bool useCache=false) { return(_getRegOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_TOUCH_bits, useCache)); } // (just a macro)
  /**
   * retrieve the (individual) touch flags/status register (this version of the function lets you check for I2C errors)
   * @param readBuff byte pointer to put the result in (if NULL, use class._readBuff). The 3 LSBits are the 3 sensor inputs respectively, see CAP1293_channel_bit()
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getTouchFlags(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_TOUCH_FLAG_REG, readBuff, useCache)); } // (just a macro)
  /**
   * retrieve the (individual) touch flags/status byte
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the touch flags byte. The 3 LSBits are the 3 sensor inputs respectively, see CAP1293_channel_bit()
   */
  uint8_t getTouchFlags(bool useCache=false) { return(_getRegRetVal(CAP1293_TOUCH_FLAG_REG, useCache)); } // (just a macro)
  /**
   * retrieve the noise flags/status register (this version of the function lets you check for I2C errors)
   * @param readBuff byte pointer to put the result in (if NULL, use class._readBuff). The 3 LSBits are the 3 sensor inputs respectively, see CAP1293_channel_bit()
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getNoiseFlagAll(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_NOISE_FLAG_REG, readBuff, useCache)); } // (just a macro)
  /**
   * retrieve the noise flags/status register
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the noise flags byte. The 3 LSBits are the 3 sensor inputs respectively, see CAP1293_channel_bit()
   */
  uint8_t getNoiseFlagAll(bool useCache=false) { return(_getRegRetVal(CAP1293_NOISE_FLAG_REG, useCache)); } // (just a macro)
  /**
   * retrieve the cnoise flags/status for just 1 sensor input (recommend using getNoiseFlags() instead)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the noise flag for 1 input
   */
  bool getNoiseFlag(uint8_t sensorIndex, bool useCache=false) { return(_getChanBit(CAP1293_NOISE_FLAG_REG, sensorIndex, useCache)); } // (just a macro)

  //note: 3LSBits CAP1293_channel_bit(), see set function of different name    // (R/W) Calibration Activate and Status register (per sensor), 3LSBits are 3 sensor channels respectively. Read for status/faillure
  /**
   * retrieve the calibration-active flags/status register (this version of the function lets you check for I2C errors).
   * (R/W) Calibration Activate and Status register (per sensor), 3LSBits are 3 sensor channels respectively. Read for status/faillure
   * @param readBuff byte pointer to put the result in (if NULL, use class._readBuff). The 3 LSBits are the 3 sensor inputs respectively, see CAP1293_channel_bit()
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getCalibOngoingAll(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_CALIB_ACTIV_REG, readBuff, useCache)); } // (just a macro)
  /**
   * retrieve the calibration-active flags/status byte.
   * (R/W) Calibration Activate and Status register (per sensor), 3LSBits are 3 sensor channels respectively. Read for status/faillure
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the calibration-active flags byte. The 3 LSBits are the 3 sensor inputs respectively, see CAP1293_channel_bit()
   */
  uint8_t getCalibOngoingAll(bool useCache=false) { return(_getRegRetVal(CAP1293_CALIB_ACTIV_REG, useCache)); } // (just a macro)
  /**
   * retrieve the calibration-active flags/status for just 1 sensor input (recommend using getCalibOngoingAll() instead)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the calibration-active flags for 1 input
   */
  bool getCalibOngoing(uint8_t sensorIndex, bool useCache=false) { return(_getChanBit(CAP1293_CALIB_ACTIV_REG, sensorIndex, useCache)); } // (just a macro)

  //note: 8bit signed, will be compared against TouchThreshRaw
  CAP1293_ERR_RETURN_TYPE getDeltaCountRaw(uint8_t sensorIndex, int8_t* readBuff, bool useCache=false) {
    return(_getRegRetErr(CAP1293_DELTA_COUNT_REG_1 + CAP1293_constr_index(sensorIndex), (uint8_t*)readBuff, 1));
  }
  int8_t getDeltaCountRaw(uint8_t sensorIndex, bool useCache=false) { return(_getRegRetVal(CAP1293_DELTA_COUNT_REG_1 + CAP1293_constr_index(sensorIndex), useCache)); }
  int16_t calcDeltaCountFull(int8_t deltaCountRaw, uint8_t deltaSense) { return((int16_t)deltaCountRaw << (7-deltaSense)); }
  //note: not recommended!, use calcDeltaCountFull directly intead (unless sensitivity changes without your knowledge).  inefficient example: getDeltaCountFull(i, _checkIsInStandbyMode(i));
  int16_t getDeltaCountFull(uint8_t sensorIndex, bool inputInStandbyMode) { // (just a macro)
    return(calcDeltaCountFull(getDeltaCountRaw(sensorIndex), inputInStandbyMode ? getStandbySensitivity() : getActiveSensitivity()));
  }
  //note: not recommended, quite inefficient and completely unnecessary if the users code is any good.  returns true if standby mode is used for sensor
  uint8_t _checkIsInStandbyMode(uint8_t sensorIndex) {
    uint8_t mainControlReg = getMainControl(); // indicates active mode(s)
    if((mainControlReg & CAP1293_MAIN_CTRL_COMBO_bits) != 0) { // Combo mode overrules Standby and Active modes
      // according to the datasheet: "If a sensor input is enabled in both the Active state and in the Standby state, the Active state settings will be used in Combo state"
      return(!getActiveChannelEnable(sensorIndex)); // if the input's bit is enabled in Active mode return 0, otherwise return 1 (sensor may be disabled in both)
    } else if((mainControlReg & CAP1293_MAIN_CTRL_COMBO_bits) != 0) { // if the sensor is in Stanby mode (only)
      return(true); // NOTE: if the sensor is not enabled in standby mode this will be meaningless
    } /*else*/ return(false); // neither Combo mode, nor Standby mode is enabled. The sensor must be in Active mode
  }

  CAP1293_ERR_RETURN_TYPE getSensReg(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_SENS_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getSensReg(bool useCache=false) { return(_getRegRetVal(CAP1293_SENS_REG, useCache)); } // (just a macro)
  //note: refer to DELTA_SENSE documentation
  uint8_t getActiveSensitivity(bool useCache=false) { return((getSensReg(useCache) & CAP1293_SENS_ACTV_SENSE_bits) >> 4); }
  uint8_t getBaseShift(bool useCache=false) { return(getSensReg(useCache) & CAP1293_SENS_BASE_SHIFT_bits); }
  
  CAP1293_ERR_RETURN_TYPE getConf_1(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_CONF_1_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getConf_1(bool useCache=false) { return(_getRegRetVal(CAP1293_CONF_1_REG, useCache)); } // (just a macro)
  bool getConf_1_TIMEOUT(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_1_REG, CAP1293_CONF_1_TIMOUT_bits, useCache)); } // (just a macro)
  bool getConf_1_DIS_DIG_NOISE(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_1_REG, CAP1293_CONF_1_DIS_DIG_NOISE_bits, useCache)); } // (just a macro)
  bool getConf_1_DIS_LF_NOISE(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_1_REG, CAP1293_CONF_1_DIS_LF_NOISE_bits, useCache)); } // (just a macro)
  bool getConf_1_MAX_DUR_EN(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_1_REG, CAP1293_CONF_1_MAX_DUR_EN_bits, useCache)); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE getConf_2(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_CONF_2_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getConf_2(bool useCache=false) { return(_getRegRetVal(CAP1293_CONF_2_REG, useCache)); } // (just a macro)
  bool getConf_2_BC_OUT_RECAL(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_2_REG, CAP1293_CONF_2_BC_OUT_RECAL_bits, useCache)); } // (just a macro)
  bool getConf_2_BLK_PWR_CTRL(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_2_REG, CAP1293_CONF_2_BLK_PWR_CTRL_bits, useCache)); } // (just a macro)
  bool getConf_2_BC_OUT_INT(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_2_REG, CAP1293_CONF_2_BC_OUT_INT_bits, useCache)); } // (just a macro)
  //note: SHOW_RF_NOISE
  bool getConf_2_DIS_LF_FLAG(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_2_REG, CAP1293_CONF_2_DIS_LF_FLAG_bits, useCache)); } // (just a macro)
  bool getConf_2_DIS_RF_NOISE(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_2_REG, CAP1293_CONF_2_DIS_RF_NOISE_bits, useCache)); } // (just a macro)
  bool getConf_2_ACAL_FAIL_INT(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_2_REG, CAP1293_CONF_2_ACAL_FAIL_INT_bits, useCache)); } // (just a macro)
  //note: active LOW
  bool getConf_2_REL_INT(bool useCache=false) { return(_getRegOneBit(CAP1293_CONF_2_REG, CAP1293_CONF_2_REL_INT_bits, useCache)); } // (just a macro)

  //note: 3LSBits CAP1293_channel_bit()
  CAP1293_ERR_RETURN_TYPE getActiveChannelsEnabled(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_ACTV_CHAN_EN_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getActiveChannelsEnabled(bool useCache=false) { return(_getRegRetVal(CAP1293_ACTV_CHAN_EN_REG, useCache)); } // (just a macro)
  bool getActiveChannelEnable(uint8_t sensorIndex, bool useCache=false) { return(_getChanBit(CAP1293_ACTV_CHAN_EN_REG, sensorIndex, useCache)); } // (just a macro)
  //note: 3LSBits CAP1293_channel_bit(),   ALERT pin & INT bit
  CAP1293_ERR_RETURN_TYPE getInterruptsEnabled(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_ALERT_EN_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getInterruptsEnabled(bool useCache=false) { return(_getRegRetVal(CAP1293_ALERT_EN_REG, useCache)); } // (just a macro)
  bool getInterruptEnable(uint8_t sensorIndex, bool useCache=false) { return(_getChanBit(CAP1293_ALERT_EN_REG, sensorIndex, useCache)); } // (just a macro)
  //note: 3LSBits CAP1293_channel_bit(),   explain repeat briefly
  CAP1293_ERR_RETURN_TYPE getRepeatsEnabled(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_REPEAT_EN_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getRepeatsEnabled(bool useCache=false) { return(_getRegRetVal(CAP1293_REPEAT_EN_REG, useCache)); } // (just a macro)
  bool getRepeatEnable(uint8_t sensorIndex, bool useCache=false) { return(_getChanBit(CAP1293_REPEAT_EN_REG, sensorIndex, useCache)); } // (just a macro)
  //note: 1st and 3rd LSBits CAP1293_GUARD_EN_FOR_x_bits    refer to guard explanation/documentation
  CAP1293_ERR_RETURN_TYPE getGuardsEnabled(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_GUARD_EN_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getGuardsEnabled(bool useCache=false) { return(_getRegRetVal(CAP1293_GUARD_EN_REG, useCache)); } // (just a macro)
  bool getGuardEnable(uint8_t sensorIndex, bool useCache=false) { return(_getChanBit(CAP1293_GUARD_EN_REG, sensorIndex, useCache)); } // (just a macro)

  //note: new name
  CAP1293_ERR_RETURN_TYPE getTimingConfReg(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_TIMING_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getTimingConfReg(bool useCache=false) { return(_getRegRetVal(CAP1293_TIMING_CONF_REG, useCache)); } // (just a macro)
  //note: CAP1293_MAX_DUR_RECAL_ENUM, default is CAP1293_MAX_DUR_RECAL_5600
  CAP1293_MAX_DUR_RECAL_ENUM getMaxDurBits(bool useCache=false) { return(static_cast<CAP1293_MAX_DUR_RECAL_ENUM>((getRptRateBits(useCache) & CAP1293_TIMING_CONF_MAX_DUR_bits) >> 4)); }
  //note: default is 5600
  uint16_t _calcMaxDurMillis(CAP1293_MAX_DUR_RECAL_ENUM maxDurBits) {
    uint8_t maxDur = static_cast<uint8_t>(maxDurBits);
    // return((16 + min(maxDur, 4)*8 + min(max(maxDur-4, 0), 5)*16 + min(max(maxDur-9, 0), 6)*32) * 35);
    return((16 + (min(maxDur, (uint8_t)4)<<3) + ((maxDur>4) ? (min(maxDur-4, 5)<<4) : 0) + ((maxDur>9) ? (min(maxDur-9, 6)<<5) : 0)) * 35); // very ugly formula, but it should work
  }
  uint16_t getMaxDurMillis(bool useCache=false) { return(_calcMaxDurMillis(getMaxDurBits(useCache))); } // (just a macro)
  uint8_t getRptRateBits(bool useCache=false) { return(getTimingConfReg(useCache) & CAP1293_TIMING_CONF_RPT_RATE_bits); }
  //note: CAP1293_RPT_RATE_LIMITS, default is 175
  uint16_t getRptRateMillis(bool useCache=false) { return((getRptRateBits(useCache)+1) * 35); }
  
  CAP1293_ERR_RETURN_TYPE getHoldThresholdBits(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_HOLD_THRSH_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getHoldThresholdBits(bool useCache=false) { return(_getRegRetVal(CAP1293_HOLD_THRSH_REG, useCache)); } // (just a macro)
  //note: CAP1293_HOLD_THRSH_LIMITS, default is 280
  uint16_t getHoldThresholdMillis(bool useCache=false) { return((getHoldThresholdBits(useCache)+1) * 35); }

  //note: (new name)
  CAP1293_ERR_RETURN_TYPE getSampConfReg(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_SAMP_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getSampConfReg(bool useCache=false) { return(_getRegRetVal(CAP1293_SAMP_CONF_REG, useCache)); } // (just a macro)
  //note: for standby, see CAP1293_STBY_CONF_SAMP_CNT_bits,  // AVG determines number of samples to be averaged (in Active mode) (to form one measurement), 3bit value where number_of_samples = (1<<(STBY_AVG))
  uint8_t getAveragingCountBits(bool useCache=false) { return((getSampConfReg(useCache) & CAP1293_SAMP_CONF_AVG_bits) >> 4); }
  uint8_t getAveragingCount(bool useCache=false) { return(1 << getAveragingCountBits(useCache)); }
  //note: 320<<bits
  CAP1293_SAMP_TM_ENUM getActiveSampleTimeBits(bool useCache=false) { return(static_cast<CAP1293_SAMP_TM_ENUM>((getSampConfReg(useCache) & CAP1293_SAMP_CONF_SAMP_TM_bits) >> 2)); }
  //note: default is 1280us
  uint16_t getActiveSampleTimeMicros(bool useCache=false) { return(320 << getActiveSampleTimeBits(useCache)); }
  //note:    // CYCLE_TIME determines the desired sensing cycle time (in Active mode), cycle time = (35ms*(TM_bits+1)) 
  uint8_t getActiveSensCycleTimeBits(bool useCache=false) { return(getSampConfReg(useCache) & CAP1293_SAMP_CONF_CYCL_TM_bits); }
  //note: CAP1293_CYCL_TM_LIMITS, default is 70
  uint8_t getActiveSensCycleTimeMillis(bool useCache=false) { return((getActiveSensCycleTimeBits(useCache)+1) * 35); }

  CAP1293_ERR_RETURN_TYPE getMultipleTouchConfReg(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_MULT_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getMultipleTouchConfReg(bool useCache=false) { return(_getRegRetVal(CAP1293_MULT_CONF_REG, useCache)); } // (just a macro)
  bool getMultipleTouchEnabled(bool useCache=false) { return(_getRegOneBit(CAP1293_MULT_CONF_REG, CAP1293_MULT_CONF_EN_bits, useCache)); } // (just a macro)
  //note: max 3,  default is 1 // B_MULT_T determines how many sensor inputs must be (simultaniously) touched to trigger an MT event. 2bit value, threshold = min(_TH_bits+1,3)
  uint8_t getMultipleTouchThreshold(bool useCache=false) { return((getMultipleTouchConfReg(useCache) & CAP1293_MULT_CONF_TH_bits) >> 2); }

  CAP1293_ERR_RETURN_TYPE getMTPatternConfReg(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_PTRN_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getMTPatternConfReg(bool useCache=false) { return(_getRegRetVal(CAP1293_PTRN_CONF_REG, useCache)); } // (just a macro)
  bool getMTPatternEnabled(bool useCache=false) { return(_getRegOneBit(CAP1293_PTRN_CONF_REG, CAP1293_PTRN_CONF_MTP_EN_bits, useCache)); } // (just a macro)
  //note: default is CAP1293_MTP_THRSH_125  // MTP_TH sets the threshold (% of (Active/Standby) sensor input threshold) for MTP touches(???), see CAP1293_MTP_THRSH_ENUM for 2bit contents
  CAP1293_MTP_THRSH_ENUM getMTP_thresholdBits(bool useCache=false) { return(static_cast<CAP1293_MTP_THRSH_ENUM>((getMTPatternConfReg(useCache) & CAP1293_PTRN_CONF_MTP_TH_bits) >> 2)); }
  //note: 12.5, 25, 37.5, 100   default is 12.5
  float getMTP_thresholdPercent(bool useCache=false) { uint8_t TH_bits = static_cast<uint8_t>(getMTP_thresholdBits(useCache));  return((TH_bits < 3) ? (TH_bits * 12.5) : 100); } // 12.5, 25, 37.5, 100
  //note:    // COMP_PTRN determines (?) whether specific sensor inputs-, or whether some number of (any) sensor inputs must be touched to trigger MTP event
  bool getMTP_patternVScount(bool useCache=false) { return(_getRegOneBit(CAP1293_PTRN_CONF_REG, CAP1293_PTRN_CONF_SOME_ANY_bits, useCache)); } // (just a macro)
  bool getMTP_INT_EN(bool useCache=false) { return(_getRegOneBit(CAP1293_PTRN_CONF_REG, CAP1293_PTRN_CONF_INT_EN_bits, useCache)); } // (just a macro)
  //note: 3LSBits (count => bitshift)  // (R/W) Mutliple Touch Pattern (selection) register. Either it's which sensor inputs are- , or it's how MANY (aboslute) sensor inputs are  'part of the pattern'
  CAP1293_ERR_RETURN_TYPE getMTP_select_or_count(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_PTRN_SEL_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getMTP_select_or_countBits(bool useCache=false) { return(_getRegRetVal(CAP1293_PTRN_SEL_REG, useCache)); } // (just a macro)
  uint8_t calcMTP_count(uint8_t MTPcountBits) {
    switch(MTPcountBits) { // 3bit value
      case(0): return(0); break; // 0 bits set
      case(1): case(2): case(4): return(1); break; // any 1 bit set
      case(3): case(5): case(6): return(2); break; // any 2 bits set
      case(7): return(3); break; // 3 bits set
      default: CAP1293debugPrint("calcMTP_count invalid input"); return(255); break; // unlikely to happen
  } }

  //note: 3LSBits CAP1293_channel_bit()      // (R) Base Count Out Of Limit register, 3 LSBits indicate whether the (auto-calibrated) base count is Out Of Limit (per sensor)
  CAP1293_ERR_RETURN_TYPE getBaseCountsOOL(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_BASE_CNT_OOL_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getBaseCountsOOL(bool useCache=false) { return(_getRegRetVal(CAP1293_BASE_CNT_OOL_REG, useCache)); } // (just a macro)
  uint8_t getBaseCountOOL(uint8_t sensorIndex, bool useCache=false) { return(_getRegOneBit(CAP1293_BASE_CNT_OOL_REG, CAP1293_channel_bit(CAP1293_constr_index(sensorIndex)), useCache)); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE getRecalibConfReg(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_RCLB_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getRecalibConfReg(bool useCache=false) { return(_getRegRetVal(CAP1293_RCLB_CONF_REG, useCache)); } // (just a macro)
  //note: refer to affected set function   // BUT_LD_TH bit, if 1 (default) then updating _ACTV_THRSH_1 will update them all, if 0 it only updates the threshold for sensor 1
  bool getRclbConf_WR_ALL_THR(bool useCache=false) { return(_getRegOneBit(CAP1293_RCLB_CONF_REG, CAP1293_RCLB_CONF_WR_ALL_THR_bits, useCache)); } // (just a macro)
  // bool getRclbConf_CLR_INTD(bool useCache=false) { return(_getRegOneBit(CAP1293_RCLB_CONF_REG, CAP1293_RCLB_CONF_CLR_INTD_bits, useCache)); } // (just a macro)
  // bool getRclbConf_CLR_NEG(bool useCache=false) { return(_getRegOneBit(CAP1293_RCLB_CONF_REG, CAP1293_RCLB_CONF_CLR_NEG_bits, useCache)); } // (just a macro)
  //note: default is 00 (see datasheet page 40??)     // NO_CLR_INTD and NO_CLR_NEG bits (default=00), determine some internal response to noise flag being set. See datasheet page 40
  uint8_t getRclbConf_INTD_NEG(bool useCache=false) { return((getRecalibConfReg(useCache) & CAP1293_RCLB_CONF_INTD_NEG_bits) >> 5); }
  //note: CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM    // NEG_DELTA_CNT determines how many consecutive negative delta counts triggers (digital) recalibration, see CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM for 2bit contents
  CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM getRclbConf_NEGDLT_CNTbits(bool useCache=false) {
    return(static_cast<CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM>((getRecalibConfReg(useCache) & CAP1293_RCLB_CONF_NEGDLT_CNT_bits) >> 3)); }
  //note: 8,16,32 or -1(None) respectively
  int8_t getRclbConf_NEGDLT_CNTval(bool useCache=false) {
    uint8_t CNTbits = getRclbConf_NEGDLT_CNTbits(useCache);
    return((CNTbits < 2) ? ((CNTbits+1)<<3) : -1);
  }
  //note: CAP1293_RCLB_CONF_CAL_CFG_ENUM     // CAL_CFG determines the number of samples and cycles for calibration, see CAP1293_RCLB_CONF_CAL_CFG_ENUM for 3bit contents
  CAP1293_RCLB_CONF_CAL_CFG_ENUM getRecalibSamplesCyclesBits(bool useCache=false) { return(static_cast<CAP1293_RCLB_CONF_CAL_CFG_ENUM>(getRecalibConfReg(useCache) & CAP1293_RCLB_CONF_CAL_CFG_bits)); }
  //note: use getRecalibSamplesCyclesBits()
  uint16_t calcRecalibSamples(CAP1293_RCLB_CONF_CAL_CFG_ENUM samplesCyclesBits) { return(16 << min(static_cast<uint8_t>(samplesCyclesBits), (uint8_t)4)); } // (just simple math)
  uint16_t calcRecalibCycles(CAP1293_RCLB_CONF_CAL_CFG_ENUM samplesCyclesBits) {
    uint8_t samplesCycles = static_cast<uint8_t>(samplesCyclesBits);
    return((samplesCycles < 5) ? (16 << samplesCycles) : (32 << samplesCycles)); // 16,32,64,128,256,1024,2048,4096 (notice the missing 512)
  }

  //note: 7bit unsigned!, will be compared against DeltaCountRaw
  CAP1293_ERR_RETURN_TYPE getActiveTouchThreshRaw(uint8_t sensorIndex, uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_TOUCH_THRSH_REG_1 + CAP1293_constr_index(sensorIndex), readBuff, 1)); }
  uint8_t getActiveTouchThreshRaw(uint8_t sensorIndex, bool useCache=false) { return(_getRegRetVal(CAP1293_TOUCH_THRSH_REG_1 + CAP1293_constr_index(sensorIndex), useCache)); }
  // note: (private) touch threshold and sensitivity both present, no cross-usage between active- and standby thresholds- and sensitivities!
  uint16_t calcTouchThreshFull(uint8_t touchThreshRaw, uint8_t deltaSense) { return((uint16_t)touchThreshRaw << (7-deltaSense)); }
  //note: not recommended, pretty inefficient. Consider using: calcTouchThreshFull(getActiveTouchThreshRaw(sensorIndex), constant_activeDELTA_SENSE)
  int16_t getActiveTouchThreshFull(uint8_t sensorIndex) { return(calcTouchThreshFull(getActiveTouchThreshRaw(sensorIndex), getActiveSensitivity())); } // (just a macro)
  
  CAP1293_ERR_RETURN_TYPE getNoiseThresholdReg(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_NOISE_THRSH_REG, readBuff, useCache)); } // (just a macro)
  //note: recommend getNoiseThreshold() instead     // (R/W) Noise Threshold register, 2bit value, see CAP1293_NOISE_THRSH_ENUM for 2bit contents
  uint8_t getNoiseThresholdReg(bool useCache=false) { return(_getRegRetVal(CAP1293_NOISE_THRSH_REG, useCache)); } // (just a macro)
  //note: CAP1293_NOISE_THRSH_REG, defualt is CAP1293_NOISE_THRSH_375
  CAP1293_NOISE_THRSH_ENUM getNoiseThreshold(bool useCache=false) { return(static_cast<CAP1293_NOISE_THRSH_ENUM>(getNoiseThresholdReg(useCache))); }
  float _calcNoiseThresholdPercent(CAP1293_NOISE_THRSH_ENUM thresholdBits) { return(25.0 + (static_cast<uint8_t>(thresholdBits) * 12.5)); }
  //note: default is 37.5
  float getNoiseThresholdPercent(bool useCache=false) { return(_calcNoiseThresholdPercent(getNoiseThreshold(useCache))); } // (just a macro)

  //note: 3LSBits CAP1293_channel_bit(), refer to power mode documentation
  CAP1293_ERR_RETURN_TYPE getStandbyChannelsEnabled(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_STBY_CHAN_EN_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getStandbyChannelsEnabled(bool useCache=false) { return(_getRegRetVal(CAP1293_STBY_CHAN_EN_REG, useCache)); } // (just a macro)
  bool getStandbyChannelEnable(uint8_t sensorIndex, bool useCache=false) { return(_getChanBit(CAP1293_STBY_CHAN_EN_REG, sensorIndex, useCache)); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE getStandbyConf(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_STBY_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getStandbyConf(bool useCache=false) { return(_getRegRetVal(CAP1293_STBY_CONF_REG, useCache)); } // (just a macro)
  bool getStandbyConf_average_or_sum(bool useCache=false) { return(_getRegOneBit(CAP1293_STBY_CONF_REG, CAP1293_STBY_CONF_AVG_SUM_bits, useCache)); } // (just a macro)
  uint8_t getStandbyConf_sample_count(bool useCache=false) { return((getStandbyConf(useCache) & CAP1293_STBY_CONF_SAMP_CNT_bits) >> 4); }
  //note: 320<<bits
  CAP1293_SAMP_TM_ENUM getStandbySampleTimeBits(bool useCache=false) { return(static_cast<CAP1293_SAMP_TM_ENUM>((getStandbyConf(useCache) & CAP1293_STBY_CONF_SAMP_TM_bits) >> 2)); }
  //note: default is 1280us
  uint16_t getStandbySampleTimeMicros(bool useCache=false) { return(320 << getStandbySampleTimeBits(useCache)); }
  //note:    // STB_CY_TIME determines the desired sensing cycle time (in Standby mode), cycle time = (35ms*(TM_bits+1)) 
  uint8_t getStandbySensCycleTimeBits(bool useCache=false) { return(getStandbyConf(useCache) & CAP1293_SAMP_CONF_CYCL_TM_bits); }
  //note: CAP1293_CYCL_TM_LIMITS, default is 70
  uint8_t getStandbySensCycleTimeMillis(bool useCache=false) { return((getStandbySensCycleTimeBits(useCache)+1) * 35); }
  //note: refer to DELTA_SENSE documentation
  CAP1293_ERR_RETURN_TYPE getStandbySensitivity(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_STBY_SENS_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getStandbySensitivity(bool useCache=false) { return(_getRegRetVal(CAP1293_STBY_SENS_REG, useCache)); } // (just a macro)

  //note: 7bit unsigned! will be compared against DeltaCountRaw
  CAP1293_ERR_RETURN_TYPE getStandbyTouchThreshRaw(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_STBY_THRSH_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getStandbyTouchThreshRaw(bool useCache=false) { return(_getRegRetVal(CAP1293_STBY_THRSH_REG, useCache)); } // (just a macro)
  //note: would be compared against DeltaCountFull
  int16_t getStandbyTouchThreshFull() { return(calcTouchThreshFull(getStandbyTouchThreshRaw(), getStandbySensitivity())); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE getBaseCountRaw(uint8_t sensorIndex, uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_BASE_COUNT_REG_1 + CAP1293_constr_index(sensorIndex), readBuff, 1)); }
  uint8_t getBaseCountRaw(uint8_t sensorIndex, bool useCache=false) { return(_getRegRetVal(CAP1293_BASE_COUNT_REG_1 + CAP1293_constr_index(sensorIndex), useCache)); }
  uint16_t calcBaseCountFull(uint8_t baseCountRaw, uint8_t baseShift) { return((uint16_t)baseCountRaw << min(baseShift, (uint8_t)8)); }
  uint16_t getBaseCountFull(uint8_t sensorIndex) { return(calcBaseCountFull(getBaseCountRaw(sensorIndex), getBaseShift())); }

  //note: proper usage: calcIdealBaseCount(either Active sampleTime bits or Standby sampleTime bits, depends on the sensor)
  uint16_t calcIdealBaseCount(CAP1293_SAMP_TM_ENUM sampleTimeBits) { return(3200 << sampleTimeBits); } // (just simple math) 3200, 6400, 12800 or 25600
  // uint16_t calcIdealBaseCount(uint16_t sampleTimeMicros) { return(sampleTimeMicros*10); } // (just simple math) 3200, 6400, 12800 or 25600
  //the following function is just WAY TOO inefficient to be used: getIdealBaseCount(_checkIsInStandbyMode(sensorIndex))
  // uint16_t getIdealBaseCount(bool inputInStandbyMode) { return(calcIdealBaseCount(inputInStandbyMode ? getStandbySampleTimeBits() : getActiveSampleTimeBits())); }

  //note: NOT 3LSBits, this uses the sensorIndex directly
  CAP1293_ERR_RETURN_TYPE getPowerButtonSelect(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_PWR_BTN_SEL_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getPowerButtonSelect(bool useCache=false) { return(_getRegRetVal(CAP1293_PWR_BTN_SEL_REG, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE getPwrBtnConf(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_PWR_BTN_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getPwrBtnConf(bool useCache=false) { return(_getRegRetVal(CAP1293_PWR_BTN_CONF_REG, useCache)); } // (just a macro)
  bool getPwrBtnConf_STBY_EN(bool useCache=false) { return(_getRegOneBit(CAP1293_PWR_BTN_CONF_REG, CAP1293_PWR_BTN_CONF_STBY_EN_bits, useCache)); } // (just a macro)
  //note: default is CAP1293_PWR_BTW_TM_1120    // STBY_PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents
  CAP1293_PWR_BTW_TM_ENUM getPwrBtnConf_STBY_TMbits(bool useCache=false) { return(static_cast<CAP1293_PWR_BTW_TM_ENUM>((getPwrBtnConf(useCache) & CAP1293_PWR_BTN_CONF_STBY_TM_bits) >> 4)); }
  //note: default is 1120ms
  uint16_t getPwrBtnConf_STBY_TMmillis(bool useCache=false) { return(280 << static_cast<uint8_t>(getPwrBtnConf_STBY_TMbits(useCache))); } // 280, 560, 1120, 2240
  bool getPwrBtnConf_ACTV_EN(bool useCache=false) { return(_getRegOneBit(CAP1293_PWR_BTN_CONF_REG, CAP1293_PWR_BTN_CONF_ACTV_EN_bits, useCache)); } // (just a macro)
  //note: default is CAP1293_PWR_BTW_TM_1120    // PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents
  CAP1293_PWR_BTW_TM_ENUM getPwrBtnConf_ACTV_TMbits(bool useCache=false) { return(static_cast<CAP1293_PWR_BTW_TM_ENUM>(getPwrBtnConf(useCache) & CAP1293_PWR_BTN_CONF_ACTV_TM_bits)); }
  //note: default is 1120ms
  uint16_t getPwrBtnConf_ACTV_TMmillis(bool useCache=false) { return(280 << static_cast<uint8_t>(getPwrBtnConf_ACTV_TMbits(useCache))); } // 280, 560, 1120, 2240

  CAP1293_ERR_RETURN_TYPE getCalibConf(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_CALIB_CONF_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getCalibConf(bool useCache=false) { return(_getRegRetVal(CAP1293_CALIB_CONF_REG, useCache)); } // (just a macro)
  //note: refer to sensitivity documentation and proximity sensor example, see also CAP1293_CALSEN_ENUM
  CAP1293_CALSEN_ENUM getCalibConf_CALSENbits(uint8_t sensorIndex, bool useCache=false) { 
    return(static_cast<CAP1293_CALSEN_ENUM>((getCalibConf(useCache) >> (2*CAP1293_constr_index(sensorIndex))) & CAP1293_CALIB_CONF_CALSEN_1_bits));
  }
  //note: refer to getAnalogCalibResult
  CAP1293_ERR_RETURN_TYPE _getAnalogCalibResultMSB(uint8_t sensorIndex, uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_CALIB_MSB_REG_1 + CAP1293_constr_index(sensorIndex), readBuff, 1)); }
  uint8_t _getAnalogCalibResultMSB(uint8_t sensorIndex, bool useCache=false) { return(_getRegRetVal(CAP1293_BASE_COUNT_REG_1 + CAP1293_constr_index(sensorIndex), useCache)); }
  //note: refer to getAnalogCalibResult
  CAP1293_ERR_RETURN_TYPE _getAnalogCalibResultLSBall(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_CALIB_x_LSB_REG, readBuff, useCache)); } // (just a macro)
  uint8_t _getAnalogCalibResultLSBall(bool useCache=false) { return(_getRegRetVal(CAP1293_CALIB_x_LSB_REG, useCache)); } // (just a macro)
  //note: (not recommended) per sensor
  uint8_t _getAnalogCalibResultLSB(uint8_t sensorIndex, bool useCache=false) { return((_getAnalogCalibResultLSBall(useCache) >> (2*CAP1293_constr_index(sensorIndex))) & 0b00000011); }
  //note: 10bit value (no unit mentioned in datasheet)
  uint16_t getAnalogCalibResult(uint8_t sensorIndex) { 
    sensorIndex = CAP1293_constr_index(sensorIndex);
    return((((uint16_t)_getAnalogCalibResultMSB(sensorIndex))<<2) | _getAnalogCalibResultLSB(sensorIndex)); // assemble 10bit value
  }
  
  CAP1293_ERR_RETURN_TYPE getProductID(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_PROD_ID_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getProductID(bool useCache=false) { return(_getRegRetVal(CAP1293_PROD_ID_REG, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE getManufacturerID(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_MF_ID_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getManufacturerID(bool useCache=false) { return(_getRegRetVal(CAP1293_MF_ID_REG, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE getRevision(uint8_t* readBuff, bool useCache=false) { return(_getRegRetErr(CAP1293_REV_REG, readBuff, useCache)); } // (just a macro)
  uint8_t getRevision(bool useCache=false) { return(_getRegRetVal(CAP1293_REV_REG, useCache)); } // (just a macro)

/////////////////////////////////////////////////////////////////////////////////////// set functions: //////////////////////////////////////////////////////////

/*
- recalib_conf (enum)
- touch thresholds (index, raw/full)
- standby:
  + chan en
  + sampling config
  + sensitivity
  + touch threshold (raw/full)
- power button (and bits)
- CALSEN (index)
*/

  CAP1293_ERR_RETURN_TYPE setMainControl(uint8_t newVal) { return(writeBytes(CAP1293_MAIN_CTRL_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setMainControl_GAIN(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_MAIN_CTRL_REG, (newVal << 6), CAP1293_MAIN_CTRL_GAIN_bits, useCache)); }
  CAP1293_ERR_RETURN_TYPE setMainControl_STBY(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_MAIN_CTRL_REG, newVal, CAP1293_MAIN_CTRL_STBY_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setMainControl_DSLEEP(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_MAIN_CTRL_REG, newVal, CAP1293_MAIN_CTRL_DSLEEP_bits, useCache)); } // (just a macro)
  //note: used for Standby sensors only in Combo mode
  CAP1293_ERR_RETURN_TYPE setMainControl_COMBO_GAIN(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_MAIN_CTRL_REG, (newVal << 2), CAP1293_MAIN_CTRL_COMBO_GAIN_bits, useCache)); }
  CAP1293_ERR_RETURN_TYPE setMainControl_COMBO(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_MAIN_CTRL_REG, newVal, CAP1293_MAIN_CTRL_COMBO_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setMainControl_INT(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_MAIN_CTRL_REG, newVal, CAP1293_MAIN_CTRL_INT_bits, useCache)); } // (just a macro)

  // CAP1293_ERR_RETURN_TYPE setMode(CAP1293_mode_ENUM newMode, bool useCache=false) { return(_setRegBits(CAP1293_MAIN_CTRL_REG, static_cast<uint8_t> newMode, CAP1293_mode_MASK, useCache)); }

  // CAP1293_ERR_RETURN_TYPE setGenStatus(uint8_t newVal) { return(writeBytes(CAP1293_GEN_STATUS_REG, &newVal, 1)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setGenStatus_BC_OUT(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_GEN_STATUS_REG, newVal, CAP1293_GEN_STATUS_BC_OUT_bits, useCache)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setGenStatus_ACAL_FAIL(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_GEN_STATUS_REG, newVal, CAP1293_GEN_STATUS_ACAL_FAIL_bits, useCache)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setGenStatus_PWR(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_GEN_STATUS_REG, newVal, CAP1293_GEN_STATUS_PWR_bits, useCache)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setGenStatus_MULT(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_GEN_STATUS_REG, newVal, CAP1293_GEN_STATUS_MULT_bits, useCache)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setGenStatus_MTP(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_GEN_STATUS_REG, newVal, CAP1293_GEN_STATUS_MTP_bits, useCache)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setGenStatus_TOUCH(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_GEN_STATUS_REG, newVal, CAP1293_GEN_STATUS_TOUCH_bits, useCache)); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE setSensReg(uint8_t newVal) { return(writeBytes(CAP1293_SENS_REG, &newVal, 1)); } // (just a macro)
  //note: refer to DELTA_SENSE documentation
  CAP1293_ERR_RETURN_TYPE setActiveSensitivity(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_SENS_REG, (newVal << 4), CAP1293_SENS_ACTV_SENSE_bits, useCache)); }
  CAP1293_ERR_RETURN_TYPE setBaseShift(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_SENS_REG, newVal, CAP1293_SENS_BASE_SHIFT_bits, useCache)); }
  
  CAP1293_ERR_RETURN_TYPE setConf_1(uint8_t newVal) { return(writeBytes(CAP1293_CONF_1_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_1_TIMEOUT(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_1_REG, newVal, CAP1293_CONF_1_TIMOUT_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_1_DIS_DIG_NOISE(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_1_REG, newVal, CAP1293_CONF_1_DIS_DIG_NOISE_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_1_DIS_LF_NOISE(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_1_REG, newVal, CAP1293_CONF_1_DIS_LF_NOISE_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_1_MAX_DUR_EN(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_1_REG, newVal, CAP1293_CONF_1_MAX_DUR_EN_bits, useCache)); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE setConf_2(uint8_t newVal) { return(writeBytes(CAP1293_CONF_2_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_2_BC_OUT_RECAL(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_2_REG, newVal, CAP1293_CONF_2_BC_OUT_RECAL_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_2_BLK_PWR_CTRL(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_2_REG, newVal, CAP1293_CONF_2_BLK_PWR_CTRL_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_2_BC_OUT_INT(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_2_REG, newVal, CAP1293_CONF_2_BC_OUT_INT_bits, useCache)); } // (just a macro)
  //note: SHOW_RF_NOISE
  CAP1293_ERR_RETURN_TYPE setConf_2_DIS_LF_FLAG(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_2_REG, newVal, CAP1293_CONF_2_DIS_LF_FLAG_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_2_DIS_RF_NOISE(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_2_REG, newVal, CAP1293_CONF_2_DIS_RF_NOISE_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setConf_2_ACAL_FAIL_INT(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_2_REG, newVal, CAP1293_CONF_2_ACAL_FAIL_INT_bits, useCache)); } // (just a macro)
  //note: active LOW
  CAP1293_ERR_RETURN_TYPE setConf_2_REL_INT(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_CONF_2_REG, newVal, CAP1293_CONF_2_REL_INT_bits, useCache)); } // (just a macro)

  //note: 3LSBits CAP1293_channel_bit()
  CAP1293_ERR_RETURN_TYPE setActiveChannelsEnabled(uint8_t newVal) { return(writeBytes(CAP1293_ACTV_CHAN_EN_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setActiveChannelEnable(uint8_t sensorIndex, bool newVal, bool useCache=false) { return(_setChanBit(CAP1293_ACTV_CHAN_EN_REG, newVal, sensorIndex, useCache)); } // (just a macro)
  //note: 3LSBits CAP1293_channel_bit(),   ALERT pin & INT bit
  CAP1293_ERR_RETURN_TYPE setInterruptsEnabled(uint8_t newVal) { return(writeBytes(CAP1293_ALERT_EN_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setInterruptEnable(uint8_t sensorIndex, bool newVal, bool useCache=false) { return(_setChanBit(CAP1293_ALERT_EN_REG, newVal, sensorIndex, useCache)); } // (just a macro)
  //note: 3LSBits CAP1293_channel_bit(),   explain repeat briefly
  CAP1293_ERR_RETURN_TYPE setRepeatsEnabled(uint8_t newVal) { return(writeBytes(CAP1293_REPEAT_EN_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setRepeatEnable(uint8_t sensorIndex, bool newVal, bool useCache=false) { return(_setChanBit(CAP1293_REPEAT_EN_REG, newVal, sensorIndex, useCache)); } // (just a macro)
  //note: 1st and 3rd LSBits CAP1293_GUARD_EN_FOR_x_bits    refer to guard explanation/documentation
  CAP1293_ERR_RETURN_TYPE setGuardsEnabled(uint8_t newVal) { return(writeBytes(CAP1293_GUARD_EN_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setGuardEnable(uint8_t sensorIndex, bool newVal, bool useCache=false) {
    return(_setRegOneBit(CAP1293_GUARD_EN_REG, newVal, CAP1293_channel_bit(CAP1293_constr_index(sensorIndex)) & (CAP1293_GUARD_EN_FOR_1_bits | CAP1293_GUARD_EN_FOR_3_bits), useCache)); } // (just a macro)

  //note: new name
  CAP1293_ERR_RETURN_TYPE setTimingConfReg(uint8_t newVal) { return(writeBytes(CAP1293_TIMING_CONF_REG, &newVal, 1)); } // (just a macro)
  //note: CAP1293_MAX_DUR_RECAL_ENUM, default is CAP1293_MAX_DUR_RECAL_5600
  CAP1293_ERR_RETURN_TYPE setMaxDurBits(CAP1293_MAX_DUR_RECAL_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_TIMING_CONF_REG, static_cast<uint8_t>(newBits) << 4, CAP1293_TIMING_CONF_MAX_DUR_bits, useCache)); }
  //note: default is 5600
  // CAP1293_MAX_DUR_RECAL_ENUM _calcMaxDurBits(uint16_t maxDurMillis) { // function not implemented, because i don't like rounding off. ENUM is meant to be legible for humans, not computers
  // CAP1293_ERR_RETURN_TYPE setMaxDurMillis(uint16_t maxDurMillis, bool useCache=false) { return(setMaxDurBits(_calcMaxDurBits(maxDurMillis), useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setRptRateBits(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_TIMING_CONF_REG, newVal, CAP1293_TIMING_CONF_RPT_RATE_bits, useCache)); }
  //note: warning, see CAP1293_RPT_RATE_LIMITS and recommend check, default is 175
  CAP1293_ERR_RETURN_TYPE setRptRateMillis(uint16_t newMillis, bool useCache=false) { return(setRptRateBits((constrain(newMillis, CAP1293_RPT_RATE_LIMITS[0], CAP1293_RPT_RATE_LIMITS[1]) / 35) - 1, useCache)); }
  
  CAP1293_ERR_RETURN_TYPE setHoldThresholdBits(uint8_t newVal) { return(writeBytes(CAP1293_HOLD_THRSH_REG, &newVal, 1)); } // (just a macro)
  //note: warning, see CAP1293_HOLD_THRSH_LIMITS and recommend check, default is 280
  CAP1293_ERR_RETURN_TYPE setHoldThresholdMillis(uint16_t newMillis) { return(setHoldThresholdBits((constrain(newMillis, CAP1293_HOLD_THRSH_LIMITS[0], CAP1293_HOLD_THRSH_LIMITS[1]) / 35) - 1)); }

  //note: (new name)
  CAP1293_ERR_RETURN_TYPE setSampConfReg(uint8_t newVal) { return(writeBytes(CAP1293_SAMP_CONF_REG, &newVal, 1)); } // (just a macro)
  //note: for standby, see also CAP1293_STBY_CONF_SAMP_CNT_bits,    // AVG determines number of samples to be averaged (in Active mode) (to form one measurement), 3bit value where number_of_samples = (1<<(STBY_AVG))
  CAP1293_ERR_RETURN_TYPE setAveragingCountBits(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_SAMP_CONF_REG, (newVal << 4), CAP1293_SAMP_CONF_AVG_bits, useCache)); }
  // //note: not recommended, only powers of 2
  // CAP1293_ERR_RETURN_TYPE setAveragingCount(uint8_t newVal, bool useCache=false) { return(setAveragingCountBits(findMSBit(newVal), useCache)); }
  //note: 320<<bits
  CAP1293_ERR_RETURN_TYPE setActiveSampleTimeBits(CAP1293_SAMP_TM_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_SAMP_CONF_REG, static_cast<uint8_t>(newBits) << 2, CAP1293_SAMP_CONF_SAMP_TM_bits, useCache)); }
  // //note: not recommended, only 320,640,1280 or 2560, default is 1280us
  // CAP1293_ERR_RETURN_TYPE setActiveSampleTimeMicros(uint16_t newMicros, bool useCache=false) { return(setActiveSampleTimeBits(static_cast<CAP1293_SAMP_TM_ENUM>(findMSBit(newMicros / 320)), useCache)); }
  //note:    // CYCLE_TIME determines the desired sensing cycle time (in Active mode), cycle time = (35ms*(TM_bits+1)) 
  CAP1293_ERR_RETURN_TYPE setActiveSensCycleTimeBits(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_SAMP_CONF_REG, newVal, CAP1293_SAMP_CONF_CYCL_TM_bits, useCache)); }
  //note: warning, see CAP1293_CYCL_TM_LIMITS and recommend check, default is 70
  CAP1293_ERR_RETURN_TYPE setActiveSensCycleTimeMillis(uint8_t newMillis, bool useCache=false) {
    return(setActiveSensCycleTimeBits((constrain(newMillis, CAP1293_CYCL_TM_LIMITS[0], CAP1293_CYCL_TM_LIMITS[1]) / 35) - 1, useCache)); }

  CAP1293_ERR_RETURN_TYPE setMultipleTouchConfReg(uint8_t newVal) { return(writeBytes(CAP1293_MULT_CONF_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setMultipleTouchEnabled(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_MULT_CONF_REG, newVal, CAP1293_MULT_CONF_EN_bits, useCache)); } // (just a macro)
  //note: max 3,  default is 1 // B_MULT_T determines how many sensor inputs must be (simultaniously) touched to trigger an MT event. 2bit value, threshold = min(_TH_bits+1,3)
  CAP1293_ERR_RETURN_TYPE setMultipleTouchThreshold(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_MULT_CONF_REG, (newVal << 2), CAP1293_MULT_CONF_TH_bits, useCache)); }

  CAP1293_ERR_RETURN_TYPE setMTPatternConfReg(uint8_t newVal) { return(writeBytes(CAP1293_PTRN_CONF_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setMTPatternEnabled(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_PTRN_CONF_REG, newVal, CAP1293_PTRN_CONF_MTP_EN_bits, useCache)); } // (just a macro)
  //note: default is CAP1293_MTP_THRSH_125  // MTP_TH sets the threshold (% of (Active/Standby) sensor input threshold) for MTP touches(???), see CAP1293_MTP_THRSH_ENUM for 2bit contents
  CAP1293_ERR_RETURN_TYPE setMTP_thresholdBits(CAP1293_MTP_THRSH_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_PTRN_CONF_REG, static_cast<uint8_t>(newBits) << 2, CAP1293_PTRN_CONF_MTP_TH_bits, useCache)); }
  // //note: 12.5, 25, 37.5, 100   default is 12.5
  // CAP1293_ERR_RETURN_TYPE setMTP_thresholdPercent(float newPercent, bool useCache=false) { /*NO, too annoying, do it yourself!*/ } // 12.5, 25, 37.5, 100
  //note:    // COMP_PTRN determines (?) whether specific sensor inputs-, or whether some number of (any) sensor inputs must be touched to trigger MTP event
  CAP1293_ERR_RETURN_TYPE setMTP_patternVScount(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_PTRN_CONF_REG, newVal, CAP1293_PTRN_CONF_SOME_ANY_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setMTP_INT_EN(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_PTRN_CONF_REG, newVal, CAP1293_PTRN_CONF_INT_EN_bits, useCache)); } // (just a macro)
  //note: 3LSBits (count => bitshift), see calcMTP_bits // (R/W) Mutliple Touch Pattern (selection) register. Either it's which sensor inputs are- , or it's how MANY (aboslute) sensor inputs are  'part of the pattern'
  CAP1293_ERR_RETURN_TYPE setMTP_select_or_count(uint8_t newVal) { return(writeBytes(CAP1293_PTRN_SEL_REG, &newVal, 1)); } // (just a macro)
  //note: see setMTP_select_or_count()
  uint8_t calcMTP_bits(uint8_t MTPcount) { return(0b00000111 >> (3-MTPcount)); }

  //note: 3LSBits CAP1293_channel_bit(), see get function of different name    // (R/W) Calibration Activate and Status register (per sensor), 3LSBits are 3 sensor channels respectively. Read for status/faillure
  CAP1293_ERR_RETURN_TYPE setCalibOngoing(uint8_t newVal) { return(writeBytes(CAP1293_CALIB_ACTIV_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE startCalibAll() { return(setCalibOngoing(0b00000111)); } // (just a macro)
  //note: stops other sensor's calibrations! use setCalibOngoing()
  CAP1293_ERR_RETURN_TYPE startCalib(uint8_t sensorIndex) {
    // return(setCalibOngoing(CAP1293_channel_bit(CAP1293_constr_index(sensorIndex)))); // sets other bits to 0 in the process
    return(_setChanBit(CAP1293_CALIB_ACTIV_REG, 1, sensorIndex, false)); // read + write in order to only affect 1 bit
  } // (just a macro)

  CAP1293_ERR_RETURN_TYPE setRecalibConfReg(uint8_t newVal) { return(writeBytes(CAP1293_RCLB_CONF_REG, &newVal, 1)); } // (just a macro)
  //note: refer to affected set function   // BUT_LD_TH bit, if 1 (default) then updating _ACTV_THRSH_1 will update them all, if 0 it only updates the threshold for sensor 1
  CAP1293_ERR_RETURN_TYPE setRclbConf_WR_ALL_THR(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_RCLB_CONF_REG, newVal, CAP1293_RCLB_CONF_WR_ALL_THR_bits, useCache)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setRclbConf_CLR_INTD(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_RCLB_CONF_REG, newVal, CAP1293_RCLB_CONF_CLR_INTD_bits, useCache)); } // (just a macro)
  // CAP1293_ERR_RETURN_TYPE setRclbConf_CLR_NEG(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_RCLB_CONF_REG, newVal, CAP1293_RCLB_CONF_CLR_NEG_bits, useCache)); } // (just a macro)
  //note: default is 00 (see datasheet page 40??)     // NO_CLR_INTD and NO_CLR_NEG bits (default=00), determine some internal response to noise flag being set. See datasheet page 40
  CAP1293_ERR_RETURN_TYPE setRclbConf_INTD_NEG(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_RCLB_CONF_REG, (newVal << 5), CAP1293_RCLB_CONF_INTD_NEG_bits, useCache)); }
  //note: CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM    // NEG_DELTA_CNT determines how many consecutive negative delta counts triggers (digital) recalibration, see CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM for 2bit contents
  CAP1293_ERR_RETURN_TYPE setRclbConf_NEGDLT_CNTbits(CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_RCLB_CONF_REG, static_cast<uint8_t>(newBits) << 3, CAP1293_RCLB_CONF_NEGDLT_CNT_bits, useCache)); }
  // //note: warning 8,16,32 or -1(None) only
  // CAP1293_ERR_RETURN_TYPE setRclbConf_NEGDLT_CNTval(int8_t newVal=-1, bool useCache=false) {
  //   return(setRclbConf_NEGDLT_CNTbits((newVal<=0) ? CAP1293_RCLB_CONF_NEGDLT_CNT_NONE : static_cast<CAP1293_RCLB_CONF_NEGDLT_CNT_ENUM>(findMSBit(newVal/8)), useCache)); }
  //note: CAP1293_RCLB_CONF_CAL_CFG_ENUM     // CAL_CFG determines the number of samples and cycles for calibration, see CAP1293_RCLB_CONF_CAL_CFG_ENUM for 3bit contents
  CAP1293_ERR_RETURN_TYPE setRecalibSamplesCyclesBits(CAP1293_RCLB_CONF_CAL_CFG_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_RCLB_CONF_REG, static_cast<uint8_t>(newBits), CAP1293_RCLB_CONF_CAL_CFG_bits, useCache)); }

  //note: 7bit unsigned!, will be compared against DeltaCountRaw
  CAP1293_ERR_RETURN_TYPE setActiveTouchThreshRaw(uint8_t sensorIndex, int8_t newVal) { return(writeBytes(CAP1293_TOUCH_THRSH_REG_1 + CAP1293_constr_index(sensorIndex), (uint8_t*)&newVal, 1)); }
  // note: (private) touch threshold and sensitivity both present, no cross-usage between active- and standby thresholds- and sensitivities!
  int8_t calcTouchThreshRaw(int16_t touchThreshFull, uint8_t deltaSense) { return(touchThreshFull >> (7-deltaSense)); }
  //note: not recommended, pretty inefficient. Consider using: calcTouchThreshFull(setActiveTouchThreshRaw(sensorIndex), constant_activeDELTA_SENSE)
  CAP1293_ERR_RETURN_TYPE setActiveTouchThreshFull(uint8_t sensorIndex, int16_t newVal) {return(setActiveTouchThreshRaw(sensorIndex, calcTouchThreshRaw(newVal, getActiveSensitivity()))); } // (just a macro)
  
  CAP1293_ERR_RETURN_TYPE setNoiseThresholdReg(uint8_t newVal) { return(writeBytes(CAP1293_NOISE_THRSH_REG, &newVal, 1)); } // (just a macro)
  //note: recommend setNoiseThreshold() instead     // (R/W) Noise Threshold register, 2bit value, see CAP1293_NOISE_THRSH_ENUM for 2bit contents
  //note: CAP1293_NOISE_THRSH_REG, defualt is CAP1293_NOISE_THRSH_375
  CAP1293_ERR_RETURN_TYPE setNoiseThreshold(CAP1293_NOISE_THRSH_ENUM newBits, bool useCache=false) { return(_setRegBits(CAP1293_NOISE_THRSH_REG, static_cast<uint8_t>(newBits), 0b00000011, useCache)); }
  // //note: default is 37.5
  // CAP1293_ERR_RETURN_TYPE setNoiseThresholdPercent(float newPrecent, bool useCache=false) { /*NO, too annoying, do it yourself!*/ } // (just a macro)

  //note: 3LSBits CAP1293_channel_bit(), refer to power mode documentation
  CAP1293_ERR_RETURN_TYPE setStandbyChannelsEnabled(uint8_t newVal) { return(writeBytes(CAP1293_STBY_CHAN_EN_REG, &newVal, 1)); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE setStandbyConf(uint8_t newVal) { return(writeBytes(CAP1293_STBY_CONF_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setStandbyConf_average_or_sum(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_STBY_CONF_REG, newVal, CAP1293_STBY_CONF_AVG_SUM_bits, useCache)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setStandbyConf_sample_count(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_STBY_CONF_REG, (newVal << 4), CAP1293_STBY_CONF_SAMP_CNT_bits, useCache)); }
  //note: 320<<bits
  CAP1293_ERR_RETURN_TYPE setStandbySampleTimeBits(CAP1293_SAMP_TM_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_STBY_CONF_REG, static_cast<uint8_t>(newBits) << 2, CAP1293_STBY_CONF_SAMP_TM_bits, useCache)); }
  // //note: not recommended, only 320,640,1280 or 2560, default is 1280us
  // CAP1293_ERR_RETURN_TYPE setStandbySampleTimeMicros(uint16_t newMicros, bool useCache=false) { return(setStandbySampleTimeBits(static_cast<CAP1293_SAMP_TM_ENUM>(findMSBit(newMicros / 320)), useCache)); }
  //note:    // STB_CY_TIME determines the desired sensing cycle time (in Standby mode), cycle time = (35ms*(TM_bits+1)) 
  CAP1293_ERR_RETURN_TYPE setStandbySensCycleTimeBits(uint8_t newVal, bool useCache=false) { return(_setRegBits(CAP1293_STBY_CONF_REG, newVal, CAP1293_SAMP_CONF_CYCL_TM_bits, useCache)); }
  //note: warning, see CAP1293_CYCL_TM_LIMITS, default is 70
  CAP1293_ERR_RETURN_TYPE setStandbySensCycleTimeMillis(uint8_t newMillis, bool useCache=false) {
    return(setStandbySensCycleTimeBits((constrain(newMillis, CAP1293_CYCL_TM_LIMITS[0], CAP1293_CYCL_TM_LIMITS[1]) / 35) - 1, useCache)); }
  //note: refer to DELTA_SENSE documentation
  CAP1293_ERR_RETURN_TYPE setStandbySensitivity(uint8_t newVal) { return(writeBytes(CAP1293_STBY_SENS_REG, &newVal, 1)); } // (just a macro)

  //note: 7bit unsigned! will be compared against DeltaCountRaw
  CAP1293_ERR_RETURN_TYPE setStandbyTouchThreshRaw(int8_t newVal) { return(writeBytes(CAP1293_STBY_THRSH_REG, (uint8_t*)&newVal, 1)); } // (just a macro)
  //note: would be compared against DeltaCountFull
  CAP1293_ERR_RETURN_TYPE setStandbyTouchThreshFull(int16_t newVal) { return(setStandbyTouchThreshRaw(calcTouchThreshRaw(newVal, getStandbySensitivity()))); } // (just a macro)

  CAP1293_ERR_RETURN_TYPE setBaseCountRaw(uint8_t sensorIndex, uint8_t newVal) { return(writeBytes(CAP1293_BASE_COUNT_REG_1 + CAP1293_constr_index(sensorIndex), &newVal, 1)); }
  uint8_t calcBaseCountRaw(uint16_t baseCountFull, uint8_t baseShift) { return(baseCountFull >> min(baseShift, (uint8_t)8)); }
  CAP1293_ERR_RETURN_TYPE setBaseCountFull(uint8_t sensorIndex, uint16_t newVal) { return(setBaseCountRaw(sensorIndex, calcBaseCountRaw(newVal, getBaseShift()))); }

  //note: NOT 3LSBits, this uses the sensorIndex directly
  CAP1293_ERR_RETURN_TYPE setPowerButtonSelect(uint8_t newVal) { return(writeBytes(CAP1293_PWR_BTN_SEL_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setPwrBtnConf(uint8_t newVal) { return(writeBytes(CAP1293_PWR_BTN_CONF_REG, &newVal, 1)); } // (just a macro)
  CAP1293_ERR_RETURN_TYPE setPwrBtnConf_STBY_EN(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_PWR_BTN_CONF_REG, newVal, CAP1293_PWR_BTN_CONF_STBY_EN_bits, useCache)); } // (just a macro)
  //note: default is CAP1293_PWR_BTW_TM_1120    // STBY_PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents
  CAP1293_ERR_RETURN_TYPE setPwrBtnConf_STBY_TMbits(CAP1293_PWR_BTW_TM_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_PWR_BTN_CONF_REG, static_cast<uint8_t>(newBits) << 4, CAP1293_PWR_BTN_CONF_STBY_TM_bits, useCache)); }
  // //note: not recommended, only 280,560,1120,2240 allowed, default is 1120ms
  // CAP1293_ERR_RETURN_TYPE setPwrBtnConf_STBY_TMmillis(uint16_t newMillis, bool useCache=false) { return(setPwrBtnConf_STBY_TMbits(static_cast<CAP1293_PWR_BTW_TM_ENUM>(findMSBit(newMillis / 280)), useCache)); }
  CAP1293_ERR_RETURN_TYPE setPwrBtnConf_ACTV_EN(bool newVal, bool useCache=false) { return(_setRegOneBit(CAP1293_PWR_BTN_CONF_REG, CAP1293_PWR_BTN_CONF_ACTV_EN_bits, useCache)); } // (just a macro)
  //note: default is CAP1293_PWR_BTW_TM_1120    // PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents
  CAP1293_ERR_RETURN_TYPE setPwrBtnConf_ACTV_TMbits(CAP1293_PWR_BTW_TM_ENUM newBits, bool useCache=false) {
    return(_setRegBits(CAP1293_PWR_BTN_CONF_REG, static_cast<uint8_t>(newBits) << 4, CAP1293_PWR_BTN_CONF_ACTV_TM_bits, useCache)); }
  //note: default is 1120ms
  // CAP1293_ERR_RETURN_TYPE setPwrBtnConf_ACTV_TMmillis(uint16_t newMillis, bool useCache=false) { return(setPwrBtnConf_ACTV_TMbits(static_cast<CAP1293_PWR_BTW_TM_ENUM>(findMSBit(newMillis / 280)), useCache)); }

  CAP1293_ERR_RETURN_TYPE setCalibConf(uint8_t newVal) { return(writeBytes(CAP1293_CALIB_CONF_REG, &newVal, 1)); } // (just a macro)
  //note: refer to sensitivity documentation and proximity sensor example, see also CAP1293_CALSEN_ENUM
  CAP1293_ERR_RETURN_TYPE setCalibConf_CALSENbits(uint8_t sensorIndex, CAP1293_CALSEN_ENUM newBits, bool useCache=false) {
    sensorIndex = CAP1293_constr_index(sensorIndex);
    return(_setRegBits(CAP1293_CALIB_CONF_REG, static_cast<uint8_t>(newBits) << (2*sensorIndex), CAP1293_CALIB_CONF_CALSEN_1_bits << (2*sensorIndex), useCache));
  }

/////////////////////////////////////////////////////////////////////////////////////// debug functions: //////////////////////////////////////////////////////////

  /**
   * attempt to retrieve the product ID and manufacturer ID and check if they match the expected values
   * @return true if reading was successful and ID values as expected
   */
  bool connectionCheck() {
    uint8_t readBuff[2];
    CAP1293_ERR_RETURN_TYPE err = requestReadBytes(CAP1293_PROD_ID_REG, readBuff, 2); // first, check if the read was successfull at all
    if(!_errGood(err)) { return(false); }
    return((readBuff[0] == CAP1293_ID_REG_DEFAULTS[0]) && (readBuff[1] == CAP1293_ID_REG_DEFAULTS[1]));
  }

  // /**
  //  * print out all the configuration values of the sensor (just for debugging)
  //  * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote successfully
  //  */
  // CAP1293_ERR_RETURN_TYPE printConfig() { // prints contents of CONF register (somewhat efficiently)
  //   // TODO!
  // }

  // reset functions?
};


// class CAP1293_thijs_EASY : private CAP1293_thijs {
//// this easy version should have a copy of all the settings (defaulting to the same values as the sensor will), which it keeps in sync.
//// uint8_t memoryMap[][2] = { }; // a list of all the register addresses in the memory map, and the data stored there [][1]
//// it may be memory-inefficient, but an oversized array (256 bytes), and a constant register map (256bits = 32 bytes) where a bit indicates that the register should exist in memory
//// the oversized array is to avoid search functions and stuff, but if you CAN implement a nice formula for skipping empty space, consider it!
//// functions to write:
//// - memSync: bulk-synchronize all the settings from the device to here, skipping gaps, variable-values and read-only stuff along the way
// };

#endif  // CAP1293_thijs_h