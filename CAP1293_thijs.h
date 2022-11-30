
// datasheet link: ...
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
- a touch is determined as (((measurement_count-base_count)>>DELTA_SENSE) >= touch_threshold)
- you will never have access to raw measurements, only 'delta' counts, which are (measurement_count-base_count)
- there are several (sometimes redundant) sensitivity settings, please read through the Sensitivity explenation below
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
  this range selection is done using CAP1293_CALIB_CONF_CALSEN_x_bits
- Analog gain (in the Main Control register) affects counts per capacitance underwater.
  the analog gain can be set using CAP1293_MAIN_CTRL_GAIN_bits (and CAP1293_MAIN_CTRL_STBY_GAIN_bits for Standby)
- DELTA_SENSE is a (badly documented / strangely implemented) secondary gain.
  The CAP1293_SENS_DELTA_SENSE_bits will shift the delta count(???) to the right after(???) being calculated, so delta = ((measurement_count-base_count)>>DELTA_SENSE)
- Touch (binary) sensitivity is determined by a threshold applied to delta (not to the measurement).
  The thresholds per sensor can be set using CAP1293_THRSH_x_REG (and CAP1293_STBY_THRSH_REG for Standby)

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
  - a very long touch (badly set thresholds?) ONLY if enabled at CAP1293_CONF_1_MAX_DUR_EN_bits, see also: CAP1293_TIMER_CONF_MAX_DUR_bits
  - a failed calibration triggers (by default, but can be turned off in some cases) another attempt at calibration
- Faillure of calibration can trigger (or be triggered by) some of the following effects:
  - if a Noise bit is set, analog calibration fails outright. Clear the noise flags and/or fix your RF/power-supply design
  - ACAL_FAIL bit indicates analog calibration faillure. Tune Gain bits or pad design to fix
  - BC_OUT bits indicates digital calibration faillure (becuase the base count is Out Of Limit, which is +-12.5% of ideal value). Tune Gain(?) and BASE_SHIFT (not sample-time???) to fix

Noise:
- RF ...
- low-frequency ...

Guard (Signal-Guard pad/perimeter):
- input 2 can be used as a Signal-Guard pad
  which other sensor input it's guarding is set using CAP1293_GUARD_EN_FOR_x_bits
  Signal-Guard pad is strongly recommended for proximity sensors (which are subject to more noise) or otherwise-noisy designs (RF/buck-boost-power)

Hold, Multi-Touch and Multi-Touch-Pattern:
- Hold ...
- Multi-Touch ...
- Multi-Touch-Pattern (MTP) ...

TODO:
- finish documentation
- basic code
- touch flag manually cleared???
- test effects of DELTA_SENSE and other sensitivity settings
- test calibration settings
- test contiguous vs continuous memory pointer movement (gaps in register addresses)
- HW testing STM32
- HW testing ESP32
- HW testing MSP430
- HW testing 328p
- check Wire.h function return values for I2C errors
- test if 'static' vars in the ESP32 functions actually are static (connect 2 sensors?)
- generalized memory map struct (also for other libraries). Could just be an enum, i just don't love #define

To write:
- 35ms multiples function based on 4bit value
- (auto-tune sensitivity/gain?)

*/

#ifndef CAP1293_thijs_h
#define CAP1293_thijs_h

#include "Arduino.h" // always import Arduino.h

//#define CAP1293debugPrint(x)  Serial.println(x)
//#define CAP1293debugPrint(x)  log_d(x)   //using the ESP32 debug printing

#ifndef CAP1293debugPrint
  #define CAP1293debugPrint(x)  ;
#endif



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
#define CAP1293_SENS_REG          0x1F  // (R/W) Sensitivity Control register, holds input sensitivity/scaling (DELTA_SENSE) and base-count scaling (BASE_SHIFT)
// (general sensor input enable registers):
#define CAP1293_CHANNEL_EN_REG    0x21  // (R/W) Sensor Input Enable register, which sensor inputs are monitored (in Active mode), 3LSBits are 3 sensor channels respectively
#define CAP1293_ALERT_EN_REG      0x27  // (R/W) (Alert) Interrupt Enable register (per sensor), 3LSBits are 3 sensor channels respectively
#define CAP1293_REPEAT_EN_REG     0x28  // (R/W) Repeat Rate Enable register (per sensor), 3LSBits are 3 sensor channels respectively
#define CAP1293_GUARD_EN_REG      0x29  // (R/W) Signal Guard Enable register, see defined _bits below

// (some timing-related registers):
#define CAP1293_TIMER_CONF_REG    0x22  // (R/W) Sensor Input Configuration (1) register, holds 2 4bit values (nibbles), see defined _bits below
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
#define CAP1293_THRSH_REG_1       0x30  // (R/W) touch Threshold register for sensor 1, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
// #define CAP1293_THRSH_1_REG       0x30  // (R/W) touch Threshold register for sensor 1, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
// #define CAP1293_THRSH_2_REG       0x31  // (R/W) touch Threshold register for sensor 2, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
// #define CAP1293_THRSH_3_REG       0x32  // (R/W) touch Threshold register for sensor 3, threshold (of delta, not measurement) that constitutes a touch, 7bit value (applied to int8_t delta value)
#define CAP1293_NOISE_THRSH_REG   0x38  // (R/W) Noise Threshold register, 2bit value, see CAP1293_NOISE_THRSH_ENUM for 2bit contents

// Standby Configuration registers:
#define CAP1293_STBY_CHAN_EN_REG  0x40  // (R/W) Standby Channel (enabled) register, 3LSBits are 3 sensor channels respectively
#define CAP1293_STBY_CONF_REG     0x41  // (R/W) Standby Configuration register, see defined _bits below
#define CAP1293_STBY_SENS_REG     0x42  // (R/W) Standby Sensitivity register, 3bit value to determine (in Standby mode) sensor data right-shift, so sensitivity multiplier = (128>>(STBY_SENS))
#define CAP1293_STBY_THRSH_REG    0x43  // (R/W) Standby Threshold register, count-delta threshold that constitutes a touch, 7bit value

// Base Count Regsiters:
#define CAP1293_BASE_COUNT_REG_1  0x50  // (R) sensor 1 Base Count (reference count value) register, default = 0xC8
// #define CAP1293_BASE_COUNT_1_REG  0x50  // (R) sensor 1 Base Count (reference count value) register, default = 0xC8
// #define CAP1293_BASE_COUNT_2_REG  0x51  // (R) sensor 2 Base Count (reference count value) register, default = 0xC8
// #define CAP1293_BASE_COUNT_3_REG  0x52  // (R) sensor 3 Base Count (reference count value) register, default = 0xC8

// Power Button Regsiters:
#define CAP1293_PWR_BTN_SEL_REG   0x60  // (R/W) Power Button (selection) register, 2bit value, can be 0, 1 or 2
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


// General Status register bits: (read only?)
#define CAP1293_GEN_STATUS_BC_OUT_bits    0b01000000 // BC_OUT indicates base count Out Of Limit for one or more sensor inputs
#define CAP1293_GEN_STATUS_ACAL_FAIL_bits 0b00100000 // ACAL_FAIL indicates analog calibration faillure for one or more sensor inputs
#define CAP1293_GEN_STATUS_PWR_bits       0b00010000 // PWR is the output flag for the Power-Button feature. Linked to the INT bit (main control register) in some way
#define CAP1293_GEN_STATUS_MULT_bits      0b00000100 // MULT indicates that Multiple Touch output ("blocking"), INT bit should not be affected
#define CAP1293_GEN_STATUS_MTP_bits       0b00000010 // MTP indicates that the Multiple Touch Pattern conditions were met (threshold crossed / pattern acceptable) for INT bit behaviour, see MPT_ALERT bit
#define CAP1293_GEN_STATUS_TOUCH_bits     0b00000001 // TOUCH indicates a touch is detected for one or more sensor inputs (but you should really just check CAP1293_TOUCH_FLAG_REG)

// Sensitivity Control register bits:
#define CAP1293_SENS_DELTA_SENSE_bits     0b01110000 // DELTA_SENSE is a 3bit value which determines the sensor data right-shift, so sensitivity multiplier = (128>>(STBY_SENS))
#define CAP1293_SENS_BASE_SHIFT_bits      0b00001111 // BASE_SHIFT is a 4bit value which determines the base count left-shift, so scaling factor = (1<<min(BASE_SHIFT, 8))

// Main Control register:
#define CAP1293_MAIN_CTRL_GAIN_bits       0b11000000 // GAIN is analog capacitance amplification for sensor inputs (in Active mode), 2bit value where gain = (1<<(GAIN))
#define CAP1293_MAIN_CTRL_PWR_STBY_bits   0b00100000 // STBY power mode enable (instead of Active mode)
#define CAP1293_MAIN_CTRL_PRW_DSLEEP_bits 0b00010000 // DSLEEP power mode enable (overrules STBY and COMBO)
#define CAP1293_MAIN_CTRL_STBY_GAIN_bits  0b00001100 // C_GAIN is analog capacitance amplification for sensor inputs (in Standby mode), 2bit value where gain = (1<<(GAIN))
#define CAP1293_MAIN_CTRL_PWR_COMBO_bits  0b00000010 // COMBO power mode enable (overrules STBY)
#define CAP1293_MAIN_CTRL_INT_bits        0b00000001 // INT interrupt flag, controls ALERT pin (used to indicate several things). Must be cleared in order to clear other status flags (like CAP1293_TOUCH_FLAG_REG)

// Configuration (1) register bits:
#define CAP1293_CONF_1_TIMOUT_bits        0b10000000 // TIMOUT enabled SMBus timeout feature. SMBus interface is reset if CLK is low for 30ms or 
#define CAP1293_CONF_1_DIS_DIG_NOISE_bits 0b00100000 // DIS_DIG_NOISE disables using the digital noise threshold (to avoid premature auto-recalibration?) (default disabled)
#define CAP1293_CONF_1_DIS_ANA_NOISE_bits 0b00010000 // DIS_ANA_NOISE whether if low-freq noise is detected, "the delta count on that channel is set to 0" (default enabled)
#define CAP1293_CONF_1_MAX_DUR_EN_bits    0b00001000 // MAX_DUR_EN enables auto-recalibration if a touch is detected for too long (see CAP1293_TIMER_CONF_MAX_DUR_bits) (default disabled)
// Configuration 2 register bits:
#define CAP1293_CONF_2_BC_OUT_RECAL_bits  0b01000000 // BC_OUT_RECAL enabled retying analog-calib. (instead of falling back to OOL-base-count) when BC_OUTx bit is set (default enabled)
#define CAP1293_CONF_2_BLK_PWR_CTRL_bits  0b00100000 // BLK_PWR_CTRL determines whether to reduce power consumption near end of sensing cycle (default enabled)
#define CAP1293_CONF_2_BC_OUT_INT_bits    0b00010000 // BC_OUT_INT enables sending an interrupt if base count is Out Of Limit for one or more sensor inputs (default disabled)
#define CAP1293_CONF_2_SHOW_RF_NOISE_bits 0b00001000 // SHOW_RF_NOISE disables the low-freq noise from setting the noise status bit(s) (default enabled)
#define CAP1293_CONF_2_DIS_RF_NOISE_bits  0b00000100 // DIS_RF_NOISE whether if RF noise is detected, "the delta count on that channel is set to 0" (default enabled)
#define CAP1293_CONF_2_ACAL_FAIL_INT_bits 0b00000010 // ACAL_FAIL_INT enables sending an interrupt if analog calibration fails for one or more sensor inputs (default disabled)
#define CAP1293_CONF_2_REL_INT_bits       0b00000001 // INT_REL_n disables sending an interrupt when a touch is released (default enabled)

// Signal Guard Enable register bits:
#define CAP1293_GUARD_EN_FOR_3_bits       0b00000100 // CS3_SG_EN enables signal guard feature around sensor input 3 (input 2 is connected to guard pad)
#define CAP1293_GUARD_EN_FOR_1_bits       0b00000001 // CS3_SG_EN enables signal guard feature around sensor input 1 (input 2 is connected to guard pad)

// Sensor Input Configuration (1) register bits:
#define CAP1293_TIMER_CONF_MAX_DUR_bits   0b11110000 // MAX_DUR determines the how long a sensor needs to detect a touch before it start auto-recalibration, see CAP1293_MAX_DUR_RECAL_ENUM for 4bit contents 
#define CAP1293_TIMER_CONF_RPT_RATE_bits  0b00001111 // RPT_RATE determines time between repeat-touch (interrupt) events, repeat rate = (35ms*(RPT_RATE_bits+1)), default = 175ms

// Averaging and Sampling Configuration register bits:
#define CAP1293_SAMP_CONF_AVG_bits        0b01110000 // AVG determines number of samples to be averaged (in Active mode) (to form one measurement), 3bit value where number_of_samples = (1<<(STBY_AVG))
#define CAP1293_SAMP_CONF_SAMP_TM_bits    0b00001100 // SAMP_TIME determines the (single-)sample time (in Active mode), see CAP1293_SAMP_TM_ENUM for 2bit contents 
#define CAP1293_SAMP_CONF_CYCL_TM_bits    0b00000011 // CYCLE_TIME determines the desired sensing cycle time (in Active mode), see CAP1293_CYCL_TM_ENUM for 2bit contents = (35ms*(TM_bits+1)) 

// Mutliple Touch Configuration register bits:
#define CAP1293_MULT_CONF_EN_bits         0b10000000 // MULT_BLK_EN enables the Multiple Touch circuitry
#define CAP1293_MULT_CONF_TH_bits         0b00001100 // B_MULT_T determines how many sensor inputs must be (simultaniously) touched to trigger an MT event. 2bit value, threshold = min(_TH_bits+1,3)
// Mutliple Touch Pattern Configuration register bits:
#define CAP1293_PTRN_CONF_MTP_EN_bits     0b10000000 // MTP_EN enables the Multiple Touch Pattern
#define CAP1293_PTRN_CONF_MTP_TH_bits     0b00001100 // MTP_TH sets the threshold (% of (Active/Standby) sensor input threshold) for MTP touches(???)
#define CAP1293_PTRN_CONF_SOME_ANY_bits   0b00001100 // COMP_PTRN determines (?) whether specific sensor inputs-, or whether some number of (any) sensor inputs must be touched to trigger MTP event

// Recalibration Configuration register bits:
#define CAP1293_RCLB_CONF_WR_ALL_THR_bits 0b10000000 // BUT_LD_TH bit, if 1 (default) then updating _ACTV_THRSH_1 will update them all, if 0 it only updates the threshold for sensor 1
// #define CAP1293_RCLB_CONF_CLR_INTD_bits   0b01000000 // NO_CLR_INTD bit, if 0 (default) the noise status bit being set also clears 'accumulation of intermediate data'   ????
// #define CAP1293_RCLB_CONF_CLR_NEG_bits    0b00100000 // NO_CLR_NEG bit, if 0 (default) the noise status bit being set also clears 'e consecutive negative delta counts counter'  ????
#define CAP1293_RCLB_CONF_INTD_NEG_bits   0b01100000 // NO_CLR_INTD and NO_CLR_NEG bits (default=00), determine some internal response to noise flag being set. See datasheet page 40
#define CAP1293_RCLB_CONF_NEGDLT_CNT_bits 0b00011000 // NEG_DELTA_CNT determines how many consecutive negative delta counts triggers (digital) recalibration
#define CAP1293_RCLB_CONF_CAL_CFG_bits    0b00000111 // CAL_CFG determines the number of samples and cycles for calibration, see CAP1293_RCLB_CONF_CAL_CFG_ENUM for 3bit contents

// Standby Configuration register bits:
#define CAP1293_STBY_CONF_AVG_SUM_bits    0b10000000 // AVG_SUM determines Standby-enabled-sensor-input values to be based on Average value OR Summation(/total-count) value (default == average)
#define CAP1293_STBY_CONF_SAMP_CNT_bits   0b01110000 // STBY_AVG determines samples taken per channel (consequitive, not paralel), 3bit value where number_of_samples = (1<<(STBY_AVG))
#define CAP1293_STBY_CONF_SAMP_TM_bits    0b00001100 // STB_SAMP_TIME determines the (single-)sample time (in Standby mode), see CAP1293_SAMP_TM_ENUM for 2bit contents 
#define CAP1293_STBY_CONF_CYCL_TM_bits    0b00000011 // STB_CY_TIME determines the desired sensing cycle time (in Standby mode), see CAP1293_CYCL_TM_ENUM for 2bit contents = (35ms*(TM_bits+1)) 

// Power Button Configuration register bits:
#define CAP1293_PWR_BTN_CONF_STBY_EN_bits 0b01000000 // STBY_PWR_EN enables the power button feature in Standby state
#define CAP1293_PWR_BTN_CONF_STBY_TM_bits 0b00110000 // STBY_PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents
#define CAP1293_PWR_BTN_CONF_ACTV_EN_bits 0b00000100 // PWR_EN enables the power button feature in Active state
#define CAP1293_PWR_BTN_CONF_ACTV_TM_bits 0b00000011 // PWR_TIME bits determine the hold-time before the bit/interrupt is activated, see CAP1293_PWR_BTW_TM_ENUM for 2bit contents

// Calibration Sensitivity Configuration register bits:
#define CAP1293_CALIB_CONF_CALSEN_3_bits  0b00110000 // CALSEN3 is the calibration gain for sensor 3, see CAP1293_CALSEN_ENUM for 2bit contents 
#define CAP1293_CALIB_CONF_CALSEN_2_bits  0b00001100 // CALSEN2 is the calibration gain for sensor 2, see CAP1293_CALSEN_ENUM for 2bit contents 
#define CAP1293_CALIB_CONF_CALSEN_1_bits  0b00000011 // CALSEN1 is the calibration gain for sensor 1, see CAP1293_CALSEN_ENUM for 2bit contents 

//// to be implemented (cleanly)?
// enum CAP1293_CHANNEL_MASK_ENUM : uint8_t { // many registers use 1 bit for each sensor, so you can also just bitshift, e.g.:  chan_1_bit = (1<<channel_index)
//   CAP1293_CHANNEL_1_MASK = 0b00000001, // sensor input channel 1 bitmask
//   CAP1293_CHANNEL_2_MASK = 0b00000010, // sensor input channel 2 bitmask
//   CAP1293_CHANNEL_3_MASK = 0b00000100  // sensor input channel 3 bitmask
// }; 

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
static const uint16_t CAP1293_RPT_RATE_LIMITS[2] = {35, 560}; // (milliseconds) Repeat Rate (RPT_RATE in _TIMER_CONF) can be set to between 35ms and 560ms

static const uint16_t CAP1293_HOLD_THRSH_LIMITS[2] = {35, 560}; // (milliseconds) press-and-hold time threshold can be set to between 35ms and 560ms

enum CAP1293_SAMP_TM_ENUM : uint8_t { // 2bit value to determine the (single-)sample time, used for SAMP_TM bits in both STBY_CONF and SAMP_CONF. Basically 320<<(2bit_value)
  CAP1293_SAMP_TM_320  = 0, // 320us
  CAP1293_SAMP_TM_640  = 1, // 640us
  CAP1293_SAMP_TM_1280 = 2, // 1.28ms (default)
  CAP1293_SAMP_TM_2560 = 3  // 2.56ms
};
enum CAP1293_CYCL_TM_ENUM : uint8_t { // 2bit value to determine the desired(!) sensing cycle time, used for CYCL_TM bits in both STBY_CONF and SAMP_CONF. Basically 35*(2bit_value)
  CAP1293_CYCL_TM_35  = 0, // 35ms
  CAP1293_CYCL_TM_70  = 1, // 70ms (default)
  CAP1293_CYCL_TM_105 = 2, // 105ms
  CAP1293_CYCL_TM_140 = 3  // 140ms
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

static const uint8_t CAP1293_ID_REG_DEFAULTS[3] = {0x6F, 0x5D, 0x00}; // default values for product ID, manufacturer ID and revision number


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
   * (private) request a specific register and read 1 byte into _readBuff (optional cache)
   * @param regAddress register byte (see list of defines at top)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE _readByteRetErr(uint8_t regAddress, bool useCache=false) {
    if( ! (useCache && (regAddress == _readBuffAddress))) { // if cache can't be used (normally true)
      CAP1293_ERR_RETURN_TYPE err = requestReadBytes(regAddress, &_readBuff, 1); _readBuffAddress = regAddress; // read into cache
      if(!_errGood(err)) { CAP1293debugPrint("_readByteRetErr read/write error: "); _readBuffAddress = CAP1293_INVALID_REG_ADDR; return(err); }
    } /*else*/ return(CAP1293_ERR_RETURN_TYPE_OK); // if cache was successfully used, this function does absolutely nothing
  }
  /**
   * (private) request a specific register and read 1 byte (optional cache)
   * @param regAddress register byte (see list of defines at top)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the byte it read (unless the read failed!)
   */
  uint8_t _readByteRetVal(uint8_t regAddress, bool useCache=false) { // (mostly a macro)
    CAP1293_ERR_RETURN_TYPE err = _readByteRetErr(regAddress, useCache);
    if(!_errGood(err)) { CAP1293debugPrint("_readByteRetVal read/write error: " /* +debugName */); }
    return(_readBuff);
  }

  /**
   * (private) retrieve one bit from a register (this version of the function DOES NOT let you check for I2C errors)
   * @param mask which bit to retrieve
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the bit (as a boolean)
   */
  bool _getOneBit(uint8_t regAddress, uint8_t mask, bool useCache=false) { return((_readByteRetVal(regAddress, useCache) & CAP1293_MAIN_CTRL_PWR_STBY_bits) != 0); } // (just a macro)

/////////////////////////////////////////////////////////////////////////////////////// get functions: //////////////////////////////////////////////////////////

  // /**
  //  * retrieve ... register (this version of the function lets you check for I2C errors)
  //  * @param readBuff byte reference to put the result in
  //  * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
  //  * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
  //  */
  // CAP1293_ERR_RETURN_TYPE get_(uint8_t& readBuff, bool useCache=false) { return(_readByteRetErr(  , useCache)); } // (just a macro)
  // /**
  //  * retrieve ... register (this version of the function DOES NOT let you check for I2C errors)
  //  * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
  //  * @return the .... byte
  //  */
  // uint8_t get_(bool useCache=false) { return(_readByteRetVal(  , useCache)); } // (just a macro)

  
  /**
   * retrieve Main Control register (this version of the function lets you check for I2C errors)
   * @param readBuff byte reference to put the result in. See CAP1293_MAIN_CTRL_x_bits for contents
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getMainControl(uint8_t& readBuff, bool useCache=false) { return(_readByteRetErr(CAP1293_MAIN_CTRL_REG, useCache)); } // (just a macro)
  /**
   * retrieve Main Control register (this version of the function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the Main Control byte. See CAP1293_MAIN_CTRL_x_bits for contents
   */
  uint8_t getMainControl(bool useCache=false) { return(_readByteRetVal(CAP1293_MAIN_CTRL_REG, useCache)); } // (just a macro)
  /**
   * retrieve GAIN bits from the Main Control register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the GAIN bits, amplification = (1<<(GAIN))
   */
  uint8_t getMainControl_GAIN(bool useCache=false) { return((_readByteRetVal(CAP1293_MAIN_CTRL_REG, useCache) & CAP1293_MAIN_CTRL_GAIN_bits) >> 6); } // (just a macro)
  /**
   * retrieve STBY bit from the Main Control register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the STBY bit, 1 means Standby power mode is active (unless overruled by Combo or Deep Sleep)
   */
  bool getMainControl_STBY(bool useCache=false) { return(_getOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_PWR_STBY_bits, useCache)); } // (just a macro)
  /**
   * retrieve DSLEEP bit from the Main Control register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the STBY bit, 1 means Deep Sleep power mode is active
   */
  bool getMainControl_DSLEEP(bool useCache=false) { return(_getOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_PRW_DSLEEP_bits, useCache)); } // (just a macro)
  /**
   * retrieve C_GAIN (Standby gain) bits from the Main Control register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the C_GAIN bits, amplification = (1<<(GAIN))
   */
  uint8_t getMainControl_C_GAIN(bool useCache=false) { return((_readByteRetVal(CAP1293_MAIN_CTRL_REG, useCache) & CAP1293_MAIN_CTRL_STBY_GAIN_bits) >> 2); } // (just a macro)
  /**
   * retrieve DSLEEP bit from the Main Control register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the STBY bit, 1 means Combo power mode is active (unless overruled by Deep Sleep)
   */
  bool getMainControl_COMBO(bool useCache=false) { return(_getOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_PWR_COMBO_bits, useCache)); } // (just a macro)
  /**
   * retrieve INT bit from the Main Control register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the INT bit, directly relates to ALERT pin
   */
  bool getMainControl_INT(bool useCache=false) { return(_getOneBit(CAP1293_MAIN_CTRL_REG, CAP1293_MAIN_CTRL_PWR_COMBO_bits, useCache)); } // (just a macro)
  /**
   * retrieve General Status register (this version of the function lets you check for I2C errors)
   * @param readBuff byte reference to put the result in. See CAP1293_GEN_STATUS_x_bits for contents
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getGenStatus(uint8_t& readBuff, bool useCache=false) { return(_readByteRetErr(CAP1293_GEN_STATUS_REG, useCache)); } // (just a macro)
  /**
   * retrieve General Status register (this version of the function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the General Status byte. See CAP1293_GEN_STATUS_x_bits for contents
   */
  uint8_t getGenStatus(bool useCache=false) { return(_readByteRetVal(CAP1293_GEN_STATUS_REG, useCache)); } // (just a macro)
  /**
   * retrieve BC_OUT bit from the General Status register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the BC_OUT bit, indicating Base Count is Out Of Limit
   */
  bool getGenStatus_BC_OUT(bool useCache=false) { return(_getOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_BC_OUT_bits, useCache)); } // (just a macro)
  /**
   * retrieve ACAL_FAIL bit from the General Status register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the ACAL_FAIL bit, indicating Analog Calibration faillure
   */
  bool getGenStatus_ACAL_FAIL(bool useCache=false) { return(_getOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_ACAL_FAIL_bits, useCache)); } // (just a macro)
  /**
   * retrieve PWR bit from the General Status register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the PWR bit, indicating Power-Button was detected (conditions were met)
   */
  bool getGenStatus_PWR(bool useCache=false) { return(_getOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_PWR_bits, useCache)); } // (just a macro)
  /**
   * retrieve MULT bit from the General Status register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the MULT bit, indicating Multiple-Touch was detected (conditions were met)
   */
  bool getGenStatus_MULT(bool useCache=false) { return(_getOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_MULT_bits, useCache)); } // (just a macro)
  /**
   * retrieve MTP bit from the General Status register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the MTP bit, indicating Multiple-Touch-Pattern was detected (conditions were met)
   */
  bool getGenStatus_MTP(bool useCache=false) { return(_getOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_MTP_bits, useCache)); } // (just a macro)
  /**
   * retrieve TOUCH bit from the General Status register (this function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the TOUCH bit, indicating a touch was detected (see touch flags register for which one)
   */
  bool getGenStatus_TOUCH(bool useCache=false) { return(_getOneBit(CAP1293_GEN_STATUS_REG, CAP1293_GEN_STATUS_TOUCH_bits, useCache)); } // (just a macro)
  /**
   * retrieve the (individual) touch flags/status register (this version of the function lets you check for I2C errors)
   * @param readBuff byte reference to put the result in. The 3 LSBits are the 3 sensor inputs respectively
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getTouchFlags(uint8_t& readBuff, bool useCache=false) { return(_readByteRetErr(CAP1293_TOUCH_FLAG_REG, useCache)); } // (just a macro)
  /**
   * retrieve the (individual) touch flags/status byte (this version of the function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the touch flags byte. The 3 LSBits are the 3 sensor inputs respectively
   */
  uint8_t getTouchFlags(bool useCache=false) { return(_readByteRetVal(CAP1293_TOUCH_FLAG_REG, useCache)); } // (just a macro)
  /**
   * retrieve the (individual) noise flags/status register (this version of the function lets you check for I2C errors)
   * @param readBuff byte reference to put the result in. The 3 LSBits are the 3 sensor inputs respectively
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getNoiseFlags(uint8_t& readBuff, bool useCache=false) { return(_readByteRetErr(CAP1293_NOISE_FLAG_REG, useCache)); } // (just a macro)
  /**
   * retrieve the (individual) noise flags/status byte (this version of the function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the noise flags byte. The 3 LSBits are the 3 sensor inputs respectively
   */
  uint8_t getNoiseFlags(bool useCache=false) { return(_readByteRetVal(CAP1293_NOISE_FLAG_REG, useCache)); } // (just a macro)

  /**
   * retrieve the (individual) calibration-active flags/status register (this version of the function lets you check for I2C errors)
   * @param readBuff byte reference to put the result in. The 3 LSBits are the 3 sensor inputs respectively
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return (bool or esp_err_t or i2c_status_e, see on defines at top) whether it wrote/read successfully
   */
  CAP1293_ERR_RETURN_TYPE getCalibActive(uint8_t& readBuff, bool useCache=false) { return(_readByteRetErr(CAP1293_CALIB_ACTIV_REG, useCache)); } // (just a macro)
  /**
   * retrieve the (individual) calibration-active flags/status byte (this version of the function DOES NOT let you check for I2C errors)
   * @param useCache (optional!, not recommended, use at own discretion) use data from _readBuff cache (if possible) instead of actually reading it from I2C (to save a little time).
   * @return the calibration-active flags byte. The 3 LSBits are the 3 sensor inputs respectively
   */
  uint8_t getCalibActive(bool useCache=false) { return(_readByteRetVal(CAP1293_CALIB_ACTIV_REG, useCache)); } // (just a macro)


  uint16_t getBaseCount(uint8_t sensorIndex) {
    sensorIndex = (sensorIndex > 2) ? 2 : sensorIndex; // constrain index to the actual number of sensor inputs
    uint8_t baseCount = _readByteRetVal(CAP1293_BASE_COUNT_REG_1 + sensorIndex);
    uint8_t baseShift = _readByteRetVal(CAP1293_SENS_REG) & CAP1293_SENS_BASE_SHIFT_bits; // LSnibble
    uint16_t baseCountReal = (baseCount << min(baseShift, (uint8_t)8));
    return(baseCountReal);
  }
  uint16_t getDeltaCount(uint8_t sensorIndex) {
    sensorIndex = (sensorIndex > 2) ? 2 : sensorIndex; // constrain index to the actual number of sensor inputs
    int8_t deltaCount = _readByteRetVal(CAP1293_DELTA_COUNT_REG_1 + sensorIndex);
    uint8_t deltaSense = (_readByteRetVal(CAP1293_SENS_REG) & CAP1293_SENS_DELTA_SENSE_bits) >> 4;
    int16_t deltaCountReal = (deltaCount << (7-deltaCountReal));
    return(deltaCountReal);
  }
/////////////////////////////////////////////////////////////////////////////////////// set functions: //////////////////////////////////////////////////////////



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

#endif  // CAP1293_thijs_h