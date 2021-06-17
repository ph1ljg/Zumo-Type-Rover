/* 
* CVl53L1X.cpp
*
* Created: 10/10/2020 14:49:43
* Author: philg
*/

//  We expect new data to be available every 50ms
#include "Includes.h"
//#include <endian.h>
// default constructor
CVL53L1X::CVL53L1X()
{
} //CVl53L1X

// default destructor
CVL53L1X::~CVL53L1X()
{
} //~CVl53L1X

/*
  initialize sensor
 */
bool CVL53L1X::Init(DistanceMode mode)
{
    uint8_t pad_i2c_hv_extsup_config = 0;
    uint16_t mm_config_outer_offset_mm = 0;
	if(!CheckId())
		return(false);
   
   
    if (!Reset())					//// reset the chip, we make assumptions later on that we are on a clean power on of the sensor
	{           // setup for 2.8V operation
          ReadRegister(PAD_I2C_HV__EXTSUP_CONFIG, &pad_i2c_hv_extsup_config);
          WriteRegister(PAD_I2C_HV__EXTSUP_CONFIG, pad_i2c_hv_extsup_config | 0x01);
	}
          // store oscillator info for later use
          ReadRegister16(OSC_MEASURED__FAST_OSC__FREQUENCY, &fast_osc_frequency);
          ReadRegister16(RESULT__OSC_CALIBRATE_VAL, &osc_calibrate_val) ;

          // static config
    if( (     WriteRegister16(DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate) && // should already be this value after reset
          WriteRegister(GPIO__TIO_HV_STATUS, 0x02) &&
          WriteRegister(SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8) && // tuning parm default
          WriteRegister(SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16) && // tuning parm default
          WriteRegister(ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01) &&
          WriteRegister(ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF) &&
          WriteRegister(ALGO__RANGE_MIN_CLIP, 0) && // tuning parm default
          WriteRegister(ALGO__CONSISTENCY_CHECK__TOLERANCE, 2) && // tuning parm default

          // general config
          WriteRegister16(SYSTEM__THRESH_RATE_HIGH, 0x0000) &&
          WriteRegister16(SYSTEM__THRESH_RATE_LOW, 0x0000) &&
          WriteRegister(DSS_CONFIG__APERTURE_ATTENUATION, 0x38) &&

          // timing config
          WriteRegister16(RANGE_CONFIG__SIGMA_THRESH, 360) && // tuning parm default
          WriteRegister16(RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192) && // tuning parm default

          // dynamic config
          WriteRegister(SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01) &&
          WriteRegister(SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01) &&
          WriteRegister(SD_CONFIG__QUANTIFIER, 2) && // tuning parm default

          // from VL53L1_preset_mode_timed_ranging_*
          // GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
          // and things don't seem to work if we don't set GPH back to 0 (which the API
          // does here).
          WriteRegister(SYSTEM__GROUPED_PARAMETER_HOLD, 0x00) &&
          WriteRegister(SYSTEM__SEED_CONFIG, 1) && // tuning parm default

          // from VL53L1_config_low_power_auto_mode
          WriteRegister(SYSTEM__SEQUENCE_CONFIG, 0x8B) && // VHV, PHASECAL, DSS1, RANGE
          WriteRegister16(DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8) &&
          WriteRegister(DSS_CONFIG__ROI_MODE_CONTROL, 2) && // REQUESTED_EFFFECTIVE_SPADS
          ReadRegister16(MM_CONFIG__OUTER_OFFSET_MM, &mm_config_outer_offset_mm) &&
          SetDistanceMode(mode) &&
          SetMeasurementTimingBudget(40000) &&
          // the API triggers this change in VL53L1_init_and_start_range() once a
          // measurement is started; assumes MM1 and MM2 are disabled
          WriteRegister16(ALGO__PART_TO_PART_RANGE_OFFSET_MM, mm_config_outer_offset_mm * 4) &&
          // set continuous mode
          StartContinuous(50)
          ))
		   {
              return false;
          }
  
    return true;
}


//  update the state of the sensor called at 20Hz
bool CVL53L1X::Update(void)
{
	uint16_t range_mm;
	if ((GetReading(range_mm)) && (range_mm <= 4000)) 
	{
		sum_mm += range_mm;
		counter++;
	}
	if (counter > 2) 
	{
		m_DistanceReading = sum_mm / counter;
		sum_mm = 0;
		counter = 0;
		return(true);
	}
	else
		return(false);
}



// check sensor ID registers
bool CVL53L1X::CheckId(void)
{
    uint8_t v1, v2;
    if (!(ReadRegister(0x010F, &v1) && ReadRegister(0x0110, &v2))) 
        return false;

    if ((v1 != 0xEA) || (v2 != 0xCC)) 
        return false;
 
    return true;
}

bool CVL53L1X::Reset(void) 
{
	if (!WriteRegister(SOFT_RESET, 0x00)) 
		return false;
	Core.delayMicroseconds(100);
	if (!WriteRegister(SOFT_RESET, 0x01)) 
		return false;
	Core.delay(1000);
	return true;
}


// set distance mode to Short, Medium, or Long based on VL53L1_SetDistanceMode()
bool CVL53L1X::SetDistanceMode(DistanceMode distance_mode)
{
	uint32_t budget_us = 0;
	if (!GetMeasurementTimingBudget(budget_us))				// save existing timing budget
		return false;

	switch (distance_mode) 
	{
	case DistanceMode::Short:								// from VL53L1_preset_mode_standard_ranging_short_range()
		if (!(// timing config
				WriteRegister(RANGE_CONFIG__VCSEL_PERIOD_A, 0x07) && WriteRegister(RANGE_CONFIG__VCSEL_PERIOD_B, 0x05) && WriteRegister(RANGE_CONFIG__VALID_PHASE_HIGH, 0x38) &&
				WriteRegister(SD_CONFIG__WOI_SD0, 0x07) &&								// dynamic config
				WriteRegister(SD_CONFIG__WOI_SD1, 0x05) &&
				WriteRegister(SD_CONFIG__INITIAL_PHASE_SD0, 6) && // tuning parm default
				WriteRegister(SD_CONFIG__INITIAL_PHASE_SD1, 6))) 
		{ // tuning parm default
			return false;
		}

		break;

	case DistanceMode::Medium:        // from VL53L1_preset_mode_standard_ranging()
        if (!(// timing config
			WriteRegister(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B) &&
			WriteRegister(RANGE_CONFIG__VCSEL_PERIOD_B, 0x09) &&
			WriteRegister(RANGE_CONFIG__VALID_PHASE_HIGH, 0x78) &&

			// dynamic config
			WriteRegister(SD_CONFIG__WOI_SD0, 0x0B) &&
			WriteRegister(SD_CONFIG__WOI_SD1, 0x09) &&
			WriteRegister(SD_CONFIG__INITIAL_PHASE_SD0, 10) && // tuning parm default
			WriteRegister(SD_CONFIG__INITIAL_PHASE_SD1, 10)))  // tuning parm default
		{
            return false;
        }

         break;
	case DistanceMode::Long:	// from VL53L1_preset_mode_standard_ranging_long_range()

		if (!(// timing config
			WriteRegister(RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F) &&
			WriteRegister(RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D) &&
			WriteRegister(RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8) &&

			// dynamic config
			WriteRegister(SD_CONFIG__WOI_SD0, 0x0F) &&
			WriteRegister(SD_CONFIG__WOI_SD1, 0x0D) &&
			WriteRegister(SD_CONFIG__INITIAL_PHASE_SD0, 14) && // tuning parm default
			WriteRegister(SD_CONFIG__INITIAL_PHASE_SD1, 14))) // tuning parm default
		 {
			return false;
         }

            break;
        default:
            // unrecognized mode - do nothing
            return false;
    }

    // reapply timing budget
    return SetMeasurementTimingBudget(budget_us);
}

// Set the measurement timing budget in microseconds, which is the time allowed for one measurement. A longer timing budget allows for more accurate measurements.
bool CVL53L1X::SetMeasurementTimingBudget(uint32_t budget_us)
{
    
	if (budget_us <= TimingGuard)						// assumes PresetMode is LOWPOWER_AUTONOMOUS
		return false;

	uint32_t range_config_timeout_us = budget_us - TimingGuard;
	if (range_config_timeout_us > 1100000) 
		return false;									// FDA_MAX_TIMING_BUDGET_US * 2
	range_config_timeout_us /= 2;
	uint8_t range_config_vcsel_period = 0;				// VL53L1_calc_timeout_register_values() begin
	if (!ReadRegister(RANGE_CONFIG__VCSEL_PERIOD_A, &range_config_vcsel_period)) 
		return false;

    uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period);		// "Update Macro Period for Range A VCSEL Period"

    // "Update Phase timeout - uses Timing A"/ Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
    // via VL53L1_get_preset_mode_timing_cfg().
    uint32_t phasecal_timeout_mclks = TimeoutMicrosecondsToMclks(1000, macro_period_us);
    if (phasecal_timeout_mclks > 0xFF) 
        phasecal_timeout_mclks = 0xFF;

    if (!( WriteRegister(PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks) &&

          // "Update MM Timing A timeout"
          // Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
          // via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
          // actually ends up with a slightly different value because it gets assigned,
          // retrieved, recalculated with a different macro period, and reassigned,
          // but it probably doesn't matter because it seems like the MM ("mode
          // mitigation"?) sequence steps are disabled in low power auto mode anyway.
          WriteRegister16(MM_CONFIG__TIMEOUT_MACROP_A, EncodeTimeout(TimeoutMicrosecondsToMclks(1, macro_period_us))) &&

          // "Update Range Timing A timeout"
          WriteRegister16(RANGE_CONFIG__TIMEOUT_MACROP_A, EncodeTimeout( TimeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us))) &&

          
          ReadRegister(RANGE_CONFIG__VCSEL_PERIOD_B, &range_config_vcsel_period)			// "Update Macro Period for Range B VCSEL Period"
         )) 
	{
        return false;
    }
    macro_period_us = calcMacroPeriod(range_config_vcsel_period);			// "Update Macro Period for Range B VCSEL Period"

    // "Update MM Timing B timeout" (See earlier comment about MM Timing A timeout.)
    return WriteRegister16(MM_CONFIG__TIMEOUT_MACROP_B, EncodeTimeout( TimeoutMicrosecondsToMclks(1, macro_period_us))) &&
	
    // "Update Range Timing B timeout"
    WriteRegister16(RANGE_CONFIG__TIMEOUT_MACROP_B, EncodeTimeout(TimeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));
}

// Get the measurement timing budget in microseconds  based on VL53L1_SetMeasurementTimingBudgetMicroSeconds()
bool CVL53L1X::GetMeasurementTimingBudget(uint32_t &budget)
{
	// assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are enabled: VHV, PHASECAL, DSS1, RANGE
	uint8_t range_config_vcsel_period_a = 0;		// "Update Macro Period for Range A VCSEL Period"
	if (!ReadRegister(RANGE_CONFIG__VCSEL_PERIOD_A, &range_config_vcsel_period_a)) 
		return false;

	uint32_t macro_period_us = calcMacroPeriod(range_config_vcsel_period_a);

	uint16_t timeout_macrop_a = 0;
	if (!ReadRegister16(RANGE_CONFIG__TIMEOUT_MACROP_A, &timeout_macrop_a)) 
		return false;

	// "Get Range Timing A timeout"
	uint32_t range_config_timeout_us = TimeoutMclksToMicroseconds(DecodeTimeout(timeout_macrop_a), macro_period_us);

	budget = 2 * range_config_timeout_us + TimingGuard;
	return true;
}

// Start continuous ranging measurements, with the given inter-measurement period in milliseconds determining how often the sensor takes a measurement.
bool CVL53L1X::StartContinuous(uint32_t period_ms)
{
    uint32_t adjusted_period_ms = period_ms + (period_ms * 64 / 1000);			// fix for actual measurement period shorter than set

    // from VL53L1_set_inter_measurement_period_ms()
    return WriteRegister32(SYSTEM__INTERMEASUREMENT_PERIOD, adjusted_period_ms * osc_calibrate_val) &&
           WriteRegister(SYSTEM__INTERRUPT_CLEAR, 0x01) && // sys_interrupt_clear_range
           WriteRegister(SYSTEM__MODE_START, 0x40); // mode_range__timed
}

// Decode sequence step timeout in MCLKs from register value based on VL53L1_decode_timeout()
uint32_t CVL53L1X::DecodeTimeout(uint16_t reg_val)
{
    return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}

// Encode sequence step timeout register value from timeout in MCLKs
uint16_t CVL53L1X::EncodeTimeout(uint32_t timeout_mclks)
{
    
    uint32_t ls_byte = 0;		// encoded format: "(LSByte * 2^MSByte) + 1"
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) 
	{
        ls_byte = timeout_mclks - 1;
        while ((ls_byte & 0xFFFFFF00) > 0) 
		{
            ls_byte >>= 1;
            ms_byte++;
        }
        return (ms_byte << 8) | (ls_byte & 0xFF);
    }
    else 
        return 0;
}

// Convert sequence step timeout from macro periods to microseconds with given macro period in microseconds (12.12 format)
uint32_t CVL53L1X::TimeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
    return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

// Convert sequence step timeout from microseconds to macro periods with given// macro period in microseconds (12.12 format)
uint32_t CVL53L1X::TimeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
    return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

// Calculate macro period in microseconds (12.12 format) with given VCSEL period assumes fast_osc_frequency has been read and stored
uint32_t CVL53L1X::CalcMacroPeriod(uint8_t vcsel_period)
{
    // from VL53L1_calc_pll_period_us()
    // fast osc frequency in 4.12 format; PLL period in 0.24 format
    uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

    // from VL53L1_decode_vcsel_period()
    uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

    // VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
    uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
    macro_period_us >>= 6;
    macro_period_us *= vcsel_period_pclks;
    macro_period_us >>= 6;

    return macro_period_us;
}

// "Setup ranges after the first one in low power auto mode by turning off FW calibration steps and programming static values"
bool CVL53L1X::SetupManualCalibration(void)
{
    uint8_t saved_vhv_init = 0;
    uint8_t saved_vhv_timeout = 0;
    uint8_t phasecal_result_vcsel_start = 0;

    return // "save original vhv configs"
			ReadRegister(VHV_CONFIG__INIT, &saved_vhv_init) &&
			ReadRegister(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, &saved_vhv_timeout) &&

			// "disable VHV init"
			WriteRegister(VHV_CONFIG__INIT, saved_vhv_init & 0x7F) &&

			// "set loop bound to tuning param"
			WriteRegister(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,
							(saved_vhv_timeout & 0x03) + (3 << 2)) && // tuning parm default (LOWPOWERAUTO_VHV_LOOP_BOUND_DEFAULT)

			// "override phasecal"
			WriteRegister(PHASECAL_CONFIG__OVERRIDE, 0x01) &&
			ReadRegister(PHASECAL_RESULT__VCSEL_START, &phasecal_result_vcsel_start) &&
			WriteRegister(CAL_CONFIG__VCSEL_START, phasecal_result_vcsel_start);
}

// check if sensor has new reading available assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
bool CVL53L1X::DataReady(void)
{
    uint8_t gpio_tio_hv_status = 0;

    return ReadRegister(GPIO__TIO_HV_STATUS, &gpio_tio_hv_status) && ((gpio_tio_hv_status & 0x01) == 0);
}

// read - return last value measured by sensor
bool CVL53L1X::GetReading(uint16_t &reading_mm)
{
    uint8_t tries = 10;
	while (!DataReady()) 
	{
		tries--;
		Core.delay(1);
		if (tries == 0) 
			return(false);
	}

	uint8_t range_status = 0;

	if(!ReadRegister(RESULT__RANGE_STATUS, &range_status) ) 
		return(false);
	
	if( !ReadRegister16(RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0, &reading_mm))
		return(false);

    // "apply correction gain"  gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
    // Basically, this appears to scale the result by 2011/2048, or about 98%/ (with the 1024 added for proper rounding).
    reading_mm = ((uint32_t)reading_mm * 2011 + 0x0400) / 0x0800;

    if (!WriteRegister(SYSTEM__INTERRUPT_CLEAR, 0x01))		// sys_interrupt_clear_range
        return false;

    switch ((DeviceError)range_status) 
	{
	case RANGECOMPLETE:
        break;

	default:
        return false;
    }

    if (!calibrated) 
	{
        calibrated = SetupManualCalibration();
    }

    return calibrated;
}

bool CVL53L1X::ReadRegister(uint16_t reg, uint8_t *value)
{
   return(I2c.ReadRegisterAdd16(VL53L1X_ADDRESS_DEFAULT,reg,value,1));
}

bool CVL53L1X::ReadRegister16(uint16_t reg, uint16_t *value)
{
     bool RetVal;
	  RetVal = I2c.ReadRegisterAdd16(VL53L1X_ADDRESS_DEFAULT,reg,(uint8_t*)value,2);

	*value = BSWAP_16(*value);
// 	uint16_t v = 0;
//     uint8_t b[2] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF) };
//     if (!dev->transfer(b, 2, (uint8_t *)&v, 2)) {
//         return false;
//     }
//     value = be16toh(v);
     return(RetVal);
}
 
bool CVL53L1X::WriteRegister(uint16_t reg, uint8_t value)
{
   return(I2c.WriteRegisterBytesAdd16(VL53L1X_ADDRESS_DEFAULT,reg,&value,1));
}

bool CVL53L1X::WriteRegister16(uint16_t reg, uint16_t value)
{
   return(I2c.WriteRegisterBytesAdd16(VL53L1X_ADDRESS_DEFAULT,reg,(uint8_t*)&value,2));

//     uint8_t b[4] = { uint8_t(reg >> 8), uint8_t(reg & 0xFF), uint8_t(value >> 8), uint8_t(value & 0xFF) };
//     return dev->transfer(b, 4, nullptr, 0);
}

bool CVL53L1X::WriteRegister32(uint16_t reg, uint32_t value)
{
   return(I2c.WriteRegisterBytesAdd16(VL53L1X_ADDRESS_DEFAULT,reg,(uint8_t*)&value,4));
//     uint8_t b[6] = { uint8_t(reg >> 8),
//                      uint8_t(reg & 0xFF),
//                      uint8_t((value >> 24) & 0xFF),
//                      uint8_t((value >> 16) & 0xFF),
//                      uint8_t((value >>  8) & 0xFF),
//                      uint8_t((value)       & 0xFF) };
//     return dev->transfer(b, 6, nullptr, 0);
}

