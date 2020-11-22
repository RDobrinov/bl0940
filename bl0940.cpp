#include "bl0940.h"

/*!
 * bl0940::bl0940
 * Class constructor
 *
 * @param stream Pointer to hardware serial connected to BL0940
 * 
*/
bl0940::bl0940(Stream* stream)
{
    this->_serial = stream;
    _softSerial = true;
    static_cast<HardwareSerial*>(_serial)->begin(BL0940_DEFAULT_BAUD_RATE, BL0940_DEFAULT_PORT_CONFIG);
    _init();
}

/*!
 * bl0940::bl0940
 * Class constructor
 *
 * @param rxPin RXD pin for SoftwareSerial
 * @param txPin TXD pin for SoftwareSerial
 * 
*/
bl0940::bl0940(uint8_t rxPin, uint8_t txPin)
{
    SoftwareSerial *stream = new SoftwareSerial(rxPin, txPin);
    _serial = stream;
    _softSerial = true;
    static_cast<SoftwareSerial*>(_serial)->begin(BL0940_DEFAULT_BAUD_RATE, BL0940_DEFAULT_PORT_CONFIG);
    static_cast<SoftwareSerial*>(_serial)->listen();
    _init();
}

/*!
 * bl0940::~bl0940
 * Class destructor
 * 
*/
bl0940::~bl0940()
{
    if(_softSerial) delete this->_serial;
}

/*!
 * bl0940::getVoltage
 * Get measured voltage
 *
 * @return Last measured voltage in Volts
 * 
 * Calculations form BL0940 Application Note
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/guide/BL0940_APPNote_TSSOP14_V1.04_EN.pdf
 * Voltage(V) = V_RMS_Reg∗Vref∗(R2+R1) / 79931∗R1∗1000
 * Vref=1.218V
 * (R2+R1)/R1*1000 = BL0940_VOLTAGE_DIVIDER. Keep in mind *both R1 and R2 are in kiloohms*
 * Vref/79931 = BL0940_V_RMS_COEFFICIENT
 */
float bl0940::getVoltage()
{
    return (((uint32_t)_rawHolder[BL0940_VRMS_HOLD_POS+2]<<16) | 
            ((uint32_t)_rawHolder[BL0940_VRMS_HOLD_POS+1]<<8) | 
            ((uint32_t)_rawHolder[BL0940_VRMS_HOLD_POS])) * (float)BL0940_V_RMS_COEFFICIENT * (float)BL0940_VOLTAGE_DIVIDER;
}

/*!
 * bl0940::getCurrent
 * Get measured current in Amperes
 *
 * @return Last measured current
 * 
 * Calculations form BL0940 Application Note
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/guide/BL0940_APPNote_TSSOP14_V1.04_EN.pdf
 * Current(A) = I_RMS_Reg∗Vref / 324004∗RL
 * Vref = 1.218V
 * Vref/324004 = BL0940_I_RMS_COEFFICIENT
 * RL = BL0940_SHUNT_RESISTANCE
 */
float bl0940::getCurrent()
{
    return (((uint32_t)_rawHolder[BL0940_IRMS_HOLD_POS+2]<<16) | 
            ((uint32_t)_rawHolder[BL0940_IRMS_HOLD_POS+1]<<8) | 
            ((uint32_t)_rawHolder[BL0940_IRMS_HOLD_POS])) * (float)BL0940_I_RMS_COEFFICIENT / (float)BL0940_SHUNT_RESISTANCE;
}

/*!
 * bl0940::getActivePower
 * Get measured Active Power in Watts
 *
 * @return Last measured active power
 *
 * Calculations form BL0940 Application Note
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/guide/BL0940_APPNote_TSSOP14_V1.04_EN.pdf
 * Active Power(W) = WATT_Reg∗VrefVref∗(R2+R1) / 4046∗RL∗R1∗1000
 * Vref = 1.218V
 * Vref*Vref/4046 = BL0940_WATT_COEFFICIENT
 * (R2+R1)/R1*1000 = BL0940_VOLTAGE_DIVIDER
 * RL = BL0940_SHUNT_RESISTANCE
 */
float bl0940::getActivePower()
{
    long powerValue = 0x7FFFFF & (((uint32_t)_rawHolder[BL0940_WATT_HOLD_POS+2]<<16) | ((uint32_t)_rawHolder[BL0940_WATT_HOLD_POS+1]<<8) | ((uint32_t)_rawHolder[BL0940_WATT_HOLD_POS]));
    powerValue = (_rawHolder[BL0940_WATT_HOLD_POS+2] & 0x80) ? powerValue - 0x7FFFFF : powerValue;
    return (float)powerValue * (float)BL0940_WATT_COEFFICIENT * (float)BL0940_VOLTAGE_DIVIDER/ (float)BL0940_SHUNT_RESISTANCE;
}

/*!
 * bl0940::getReactivePower
 * Calculate Reactive Power in VAr
 *
 * @return Calculated Reactive power
 * 
 * Calculations from Power Triangle
 */
float bl0940::getReactivePower()
{
    return getActivePower()*tan(getPhaseAngle());
}

/*!
 * bl0940::getApparentPower
 * Calculate Apparent Power in VA
 *
 * @return Calculated Apparent power
 * 
 * Calculations from Power Triangle
 */
float bl0940::getApparentPower()
{
    return getActivePower()/getPowerFactor(false);
}

/*! Energy calculation */

/*!
 * bl0940::getEnergy
 * Get measured Energy in kWh
 *
 * @return Total measured Energy
 */
float bl0940::getEnergy()
{
    return getEnergy(((uint32_t)_rawHolder[BL0940_CF_CNT_HOLD_POS+2]<<16) | ((uint32_t)_rawHolder[BL0940_CF_CNT_HOLD_POS+1]<<8) | ((uint32_t)_rawHolder[BL0940_CF_CNT_HOLD_POS]));
}

/*!
 * bl0940::getEnergy
 * Calculate Energy in kWh for number of CF pulses
 *
 * @param Counted Energy pulses
 * @return Total measured Energy
 *
 * Calculations form BL0940 Application Note
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/guide/BL0940_APPNote_TSSOP14_V1.04_EN.pdf
 * tcf = 1638.4*256 / WATT
 * Active Energy (kWh ) = CF * 1638.4*256*Vref*Vref*(R2+R1) / 3600000*4046*RL*R1*1000
 * Vref = 1.218V
 * Vref*Vref/4046 = BL0940_WATT_COEFFICIENT
 * (R2+R1)/R1*1000 = BL0940_VOLTAGE_DIVIDER
 * RL = BL0940_SHUNT_RESISTANCE
 * 1638.4*256 = BL0940_ACTIVE_ENERGY_COEFFICIENT
 *                             BL0940_ACTIVE_ENERGY_COEFFICIENT * BL0940_WATT_COEFFICIENT * BL0940_VOLTAGE_DIVIDER
 * Active Energy (kWh ) = CF * -----------------------------------------------------------------------------------
 *                                                    3600000 * BL0940_SHUNT_RESISTANCE
 */
float bl0940::getEnergy(uint32_t cf)
{
    return (float)cf * ((BL0940_ACTIVE_ENERGY_COEFFICIENT * BL0940_WATT_COEFFICIENT * (float) BL0940_VOLTAGE_DIVIDER) / ( 3600000.0 * (float) BL0940_SHUNT_RESISTANCE ));
}

/*!
 * bl0940::getEnergyDelta
 * Calculate Energy delta in kWh between two function calls
 *
 * @return Measured Energy delta 
 */
float bl0940::getEnergyDelta()
{
    float energyDelta = 0;
    uint32_t cf = ((uint32_t)_rawHolder[BL0940_CF_CNT_HOLD_POS+2]<<16) | ((uint32_t)_rawHolder[BL0940_CF_CNT_HOLD_POS+1]<<8) | ((uint32_t)_rawHolder[BL0940_CF_CNT_HOLD_POS]);
    energyDelta = ( cf < _lastCF ) ? getEnergy( 0xFFFFFF - _lastCF + cf ) : getEnergy( cf - _lastCF );
    _lastCF = cf;
    return energyDelta;
}
/*! End Energy calculation */

/*!
 * bl0940::getPhaseAngle
 * Get Phase Angle in Radians
 *
 * @return Phase Angle
 * 
 * Calculations form BL0940 Calibration-free Metering IC Datasheet
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/datasheet/BL0940_V1.1_en.pdf
 * PA = 2*pi*CORNER*Fc/Fo
 * Fc = AC Frequency
 * Fo = BL0940_SAMPLING_FREQUENCY
 */
float bl0940::getPhaseAngle()
{
  return 2*PI*((uint16_t)_rawHolder[BL0940_CORNER_HOLD_POS+1]<<8 | _rawHolder[BL0940_CORNER_HOLD_POS])*(float)_cfg.acFreq/(float)BL0940_SAMPLING_FREQUENCY;
}

/*!
 * bl0940::getPowerFactor
 * Get Power factor
 *
 * @param Result in percentage ( default true )
 * @return Power factor 
 */
float bl0940::getPowerFactor(bool percentage)
{   
    return cos(getPhaseAngle()) * (float)((percentage) ? 100 : 1);
}

/*!
 * bl0940::getTemperature
 * Get Intrenal temerature in °C
 *
 * @return Intrenal temperature 
 * 
 * Calculations form BL0940 Calibration-free Metering IC Datasheet
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/datasheet/BL0940_V1.1_en.pdf
 * Tx=(170/448)(TPS1/2-32)-45
 */
float bl0940::getTemperature()
{
  return BL0940_TEMPERATURE_COEFFICIENT * 
        ((float)(((uint32_t)(_rawHolder[BL0940_TPS1_HOLD_POS+1] & 0x0F)<<8) | ((uint32_t)_rawHolder[BL0940_TPS1_HOLD_POS])) / 2 -32) - 45;
}

/*!
 * bl0940::getState
 * Get last communication state 
 *
 * @return bl0940States ( see header file ) 
 */
bl0940States bl0940::getState()
{
    return _state;
}

/*!
 * bl0940::getCurrentConfig
 * Get current register configuration 
 *
 * @return bl0940Config_t structure ( see header file ) 
 */
bl0940Config_t bl0940::getCurrentConfig()
{
    return _cfg;
}

/*!
 * bl0940::setRMSUpdate
 * Set internal RMS register update rate
 * !!!Do not update chip mode register!!!
 *
 * @param BL0940_RMS_REG_UPDATE_RATE_400MS or BL0940_RMS_REG_UPDATE_RATE_800MS
 */
void bl0940::setRMSUpdate(uint8_t val)
{
    if((val != BL0940_RMS_REG_UPDATE_RATE_400MS) && (val != BL0940_RMS_REG_UPDATE_RATE_800MS) ) return;
    _setParam(&_cfg.modeRegister, BL0940_RMS_UPDATE_SEL_BITS, val);
    _cfg.rmsUpdate = (val == BL0940_RMS_REG_UPDATE_RATE_400MS) ? 400 : 800;
}

/*!
 * bl0940::setACFrequency
 * Set internal AC frequency selector
 * !!!Do not update chip mode register!!!
 *
 * @param BL0940_AC_FREQ_50HZ or BL0940_AC_FREQ_60HZ
 */
void bl0940::setACFrequency(uint8_t val)
{
    if((val != BL0940_AC_FREQ_50HZ) && (val != BL0940_AC_FREQ_60HZ) ) return;
    _setParam(&_cfg.modeRegister, BL0940_AC_FREQ_SEL_BIT, val);
}

/*!
 * bl0940::setCFPinMode
 * Set internal CF output function
 * !!!Do not update chip mode register!!!
 *
 * @param BL0940_CF_ENERGY_PULSE or BL0940_CF_ALARM_PULSE
 */
void bl0940::setCFPinMode(uint8_t val)
{
    if((val != BL0940_CF_ENERGY_PULSE) && (val != BL0940_CF_ALARM_PULSE) ) return;
    _setParam(&_cfg.modeRegister, BL0940_CF_UNABLE_BIT, val);    
}

/*!
 * bl0940::setTSwitch
 * Set internal Temperature switch - TPS_CTRL[15]
 * !!!Do not update chip Temperature mode control register!!!
 *
 * @param BL0940_TEMPERATUTE_CTRL_ON or BL0940_TEMPERATUTE_CTRL_OFF
 */
void bl0940::setTSwitch(uint8_t val)
{
    if((val != BL0940_TEMPERATUTE_CTRL_ON) && (val != BL0940_TEMPERATUTE_CTRL_OFF) ) return;
    _setParam(&_cfg.tpsctrlRegister, BL0940_TEMPERATURE_SWITCH_BIT, val); 
}

/*!
 * bl0940::setTSwitch
 * Set internal Alarm switch - TPS_CTRL[14]
 * !!!Do not update chip Temperature mode control register!!!
 *
 * @param BL0940_TEMPERATUTE_CTRL_ON or BL0940_TEMPERATUTE_CTRL_OFF
 */
void bl0940::setASwitch(uint8_t val)
{
    if((val != BL0940_TEMPERATURE_ALARM_ON) && (val != BL0940_OVERCURRENT_ALARM_ON) ) return;
    _setParam(&_cfg.tpsctrlRegister, BL0940_ALARM_SWITCH_BIT, val);    
}

/*!
 * bl0940::setTSwitch
 * Set internal Temperature measurement selection - TPS_CTRL[13:12]
 * !!!Do not update chip Temperature mode control register!!!
 *
 * @param One of BL0940_TEMPERATURE_AUTO, BL0940_TEMPERATURE_SAME, BL0940_TEMPERATURE_INTERNAL or BL0940_TEMPERATURE_EXTERNAL
 */
void bl0940::setTSelector(uint8_t val)
{
    if((val != BL0940_TEMPERATURE_AUTO) && (val != BL0940_TEMPERATURE_SAME) 
       && (val != BL0940_TEMPERATURE_INTERNAL) && (val != BL0940_TEMPERATURE_EXTERNAL) ) return;
    _setParam(&_cfg.tpsctrlRegister, BL0940_TEMPERATURE_SELECTOR_BIT, val);    
}

/*!
 * bl0940::setTInterval
 * Set internal Temperature measurement interval - TPS_CTRL[11:10]
 * !!!Do not update chip Temperature mode control register!!!
 *
 * @param One of BL0940_TEMPERATURE_INTERVAL_50MS, BL0940_TEMPERATURE_INTERVAL_100MS, BL0940_TEMPERATURE_INTERVAL_200MS or BL0940_TEMPERATURE_INTERVAL_400MS
 */
void bl0940::setTInterval(uint8_t val)
{
    if((val != BL0940_TEMPERATURE_INTERVAL_50MS) && (val != BL0940_TEMPERATURE_INTERVAL_100MS) 
       && (val != BL0940_TEMPERATURE_INTERVAL_200MS) && (val != BL0940_TEMPERATURE_INTERVAL_400MS) ) return;
    _setParam(&_cfg.tpsctrlRegister, BL0940_TEMPERATURE_INTERVAL_BIT, val);
}

/*!
 * bl0940::setExtTThreshold
 * Set internal External temperature measurement alarm threshold setting - TPS_CTRL[9:0]
 * !!!Do not update chip Temperature mode control register!!!
 *
 * @param Alarm threshold value in SAR ADC ( full scale is 1024 )
 */
void bl0940::setExtTThreshold(uint16_t val)
{
    val &= 0x3FF;
    _setParam(&_cfg.modeRegister, BL0940_CF_UNABLE_BIT, (uint8_t) BL0940_CF_ALARM_PULSE);
    _setParam(&_cfg.tpsctrlRegister, BL0940_EXT_TEMPERATURE_THRHOLD_BIT, val);
}

/*!
 * bl0940::readRegister
 * Read value from BL0940 register
 *
 * @param Register address ( see definitions in header file )
 * @return Register value
 */
long bl0940::readRegister(uint8_t regAddress)
{
    long regValue = 0;
    _frame.Bytes = 6;
    _frame.Index = 2;
    _frame.Payload[BL0940_FRM_HEAD_POS] = BL0940_READ_CMD;
    _frame.Payload[BL0940_FRM_ADDR_POS] = regAddress;
    _sendFrame();
    _rcvFrame();
    if( _state == NO_ERROR )
    {   
        regValue = ((uint32_t)_frame.Payload[BL0940_FRM_DATA_H_POS]<<16 | 
                    (uint32_t)_frame.Payload[BL0940_FRM_DATA_M_POS]<<8 | 
                    (uint32_t)_frame.Payload[BL0940_FRM_DATA_L_POS]);
        regValue = regValue & 0x007FFFFF;
        if( (_frame.Payload[BL0940_FRM_ADDR_POS] == BL0940_I_WAVE_REG_ADDR) || 
            (_frame.Payload[BL0940_FRM_ADDR_POS] == BL0940_V_WAVE_REG_ADDR) || 
            (_frame.Payload[BL0940_FRM_ADDR_POS] == BL0940_WATT_REG_ADDR) )
        {
            return ((_frame.Payload[BL0940_FRM_DATA_H_POS] & 0x80)) ? regValue-0x7FFFFF : regValue;
        }
        return regValue;
    }
    return regValue | BL0940_MAX_REG_VALUE;
}

/*!
 * bl0940::writeRegister
 * Write single BL0940 register and check value written
 * No sanity check, so be careful ( should be in private members in future releases )
 *
 * @param Register address ( see definitions in header file )
 * @param Value to write info register
 * @return Success
 */
bool bl0940::writeRegister(uint8_t regAddress, uint32_t regValue)
{
    if((BL0940_I_FAST_RMS_CTRL_REG_ADDR > regAddress) || (BL0940_TPS2_B_REG_ADDR < regAddress)) return false;
    _unlockWRProt();
    if (_cfg.wprot) return false;
    _frame.Bytes = 6;
    _frame.Index = 6;
    _frame.Payload[BL0940_FRM_HEAD_POS] = BL0940_WRITE_CMD;
    _frame.Payload[BL0940_FRM_ADDR_POS] = regAddress;
    _frame.Payload[BL0940_FRM_DATA_H_POS] = ( (regValue>>16) & 0xFF );
    _frame.Payload[BL0940_FRM_DATA_M_POS] = ( (regValue>>8) & 0xFF );
    _frame.Payload[BL0940_FRM_DATA_L_POS] = ( regValue & 0xFF );
    _crcCalc(_frame.Payload, _frame.Bytes, true);
    _sendFrame();
    return (readRegister(regAddress) == regValue);
}

/*!
 * bl0940::writeModeRegister
 * Write internal mode register value to chip
 * Execute after setRMSUpdate, setACFrequency or setCFPinMode updates
 * to write modified internal params
 *
 * @return Success
 */
bool bl0940::writeModeRegister()
{
    if(writeRegister(BL0940_MODE_REG_ADDR, _cfg.modeRegister))
    {
        _cfg.rmsUpdate = (_cfg.modeRegister & BL0940_RMS_UPDATE_SEL_BITS) ? 800 : 400;
        _cfg.acFreq = (_cfg.modeRegister & BL0940_AC_FREQ_SEL_BIT) ? 60 : 50;
        return true;
    }
    return false;
}

/*!
 * bl0940::writeTpsRegister
 * Write Temperature mode control register value to chip
 * Execute after setTSwitch, setASwitch, setTSelector, setTInterval or setExtTThreshold
 * to write modified internal params
 *
 * @return Success
 */
bool bl0940::writeTpsRegister()
{
    return writeRegister(BL0940_TPS_CTRL_REG_ADDR, _cfg.tpsctrlRegister);
}

/*!
 * vbox@bare-metal:~$ hexdump -v -e '1/1 "0x%02x "'  < /dev/ttyUSB0
 * 0x50 0xaa 0x55 0xdd 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x06 0x02 0x00 0x4a 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xb2 0x01 0x00 0xfe 0x03 0x00 0x73
 * 0x50 0xaa 0x55 0xc1 0x02 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x15 0x02 0x00 0xda 0x01 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0x00 0xb2 0x01 0x00 0xfe 0x03 0x00 0xf1
 * -----
 * OTA update: Start
 * OTA update: Progress 100%
 * OTA update: End
 * Ncat: Connection from 10.0.0.129.
 * Ncat: Connection from 10.0.0.129:61535.
 * [SND] 50 AA 
 * [RCV] 50 AA 55 A2 02 00 00 00 00 00 00 00 5A E1 70 58 02 00 F4 FF FF 00 00 00 00 00 00 00 00 00 D1 01 00 00 00 00 ED
 * [SND] 50 AA 
 * [RCV] 50 AA 55 FA 02 00 00 00 00 28 00 00 69 3A 6F 4A 02 00 00 00 00 0D 00 00 00 00 00 00 00 00 D3 01 00 00 00 00 F7 
 * ---
 * [SND] 50 AA 
 * [RCV] 50 AA 55 5C 03 00 33 00 00 03 00 00 4E 69 6E E4 01 00 F4 FF FF 01 00 00 00 00 00 00 00 00 D2 01 00 00 00 00 F5 
 * [RHL] 5C 03 00 33 00 00 4E 69 6E F4 FF FF 00 00 00 8F 03 D2 01 00 00 
 * [SND] 50 0C 
 * [RCV] 50 0C A7 03 00 F9 
 * _state: NO_ERROR
 * [  V] 219.97 [COS] 0.96 [PHA] 0.29 [TMP] 31.27 [WAT] -0.01 [ENE] 0.00 [DLT] 0.00
 * [CFG] AC_FREQ: 50, RMS_UPDATE: 400, MODE_REG: [SND] 50 18 
 * [RCV] 50 18 00 00 00 97 
 * _state: NO_ERROR
 * 0
 */

/*!
 * bl0940::readValues
 * Read full electrical parameter data packet, Phase Angle register value,
 * parse received frame and copy data to internal holder
 *
 * @return Success
 */
bool bl0940::readValues()
{
    if(( (millis() - _frame.lastRcv) < _cfg.rmsUpdate ) && (_frame.Payload[BL0940_FRM_ADDR_POS] == BL0940_READALL_REG_ADDR )) return true;
    _frame.Bytes = BL0940_MAX_FRAME;
    _frame.Index = 2;
    _frame.Payload[BL0940_FRM_HEAD_POS] = BL0940_READ_CMD;
    _frame.Payload[BL0940_FRM_ADDR_POS] = BL0940_READALL_REG_ADDR;
    _sendFrame();
    _rcvFrame();
    if( _state == NO_ERROR )
    {
        long pa = readRegister(BL0940_CORNER_REG_ADDR);
        if(pa < BL0940_MAX_REG_VALUE)
        {
          _rawHolder[BL0940_CORNER_HOLD_POS] = ( pa & 0xFF );
          _rawHolder[BL0940_CORNER_HOLD_POS + 1] = ( pa>>8 & 0xFF );
        }
    }
    return (_state == NO_ERROR);
}

/*! private */
/*!
 * bl0940::_init
 * Internal variables initialization
 */
void bl0940::_init()
{
    _cfg = {
        .modeRegister = BL0940_MODE_DEFAULT,
        .tpsctrlRegister = BL0940_TPS_CTRL_DEFAULT,
        .rmsUpdate = BL0940_DEFAULT_RMS_UPDATE,
        .acFreq = BL0940_DEFAULT_AC_FREQUENCY,
        .read = false,
        .wprot = true,
    };
    _lastCF = 0;
    _frame.lastRcv = 0;
    setTSelector(BL0940_TEMPERATURE_INTERNAL);
    _state = ( writeModeRegister() &&  writeTpsRegister() ) ? NO_ERROR : COMM_ERROR;
}

/*!
 * bl0940::_unlockWRProt
 * Unlock user operation register and populate write protection status
 */
void bl0940::_unlockWRProt()
{
    if(!_cfg.wprot) return;
    _frame.Bytes = 6;
    _frame.Index = 6;
    _frame.Payload[BL0940_FRM_HEAD_POS] = BL0940_WRITE_CMD;
    _frame.Payload[BL0940_FRM_ADDR_POS] = BL0940_USR_WRPROT_REG_ADDR;
    _frame.Payload[BL0940_FRM_DATA_H_POS] = 0x00;
    _frame.Payload[BL0940_FRM_DATA_M_POS] = 0x00;
    _frame.Payload[BL0940_FRM_DATA_L_POS] = BL0940_UNLOCK_USER_REG;
    _crcCalc(_frame.Payload, _frame.Bytes, true);
    _sendFrame();
    _cfg.wprot = ( (readRegister(BL0940_USR_WRPROT_REG_ADDR) == BL0940_UNLOCK_USER_REG) ? false : true );
}

/*!
 * bl0940::_sendFrame
 * Send prepared frame to chip
 */
void bl0940::_sendFrame()
{
    _serial->write(_frame.Payload, _frame.Index);
}

/*!
 * bl0940::_rcvFrame
 * Receive and parse single frame from BL0940 chip.
 * Update internal status
 * 
 */
void bl0940::_rcvFrame()
{
    int bytesAvailable;
    _frame.lastRcv = millis();
    while( (_frame.Index < _frame.Bytes) && !((millis()-_frame.lastRcv) > BL0940_TIMEOUT))
    {
        bytesAvailable = _serial->available();
        if( bytesAvailable > 0)
        {
            for( int bytesIndex=0; bytesIndex<bytesAvailable; bytesIndex++)
            {
                _frame.Payload[_frame.Index++] = (uint8_t)_serial->read();
            }
            _frame.lastRcv = millis();         //Reset timeout counter
        }
    }

    if( _frame.Index != _frame.Bytes )
    {
        _state = BADFRAME_ERR;
        return;
    }

    if(BL0940_READALL_REG_ADDR == _frame.Payload[BL0940_FRM_ADDR_POS])
    {
        //***Special case*** CRC is calculated only for incomming frame bytes
        if(!_crcCalc((_frame.Payload+BL0940_FRM_SEND_MODE_HEAD_POS), BL0940_SEND_MODE_FRAME_BYTES))
        {
            _state = CRCFRAME_ERR;
            return;    
        }
        memcpy( _rawHolder, (_frame.Payload+BL0940_FRM_SEND_MODE_HEAD_POS + BL0940_IFRMS_FRM_POS), 6);                                // I_FAST_RMS and I_RMS values to holder
        memcpy( (_rawHolder + BL0940_VRMS_HOLD_POS), (_frame.Payload+BL0940_FRM_SEND_MODE_HEAD_POS + BL0940_VRMS_FRM_POS), 3 );       // V_RMS to holder
        memcpy( (_rawHolder + BL0940_WATT_HOLD_POS), (_frame.Payload+BL0940_FRM_SEND_MODE_HEAD_POS + BL0940_WATT_FRM_POS), 3 );       // WATT to holder
        memcpy( (_rawHolder + BL0940_CF_CNT_HOLD_POS), (_frame.Payload+BL0940_FRM_SEND_MODE_HEAD_POS + BL0940_CF_CNT_FRM_POS), 3 );   // CF to holder
        memcpy( (_rawHolder + BL0940_TPS1_HOLD_POS), (_frame.Payload+BL0940_FRM_SEND_MODE_HEAD_POS + BL0940_TPS1_FRM_POS), 2 );       // TPS1 to holder
        memcpy( (_rawHolder + BL0940_TPS2_HOLD_POS), (_frame.Payload+BL0940_FRM_SEND_MODE_HEAD_POS + BL0940_TPS2_FRM_POS), 2 );       // TPS1 to holder
    }
    else
    {
        if(!_crcCalc(_frame.Payload, _frame.Bytes))
        {
            _state = CRCFRAME_ERR;
            return;    
        }
    }
    _state = NO_ERROR;
    return;
}

/*!
 * bl0940::_readConfig
 * Not used. Read and populate User mode selection register and Temperature mode control register values
 * to internal register values. *** NOT TESTED ***
 * 
 */
void bl0940::_readConfig()
{
    long readout = readRegister(BL0940_MODE_REG_ADDR);
    if( readout < 0xFFFF )
    {
        _cfg.modeRegister = readout & 0xFFFF;
        _cfg.read = true;
    }
    else
    {
        _cfg.read = false;
    }
    
    readout = readRegister(BL0940_TPS_CTRL_REG_ADDR);
    if( readout < 0xFFFF )
    {
        _cfg.tpsctrlRegister = readout & 0xFFFF;
        _cfg.read &= true;
    }
    else
    {
        _cfg.read = false;
    }
}

/*!
 * bl0940::_setParam
 * Set single param to internal register
 * 
 * @param Register address ( see header file )
 * @param Bits mask ( see header file )
 * @param Value to set ( from header file )
 */
void bl0940::_setParam(uint16_t* reg, uint16_t mask, uint16_t value)
{
    *reg &= ~mask;
    *reg |= value;
}

/*!
 * bl0940::_setParam
 * Set single param to internal register
 * 
 * @params same as above instead 8bit values for mode register
 */
void bl0940::_setParam(uint16_t* reg, uint16_t mask, uint8_t value)
{
    _setParam(reg, mask, (uint16_t)((uint16_t)value<<8));
}

/*!
 * bl0940::_crcCalc
 * Calculate and check frame checksum
 * 
 * @param Pointer to frame
 * @param Number of bytes for calcluation
 * @param Set or check frame checksum
 */
bool bl0940::_crcCalc( uint8_t *buf, uint8_t bytes, bool set )
{
    uint8_t crc = 0;
    /*!
     * A strange special case. When BL0940_READ_CMD with BL0940_READALL_REG_ADDR is sent to chip, 
     * responce frame CRC is calculated with BL0940_READALL_REG_ADDR excluded.
     * To do right check first two bytes BL0940_READ_CMD and BL0940_READALL_REG_ADDR are stripped
     * and calculaton begin at BL0940_FRM_SEND_MODE_HEAD_POS with BL0940_READ_CMD as initial value.
     * See void bl0940::_rcvFrame()
     */
    if( bytes == BL0940_SEND_MODE_FRAME_BYTES)
    {
        crc = BL0940_READ_CMD;
    }
    for( int i = 0; i < (bytes-1); i++ )
    {
        crc += buf[i];
    }
    crc = ~crc;
    if(set)
    {
        buf[bytes-1] = crc;
        return true;
    }
    return ( crc == buf[bytes-1]);
}
