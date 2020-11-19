
#ifndef _BL0940_H_
#define _BL0940_H_

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/*!
 * From BL0940 Calibration-free Metering IC Datasheet
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/datasheet/BL0940_V1.1_en.pdf
 */

//Electrical parameter register (read only) 
#define BL0940_I_FAST_RMS_REG_ADDR          0x00
#define BL0940_I_WAVE_REG_ADDR              0x01
#define BL0940_V_WAVE_REG_ADDR              0x03
#define BL0940_I_RMS_REG_ADDR               0x04
#define BL0940_V_RMS_REG_ADDR               0x06
#define BL0940_WATT_REG_ADDR                0x08
#define BL0940_CF_CNT_REG_ADDR              0x0A
#define BL0940_CORNER_REG_ADDR              0x0C
#define BL0940_TPS1_REG_ADDR                0x0E
#define BL0940_TPS2_REG_ADDR                0x0F
#define BL0940_READALL_REG_ADDR             0xAA

//User operated register (read and write)
#define BL0940_I_FAST_RMS_CTRL_REG_ADDR     0x10
#define BL0940_I_RMSOS_REG_ADDR             0x13
#define BL0940_WATTOS_REG_ADDR              0x15
#define BL0940_WA_CREEP_REG_ADDR            0x17
#define BL0940_MODE_REG_ADDR                0x18
#define BL0940_SOFT_RESET_REG_ADDR          0x19
#define BL0940_USR_WRPROT_REG_ADDR          0x1A
#define BL0940_TPS_CTRL_REG_ADDR            0x1B
#define BL0940_TPS2_A_REG_ADDR              0x1C
#define BL0940_TPS2_B_REG_ADDR              0x1D

#define BL0940_READ_CMD                     0x50    //0x58 in docs, bw-shp10 firmware send 0x50
#define BL0940_WRITE_CMD                    0xA0    //0xA8 in docs. Many thanks to Theo Arends @ https://github.com/arendst/ 
#define BL0940_UNLOCK_USER_REG              0x55

#define BL0940_FRM_HEAD_POS                 0x00
#define BL0940_FRM_ADDR_POS                 0x01
#define BL0940_FRM_SEND_MODE_HEAD_POS       0x02
#define BL0940_FRM_DATA_H_POS               0x04
#define BL0940_FRM_DATA_M_POS               0x03
#define BL0940_FRM_DATA_L_POS               0x02
#define BL0940_FRM_CRC_POS                  0x05

#define BL0940_IFRMS_FRM_POS                0x01
#define BL0940_IRMS_FRM_POS                 0x04
#define BL0940_VRMS_FRM_POS                 0x0A
#define BL0940_WATT_FRM_POS                 0x10
#define BL0940_CF_CNT_FRM_POS               0x16
#define BL0940_TPS1_FRM_POS                 0x1C
#define BL0940_TPS2_FRM_POS                 0x1F
#define BL0940_CRC_FRM_POS                  0x22

#define BL0940_IFRMS_HOLD_POS               0x00
#define BL0940_IRMS_HOLD_POS                0x03
#define BL0940_VRMS_HOLD_POS                0x06
#define BL0940_WATT_HOLD_POS                0x09
#define BL0940_CF_CNT_HOLD_POS              0x0C
#define BL0940_CORNER_HOLD_POS              0x0F
#define BL0940_TPS1_HOLD_POS                0x11
#define BL0940_TPS2_HOLD_POS                0x13

#define BL0940_MAX_REG_VALUE                0x1000000
#define BL0940_MAX_FRAME                    37
#define BL0940_SEND_MODE_FRAME_BYTES        35
#define BL0940_TIMEOUT                      50

#define BL0940_RMS_UPDATE_SEL_BITS          0b0000000100000000
#define BL0940_AC_FREQ_SEL_BIT              0b0000001000000000
#define BL0940_CF_UNABLE_BIT                0b0001000000000000
#define BL0940_RMS_REG_UPDATE_RATE_400MS    0b00000000
#define BL0940_RMS_REG_UPDATE_RATE_800MS    0b00000001
#define BL0940_AC_FREQ_50HZ                 0b00000000
#define BL0940_AC_FREQ_60HZ                 0b00000010
#define BL0940_CF_ENERGY_PULSE              0b00000000
#define BL0940_CF_ALARM_PULSE               0b00010000
#define BL0940_TEMPERATURE_SWITCH_BIT       0b1000000000000000
#define BL0940_ALARM_SWITCH_BIT             0b0100000000000000
#define BL0940_TEMPERATURE_SELECTOR_BIT     0b0011000000000000
#define BL0940_TEMPERATURE_INTERVAL_BIT     0b0000110000000000
#define BL0940_EXT_TEMPERATURE_THRHOLD_BIT  0b0000001111111111
#define BL0940_TEMPERATUTE_CTRL_ON          0b00000000
#define BL0940_TEMPERATUTE_CTRL_OFF         0b10000000
#define BL0940_TEMPERATURE_ALARM_ON         0b00000000
#define BL0940_OVERCURRENT_ALARM_ON         0b01000000
#define BL0940_TEMPERATURE_AUTO             0b00000000
#define BL0940_TEMPERATURE_SAME             0b00010000
#define BL0940_TEMPERATURE_INTERNAL         0b00100000
#define BL0940_TEMPERATURE_EXTERNAL         0b00110000
#define BL0940_TEMPERATURE_INTERVAL_50MS    0b00000000
#define BL0940_TEMPERATURE_INTERVAL_100MS   0b00000100
#define BL0940_TEMPERATURE_INTERVAL_200MS   0b00001000
#define BL0940_TEMPERATURE_INTERVAL_400MS   0b00001100

#define BL0940_TPS_CTRL_DEFAULT             0b0000011111111111      
                                                                    // Temperature measurement switch on
                                                                    // Temperature alarm ON - controled by MODE register
                                                                    // Automatic temperature measurement
                                                                    // 100mS Measurement interval
                                                                    // ~140°C External alarm threshold

#define BL0940_MODE_DEFAULT                 0b0000000000000000      // Energy pulse on CF pin
                                                                    // 50Hz AC frequency
                                                                    // 400mS RMS register update

#define BL0940_DEFAULT_BAUD_RATE            4800
#define BL0940_DEFAULT_PORT_CONFIG          SERIAL_8N1
#define BL0940_DEFAULT_RMS_UPDATE           400
#define BL0940_DEFAULT_AC_FREQUENCY         50

/*!
 *
 * From BL0940 Calibration-free Metering IC Datasheet
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/datasheet/BL0940_V1.1_en.pdf
 * 
 * and BL0940 Application Note
 * http://www.belling.com.cn/media/file_object/bel_product/BL0940/guide/BL0940_APPNote_TSSOP14_V1.04_EN.pdf
 * 
 * The current RMS conversion formula: I_RMS = 324004∗𝐼(𝐴) / 𝑉𝑟𝑒𝑓
 * The voltage RMS conversion formula: V_RMS = 79931∗𝑉(𝑉) / 𝑉𝑟𝑒𝑓
 * 𝑉𝑟𝑒𝑓 is the reference voltage, the typical value is 1.218V.
 * 
 */

#define BL0940_I_RMS_COEFFICIENT            3.7592E-06      // 3.75921284922408E-06
#define BL0940_V_RMS_COEFFICIENT            1.5238E-05      // 1.52381428982497E-05
#define BL0940_WATT_COEFFICIENT             3.6666E-04      // 0.000366664
#define BL0940_VOLTAGE_DIVIDER              1.995           // Blitzwolf BW-SHP10 socket
#define BL0940_SHUNT_RESISTANCE             1               // Value in milliOhms ( Blitzwolf BW-SHP10 socket )
#define BL0940_TEMPERATURE_COEFFICIENT      0.379464286     // From docs = 170/448
#define BL0940_ACTIVE_ENERGY_COEFFICIENT    419430.4        // From docs = 1638.8*256

#define BL0940_SAMPLING_FREQUENCY           1000000

enum bl0940States { NO_ERROR, TIMEOUT_ERR, BADFRAME_ERR, CRCFRAME_ERR, READY, COMM_ERROR };

/*!
 * One full measurement frame from device
 * 0x55           HEADER ( 0x58 in docs )
 * 0xc1 0x02 0x00 I_FAST_RMS
 * 0x00 0x00 0x00 I_RMS
 * 0x00 0x00 0x00 
 * 0x15 0x02 0x00 V_RMS
 * 0xda 0x01 0x00 
 * 0x00 0x00 0x00 WATT
 * 0x00 0x00 0x00 
 * 0x00 0x00 0x00 CF_CNT
 * 0x00 0x00 0x00 
 * 0xb2 0x01 0x00 TPS1
 * 0xfe 0x03 0x00 TPS2
 * 0xf1           CHECKSUM
 */
struct frame_t 
{
    uint8_t Bytes;
    uint8_t Index;
    uint8_t Payload[BL0940_MAX_FRAME];
    uint64_t lastRcv;
    uint64_t lastUpdate;
};

struct bl0940Config_t
{
    uint16_t modeRegister;
    uint16_t tpsctrlRegister;
    uint16_t rmsUpdate;
    uint8_t  acFreq;
    bool read;
    bool wprot;
};

class bl0940
{
    public:
        bl0940( HardwareSerial* hwSerial );
        ~bl0940();

        float getVoltage();
        float getCurrent();
        float getActivePower();
        float getReactivePower();
        float getApparentPower();
        float getEnergy();
        float getEnergy(uint32_t cf);
        float getEnergyDelta();
        float getPhaseAngle();
        float getPowerFactor(bool percentage=true);
        float getTemperature();
        bl0940States getState();
        bl0940Config_t getCurrentConfig();

        void setRMSUpdate(uint8_t val = BL0940_RMS_REG_UPDATE_RATE_400MS);
        void setACFrequency(uint8_t val = BL0940_AC_FREQ_50HZ);
        void setCFPinMode(uint8_t val = BL0940_CF_ENERGY_PULSE);
        void setTSwitch(uint8_t val = BL0940_TEMPERATUTE_CTRL_ON);
        void setASwitch(uint8_t val = BL0940_TEMPERATURE_ALARM_ON);
        void setTSelector(uint8_t val = BL0940_TEMPERATURE_AUTO);
        void setTInterval(uint8_t val = BL0940_TEMPERATURE_INTERVAL_100MS);
        void setExtTThreshold(uint16_t val);

        long readRegister(uint8_t regAddress);
        bool writeModeRegister();
        bool writeTpsRegister();
        bool writeRegister(uint8_t regAddress, uint32_t regValue);

        bool readValues();

    private:
        Stream* _serial;
        uint8_t _rawHolder[21];
        bl0940States _state;

        frame_t    _frame;
        bl0940Config_t _cfg;

        uint32_t  _lastCF;

        void _init();
        void _unlockWRProt();
        void _sendFrame();
        void _rcvFrame();
        void _readConfig();
        void _setParam(uint16_t* reg, uint16_t mask, uint16_t value);
        void _setParam(uint16_t* reg, uint16_t mask, uint8_t value);
        bool _crcCalc(uint8_t *buf, uint8_t bytes, bool set=false);
};

#endif  //_BL0940_H_
