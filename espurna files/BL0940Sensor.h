// -----------------------------------------------------------------------------
// BL0940 based power monitor
// -----------------------------------------------------------------------------


#if SENSOR_SUPPORT && BL0940_SUPPORT

#pragma once

#include "Arduino.h"
#include "BaseSensor.h"

#include <bl0940.h>

class BL0940Sensor : public BaseSensor {

    public:

        // ---------------------------------------------------------------------
        // Public
        // ---------------------------------------------------------------------

        BL0940Sensor(HardwareSerial* hwSerial): BaseSensor() 
        {
            _sensor_id = SENSOR_BL0940_ID;
            _serial = hwSerial;
            _dirty = true;
        }

        ~BL0940Sensor() 
        {
            if (_bl0940) delete _bl0940;
        }

        // Initialization method, must be idempotent
        void begin() 
        {
            if (!_dirty) return;

            if (_bl0940) delete _bl0940;
            _bl0940 = new bl0940(_serial);
            _count = 9;
            _ready = true;
            _dirty = false;
        }

        // Descriptive name of the sensor
        String description() 
        {
            char buffer[20];
            snprintf(buffer, sizeof(buffer), "BL0940 @ HwSerial");
            return String(buffer);
        }

        // Descriptive name of the slot # index
        String slot(unsigned char index) 
        {
            return description();
        };

        // Address of the sensor (it could be the GPIO or I2C address)
        String address(unsigned char index) 
        {
            return String("1");
        }

        // Type for slot # index
        unsigned char type(unsigned char index) 
        {
            if (index == 0) return MAGNITUDE_VOLTAGE;
            if (index == 1) return MAGNITUDE_CURRENT;
            if (index == 2) return MAGNITUDE_POWER_ACTIVE;
            if (index == 3) return MAGNITUDE_POWER_APPARENT;
            if (index == 4) return MAGNITUDE_POWER_REACTIVE;
            if (index == 5) return MAGNITUDE_POWER_FACTOR;
            if (index == 6) return MAGNITUDE_ENERGY;
            if (index == 7) return MAGNITUDE_ENERGY_DELTA;
            if (index == 8) return MAGNITUDE_TEMPERATURE;
            return MAGNITUDE_NONE;
        }

        signed char decimals(unsigned char type) 
        { 
	    // These numbers of decimals correspond to maximum sensor resolution settings
            switch (type) 
            {  
                case MAGNITUDE_VOLTAGE:         return 1; 
                case MAGNITUDE_CURRENT:         return 3;
                case MAGNITUDE_POWER_ACTIVE:    return 1;
                case MAGNITUDE_POWER_APPARENT:  return 1;
                case MAGNITUDE_POWER_REACTIVE:  return 1;
                case MAGNITUDE_POWER_FACTOR:    return 0;
                case MAGNITUDE_ENERGY:          return 3;
                case MAGNITUDE_ENERGY_DELTA:    return 3;
                case MAGNITUDE_TEMPERATURE:     return 2;
            }
            return -1;
	    }

        // Current value for slot # index
        double value(unsigned char index) 
        {
            if (index == 0) return _bl0940->getVoltage();
            if (index == 1) return _bl0940->getCurrent();
            if (index == 2) return _bl0940->getActivePower();
            if (index == 3) return _bl0940->getApparentPower();
            if (index == 4) return _bl0940->getReactivePower();
            if (index == 5) return _bl0940->getPowerFactor();
            if (index == 6) return _bl0940->getEnergy();
            if (index == 7) return _bl0940->getEnergyDelta();
            if (index == 8) return _bl0940->getTemperature();
            return 0.0;
        }

        virtual void pre() 
        {
            _error = (_bl0940->readValues()) ? SENSOR_ERROR_OK : SENSOR_ERROR_TIMEOUT;
        }

    protected:

        // ---------------------------------------------------------------------
        // Protected
        // ---------------------------------------------------------------------

        HardwareSerial * _serial = NULL;
        bl0940* _bl0940 = NULL;

};

#endif // SENSOR_SUPPORT && BL0940_SUPPORT
