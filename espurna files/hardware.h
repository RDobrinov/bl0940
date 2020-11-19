#elif defined(BW_SHP10_16A)

    // Info
    #define MANUFACTURER        "BlitzWolf"
    #define DEVICE              "SHP10-16A"



    // Buttons
    #define BUTTON1_PIN         13
    #define BUTTON1_MODE        BUTTON_PUSHBUTTON | BUTTON_SET_PULLUP | BUTTON_DEFAULT_HIGH
    #define BUTTON1_RELAY       1

    // Hidden button will enter AP mode if dblclick and reset the device when long-long-clicked
    #define RELAY1_PIN          5
    #define RELAY1_TYPE         RELAY_TYPE_NORMAL

    // Light
    #define LED1_PIN            4
    #define LED1_PIN_INVERSE    1
    #define LED1_MODE           LED_MODE_WIFI

    // BL0940
    #define SENSOR_SUPPORT      1
    #ifndef BL0940_SUPPORT
    #define BL0940_SUPPORT      1
    #endif

    #ifndef BL0940_SENSOR_RMS
    #define BL0940_SENSOR_RMS   1   // 0 for 400msec, 1 for 800msec
    #endif
    #ifndef BL0940_SENSOR_FREQ
    #define BL0940_SENSOR_FREQ  0   // 0 for 50Hz, 1 for 60Hz
    #endif

    #define DEBUG_SERIAL_SUPPORT    0
    #define SENSOR_DEBUG            0
//
