#ifndef	_COMMON_H
#define	_COMMON_H

#include <SLFS.h>
#include <WiFi.h>

// IO definitions
// Volage on analog pins must be < 1.46V <=> 4096 (12 bits ADC)

#define LED_RED          29    // GPIO 9 + PWM
#define LED_BLUE         8     // GPIO 7 
#define LED_GREEN        27    // GPIO 8

#define GPIO3    2             // GPIO + Analog Input
#define GPIO4    6             // GPIO + Analog Input
#define GPIO5    33            // GPIO + Analog Input

#define GPIO10   9             // GPIO + PWM + I2C SCL
#define GPIO11   10            // GPIO + PWM + I2C SDA
#define GPIO23   32            // GPIO (JTAG TDI)
#define GPIO24   38            // GPIO + PWM (JTAG TDO)
#define GPIO28   19            // GPIO (mode switch input)
#define GPIO13   3             // GPIO (configured as general purpose input)
#define GPIO12   4             // GPIO + Interrupt (configured as general purpose output)
#define SWITCH_INPUT     19    // This is the mode switch (configuration / AP mode or normal usage as wifi station). Also exported in the OSC message
#define SWITCH2_INPUT    3     //  Free to use on the side of the board, configured as an input with pullup in the FW and exported in the OSC message
#define REMOTE_OUTPUT    4     //  Free to use on the side of the board, configured as an output in the FW and controlled by OSC message


// DEFAULTS
#define DEFAULT_UDP_PORT  8888
#define DEFAULT_UDP_SERVICE_PORT  9999
#define DEFAULT_SAMPLE_RATE  5
#define VERSION_DATE        "R-IoT Bitalino v2.043 - IRCAM-PLUX 2017-2021"
#define PARAMS_FILENAME     "params.txt"
#define WEB_SERVER_DELAY    100          // Time to press on the switch to start the webserver      

#define GYRO_NOISEGATE      50

// Sensor orientation
#define TOP      0
#define BOTTOM   1

// Declination at Paris, FRANCE (about 1 minute)
#define DECLINATION    0.01
//#define DECLINATION  13.8; // Declination at Danville, California is 13 degrees 48 minutes and 47 seconds on 2014-04-04

#define MAX_CLIENTS    5
#define STATION_MODE   0
#define AP_MODE        1

typedef union uWord {
	short int Value;
  	unsigned char Val[sizeof(short int)];
} Word;

// For Mag calibration stages
typedef enum coords {X, Y, Z};
typedef enum CalStage {START, X_PLUS_0, X_PLUS_180, X_MINUS_0, X_MINUS_180, Y_PLUS_0, Y_PLUS_180, Y_MINUS_0, Y_MINUS_180, Z_PLUS_0, Z_PLUS_180, Z_MINUS_0, Z_MINUS_180, END}; 


#define IPV4_SIZE  4
#define MAX_SERIAL 80
#define MAX_STRING_LEN  80
#define MAX_STRING      200

// Serial Messages
#define TEXT_WIFI_MODE    "mode"
#define TEXT_SSID	  "ssid"
#define TEXT_OWNIP	  "ownip"
#define TEXT_DESTIP	  "destip"
#define TEXT_GATEWAY	  "gateway"
#define TEXT_DNS	  "dns"
#define TEXT_MASK	  "mask"
#define TEXT_PORT	  "port"
#define TEXT_MASTER_ID	  "masterid"
#define TEXT_SAMPLE_RATE  "samplerate"
#define TEXT_SECURITY     "security"
#define TEXT_PASSWORD     "pass"
#define TEXT_DHCP         "dhcp"
#define TEXT_STANDALONE   "standalone"

// Offsets & calibration matrix
#define TEXT_ACC_OFFSETX  "acc_offsetx"
#define TEXT_ACC_OFFSETY  "acc_offsety"
#define TEXT_ACC_OFFSETZ  "acc_offsetz"

#define TEXT_GYRO_OFFSETX  "gyr_offsetx"
#define TEXT_GYRO_OFFSETY  "gyr_offsety"
#define TEXT_GYRO_OFFSETZ  "gyr_offsetz"

#define TEXT_MAG_OFFSETX  "mag_offsetx"
#define TEXT_MAG_OFFSETY  "mag_offsety"
#define TEXT_MAG_OFFSETZ  "mag_offsetz"

#define TEXT_BETA          "beta"


#define MIN_SAMPLE_RATE     3
#define MAX_SAMPLE_RATE	    1000


void ParseIP(char *TheString, IPAddress *TheIP);
unsigned char SkipToValue(char *StringBuffer);
unsigned char SkipToNextValue(char *StringBuffer, unsigned char StartIndex);
unsigned char GrabLine(char *StringBuffer);
unsigned int StringLength(char* StringBuffer);
char * ftoa(double f, char * buf, int precision);
void SetLedColor(boolean red, boolean green, boolean blue);



#endif
