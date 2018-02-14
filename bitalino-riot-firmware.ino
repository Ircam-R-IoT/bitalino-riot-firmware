/* R-IoT :
 Texas Instrument CC3200 Internet of Things / Sensor hub / Dev Platform
 80 MHz 32 Bit ARM MCU + Wifi stack / modem
 
 IRCAM - Emmanuel FLETY - Music Bricks - Rapid Mix
 
 Rev History :
 
 2.0 : moving to the LSM9DS1 motion sensor, new PCB from PLUX and secondary UART. UART0 : FTDI / UART1 : Bitalino
 
 1.7 : Massive improvement in calibration and Euler angles - added bitalino support in this specific version
 
 1.5 : adding a AP style connection to allow streaming to multiple computers / devices
 
 1.4 :  loads of fixing in the calibration process for the absolute angles (madgwick)
 and webserver
 
 */

#include <stdio.h>
#include <strings.h>
#include <SPI.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <bitalino1.h>

// Handles the file system of the FLASH, to store parameters
#include <SLFS.h>

#include "common.h"
#include "LSM9DS1.h"
#include "osc.h"
#include "web.h"

/////////////////////////////////////////////////////////////
// DEFAULT parameters
// your network name also called SSID
byte mac[6];
const char TheSSID[] = "riot";
const uint16_t TheDestPort = DEFAULT_UDP_PORT;
const uint8_t TheLocalIP[] = {
  192,168,1,40};
const uint8_t TheSubnetMask[] = {
  255,255,255,0};
const uint8_t TheGatewayIP[] = {
  192,168,1,1};
const uint8_t TheDestIP[] = {
  192,168,1,100};
const unsigned long TheSampleRate = DEFAULT_SAMPLE_RATE;
const uint8_t TheID = 0; 

/////////////////////////////////////////////////////////////
// Global vars
WiFiServer server(80);
WiFiClient client;
byte APorStation = STATION_MODE;
char ssid[32];
char ssidAP[32];
char password[32] = "12345678";
IPAddress LocalIP;
IPAddress APIP;
IPAddress SubnetMask;
IPAddress GatewayIP;
IPAddress DestIP;
uint16_t DestPort;
uint8_t ModuleID;
unsigned long SampleRate;
boolean UseDHCP = true;
boolean UseSecurity = false;
int status = WL_IDLE_STATUS;
int statusAP = false;
int PacketStatus;
boolean ConfigurationMode = false;
boolean AcceptOSC = true;
byte PageToDisplay = CONFIG_WEB_PAGE;
unsigned int ConfigModePressCounter = 0;
unsigned int ConfigModeAllowCounter = 1000; // allow config only shortly after start-up
int TempInt = 0;
boolean BlinkStatus = 0;

char packetBuffer[255]; //buffer to hold incoming packet
WiFiUDP UdpPacket;
WiFiUDP ConfigPacket;
OscBuffer RawSensors;
OscBuffer Message;
OscBuffer BitalinoData;

BITalinoFrame frame;
word FrameAmount = 0;
// Defines if we run the module in standalone or with bitalino support by default
// When in standalone mode, we can talk to the module via the serial port
// and get Serial.print debug / info / diag while this is muted in bitalino mode
// as the serial port is used exclusively for talking to the bitalino MCU.
boolean StandAloneMode = false;  

unsigned long ElapsedTime = 0;
unsigned long ElapsedTime2 = 0;

////////////////////////////////////////////////////////////
// Sensor storage
short unsigned int SwitchState, ActualSwitchState;
short unsigned int SwitchState2, ActualSwitchState2;
short unsigned int RemoteOutputState = LOW;

Word AccelerationX, AccelerationY, AccelerationZ;
Word GyroscopeX, GyroscopeY, GyroscopeZ;
Word MagnetometerX, MagnetometerY, MagnetometerZ;
Word Temperature;
int AnalogInput1, AnalogInput2;
byte CommunicationMode = SPI_MODE;
//byte CommunicationMode = I2C_MODE;

// Defines whether you want the "raw" value with the stored offset or not
byte SendCalibrated = true;
//byte SendCalibrated = false;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Absolute angle (madgwick)
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define PI 3.14159265358979323846264338327950
// There is a tradeoff in the beta parameter between accuracy and response speed.
// In the original Madgwick study, beta of 0.041 (corresponding to GyroMeasError of 2.7 degrees/s) was found to give optimal accuracy.
// However, with this value, the LSM9SD0 response time is about 10 seconds to a stable initial quaternion.
// Subsequent changes also require a longish lag time to a stable output, not fast enough for a quadcopter or robot car!
// By increasing beta (GyroMeasError) by about a factor of fifteen, the response time constant is reduced to ~2 sec
// I haven't noticed any reduction in solution accuracy. This is essentially the I coefficient in a PID control sense; 
// the bigger the feedback coefficient, the faster the solution converges, usually at the expense of accuracy. 
// In any case, this is the free parameter in the Madgwick filtering and fusion scheme.
// Beta is called the rate of convergence of the filter. Higher value lead to a noisy output but fast response
// If beta = 0.0 => uses gyro only. A very high beta like 2.5 uses almost no gyro and only accel + magneto.

#define BETA_DEFAULT 0.4f   // Much faster - noisier
#define BETA_MAX     2.0f

float beta = BETA_DEFAULT;
float madgwick_beta_max = BETA_MAX;
float madgwick_beta_gain = 1.0f;

int gyroOffsetAutocalTime = 5000; //ms = 1000 samples @5ms
long gyroOffsetAutocalThreshold = 100; //LSB
long gyroOffsetAutocalCounter; //internal
boolean gyroOffsetAutocalOn = false;
boolean gyroOffsetCalDone = false;
long gyroOffsetCalElapsed = 0;
long gyroOffsetAutocalMin[3];
long gyroOffsetAutocalMax[3];
long gyroOffsetAutocalSum[3];
long magOffsetAutocalMin[3];
long magOffsetAutocalMax[3];
long accOffsetAutocalSum[3];

float pitch, yaw, roll, heading;
float Declination = DECLINATION;
float deltat = 0.005f;        // integration interval for both filter schemes - 5ms by default

int gyro_bias[3] = { 0, 0, 0};
int accel_bias[3] = { 0, 0, 0};
int mag_bias[3] = { 0, 0, 0};
int bias_samples = 32;

float abias[3] = { 0., 0., 0.};
float gbias[3] = { 0., 0., 0.};
float mbias[3] = { 0., 0., 0.};

float gRes, aRes, mRes;		// Resolution = Sensor range / 2^15
float a_x, a_y, a_z, g_x, g_y, g_z, m_x, m_y, m_z; // variables to hold latest sensor data values
float gyro_norm; // used to tweak Beta
float mag_nobias[3];

float q1 = 1.0f, q2 = 0.0f, q3 = 0.0f, q4 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame

/////////////////////////////////////////////////////////////////
// Serial port message / buffers / temporary strings
char SerialBuffer[MAX_SERIAL];
unsigned char SerialIndex = 0;
boolean FlagSerial = FALSE;
char StringBuffer[MAX_STRING];


// To get printf to work, we redirect STDOUT and the myWrite functions
ssize_t myWrite(void *cookie, const char *buf, size_t n)
{
  return Serial.write((uint8_t*)buf, n);
}

cookie_io_functions_t myVectors = { 
  0, myWrite, 0, 0 };


void setup() {
  // Basic I/Os
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  
  // Analog Inputs
  pinMode(GPIO4, INPUT);
  pinMode(GPIO5, INPUT);
  
  // This is the former RIOT general purpose switch input, on another I/O for this version
  // SWITCH_INPUT can be used in place of GPIO28, see common.h for I/0 definitions
  pinMode(GPIO28, INPUT_PULLUP);  // Used to trigger configuration mode
  
  // This is an addition GPIO configured as an input and exported in the OSC message
  // SWITCH2_INPUT can be used in place of GPIO13, see common.h for I/0 definitions
  pinMode(GPIO13, INPUT_PULLUP);
  
  // This is an addition GPIO configured as an input and exported in the OSC message
  // REMOTE_OUTPUT can be used in place of GPIO12, see common.h for I/0 definitions
  pinMode(GPIO12, OUTPUT);
  digitalWrite(REMOTE_OUTPUT, RemoteOutputState);
  
    // POWER On indicator  
  SetLedColor(1,0,0);  // RED
  
  //Initialize serial. Serial1 is for the bitalino board
  Serial.begin(115200);
  Serial1.begin(115200);
  
  // Needed to have printf working
  stdout = fopencookie((void *)0, "w", myVectors);
  setlinebuf(stdout);
  
  // Starts the file system
  // This has to be done asap to determine if we are in bitalino mode or 
  // standalone mode.
  SerFlash.begin();

  // Retrieve saved params in FLASH using the file system
  LoadParams();
 
  // Bitalino framework init attempt
  if(!StandAloneMode)
  {
    char verStr[30];
    boolean ok;
    BITalino.begin();
    
    ok = BITalino.version(verStr, sizeof verStr);    // get device version string
    
    // Debug - uncomment if needed
    /*Serial.print("version: ");
    Serial.println(verStr);
    Serial.print("Ok=");
    Serial.println(ok);*/
    
    // Here, some checks are needed to ensure we "found" the bitalino. 
    // Either check the version string or something else.
    if(!ok)
    {
      StandAloneMode = true;
      Serial.println("Bitalino not found, running in standalone R-IoT mode");
    }
    else
    {
      StandAloneMode = false;  // Not absolutely needed but safety
      Serial.println("Bitalino found, initializing at 1000 Hz");
      ok = BITalino.battery(10);  // set battery threshold (optional)
      ok = BITalino.start(1000, 0x3F, false);   // start acquisition of all channels at 1000 Hz
      sprintf(StringBuffer, "/%u/bitalino\0",ModuleID);
      PrepareOSC(&BitalinoData, StringBuffer, 'i', 11);    // Seq # + 4 digital + 6 analog
    }
    
  }
  
  Serial.println(VERSION_DATE);
  Serial.println("Params Loaded");

  // Check if we are going in configuration mode
  // 2-3 second shorting the pin to ground during boot
  while(!digitalRead(SWITCH_INPUT))
  {
    delay(20);
    TempInt++;
    if(BlinkStatus)
    {
      BlinkStatus = 0;
      SetLedColor(1,0,0);
    }
    else
    {
      BlinkStatus = 1;
      SetLedColor(0,0,0);
    }
    if(TempInt > WEB_SERVER_DELAY)
    {
      ConfigurationMode = true;
      Serial.println("Configuration / Web Server Mode");
      SetLedColor(1,0,0);  // RED
      break;
    }
  }

  // Init motion sensor
  Serial.println("Init Motion Sensor");
  
  // Start SPI with defaults
  SPI.begin();
  // SPI settings
  // 16 MHz max bit rate, clock divider 1:2 => 8 MHZ SPI clock
  SPI.setClockDivider(SPI_CLOCK_DIV2);
  SPI.setDataMode(SPI_MODE0);
  
  // Sensor HW comm settings
  if(CommunicationMode == SPI_MODE)
  {
    // Motion sensor I/Os
    pinMode(ACC_CS, OUTPUT);
    pinMode(MAG_CS, OUTPUT);
    digitalWrite(ACC_CS, HIGH);
    digitalWrite(MAG_CS, HIGH);
  }
  else
  {
    Wire.begin();
  } 
  
  delay(10);
  RebootLSM9DS1();
  delay(10);
  
  //////////////////////////////////////////
  //// TEMP / DEBUG
  
  /*unsigned char SensorToken;
  while(1)
  {
    SensorToken = ReadAccLSM9DS1(WHO_AM_I);
    Serial.print("WhoAmI: 0x");
    Serial.println(SensorToken, HEX);
    delay(100);
  }
 */
  InitLSM9DS1();

  delay(40);
  SetLedColor(0,1,0);
  
  // Scaling to obtain gs and deg/s 	
  // Must match the init settings of the LSM9DS0
  // Beware, absolute dynamic range of each sensor isn't equivalent.
  // Accelerometers are using the whole dynamic range, gyro / mag aren't
  gRes = 2000.0 / 32768; 	// +- 2000 deg/s
  aRes = 8.0 / 32768.0;         // +- 8g
  mRes = 2.0 / 32768.0;         // +- 2 gauss


  // Finalize the bias unit conversion
  for(int i = 0 ; i < 3 ; i++)
  {
    gbias[i] = gRes * (float)gyro_bias[i];
    abias[i] = aRes * (float)accel_bias[i];
    mbias[i] = mRes * (float)mag_bias[i];
  }

  if(!ConfigurationMode)
  {
    SetLedColor(0,1,0);
    
    if(APorStation == STATION_MODE)
    {
      // Attempt to connect to Wifi network:
      Serial.print("R-IoT connecting to: ");
      // print the network name (SSID);
      Serial.println(ssid); 
      Connect(); 
    }
    else  // AP mode
    {
      // attempt to connect to Wifi network:
      Serial.print("R-IoT creates network: ");
      // print the network name (SSID);
      Serial.println(ssid); 
      
      // Creates the AP & config
      APIP = IPAddress(TheGatewayIP);
      WiFi.config(APIP);
      if(!UseSecurity)
        WiFi.beginNetwork((char *)ssid);
      else
        WiFi.beginNetwork((char *)ssid, (char*)password);
    }
    
    if(StandAloneMode)
      WiFi.macAddress(mac);
    
    // Prep the UDP packet
    UdpPacket.begin(DestPort);
    UdpPacket.beginPacket(DestIP, DestPort);
    
    // Open the service port to talk to the module (config, calibration)
    ConfigPacket.begin(DEFAULT_UDP_SERVICE_PORT);

    // Prepare the OSC message structure   
    // Now we send all data at once in a single message with only floats
    sprintf(StringBuffer, "/%u/raw\0",ModuleID);
    PrepareOSC(&RawSensors, StringBuffer, 'f', 22);    // All float
    
  } // END OF IF NORMAL (!CONFIG) MODE

  // If in configuration mode we setup a webserver for configuring the unit
  // The module becomes an AP with DHCP
  else
  {
    APIP = IPAddress(TheGatewayIP);

    WiFi.config(APIP);
    randomSeed(analogRead(GPIO5));
    sprintf(ssidAP, "RIOT-%04x\0",random(16000));
    if(StandAloneMode)
    {
      Serial.print("Setting up Access Point named: ");
      Serial.println(ssidAP);
    }
    WiFi.beginNetwork((char *)ssidAP);
    WiFi.macAddress(mac);
  }

  ElapsedTime = millis();
  ElapsedTime2 = millis();  
}


void loop() {

  if(!ConfigurationMode)
  {
    if((millis() - ElapsedTime2) > 300) // Perform the check not too often
    {
      ElapsedTime2 = millis();
      if(APorStation == STATION_MODE)
      {
        int CurrentStatus = WiFi.status();
        if(CurrentStatus != WL_CONNECTED)
        {
          // print dots while we wait to connect and blink the power led
          Serial.print(".");
          
          if(BlinkStatus)
          {
             BlinkStatus = 0;
             SetLedColor(0,0,0);
          }
          else
          {
            BlinkStatus = 1;
            SetLedColor(0,1,0);
          }
        }
        // Newly connected to the network, locks until DHCP answers
        // if enabled 
        if((status != WL_CONNECTED) && (WiFi.status() == WL_CONNECTED))
        {
          status = WiFi.status();
          SetLedColor(0,0,1); // Blue
          Serial.println("\nConnected to the network");  

          while ((WiFi.localIP() == INADDR_NONE))
          {
            // print dots while we wait for an ip addresss
            Serial.print(".");
            delay(300);  
          }
           
          // you're connected now, so print out the status  
          printCurrentNet();
          printWifiData();

        }
        
        // Disconnected from the network, try to reconnect
        if((status == WL_CONNECTED) && (WiFi.status() != WL_CONNECTED))
        {
          Serial.println("Network Lost, trying to reconnect");
          status = WiFi.status();
        }
      } // End of Station mode connection
      else  // AP mode
      {      
        if(WiFi.localIP() == INADDR_NONE)   // Indicates AP isn't ready yet
        {        
          // print dots while we wait to connect and blink the power led
          if(StandAloneMode)
            Serial.print(".");
          
          if(BlinkStatus)
          {  
            BlinkStatus = 0;
            SetLedColor(0,0,0);
          }
          else
          {
            BlinkStatus = 1;
            SetLedColor(0,1,0);
          }
        }  
        else if (!statusAP)
        {
          statusAP = true;
          SetLedColor(0,0,1);
      
          Serial.println("AP active.");
          printCurrentNet();
          printWifiData();
        }
      } // End of AP mode connection
    } // end of IF(elapsed time)

    if(AcceptOSC)
    {
      // Parses incoming OSC messages
      int packetSize = ConfigPacket.parsePacket();
      if (packetSize)
      {
        // Debug Info
         //Serial.print("Received packet of size ");
         //Serial.println(packetSize);
         //Serial.print("From ");
         //IPAddress remoteIp = ConfigPacket.remoteIP();
         //Serial.print(remoteIp);
         //Serial.print(", port ");
         //Serial.println(ConfigPacket.remotePort());

        // read the packet into packetBufffer
        int Index = 0;
        int len = ConfigPacket.read(packetBuffer, 255);
        if (len > 0) packetBuffer[len] = 0;          // Add a terminator in the buffer
        
        //Debug
        /*Serial.println("Contents:");
        Serial.println(packetBuffer);
        Serial.println("Packet Len:");
        Serial.println(len);
        for (int i=0 ; i < len ; i++)
        {
          if(packetBuffer[i] == '\0')
            printf("_");
          else
            printf("%c",packetBuffer[i]);
        }
        printf("\n");
        
        printf("Generated Osc Message:\n");
        for (int i=0 ; i < len ; i++)
        {
          if(Message.buf[i] == '\0')
            printf("_");
          else
            printf("%c",Message.buf[i]);
        }
        printf("\n");*/
        
        // Actual parsing
        // Checks that's for the proper ID / module
        sprintf(StringBuffer, "/%u/\0",ModuleID);
        if(!strncmp(packetBuffer, StringBuffer, strlen(StringBuffer)))
        {  // that's for us
          char *pUDP = packetBuffer;
          int Index = strlen(StringBuffer);  // Skips the ID
         
         // Parsing / decoding (basic)
          if(!strncmp(&(pUDP[Index]), "output", 6))
          {
            Index += strlen("output");
            Index = OscSkipToValue(pUDP, Index);
            
            //Serial.println("After OSC SKIP");
            //printf("Index = %d\n", Index);
            
            int OscRemoteValue, TempInt;
            TempInt = pUDP[Index];
            OscRemoteValue = TempInt << 24;
            Index++;
            TempInt = pUDP[Index];
            OscRemoteValue += OscRemoteValue || (TempInt << 16);
            Index++;
            TempInt = pUDP[Index];
            OscRemoteValue += OscRemoteValue || (TempInt << 8);
            Index++;
            TempInt = pUDP[Index];
            OscRemoteValue += OscRemoteValue || TempInt;
           
           RemoteOutputState = OscRemoteValue;
            if(RemoteOutputState)
              RemoteOutputState = HIGH;
            else
              RemoteOutputState = LOW;
            // Debug
            printf("Remote Control Output update = %d\n", RemoteOutputState);
            digitalWrite(REMOTE_OUTPUT,RemoteOutputState);
          }
          // add here other keywords like changing the sample rate and saving 
          // data => see parse serial
        }
      }
    }
    
    // The main sampling loop
    if((millis() - ElapsedTime >= SampleRate) && !ConfigurationMode &&
    (((WiFi.status()==WL_CONNECTED) && (APorStation==STATION_MODE)) ||
    (statusAP && (APorStation==AP_MODE))))
    {       
      ElapsedTime = millis();
      SetLedColor(0,0,1);    // Turns blue
      
      SwitchState = digitalRead(SWITCH_INPUT);
      ActualSwitchState = !SwitchState;
      
      // Second input reading
      SwitchState2 = digitalRead(SWITCH2_INPUT);
      ActualSwitchState2 = !SwitchState2;
      
      // Debug
      ReadAccel();
      ReadGyro();
      ReadMagneto();
      ReadTemperature();
      
      // Comment those 2 if you don't need the analog inputs to be exported by OSC
      AnalogInput1 = analogRead(GPIO4);
      AnalogInput2 = analogRead(GPIO5);
      
      // Allow configuration mode only shortly after start-up
      if(ConfigModeAllowCounter > 0)
      {
        --ConfigModeAllowCounter;
        if(!SwitchState)
        {
          ConfigModePressCounter++;
          if(ConfigModePressCounter > 600)
          {
            ConfigModeAllowCounter = 0;
            ConfigModePressCounter = 0;
            CalibrateAccGyroMag();
          }
        }
        else
        {
          // end of initial period that allows for calibration
          ConfigModePressCounter = 0;
        }
      }      
      if((millis() - gyroOffsetCalElapsed > gyroOffsetAutocalTime) && gyroOffsetCalDone)
      {
        gyroOffsetCalElapsed = millis();
        gyroOffsetCalDone = false;
      }
      
      if(!gyroOffsetCalDone && gyroOffsetAutocalOn)
      {
        gyroOffsetCalibration();      
      }
    
      g_x = (gRes * (float)GyroscopeX.Value) - gbias[0];   // Convert to degrees per seconds, remove gyro biases
      g_y = (gRes * (float)GyroscopeY.Value) - gbias[1];
      g_z = (gRes * (float)GyroscopeZ.Value) - gbias[2];

      a_x = (aRes * (float)AccelerationX.Value) - abias[0];   // Convert to g's, remove accelerometer biases
      a_y = (aRes * (float)AccelerationY.Value) - abias[1];
      a_z = (aRes * (float)AccelerationZ.Value) - abias[2];

      m_x = (mRes * (float)MagnetometerX.Value) - mbias[0];     // Convert to Gauss and correct for calibration
      m_y = (mRes * (float)MagnetometerY.Value) - mbias[1];
      m_z = (mRes * (float)MagnetometerZ.Value) - mbias[2];   

      // compute the squared norm of the gyro data => rough estimation of the movement
      gyro_norm = g_x * g_x + g_y * g_y + g_z * g_z;
      mag_nobias[0] = m_x;
      mag_nobias[1] = m_y;
      mag_nobias[2] = m_z;
  
      ////////////////////////////////////////////////////////////////////////////////////
      // Note regarding the sensor orientation & angles :
      // We alter the sensor sign in order to "redefine gravity" and axis so that it behaves
      // the same whether the sensor is up or down. However, for the heading computation and
      // correction against angles, the "real" sensor orientation and pitch / roll proper 
      // signing must be used. We therefore do a double signe inversion when the sensor 
      // is on the bottom. [looks crappy but works and for good reasons]
      
      // Different orientation of the LS1 vs LS0 for the magnetometers
      MadgwickAHRSupdate(a_x, a_y, a_z, g_x*PI/180.0f, g_y*PI/180.0f, g_z*PI/180.0f, m_x, m_y, -m_z);
      //MadgwickAHRSupdate(a_x, a_y, a_z, g_x*PI/180.0f, g_y*PI/180.0f, g_z*PI/180.0f, m_x, m_y, m_z);
      yaw   = atan2(2.0f * (q2 * q3 + q1 * q4), q1*q1 + q2*q2 - q3*q3 - q4*q4);   
      pitch = -asin(2.0f * ((q2*q4) - (q1*q3)));
      roll  = atan2(2.0f * (q1*q2 + q3*q4), (q1*q1) - (q2*q2) - (q3*q3) + (q4*q4));
    
      // Compute heading *BEFORE* the final export of yaw pitch roll to save float computation of deg2rad / rad2deg
      ComputeHeading(); 
      
      /////////////////////////////////////////////////////////////////////////////////
      // Degree per second conversion and declination correction 
      pitch *= 180.0f / PI;
      yaw   *= 180.0f / PI; 
      yaw   -= Declination; 
      roll  *= 180.0f / PI;

      // Update sensors data in the main OSC message
      char *pData = RawSensors.pData;
     
      float TempFloat;
      FloatToBigEndian(pData, &a_x);
      pData += sizeof(float);
      FloatToBigEndian(pData, &a_y);
      pData += sizeof(float);
      FloatToBigEndian(pData, &a_z);
      pData += sizeof(float);
      
      // Rescaling to deg/s to have a more human readable value
      // and decent numbers similar to accel
      float g_x_scaled, g_y_scaled, g_z_scaled;
      g_x_scaled = g_x / 1000.;  // Sending °/s
      g_y_scaled = g_y / 1000.;  // Sending °/s
      g_z_scaled = g_z / 1000.;  // Sending °/s
      
      FloatToBigEndian(pData, &g_x_scaled);
      pData += sizeof(float);
      FloatToBigEndian(pData, &g_y_scaled);
      pData += sizeof(float);
      FloatToBigEndian(pData, &g_z_scaled);
      pData += sizeof(float);
       
      FloatToBigEndian(pData, &m_x);
      pData += sizeof(float);
      FloatToBigEndian(pData, &m_y);
      pData += sizeof(float);
      FloatToBigEndian(pData, &m_z);
      pData += sizeof(float);
      // Temperature
      TempFloat = (float)Temperature.Value;
      // Conversion to decimal °C here before float export
      TempFloat = (TempFloat / (float)(LSM_TEMP_SCALE)) + (float)(LSM_BIAS_TEMPERATURE);
      FloatToBigEndian(pData, &TempFloat);
      pData += sizeof(float);
      
      TempFloat = (float)ActualSwitchState;
      FloatToBigEndian(pData, &TempFloat);
      pData += sizeof(float);
      
      TempFloat = (float)ActualSwitchState2;
      FloatToBigEndian(pData, &TempFloat);
      pData += sizeof(float);
    
      // Analog Inputs (12 bits)
      TempFloat = (float)AnalogInput1;
      FloatToBigEndian(pData, &TempFloat);
      pData += sizeof(float);  
      TempFloat = (float)AnalogInput2;
      FloatToBigEndian(pData, &TempFloat);
      pData += sizeof(float);  

      // Quaternions
      FloatToBigEndian(pData, &q1);
      pData += sizeof(float);
      FloatToBigEndian(pData, &q2);
      pData += sizeof(float);
      FloatToBigEndian(pData, &q3);
      pData += sizeof(float);
      FloatToBigEndian(pData, &q4);
      pData += sizeof(float);

      // Euler Angles + Heading
      FloatToBigEndian(pData, &yaw);
      pData += sizeof(float);
      FloatToBigEndian(pData, &pitch);
      pData += sizeof(float);
      FloatToBigEndian(pData, &roll);
      pData += sizeof(float);
      FloatToBigEndian(pData, &heading);
      
      UdpPacket.write((uint8_t*)RawSensors.buf, RawSensors.PacketSize);
      UdpPacket.endPacket(); 

      if (FrameAmount == 1 && !StandAloneMode)
      {
        // Update the bitalino data/sensors data in the bitalino OSC message
        pData = BitalinoData.pData;
        int k;
        
        // Sequence #
        ShortToBigEndian(pData, frame.seq);
        pData += sizeof(int);
        
        // digital first 
        for(k=0 ; k < 4 ; k++)
        {
           ShortToBigEndian(pData, frame.digital[k]);
           pData += sizeof(int);
        }
        // then analog 
        for(k=0 ; k < 6 ; k++)
        {
           ShortToBigEndian(pData, frame.analog[k]);
           pData += sizeof(int);
        } 
        UdpPacket.write((uint8_t*)BitalinoData.buf, BitalinoData.PacketSize);
        UdpPacket.endPacket();
        FrameAmount = 0;
      }

      SetLedColor(0,0,0);     
    }
    
    // Process the bitalino serial stream separately from the first UART
    if(Serial.available())
    {
       //////////////////////////////////////////////////////////////////////////////////
      // Incoming serial message (config, control)
     if(GrabSerialMessage())
      {
        ProcessSerial();
        FlagSerial = false;
      }
    }
    if(Serial1.available())
    {
      FrameAmount = BITalino.readSingle(&frame);
    }
  }
  //////////////////////////////////////////////////////////////////////////////////
  // Handles the web server for configuration via the webpage
  else
  {
    if(WiFi.localIP() == INADDR_NONE)
    {        
      if((millis() - ElapsedTime2) > 100) // Blinks faster than during normal mode
      {
        ElapsedTime2 = millis();
        // print dots while we wait to connect and blink the power led
        if(StandAloneMode)
          Serial.print(".");
        
        if(BlinkStatus)
        {
          BlinkStatus = 0;
          SetLedColor(0,0,0);
        }
        else
        {
          BlinkStatus = 1;
          SetLedColor(0,1,0);
        }
      }  
    }

    else if (!statusAP) {
      statusAP = true;
      SetLedColor(0,0,1);
      Serial.println("AP active.");
      printCurrentNet();
      printWifiData();
      Serial.println("Starting webserver on port 80");

      server.begin();
      Serial.println("Webserver started!");
    }

    if(statusAP) // We can accept clients
    {
      char c;
      char LocalHttpBuffer[300];
      int HttpBufferIndex = 0;
      unsigned int Index, Rank;

      client = server.available();

      if (client) {
        if(StandAloneMode)
          Serial.println("new client");
        // an http request ends with a blank line
        boolean currentLineIsBlank = true;
        boolean StayConnected = true;
        
        while (client.connected() && StayConnected )
        {
          if (client.available())
          {

            c = client.read();
            Serial.write(c);

            if(c != '\r')
              LocalHttpBuffer[HttpBufferIndex++] = c;
            if(!currentLineIsBlank && c == '\n')
            { 
              LocalHttpBuffer[HttpBufferIndex++] = '\0';
              // Process HTTP contents
              //Serial.println("new http line - processing");
              //Serial.print(LocalHttpBuffer);
              HttpBufferIndex = 0;
              if(!strncmp(LocalHttpBuffer, "GET / ", 6))
                PageToDisplay = CONFIG_WEB_PAGE;

              else if(!strncmp(LocalHttpBuffer, "GET /params", 11))
              { // Apply settings request - parsing all parameters, stores, update, reboot
                char *pHtml = LocalHttpBuffer;
                // position the pointer on the first param
                while(*pHtml != '?' && *pHtml != '\0')  pHtml++;
                pHtml++;  // skips the ?
                while(*pHtml != '\0')
                {
                  pHtml += GrabNextParam(pHtml, StringBuffer);
                  //Serial.print("Param found: ");
                  //Serial.println(StringBuffer);

                  // Parsing params withing the submitted URL
                  if(!strncmp("ssid", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    strcpy(ssid, &(StringBuffer[Index]));
                    Serial.print("Updated SSID: ");
                    Serial.println(ssid);
                  }
                  
                  if(!strncmp("pass", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    
                    strcpy(password, &(StringBuffer[Index]));
                    Serial.print("Updated password: ");
                    Serial.println(password);
                  }
                  
                  if(!strncmp("security", StringBuffer, 8))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "WPA2", 4))
                      UseSecurity = true;
                    else
                      UseSecurity = false;
 
                    Serial.print("Updated Security: ");
                    Serial.println(UseSecurity);
                  }
                  
                  if(!strncmp("mode", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "station", 7))
                      APorStation = STATION_MODE;
                    else
                      APorStation = AP_MODE;

                    Serial.print("Updated Mode: ");
                    Serial.println(APorStation);
                    
                  }
                  
                  if(!strncmp("type", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    if(!strncmp(&(StringBuffer[Index]), "static", 6))
                      UseDHCP = false;
                    else
                      UseDHCP = true; 
                    
                    Serial.print("Updated DHCP: ");
                    Serial.println(UseDHCP);
                    
                  }

                  if(!strncmp("ip", StringBuffer, 2))
                  {
                    Rank = atoi(&(StringBuffer[2]));
                    Index = SkipToValue(StringBuffer);
                    LocalIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("dip", StringBuffer, 3))
                  {
                    Rank = atoi(&(StringBuffer[3]));
                    Index = SkipToValue(StringBuffer);
                    DestIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("gw", StringBuffer, 2))
                  {
                    Rank = atoi(&(StringBuffer[1]));
                    Index = SkipToValue(StringBuffer);
                    GatewayIP[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("msk", StringBuffer, 3))
                  {
                    Rank = atoi(&(StringBuffer[1]));
                    Index = SkipToValue(StringBuffer);
                    SubnetMask[Rank-1] = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("port", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    DestPort = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("id", StringBuffer, 2))
                  {
                    Index = SkipToValue(StringBuffer);
                    ModuleID = atoi(&(StringBuffer[Index]));
                  }

                  if(!strncmp("rate", StringBuffer, 4))
                  {
                    Index = SkipToValue(StringBuffer);
                    SampleRate = atoi(&(StringBuffer[Index]));
                  }
                } // End of WHILE(PARSING PARAMETERS)
                // Save Params
                SaveFlashPrefs();
                Serial.println("Params updated and saved");
                PageToDisplay = PARAMS_WEB_PAGE;

              }
            }

            ////////////////////////////////////////////////////////////////////////
            // if you've gotten to the end of the line (received a newline
            // character) and the line is blank, the http request has ended,
            // so you can send a reply
            if (c == '\n' && currentLineIsBlank) 
            {
              Serial.println("sending webpage");
              // send a standard http response header
              switch(PageToDisplay)
              {
              case CONFIG_WEB_PAGE:
                SendConfigWebPage();
                StayConnected = false;
                delay(100);
                break;

              case PARAMS_WEB_PAGE:
                SendParamsWebpage();
                StayConnected = false;
                delay(100);
                break;

              default:
                client.println(HTTP_RESPONSE_0);
                client.println(HTTP_RESPONSE_1);
                client.println(HTTP_RESPONSE_2);
                client.println();
                delay(100);
                StayConnected = false;
                break;
              }
            }
            if (c == '\n')
            {
              // you're starting a new line
              currentLineIsBlank = true;
            }
            else if (c != '\r')
            {
              // you've gotten a character on the current line
              currentLineIsBlank = false;
            }
          }
        }
        // give the web browser time to receive the data
        delay(100);

        // close the connection:
        client.stop();
        Serial.println("client disconnected");
      } 
    }
  }
}


void Connect(void)
{
  if(!UseSecurity)
    WiFi.begin(ssid);
  else
    WiFi.begin(ssid, password); // If security is needed

  // if static IP - this still has problems with profiles and CC3200 API
  // [EDIT 02/2017 - seems to work fine in energia 17]
  if(!UseDHCP)
    WiFi.config(LocalIP, GatewayIP, GatewayIP, SubnetMask);
}


void SendConfigWebPage(void)
{
  client.println(HTTP_RESPONSE_0);
  client.println(HTTP_RESPONSE_1);
  client.println(HTTP_RESPONSE_2);
  client.println();
  for(int i = 0 ; i < HTML_HEADER_CSS_SIZE ; i++)
  {
    client.println(pHTML_HEADER_CSS[i]);
  }
  
  client.println();
  client.println("</span></h1>\n<br/><br/><hr>");
  client.println("<h1>R-IoT Configuration Page</h1>");
  client.println("<p><table><tr><td><strong>Module Information</strong></td></tr></table>");
  sprintf(StringBuffer, "<table><tr><td>MAC: %02x:%02x%:%02x:%02x:%02x:%02x</td></tr>\0",mac[0], mac[1], mac[2], mac[3], mac[5], mac[5]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<tr><td>ID: %u</td></tr>\0",ModuleID);
  client.println(StringBuffer);
  
  client.println("<tr><td>Beta = \0");
  client.println(beta);
  client.println("</td></tr><br/>");
    
  sprintf(StringBuffer, "<tr><td>Firmware: %s</td></tr></table><br/><br/>\0", VERSION_DATE);
  client.println(StringBuffer);
  client.println("<table><tr><td><strong>Network Configuration</strong></td></tr></table>");
  client.println("<form method=\"GET\" action=\"params\"><table><tr><td>");
  
  client.print("<tr><td>WIFI MODE:</td><td><select name=\"mode\">");
  delay(10);
  if(APorStation == STATION_MODE)
    client.println("<option selected=\"selected\">station</option><option>AP</option></select></td></tr>");
  else
    client.println("<option>station</option><option selected=\"selected\">AP</option></select></td></tr>"); 
  delay(10);
  
  client.print("<tr><td>IP TYPE:</td><td><select name=\"type\">");
  delay(10);
  if(!UseDHCP)
    client.println("<option selected=\"selected\">static</option><option>DHCP</option></select></td></tr>");
  else
    client.println("<option>static</option><option selected=\"selected\">DHCP</option></select></td></tr>"); 
  delay(10);
  
  sprintf(StringBuffer,"<tr><td>SSID:</td><td><input type=\"text\" size=\"32\" maxlength=\"32\" name=\"ssid\" value=\"%s\"></td></tr>\0", ssid);
  client.println(StringBuffer);
  
  client.println("<tr><td>SECURITY:</td><td><select name=\"security\">");
  delay(10);
  if(!UseSecurity)
    client.println("<option selected=\"selected\">None</option><option>WPA2</option></select></td></tr>");
  else
    client.println("<option>None</option><option selected=\"selected\">WPA2</option></select></td></tr>");
  
  delay(10);
  client.print("<tr><td>PASSWD:</td><td><input type=\"text\" size=\"32\" maxlength=\"32\" name=\"pass\"");
  delay(10);
  sprintf(StringBuffer, " value=\"%s\"></td></tr>\0", password);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>IP:</td><td><input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi1\" value=\"%u\">.\0", LocalIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi2\" value=\"%u\">.\0", LocalIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi3\" value=\"%u\">.\0", LocalIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"ipi4\" value=\"%u\"></td></tr>\0", LocalIP[3]);
  client.println(StringBuffer);

  sprintf(StringBuffer, "<tr><td>DEST IP:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"dip1\" value=\"%u\">.\0", DestIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip2\" value=\"%u\">.\0", DestIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip3\" value=\"%u\">.\0", DestIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"dip4\" value=\"%u\"></td></tr>\0", DestIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>GATEWAY:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"gw1\" value=\"%u\">.\0", GatewayIP[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw2\" value=\"%u\">.\0", GatewayIP[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw3\" value=\"%u\">.\0", GatewayIP[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"gw4\" value=\"%u\"></td></tr>\0", GatewayIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>MASK:</td><td><input type=\"text\" size=\"1\" maxlength=\"3\" name=\"msk1\" value=\"%u\">.\0", SubnetMask[0]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk2\" value=\"%u\">.\0", SubnetMask[1]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk3\" value=\"%u\">.\0", SubnetMask[2]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<input type=\"text\" size=\"3\" maxlength=\"3\" name=\"msk4\" value=\"%u\"></td><br/></tr>\0", SubnetMask[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PORT:</td><td><input type=\"text\" size=\"4\" maxlength=\"6\" name=\"port\" value=\"%u\"></td></tr>\0", DestPort);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>ID:</td><td><input type=\"text\" size=\"4\" maxlength=\"3\" name=\"id\" value=\"%u\"></td></tr/>\0", ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SAMPLERATE:</td><td><input type=\"text\" size=\"4\" maxlength=\"4\" name=\"rate\" value=\"%u\"></td></tr>\0", SampleRate);
  client.println(StringBuffer);

  client.println("<tr><td><br/></td></tr><tr><td><input type=\"submit\" value=\"Submit\"></td></tr></table></form>");

  client.println("<br /></body></html>");
  delay(1);
  client.println();
}


void SendParamsWebpage(void)
{
  client.println(HTTP_RESPONSE_0);
  client.println(HTTP_RESPONSE_1);
  client.println(HTTP_RESPONSE_2);
  client.println();
  for(int i = 0 ; i < HTML_HEADER_CSS_SIZE ; i++)
  {
    client.println(pHTML_HEADER_CSS[i]);
  }

  client.println();
  client.println("</span></h1>\n<br/><br/><hr>");
  client.println("<h1>R-IoT Configuration *SAVED* - OK</h1>");
  client.println("<p><table><tr><td><strong>Module Information</strong></td></tr></table>");
  sprintf(StringBuffer, "<table><tr><td>MAC: %02x:%02x%:%02x:%02x:%02x:%02x</td></tr>\0",mac[0], mac[1], mac[2], mac[3], mac[5], mac[5]);
  client.println(StringBuffer);
  sprintf(StringBuffer,"<tr><td>ID: %u</td></tr><tr><td>\0",ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "Firmware: %s</td></tr></table><br/><br/>\0", VERSION_DATE);
  client.println(StringBuffer);
  client.println("<table><tr><td><strong>Network Configuration</strong></td></tr></table>");
  client.println("<table><tr><td>");
  
  client.print("<tr><td>WIFI MODE:</td>");
  if(APorStation == STATION_MODE)
    client.println("<td> Station </td></tr>");
  else
    client.println("<td> Access Point </td></tr>");
  client.print("<tr><td>IP TYPE:</td>");
  if(!UseDHCP)
    client.println("<td> Static IP </td></tr>");
  else
    client.println("<td> DHCP </td></tr>");
  sprintf(StringBuffer,"<tr><td>SSID:</td><td>%s</td></tr>\0", ssid);
  client.println(StringBuffer);
  client.println("<tr><td>SECURITY:</td>");
  sprintf(StringBuffer, "<td> %d </td></tr>\0", UseSecurity);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PASSWD:</td><td> %s </td></tr>\0", password);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>IP:</td><td>%u.%u.%u.%u</td></tr>\0", LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>DEST IP:</td><td>%u.%u.%u.%u</td></tr>\0", DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>GATEWAY:</td><td>%u.%u.%u.%u</td></tr>\0", GatewayIP[0], GatewayIP[1], GatewayIP[2], GatewayIP[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SUBNET MASK:</td><td>%u.%u.%u.%u</td></tr>\0", SubnetMask[0], SubnetMask[1], SubnetMask[2], SubnetMask[3]);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>PORT:</td><td>%u</td></tr>\0", DestPort);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>MODULE ID:</td><td>%u</td></tr>\0", ModuleID);
  client.println(StringBuffer);
  sprintf(StringBuffer, "<tr><td>SAMPLE RATE:</td><td>%u</td></tr>\0", SampleRate);
  client.println(StringBuffer);

  client.println("<tr><td><br/></td></tr></table>");

  client.println("<br /></body></html>"); 
  delay(1);
  client.println();

}

unsigned int GrabNextParam(char *pBuffer, char *ParamString)
{
  unsigned int Len = 0;

  // URL / param string looks like below
  //  /params?type=static+IP&ssid=RIOT-36b9&type=None&pass=&ipi1=192&ipi2=168&ipi3=1&ipi4=1&dip
  // Looks up the & that split between the parameters
  // uses a fair search limit of 50 chars to avoid crashing in case something goes wrong
  while(Len < 50 && *pBuffer != '&' && *pBuffer != ' ')
  {
    ParamString[Len] = *pBuffer;
    pBuffer++;
    Len++;
  }
  pBuffer++; // skips the &
  ParamString[Len] = '\0';  // terminates the string
  Len++;
  return(Len);
}


void printWifiData() {
  // print your WiFi IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);

  // print your MAC address:  
  printf("MAC address: %02x:%02x:%02x:%02x:%02x:%02x\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  Serial.print("Dest. IP Address: ");
  Serial.println(DestIP);

  // print your subnet mask:
  IPAddress subnet = WiFi.subnetMask();
  Serial.print("NetMask: ");
  Serial.println(subnet);

  // print your gateway address:
  IPAddress gateway = WiFi.gatewayIP();
  Serial.print("Gateway: ");
  Serial.println(gateway);

  printf("UDP/OSC Port=%u\n", DestPort);    
  printf("Module ID=%u\n", ModuleID);    
  printf("Sample Period (ms)=%u\n", SampleRate);
}


void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("signal strength (RSSI) in dB:");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Implementation of Sebastian Madgwick's "...efficient orientation filter for... inertial/magnetic sensor arrays"
// (see http://www.x-io.co.uk/category/open-source/ for examples and more details)
// which fuses acceleration, rotation rate, and magnetic moments to produce a quaternion-based estimate of absolute
// device orientation -- which can be converted to yaw, pitch, and roll. 
void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q1mx, _2q1my, _2q1mz, _2q2mx, _2bx, _2bz, _4bx, _4bz, _2q1, _2q2, _2q3, _2q4, _2q1q3, _2q3q4, q1q1, q1q2, q1q3, q1q4, q2q2, q2q3, q2q4, q3q3, q3q4, q4q4;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
          Serial.println("Mag data invalid - no update");
	  return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz);
	qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy);
	qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx);
	qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = AccurateInvSqrt(ax * ax + ay * ay + az * az);
                //recipNorm = 1. / sqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = AccurateInvSqrt(mx * mx + my * my + mz * mz);
                //recipNorm = 1. / sqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q1mx = 2.0f * q1 * mx;
		_2q1my = 2.0f * q1 * my;
		_2q1mz = 2.0f * q1 * mz;
		_2q2mx = 2.0f * q2 * mx;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q4 = 2.0f * q4;
		_2q1q3 = 2.0f * q1 * q3;
		_2q3q4 = 2.0f * q3 * q4;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q1q4 = q1 * q4;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q2q4 = q2 * q4;
		q3q3 = q3 * q3;
		q3q4 = q3 * q4;
		q4q4 = q4 * q4;

		// Reference direction of Earth's magnetic field
		hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
		hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s1 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s2 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1 - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		s3 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
		recipNorm = AccurateInvSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q1 += qDot1 * deltat;
	q2 += qDot2 * deltat;
	q3 += qDot3 * deltat;
	q4 += qDot4 * deltat;

	// Normalise quaternion
	recipNorm = AccurateInvSqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
        //recipNorm = 1. / sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	q4 *= recipNorm;
}


//////////////////////////////////////////////////////////////////
// Computation below got adapted from Freescale application note
// for a tilt compensated compass, formerly using accelerometers
// as inclinometers.
// We however directly grab the stable Pitch / Roll angles from Madgwick
// To de-rotate the mag data. This way we are un-sensitive to shaking
// (classic algorithm uses static accel data to get absolute angles)
//
// Beware, the computation below expects angles provided in rad
// so the routine must be called when pitch and roll are still expressed
// in that unit. Cheezy dirty cheap optimization but embedded rulz
void ComputeHeading(void)
{
  float iSin, iCos; /* sine and cosine */
  float iBpx, iBpy, iBpz, LocalPitch, LocalRoll;
  float iBfx, iBfy, iBfz;  // de rotated values of the mag sensors
  	
  // We work with the calibrated values of the MAG sensors
  // (hard iron offset removed)
  iBpx = mag_nobias[0];
  iBpy = mag_nobias[1];
  iBpz = mag_nobias[2];

  LocalRoll = roll;
  LocalPitch = pitch;
	
  iBpz = iBpz * -1.0;    // Z mag axis is inverted on the LSM9DS0
  
  /* calculate sin and cosine of roll angle Phi */
  iSin = sin(LocalRoll);
  iCos = cos(LocalRoll); 
  /* de-rotate by roll angle Phi */  
  iBfy = (iBpy * iCos) - (iBpz * iSin);/* Eq 19 y component */
  iBpz = (iBpy * iSin) + (iBpz * iCos);/* Bpy*sin(Phi)+Bpz*cos(Phi)*/

  /* calculate sin and cosine of pitch angle Theta */
  iSin = sin(LocalPitch);
  iCos = cos(LocalPitch);	
  /* de-rotate by pitch angle Theta */
  iBfx = (iBpx * iCos) + (iBpz * iSin); /* Eq 19: x component */
  iBfz = (-iBpx * iSin) + (iBpz * iCos);/* Eq 19: z component */
	
  /* calculate current yaw/heading */
  heading = atan2(-iBfy, iBfx); /* Eq 22 */
  heading = (heading * 180.0) / PI;
}


//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
// (thank you quake)
float InvSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  return y;
}


// Variant with 1/3 of the error of the code above
// https://pizer.wordpress.com/2008/10/12/fast-inverse-square-root/
float AccurateInvSqrt(float x){
  uint32_t i = 0x5F1F1412 - (*(uint32_t*)&x >> 1);
  float tmp = *(float*)&i;
  return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
}


void magOffsetCalibration(void)
{
  boolean QuitLoop = false;
  boolean LedState = 0;
  
  
  resetMagOffsetCalibration();
  while(!digitalRead(SWITCH_INPUT))
    delay(20);
  delay(200);
  
  while(!QuitLoop)
  {
    delay(50);
    ReadMagneto();
    magOffsetAutocalMax[0] = max(magOffsetAutocalMax[0], MagnetometerX.Value);
    magOffsetAutocalMax[1] = max(magOffsetAutocalMax[1], MagnetometerY.Value);
    magOffsetAutocalMax[2] = max(magOffsetAutocalMax[2], MagnetometerZ.Value);
    magOffsetAutocalMin[0] = min(magOffsetAutocalMin[0], MagnetometerX.Value);
    magOffsetAutocalMin[1] = min(magOffsetAutocalMin[1], MagnetometerY.Value);
    magOffsetAutocalMin[2] = min(magOffsetAutocalMin[2], MagnetometerZ.Value);
    
    LedState = 1 - LedState;
    SetLedColor(LedState, LedState, LedState); // flashing white
    
    if(!digitalRead(SWITCH_INPUT))
    {
      SetLedColor(0, 0, 1);  // blue
      QuitLoop = true;
      
    } 
  }
  while(!digitalRead(SWITCH_INPUT))  delay(20);

  Serial.println("Mag got recalibrated");
  Serial.print("offsets: ");

  for(int i = 0 ; i < 3 ; i++)
  {
     mag_bias[i] = (magOffsetAutocalMax[i] + magOffsetAutocalMin[i]) / 2;
     mbias[i] = mRes * (float)mag_bias[i];
    
     Serial.print(mag_bias[i]);
     Serial.print(" ; ");
  }
  Serial.println();
}

//=====================================================================================================
// function gyroOffsetCalibration
//=====================================================================================================
//
// calibrate the offset of the gyroscope
//
void gyroOffsetCalibration(void)
{
    //gyro offset autocalibration
    // update min, max, sum and counter
    gyroOffsetAutocalMax[0] = max(gyroOffsetAutocalMax[0], GyroscopeX.Value);
    gyroOffsetAutocalMax[1] = max(gyroOffsetAutocalMax[1], GyroscopeY.Value);
    gyroOffsetAutocalMax[2] = max(gyroOffsetAutocalMax[2], GyroscopeY.Value);
    
    gyroOffsetAutocalMin[0] = min(gyroOffsetAutocalMin[0], GyroscopeX.Value);
    gyroOffsetAutocalMin[1] = min(gyroOffsetAutocalMin[1], GyroscopeY.Value);
    gyroOffsetAutocalMin[2] = min(gyroOffsetAutocalMin[2], GyroscopeZ.Value);
    
    gyroOffsetAutocalSum[0] += GyroscopeX.Value;
    gyroOffsetAutocalSum[1] += GyroscopeY.Value;
    gyroOffsetAutocalSum[2] += GyroscopeZ.Value;
    gyroOffsetAutocalCounter++;
    
    // if the max-min differences are above the threshold, reset the counter and values
    if((gyroOffsetAutocalMax[0]-gyroOffsetAutocalMin[0] > gyroOffsetAutocalThreshold) 
       ||(gyroOffsetAutocalMax[1]-gyroOffsetAutocalMin[1] > gyroOffsetAutocalThreshold)
       ||(gyroOffsetAutocalMax[2]-gyroOffsetAutocalMin[2] > gyroOffsetAutocalThreshold)) 
    {
        resetGyroOffsetCalibration();
        resetAccOffsetCalibration();
    }
    
    // check if there are enough stable samples. If yes, update the offsets and the "calibrate" flag
    if(gyroOffsetAutocalCounter >= (gyroOffsetAutocalTime / 1000. * SampleRate) )
    { // update bias
      for(int i = 0 ; i < 3 ; i++)
      {
        gyro_bias[i] = (gyroOffsetAutocalSum[i] * 1.0)/ gyroOffsetAutocalCounter;
        gbias[i] = gRes * (float)gyro_bias[i];
      }
      gyroOffsetCalDone = true;
      gyroOffsetCalElapsed = millis();      
    }
}


// Reset gyro offset auto calibration / min / max
void resetGyroOffsetCalibration(void) {
  gyroOffsetAutocalCounter = 0;
      
  for(int i = 0 ; i<3 ; i++)
  {                            
    gyroOffsetAutocalMin[i] = 40000;
    gyroOffsetAutocalMax[i] = -40000;
    gyroOffsetAutocalSum[i] = 0;    
    gyro_bias[i] = 0;
    gbias[i] = 0.0;
   }
}

// Reset gyro offset auto calibration / min / max
void resetMagOffsetCalibration(void) {

  for(int i = 0 ; i<3 ; i++)
  {  
    magOffsetAutocalMin[i] = 40000;
    magOffsetAutocalMax[i] = -40000;
    mag_bias[i] = 0;
    mbias[i] = 0.0;
  }
}

// Reset gyro offset auto calibration / min / max
void resetAccOffsetCalibration(void) {
  for(int i = 0 ; i<3 ; i++)
  {                            
    accel_bias[i] = 0;
    abias[i] = 0.0;
    accOffsetAutocalSum[i] = 0;
  }
}


void CalibrateAccGyroMag(void)
{
  boolean QuitLoop = false;
  boolean LedState = 0;

  Serial.println("ACC+GYRO+MAG Calibration Started");
  Serial.println("Please place the module on a flat and stable surface");

  while(!digitalRead(SWITCH_INPUT))
    delay(20);
  
  resetAccOffsetCalibration();
  resetGyroOffsetCalibration();
  resetMagOffsetCalibration();
  
  while(!QuitLoop)
  {
    delay(10);
    ReadAccel();
    ReadGyro();
    
    magOffsetAutocalMax[0] = max(magOffsetAutocalMax[0], MagnetometerX.Value);
    magOffsetAutocalMax[1] = max(magOffsetAutocalMax[1], MagnetometerY.Value);
    magOffsetAutocalMax[2] = max(magOffsetAutocalMax[2], MagnetometerZ.Value);
    magOffsetAutocalMin[0] = min(magOffsetAutocalMin[0], MagnetometerX.Value);
    magOffsetAutocalMin[1] = min(magOffsetAutocalMin[1], MagnetometerY.Value);
    magOffsetAutocalMin[2] = min(magOffsetAutocalMin[2], MagnetometerZ.Value);    
    
    LedState = 1 - LedState;
    SetLedColor(LedState, 0, 0);
    if(!digitalRead(SWITCH_INPUT))
    {
      SetLedColor(LedState, 0, 0);
      Serial.println("Calibration interrupted");
      return;
    } 
  
    //gyro offset autocalibration
    // update min, max, sum and counter
    gyroOffsetAutocalMax[0] = max(gyroOffsetAutocalMax[0], GyroscopeX.Value);
    gyroOffsetAutocalMax[1] = max(gyroOffsetAutocalMax[1], GyroscopeY.Value);
    gyroOffsetAutocalMax[2] = max(gyroOffsetAutocalMax[2], GyroscopeZ.Value);
    
    gyroOffsetAutocalMin[0] = min(gyroOffsetAutocalMin[0], GyroscopeX.Value);
    gyroOffsetAutocalMin[1] = min(gyroOffsetAutocalMin[1], GyroscopeY.Value);
    gyroOffsetAutocalMin[2] = min(gyroOffsetAutocalMin[2], GyroscopeZ.Value);
    
    gyroOffsetAutocalSum[0] += GyroscopeX.Value;
    gyroOffsetAutocalSum[1] += GyroscopeY.Value;
    gyroOffsetAutocalSum[2] += GyroscopeZ.Value;
    
    accOffsetAutocalSum[0] += AccelerationX.Value;
    accOffsetAutocalSum[1] += AccelerationY.Value;
    accOffsetAutocalSum[2] += AccelerationZ.Value - (int)(1./aRes);
    
    gyroOffsetAutocalCounter++;
    
    // if the max-min differences are above the threshold, reset the counter and values
    if((gyroOffsetAutocalMax[0]-gyroOffsetAutocalMin[0] > gyroOffsetAutocalThreshold) 
       ||(gyroOffsetAutocalMax[1]-gyroOffsetAutocalMin[1] > gyroOffsetAutocalThreshold)
       ||(gyroOffsetAutocalMax[2]-gyroOffsetAutocalMin[2] > gyroOffsetAutocalThreshold)) 
    {
        resetGyroOffsetCalibration();
        resetAccOffsetCalibration();
        
    }
    
    // check if there are enough stable samples. If yes, update the offsets and the "calibrate" flag
    if(gyroOffsetAutocalCounter >= (gyroOffsetAutocalTime / 1000. * SampleRate) )
    { 
      // update bias
      Serial.println("Gyro Stable");
        
      for(int i = 0 ; i < 3 ; i++)
      {
        gyro_bias[i] = (gyroOffsetAutocalSum[i] * 1.0)/ gyroOffsetAutocalCounter;
        gbias[i] = gRes * (float)gyro_bias[i];
        accel_bias[i] = (accOffsetAutocalSum[i] * 1.0)/ gyroOffsetAutocalCounter;
        abias[i] = aRes * (float)accel_bias[i];
      }
      QuitLoop = true; 
    }
  }
  
  sprintf(StringBuffer, "*** FOUND Bias acc= %d %d %d", accel_bias[0], accel_bias[1], accel_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);
  sprintf(StringBuffer,"*** FOUND Bias gyro= %d %d %d", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);
  
  SetLedColor(1, 1, 1);
  int WinkCounter = 0;
  while(digitalRead(SWITCH_INPUT))  // Waits for the GP switch to proceed to mag calibration
  {
    delay(20);
    WinkCounter++;
    if(WinkCounter > 50)
    {
      SetLedColor(1, 1, 1);
      delay(100);
      SetLedColor(0, 0, 0);
      WinkCounter = 0;
    }
  }  
  
  resetMagOffsetCalibration();
  while(!digitalRead(SWITCH_INPUT))
    delay(20);
  delay(200);

  sprintf(StringBuffer, "*** Proceeding to MAG calibration - Max out all axis **** ");
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);
  
  QuitLoop = false;
  while(!QuitLoop)
  {
    delay(50);
    ReadMagneto();
    magOffsetAutocalMax[0] = max(magOffsetAutocalMax[0], MagnetometerX.Value);
    magOffsetAutocalMax[1] = max(magOffsetAutocalMax[1], MagnetometerY.Value);
    magOffsetAutocalMax[2] = max(magOffsetAutocalMax[2], MagnetometerZ.Value);
    magOffsetAutocalMin[0] = min(magOffsetAutocalMin[0], MagnetometerX.Value);
    magOffsetAutocalMin[1] = min(magOffsetAutocalMin[1], MagnetometerY.Value);
    magOffsetAutocalMin[2] = min(magOffsetAutocalMin[2], MagnetometerZ.Value);
   
    LedState = 1 - LedState;
    SetLedColor(1, 1, 1);  
  
    if(!digitalRead(SWITCH_INPUT))
    {
      SetLedColor(0, 1, 0);
      QuitLoop = true;
    } 
  }
  while(!digitalRead(SWITCH_INPUT))
    delay(20);
  for(int i = 0 ; i < 3 ; i++)
  {
     mag_bias[i] = (magOffsetAutocalMax[i] + magOffsetAutocalMin[i]) / 2;
     mbias[i] = mRes * (float)mag_bias[i];
  }

  sprintf(StringBuffer,"*** FOUND Bias mag= %d %d %d", mag_bias[0], mag_bias[1], mag_bias[2]);
  printf("%s\n", StringBuffer);
  PrintToOSC(StringBuffer);
  
  SaveFlashPrefs();
}


boolean GrabSerialMessage(void)
{
  char TheChar;
  if (Serial.available())
  {
    while(Serial.available() && (SerialIndex < MAX_SERIAL))
    {
      TheChar = Serial.read();
      //Serial.print(TheChar);
      if((TheChar == '\n') || (TheChar == '\r'))
      {
        SerialBuffer[SerialIndex] = '\0';
        SerialIndex = 0;
        strcpy(StringBuffer, SerialBuffer);
        FlagSerial = true; 
        return(true);
      }
      else
      {
        SerialBuffer[SerialIndex++] = TheChar;
      }
    }
    if(SerialIndex >= MAX_SERIAL)
      SerialIndex = 0;
  }
  return(false);
}

void ProcessSerial(void)
{
  unsigned char i, j;
  unsigned char Index;

  byte temp_ip[4];

  // Debug
  //printf("Cmd: %s - OK\n",StringBuffer);

  // Send current config to the configuration app
  if(!strncmp("cfgrequest",StringBuffer,10))
  {
    // Outputs all the configuration  
    printf("%s %d\n", TEXT_DHCP, UseDHCP);
    printf("%s %s\n", TEXT_SSID, ssid);
    printf("%s %d\n", TEXT_WIFI_MODE, APorStation);
    printf("%s %d\n", TEXT_SECURITY, UseSecurity);
    printf("%s %s\n", TEXT_PASSWORD, password);
    printf("%s %u.%u.%u.%u\n", TEXT_OWNIP, LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3] );
    printf("%s %u.%u.%u.%u\n", TEXT_DESTIP, DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
    printf("%s %u.%u.%u.%u\n", TEXT_GATEWAY, GatewayIP[0], GatewayIP[1],GatewayIP[2],GatewayIP[3]);
    printf("%s %u.%u.%u.%u\n", TEXT_MASK, SubnetMask[0], SubnetMask[1], SubnetMask[2], SubnetMask[3] );
    printf("%s %u\n", TEXT_PORT, DestPort);
    printf("%s %u\n", TEXT_MASTER_ID, ModuleID);
    printf("%s %u\n", TEXT_SAMPLE_RATE, SampleRate);
    
    // All offsets as lists + rotation matrix
    printf("%s %d\n", TEXT_ACC_OFFSETX, accel_bias[0]);
    printf("%s %d\n", TEXT_ACC_OFFSETY, accel_bias[1]);
    printf("%s %d\n", TEXT_ACC_OFFSETZ, accel_bias[2]);
   
    printf("%s %d\n", TEXT_GYRO_OFFSETX, gyro_bias[0]);
    printf("%s %d\n", TEXT_GYRO_OFFSETY, gyro_bias[1]);
    printf("%s %d\n", TEXT_GYRO_OFFSETZ, gyro_bias[2]);
    
    printf("%s %d\n", TEXT_MAG_OFFSETX, mag_bias[0]);
    printf("%s %d\n", TEXT_MAG_OFFSETY, mag_bias[1]);
    printf("%s %d\n", TEXT_MAG_OFFSETZ, mag_bias[2]);   
   
    printf("%s %f\n", TEXT_BETA, beta); 
      
    printf("refresh\n");
  }

  // Ping / Echo question/answer from the GUI
  else if(!strncmp("ping",StringBuffer,4))
  {
    printf("echo\n");	// a simple ASCII echo answer to let the GUI know the COM port is the right one
    return;
  }

  else if(!strncmp("savecfg",StringBuffer,7))	// Saves config to FLASH
  {
    SaveFlashPrefs();
    // Reboot is needed to use new settings
    //Reboot();	
  }
  
  else if(!strncmp(TEXT_WIFI_MODE,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    sscanf(&StringBuffer[Index],"%d", &APorStation);
    return;	
  }
  
  else if(!strncmp(TEXT_DHCP,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    sscanf(&StringBuffer[Index],"%d", &UseDHCP);
    return;	
  }
  else if(!strncmp(TEXT_SSID,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    memset(ssid, '\0', sizeof(ssid));
    sscanf(&StringBuffer[Index],"%s", ssid);
    return;	
  }
  else if(!strncmp(TEXT_OWNIP,StringBuffer,5))
  {
    Index = SkipToValue(StringBuffer);
    //printf("%s\n",&StringBuffer[Index]);
    ParseIP(&StringBuffer[Index], &LocalIP);
    //printf("own ip update %u.%u.%u.%u\n",pucIP_Addr[0], pucIP_Addr[1], pucIP_Addr[2], pucIP_Addr[3]);
    return;
  }	
  else if(!strncmp(TEXT_DESTIP,StringBuffer,6))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &DestIP);
    return;
  }	
  else if(!strncmp(TEXT_GATEWAY,StringBuffer,7))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &GatewayIP);
    return;
  }	
  else if(!strncmp(TEXT_DNS,StringBuffer,3))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &GatewayIP);
    return;
  }	
  else if(!strncmp(TEXT_MASK,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    ParseIP(&StringBuffer[Index], &SubnetMask);
    return;
  }	
  else if(!strncmp(TEXT_PORT,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    DestPort = atoi(&StringBuffer[Index]);
    return;
  }	
  else if(!strncmp(TEXT_MASTER_ID,StringBuffer,8))
  {
    Index = SkipToValue(StringBuffer);
    ModuleID = atoi(&StringBuffer[Index]);
    return;
  }
  else if(!strncmp(TEXT_SAMPLE_RATE,StringBuffer,10))
  {
    Index = SkipToValue(StringBuffer);
    SampleRate = atoi(&StringBuffer[Index]);
    if(SampleRate < MIN_SAMPLE_RATE)
      SampleRate = MIN_SAMPLE_RATE;

    if(SampleRate > MAX_SAMPLE_RATE)
      SampleRate = MAX_SAMPLE_RATE;

    return;
  }	

  else if(!strncmp(TEXT_ACC_OFFSETX,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    accel_bias[0] = atoi(&StringBuffer[Index]);
    abias[0] = (float)accel_bias[0]*aRes;
    return;
  }
  else if(!strncmp(TEXT_ACC_OFFSETY,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    accel_bias[1] = atoi(&StringBuffer[Index]);
    abias[1] = (float)accel_bias[1]*aRes;
    return;
  }
  else if(!strncmp(TEXT_ACC_OFFSETZ,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    accel_bias[2] = atoi(&StringBuffer[Index]);
    abias[2] = (float)accel_bias[2]*aRes;
    return;
  }
  
  else if(!strncmp(TEXT_GYRO_OFFSETX,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    gyro_bias[0] = atoi(&StringBuffer[Index]);
    gbias[0] = (float)gyro_bias[0]*gRes;
    return;
  }
  else if(!strncmp(TEXT_GYRO_OFFSETY,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    gyro_bias[1] = atoi(&StringBuffer[Index]);
    gbias[1] = (float)gyro_bias[1]*gRes;
    return;
  }
  else if(!strncmp(TEXT_GYRO_OFFSETZ,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    gyro_bias[2] = atoi(&StringBuffer[Index]);
    gbias[2] = (float)gyro_bias[2]*gRes;
    return;
  }
  
  else if(!strncmp(TEXT_MAG_OFFSETX,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    mag_bias[0] = atoi(&StringBuffer[Index]);
    mbias[0] = (float)mag_bias[0]*mRes;
    return;
  }
  else if(!strncmp(TEXT_MAG_OFFSETY,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    mag_bias[1] = atoi(&StringBuffer[Index]);
    mbias[1] = (float)mag_bias[1]*mRes;
    return;
  }
  else if(!strncmp(TEXT_MAG_OFFSETZ,StringBuffer,11))
  {
    Index = SkipToValue(StringBuffer);
    mag_bias[2] = atoi(&StringBuffer[Index]);
    mbias[2] = (float)mag_bias[2]*mRes;
    return;
  }
  
  else if(!strncmp(TEXT_BETA,StringBuffer,4))
  {
    Index = SkipToValue(StringBuffer);
    beta = atof(&StringBuffer[Index]);
    return;
  }


  else if(!strncmp("defaults",StringBuffer,8))
  {
    // Re open in write mode
    SerFlash.close();
    if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE))
    {
      Serial.println("Restoring defaults");
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Please Reboot");
      while(1);
    }	
  }
}

void LoadParams(void)
{
  // Check if file exists
  if(!SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_READ))
  {
    SerFlash.close();
    // Creates the file
    Serial.println("Param File not found");
    if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_CREATE(512, _FS_FILE_OPEN_FLAG_COMMIT)))
    {
      // Re open in write mode
      Serial.println("Param File created and opened for writing");
      Serial.println("Restoring defaults");
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Please Reboot");
      while(1);
      // REBOOT NEEDED 
    }
  }
  else
  {
    Serial.println("Found Param file, parsing");
    
     printf("This is a printf test\n");
    
    int FormatToken;
    GrabLine(StringBuffer);
    // checks if the file is properly formatted with the 0x55 header token
    if(strncmp(StringBuffer, "0x55", 4))
    {
      Serial.println("Restoring defaults");
      SerFlash.close();
      SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE);
      RestoreDefaults();
      SerFlash.close();
      Serial.println("Reboot Device");
    }
    else
    {
      // Parses the preferences
      GrabLine(StringBuffer);
      strcpy(ssid, StringBuffer);
      APorStation = atoi(StringBuffer);
      GrabLine(StringBuffer);
      strcpy(ssid, StringBuffer);
      GrabLine(StringBuffer);
      UseSecurity = atoi(StringBuffer);
      GrabLine(StringBuffer);
      strcpy(password, StringBuffer);
      GrabLine(StringBuffer);
      UseDHCP = atoi(StringBuffer);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &LocalIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &DestIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &GatewayIP);
      GrabLine(StringBuffer);
      ParseIP(StringBuffer, &SubnetMask);
      GrabLine(StringBuffer);
      DestPort = atoi(StringBuffer);      
      GrabLine(StringBuffer);
      ModuleID = atoi(StringBuffer);      
      GrabLine(StringBuffer);
      SampleRate = atoi(StringBuffer);
       
      if(!SampleRate)
      {
        Serial.println("Min Sample Rate is 3 ms");
        SampleRate = MIN_SAMPLE_RATE;
      }
      deltat = (float)SampleRate / 1000.0f;

      // Loading calibration data
      GrabLine(StringBuffer);
      accel_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      accel_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      accel_bias[2] = atoi(StringBuffer);
      

      GrabLine(StringBuffer);
      gyro_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      gyro_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      gyro_bias[2] = atoi(StringBuffer);
      

      GrabLine(StringBuffer);
      mag_bias[0] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      mag_bias[1] = atoi(StringBuffer);
      GrabLine(StringBuffer);
      mag_bias[2] = atoi(StringBuffer);
      
      printf("Wifi Mode = ");
      if(APorStation == STATION_MODE)
        printf("Station\n");
      else
        printf("Access Point\n");
      printf("WiFi Encryption = %d\n", UseSecurity);
      printf("WiFi Password = %s\n", password);
      printf("Use DHCP = %d\n", UseDHCP);
      printf("Loaded Accel Offsets: %d %d %d\n", accel_bias[0], accel_bias[1], accel_bias[2]);
      printf("Loaded Gyro Offsets: %d %d %d\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
      printf("Loaded Mag Offsets: %d %d %d\n", mag_bias[0], mag_bias[1], mag_bias[2]);
      Serial.print("Madgwick Specifics: Beta =");
      Serial.println(beta);
      
      SerFlash.close();
    }
  }
}

void SaveFlashPrefs(void)
{
  int writeStatus;

  // File was opened for Read so far, re open in write mode
  SerFlash.close();
  if(SerFlash.open(PARAMS_FILENAME, FS_MODE_OPEN_WRITE))
  {
    Serial.println("Saving prefs in FLASH");

    // Format token 0x55
    writeStatus = SerFlash.write((uint8_t*)("0x55\n"),5);
    
    // Station or AP mode
    sprintf(StringBuffer, "%d\n", APorStation);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    
    // ssid
    sprintf(StringBuffer, "%s\n", ssid);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // use security ?
    sprintf(StringBuffer, "%d\n", UseSecurity);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    
    sprintf(StringBuffer, "%s\n", password);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%d\n", UseDHCP);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", LocalIP[0], LocalIP[1], LocalIP[2], LocalIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", DestIP[0], DestIP[1], DestIP[2], DestIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", GatewayIP[0], GatewayIP[1],GatewayIP[2],GatewayIP[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u.%u.%u.%u\n", SubnetMask[0], SubnetMask[1],SubnetMask[2],SubnetMask[3]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", DestPort);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", ModuleID);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    sprintf(StringBuffer, "%u\n", SampleRate);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Calibration data
    // Accel offset
    sprintf(StringBuffer, "%d\n", accel_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", accel_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", accel_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Gyro offset
    sprintf(StringBuffer, "%d\n", gyro_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", gyro_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", gyro_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

    // Mag offset
    sprintf(StringBuffer, "%d\n", mag_bias[0]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", mag_bias[1]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
    sprintf(StringBuffer, "%d\n", mag_bias[2]);
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer)); 

    SerFlash.close();
    Serial.println("Done");
  }
  else
  {
    Serial.println("Error saving params");
  }
}


// Assumes the file is already opened for writing
void RestoreDefaults(void)
{
  int writeStatus;
  
  // Clears the "file"
  //for(int i = 0 ; i < 500 ; i++)
  //  writeStatus = SerFlash.write((uint8_t*)0x00,1);
  
  // Format token 0x55
  writeStatus = SerFlash.write((uint8_t*)("0x55\n"),5);
  
  // Use Station mode
  sprintf(StringBuffer, "%d\n", STATION_MODE);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  // ssid
  sprintf(StringBuffer, "%s\n", TheSSID);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Use Security = false
  sprintf(StringBuffer, "%d\n", false);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  // default password = 12345678
  sprintf(StringBuffer, "12345678\n");
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Use DHCP = true
  sprintf(StringBuffer, "%d\n", true);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Fixed / static IP
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheLocalIP[0], TheLocalIP[1], TheLocalIP[2], TheLocalIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Destination computer IP
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheDestIP[0], TheDestIP[1], TheDestIP[2], TheDestIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheGatewayIP[0], TheGatewayIP[1],TheGatewayIP[2],TheGatewayIP[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  sprintf(StringBuffer, "%u.%u.%u.%u\n", TheSubnetMask[0], TheSubnetMask[1],TheSubnetMask[2],TheSubnetMask[3]);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u\n", TheDestPort);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
  
  sprintf(StringBuffer, "%u\n", TheID);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  sprintf(StringBuffer, "%u\n", TheSampleRate);
  writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));

  // Calibration data
  // Accel offset
  sprintf(StringBuffer, "0\n");
  // reseting all offsets to zero
  for(int i=0 ; i < 9 ; i++)
    writeStatus = SerFlash.write((uint8_t*)StringBuffer, StringLength(StringBuffer));
}


void EmptyString(char* TheString, int size)
{
  for(int i = 0 ; i < size ; i++)
    TheString[i] = '\0';
}

void PrintToOSC(char *StringMessage)
{
  char LocalString[50];
  
  if(WiFi.status() != WL_CONNECTED)
    return;
  UdpPacket.beginPacket(DestIP, DestPort);
  sprintf(LocalString, "/%d/message", ModuleID);
  StringToOsc(&Message, LocalString, StringMessage);
  UdpPacket.write((uint8_t*)Message.buf, Message.PacketSize);
  UdpPacket.endPacket();
  delay(20);
}






