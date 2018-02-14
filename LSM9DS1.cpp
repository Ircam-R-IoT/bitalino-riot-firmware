// LSM9DS1 Driver
// Emmanuel FLETY - March 2017

#include <energia.h>
#include <SPI.h>
#include <Wire.h>
#include "common.h"
#include "LSM9DS1.h"

extern Word AccelerationX;
extern Word AccelerationY;
extern Word AccelerationZ;
extern Word GyroscopeX;
extern Word GyroscopeY;
extern Word GyroscopeZ;
extern Word MagnetometerX;
extern Word MagnetometerY;
extern Word MagnetometerZ;

extern Word Temperature;
extern byte CommunicationMode;


unsigned char CheckWhoAmI(unsigned char Byte)
{
  switch(Byte)
  {
	case WHO_AM_I_TOKEN:
          return(1);

	default:
	  return(0);
  }
}


void InitLSM9DS1(void)
{	
  
		
  // We initialize the sensors (accel + gyro) in normal mode to get the proper
  // 16 bit resolution
  // We use an ODR > 400 Hz as we used to sample the motion sensor at 2ms (<=> 500 Hz)
  // All axis X,Y,Z enabled
  
  // TODO : set sensitivity back to +- 8G, 2 gaus
  	
  // Gyroscope configuration
  WriteAccLSM9DS1(CTRL_REG1_G, 0b10111000);   // 476 Hz ODR + 100 Hz cut-off / 2000°/s 
  WriteAccLSM9DS1(CTRL_REG2_G, 0b00000000);   // HPF & LPF2 Filters bypassed
  WriteAccLSM9DS1(CTRL_REG3_G, 0b00000000);
  WriteAccLSM9DS1(CTRL_REG4_G, 0b00111000);   // Gyro axis enable
  
  // Accelerometer configuration
  WriteAccLSM9DS1(CTRL_REG5_XL, 0b00111000);	// Accel axis enable
  //WriteAccLSM9DS1(CTRL_REG6_XL, 0b00010000);	// ODR = Gyro ODR / +-4g / LP BW based on ODR = 211 Hz
  WriteAccLSM9DS1(CTRL_REG6_XL, 0b00011000);	// ODR = Gyro ODR / +-8g / LP BW based on ODR = 211 Hz
  WriteAccLSM9DS1(CTRL_REG8,    0b01000100);  // BDU on / 4-wire SPI / Auto INC
  WriteAccLSM9DS1(CTRL_REG9,    0b00000100);  // SPI only / I2C disable        
    
  // Magneto configuration
  WriteMagLSM9DS1(CTRL_REG1_M, 0b11010100);	// Temp comp. EN / Hi Perf / 20 Hz
  WriteMagLSM9DS1(CTRL_REG2_M, 0b00000000);   // +- 4 gauss
  WriteMagLSM9DS1(CTRL_REG3_M, 0b10000000);   // I2C dis. / SPI R+W / continuous output
  WriteMagLSM9DS1(CTRL_REG4_M, 0b00001000);   // Z axis Hi Perf
  WriteMagLSM9DS1(CTRL_REG5_M, 0b00000000);   // Block update BDU Off
}


void RebootLSM9DS1(void)
{
  WriteAccLSM9DS1(CTRL_REG8,0b01000101);      // BIT 0 = SOFT RESET
  delay(10);	// wait reboot finished
}

void WriteAccLSM9DS1(unsigned char RegAddress, unsigned char Data)
{
  
  if(CommunicationMode == SPI_MODE)
  {
    RegAddress = RegAddress & 0b01111111; // Write and MS bit = 0, no auto-increment
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);		
    SPI.transfer(RegAddress);
    SPI.transfer(Data);
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_ACCELGYRO);
    Wire.write(RegAddress);
    Wire.write(Data);
    Wire.endTransmission();
  }
}


// Single Byte read routine
unsigned char ReadAccLSM9DS1(unsigned char RegAddress)
{
  unsigned char DataLSM9DS1;
  
  if(CommunicationMode == SPI_MODE)
  {	
    RegAddress = RegAddress | 0b10000000; // Read bit =1 and MS bit = 0 (no auto-increment)
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);
    SPI.transfer(RegAddress);
    DataLSM9DS1 = SPI.transfer(0xFF); // dummy write
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_MAG);
    Wire.write(RegAddress);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS1_ADDRESS_MAG, 1);
    // Wait around until enough data is available
    while (Wire.available() < 1);
    DataLSM9DS1 = Wire.read();
    Wire.endTransmission();
  }
  
  return(DataLSM9DS1);
}


void WriteMagLSM9DS1(unsigned char RegAddress, unsigned char Data)
{
  
  if(CommunicationMode == SPI_MODE)
  {
    RegAddress = RegAddress & 0b01111111; // Write and MS bit = 0, no auto-increment
    // Asserting the chip
    digitalWrite(MAG_CS, LOW);
    SPI.transfer(RegAddress);
    SPI.transfer(Data);
    digitalWrite(MAG_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_MAG);
    Wire.write(RegAddress);
    Wire.write(Data);
    Wire.endTransmission();
  }
}


// Single Byte read routine
unsigned char ReadMagLSM9DS1(unsigned char RegAddress)
{
  unsigned char DataLSM9DS1;
  
  if(CommunicationMode == SPI_MODE)
  {
    RegAddress = RegAddress | 0b10000000; // Read bit =1 and MS bit = 0 (no auto-increment)
    // Asserting the chip
    digitalWrite(MAG_CS, LOW);
    SPI.transfer(RegAddress);
    DataLSM9DS1 = SPI.transfer(0xFF); // dummy write
    digitalWrite(MAG_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_MAG);
    Wire.write(RegAddress);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS1_ADDRESS_MAG, 1);
    // Wait around until enough data is available
    while (Wire.available() < 1);
    DataLSM9DS1 = Wire.read();
    Wire.endTransmission();
  }
  
  return(DataLSM9DS1);
}


void ReadAccel(void)
{
  unsigned char LSB, MSB;  
  if(CommunicationMode == SPI_MODE)
  {
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);
    SPI.transfer(OUT_X_L_A | READ_AND_AUTOINCREMENT);
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    
    // We swap X and Y based on the new chip (LS1) vs. the former one (LS0)
    // and chip orientation on PCB. 
    // In addition, axis orientation is inverted
    // Could use a axis order swapper + Sign
    // Orientation of the chip is 3x mom. switches / led down, power switch / antenna up
    AccelerationY.Val[0] = LSB;
    AccelerationY.Val[1] = MSB;
	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    AccelerationX.Val[0] = LSB;
    AccelerationX.Val[1] = MSB;
	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    AccelerationZ.Val[0] = LSB;
    AccelerationZ.Val[1] = MSB;	
    digitalWrite(ACC_CS, HIGH);	
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_ACCELGYRO);
    Wire.write(OUT_X_L_A | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS1_ADDRESS_ACCELGYRO, 6);
    // Wait around until enough data is available
    while (Wire.available() < 6);
    LSB = Wire.read();
    MSB = Wire.read();
    AccelerationY.Val[0] = LSB;
    AccelerationY.Val[1] = MSB;
    
	
    LSB = Wire.read();
    MSB = Wire.read();
    AccelerationX.Val[0] = LSB;
    AccelerationX.Val[1] = MSB;
    
	
    LSB = Wire.read();
    MSB = Wire.read();
    AccelerationZ.Val[0] = LSB;
    AccelerationZ.Val[1] = MSB;	
    Wire.endTransmission();
  }
  // Sign
  AccelerationX.Value = -AccelerationX.Value;
  AccelerationY.Value = -AccelerationY.Value;
}


void ReadGyro(void)
{
  unsigned char LSB, MSB;
  
  if(CommunicationMode == SPI_MODE)
  {
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);
    SPI.transfer(OUT_X_L_G | READ_AND_AUTOINCREMENT);
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    GyroscopeY.Val[0] = LSB;
    GyroscopeY.Val[1] = MSB;	
	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    GyroscopeX.Val[0] = LSB;
    GyroscopeX.Val[1] = MSB;
  	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    GyroscopeZ.Val[0] = LSB;
    GyroscopeZ.Val[1] = MSB;
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_ACCELGYRO);
    Wire.write(OUT_X_L_G | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS1_ADDRESS_ACCELGYRO, 6);
    // Wait around until enough data is available
    while (Wire.available() < 6);
    LSB = Wire.read();
    MSB = Wire.read();
    GyroscopeY.Val[0] = LSB;
    GyroscopeY.Val[1] = MSB;
  	
	
    LSB = Wire.read();
    MSB = Wire.read();
    GyroscopeX.Val[0] = LSB;
    GyroscopeX.Val[1] = MSB;
    
  	
    LSB = Wire.read();
    MSB = Wire.read();
    GyroscopeZ.Val[0] = LSB;
    GyroscopeZ.Val[1] = MSB;
    Wire.endTransmission();
  }
  
  // Sign
  GyroscopeY.Value = -GyroscopeY.Value;
  GyroscopeX.Value = -GyroscopeX.Value;
}


void ReadMagneto(void)
{
  unsigned char LSB, MSB;

  if(CommunicationMode == SPI_MODE)
  {
    // Asserting the chip
    digitalWrite(MAG_CS, LOW);
    SPI.transfer(OUT_X_L_M | MAG_READ_AND_AUTOINCREMENT);
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    MagnetometerY.Val[0] = LSB;
    MagnetometerY.Val[1] = MSB;
    
    	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    MagnetometerX.Val[0] = LSB;
    MagnetometerX.Val[1] = MSB;
    
    
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    MagnetometerZ.Val[0] = LSB;
    MagnetometerZ.Val[1] = MSB;	
    digitalWrite(MAG_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_ACCELGYRO);
    Wire.write(OUT_X_L_M | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS1_ADDRESS_ACCELGYRO, 6);
    // Wait around until enough data is available
    while (Wire.available() < 6);
    LSB = Wire.read();
    MSB = Wire.read();
    MagnetometerY.Val[0] = LSB;
    MagnetometerY.Val[1] = MSB;
   
    LSB = Wire.read();
    MSB = Wire.read();
    MagnetometerX.Val[0] = LSB;
    MagnetometerX.Val[1] = MSB;
  	
    LSB = Wire.read();
    MSB = Wire.read();
    MagnetometerZ.Val[0] = LSB;
    MagnetometerZ.Val[1] = MSB;
    Wire.endTransmission();
  }
  // Sign
  MagnetometerX.Value = -MagnetometerX.Value;
  MagnetometerZ.Value = -MagnetometerZ.Value;
  
}

void ReadTemperature(void)
{
  unsigned char LSB, MSB;
        
  // Temperature 12 bit / 8 LSB per degre centigrade
  if(CommunicationMode == SPI_MODE)
  {
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);
    SPI.transfer(OUT_TEMP_L | READ_AND_AUTOINCREMENT);
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    Temperature.Val[0] = LSB;
    Temperature.Val[1] = MSB;
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS1_ADDRESS_MAG);
    Wire.write(OUT_TEMP_L | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS1_ADDRESS_MAG, 2);
    // Wait around until enough data is available
    while (Wire.available() < 2);
    LSB = Wire.read();
    MSB = Wire.read();
    Temperature.Val[0] = LSB;
    Temperature.Val[1] = MSB;	
    Wire.endTransmission();
  }
}


