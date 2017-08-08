// LSM9DS0 Driver
// Emmanuel FLETY - March 2014

// ported to arduino and alike : march 2015

#include <energia.h>
#include <SPI.h>
#include <Wire.h>
#include "common.h"
#include "LSM9DS0.h"

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

void InitLSM9DS0(void)
{	
  if(CommunicationMode == SPI_MODE)
  {
    // Motion sensor I/Os
    pinMode(ACC_CS, OUTPUT);
    pinMode(GYR_CS, OUTPUT);
    digitalWrite(ACC_CS, HIGH);
    digitalWrite(GYR_CS, HIGH);
  }
  else
  {
    Wire.begin();
  } 
  delay(1);
		
  // We initialize the sensors (accel + gyro) in normal mode to get the proper
  // 16 bit resolution
  // We use an ODR of 400 as we used to sample the motion sensor at 2ms (<=> 500 Hz)
  // All axis X,Y,Z enabled
  	
  WriteAccLSM9DS0(CTRL_REG0_XM, 0b00000000);	// FIFO disable
  WriteAccLSM9DS0(CTRL_REG1_XM, 0b10001111);	// ODR 400Hz - BDU enable - All axis ON
  WriteAccLSM9DS0(CTRL_REG2_XM, 0b01011000);	// LP filter 194 Hz / +-8g / 4 wire SPI
  //WriteAccLSM9DS0(CTRL_REG2_XM, 0b01000000);	// LP filter 194 Hz / +-2g / 4 wire SPI
  WriteAccLSM9DS0(CTRL_REG3_XM, 0b00000000);	// Disable all interrupts on INT1
  WriteAccLSM9DS0(CTRL_REG4_XM, 0b00000000);	// Disable all interrupts on INT2
  WriteAccLSM9DS0(CTRL_REG5_XM, 0b11101100);	// Temp enable / Mag res high / 25 Hz mag rate
  //WriteAccLSM9DS0(CTRL_REG6_XM, 0b00100000);	// Mag scale +-4 gauss
  WriteAccLSM9DS0(CTRL_REG6_XM, 0b00000000);	// Mag scale +-2 gauss
  //WriteAccLSM9DS0(CTRL_REG6_XM, 0b01100000);	// Mag scale +-12 gauss
  WriteAccLSM9DS0(CTRL_REG7_XM, 0b00000000);	// HPF dis. / Cont. conversion for mag.
  WriteAccLSM9DS0(FIFO_CTRL_REG_A, 0b00000000);	// FIFO disable, bypass mode
	
  // Gyro init
  WriteGyrLSM9DS0(CTRL_REG1_G, 0b10111111);	// 380 Hz ODR + 100 Hz cut-off - normal mode
  WriteGyrLSM9DS0(CTRL_REG2_G, 0b00000100);	// @ODR 380 Hz, 1.8Hz HPF
  WriteGyrLSM9DS0(CTRL_REG3_G, 0b00000000);
  WriteGyrLSM9DS0(CTRL_REG4_G, 0b10100000);	// 4 wire SPI, LSB first (big endian), 2000 °/s
  //WriteGyrLSM9DS0(CTRL_REG4_G, 0b10000000);	// 4 wire SPI, LSB first (big endian), 245 °/s
  WriteGyrLSM9DS0(CTRL_REG5_G, 0b00000000);	// HPF disabled, normal mode, FIFO disabled
}

void PowerDownLSM9DS0(void)
{
  WriteAccLSM9DS0(CTRL_REG1_XM, 0b00001111);	// ODR 0Hz = power down
  WriteAccLSM9DS0(CTRL_REG7_XM, 0b00000011);	// Magnetic sensor power down
  WriteGyrLSM9DS0(CTRL_REG1_G, 0b10110111);	// 380 Hz ODR + 100 Hz cut-off - Power down mode
}


void PowerUpLSM9DS0(void)
{
  WriteAccLSM9DS0(CTRL_REG1_XM, 0b10001111);	// ODR 400Hz - BDU enable - All axis ON
  WriteAccLSM9DS0(CTRL_REG7_XM, 0b00000000);	// HPF dis. / Cont. conversion for mag.
  WriteGyrLSM9DS0(CTRL_REG1_G, 0b10111111);	// 380 Hz ODR + 100 Hz cut-off - normal mode
}



void WriteAccLSM9DS0(unsigned char RegAddress, unsigned char Data)
{
  
  if(CommunicationMode == SPI_MODE)
  {
    RegAddress = RegAddress & 0b00111111; // Write and MS bit = 0, no auto-increment
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);		
    SPI.transfer(RegAddress);
    SPI.transfer(Data);
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(RegAddress);
    Wire.write(Data);
    Wire.endTransmission();
  }
}


// Single Byte read routine
unsigned char ReadAccLSM9DS0(unsigned char RegAddress)
{
  unsigned char DataLSM9DS0;
  
  if(CommunicationMode == SPI_MODE)
  {	
    RegAddress = RegAddress | 0b10000000; // Read bit =1 and MS bit = 0 (no auto-increment)
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);
    SPI.transfer(RegAddress);
    DataLSM9DS0 = SPI.transfer(0xFF); // dummy write
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(RegAddress);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS0_ADDRESS_ACCELMAG, 1);
    // Wait around until enough data is available
    while (Wire.available() < 1);
    DataLSM9DS0 = Wire.read();
    Wire.endTransmission();
  }
  
  return(DataLSM9DS0);
}


void WriteGyrLSM9DS0(unsigned char RegAddress, unsigned char Data)
{
  
  if(CommunicationMode == SPI_MODE)
  {
    RegAddress = RegAddress & 0b00111111; // Write and MS bit = 0, no auto-increment
    // Asserting the chip
    digitalWrite(GYR_CS, LOW);
    SPI.transfer(RegAddress);
    SPI.transfer(Data);
    digitalWrite(GYR_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_GYRO);
    Wire.write(RegAddress);
    Wire.write(Data);
    Wire.endTransmission();
  }


}


// Single Byte read routine
unsigned char ReadGyrLSM9DS0(unsigned char RegAddress)
{
  unsigned char DataLSM9DS0;
  
  if(CommunicationMode == SPI_MODE)
  {
    RegAddress = RegAddress | 0b10000000; // Read bit =1 and MS bit = 0 (no auto-increment)
    // Asserting the chip
    digitalWrite(GYR_CS, LOW);
    SPI.transfer(RegAddress);
    DataLSM9DS0 = SPI.transfer(0xFF); // dummy write
    digitalWrite(GYR_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_GYRO);
    Wire.write(RegAddress);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS0_ADDRESS_GYRO, 1);
    // Wait around until enough data is available
    while (Wire.available() < 1);
    DataLSM9DS0 = Wire.read();
    Wire.endTransmission();
  }
  
  return(DataLSM9DS0);
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
    AccelerationX.Val[0] = LSB;
    AccelerationX.Val[1] = MSB;
	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    AccelerationY.Val[0] = LSB;
    AccelerationY.Val[1] = MSB;
	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    AccelerationZ.Val[0] = LSB;
    AccelerationZ.Val[1] = MSB;	
    digitalWrite(ACC_CS, HIGH);	
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(OUT_X_L_A | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS0_ADDRESS_ACCELMAG, 6);
    // Wait around until enough data is available
    while (Wire.available() < 6);
    LSB = Wire.read();
    MSB = Wire.read();
    AccelerationX.Val[0] = LSB;
    AccelerationX.Val[1] = MSB;
	
    LSB = Wire.read();
    MSB = Wire.read();
    AccelerationY.Val[0] = LSB;
    AccelerationY.Val[1] = MSB;
	
    LSB = Wire.read();
    MSB = Wire.read();
    AccelerationZ.Val[0] = LSB;
    AccelerationZ.Val[1] = MSB;	
    Wire.endTransmission();
  }
}


void ReadGyro(void)
{
  unsigned char LSB, MSB;
  
  if(CommunicationMode == SPI_MODE)
  {
    // Asserting the chip
    digitalWrite(GYR_CS, LOW);
    SPI.transfer(OUT_X_L_G | READ_AND_AUTOINCREMENT);
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    GyroscopeX.Val[0] = LSB;
    GyroscopeX.Val[1] = MSB;	
	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    GyroscopeY.Val[0] = LSB;
    GyroscopeY.Val[1] = MSB;
  	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    GyroscopeZ.Val[0] = LSB;
    GyroscopeZ.Val[1] = MSB;
    digitalWrite(GYR_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_GYRO);
    Wire.write(OUT_X_L_G | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS0_ADDRESS_GYRO, 6);
    // Wait around until enough data is available
    while (Wire.available() < 6);
    LSB = Wire.read();
    MSB = Wire.read();
    GyroscopeX.Val[0] = LSB;
    GyroscopeX.Val[1] = MSB;	
	
    LSB = Wire.read();
    MSB = Wire.read();
    GyroscopeY.Val[0] = LSB;
    GyroscopeY.Val[1] = MSB;
  	
    LSB = Wire.read();
    MSB = Wire.read();
    GyroscopeZ.Val[0] = LSB;
    GyroscopeZ.Val[1] = MSB;
    Wire.endTransmission();
  }
}



void ReadMagneto(void)
{
  unsigned char LSB, MSB;

  if(CommunicationMode == SPI_MODE)
  {
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);
    SPI.transfer(OUT_X_L_M | READ_AND_AUTOINCREMENT);
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    MagnetometerX.Val[0] = LSB;
    MagnetometerX.Val[1] = MSB;
    	
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    MagnetometerY.Val[0] = LSB;
    MagnetometerY.Val[1] = MSB;
    
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    MagnetometerZ.Val[0] = LSB;
    MagnetometerZ.Val[1] = MSB;	
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(OUT_X_L_M | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS0_ADDRESS_ACCELMAG, 6);
    // Wait around until enough data is available
    while (Wire.available() < 6);
    LSB = Wire.read();
    MSB = Wire.read();
    MagnetometerX.Val[0] = LSB;
    MagnetometerX.Val[1] = MSB;	
	
    LSB = Wire.read();
    MSB = Wire.read();
    MagnetometerY.Val[0] = LSB;
    MagnetometerY.Val[1] = MSB;
  	
    LSB = Wire.read();
    MSB = Wire.read();
    MagnetometerZ.Val[0] = LSB;
    MagnetometerZ.Val[1] = MSB;
    Wire.endTransmission();
  }
}

void ReadTemperature(void)
{
  unsigned char LSB, MSB;
        
  // Temperature 12 bit / 8 LSB per degre centigrade
  if(CommunicationMode == SPI_MODE)
  {
    // Asserting the chip
    digitalWrite(ACC_CS, LOW);
    SPI.transfer(OUT_TEMP_L_XM | READ_AND_AUTOINCREMENT);
    LSB = SPI.transfer(0xFF);
    MSB = SPI.transfer(0xFF);
    Temperature.Val[0] = LSB;
    Temperature.Val[1] = MSB;
    digitalWrite(ACC_CS, HIGH);
  }
  else
  {
    Wire.beginTransmission(LSM9DS0_ADDRESS_ACCELMAG);
    Wire.write(OUT_TEMP_L_XM | JUST_READ);
    Wire.endTransmission();
    Wire.requestFrom(LSM9DS0_ADDRESS_ACCELMAG, 2);
    // Wait around until enough data is available
    while (Wire.available() < 2);
    LSB = Wire.read();
    MSB = Wire.read();
    Temperature.Val[0] = LSB;
    Temperature.Val[1] = MSB;	
    
    Wire.endTransmission();
  }
}


