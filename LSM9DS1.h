#ifndef	_LSM9DS1_H
#define	_LSM9DS1_H

// LSM9DS1 Driver
// Emmanuel FLETY - March 2017

// ported to arduino and alike : June 2017

#define NORMAL_CONFIG_ACCEL	0b01110111
#define LP_CONFIG_ACCEL		0b00001000

#define SPI_MODE                0x00
#define I2C_MODE                0x01

// I2C addresses
// Vérifier pour LSM9DS1
#define LSM9DS1_ADDRESS_ACCELGYRO         (0x1D)       
#define LSM9DS1_ADDRESS_MAG               (0x6B)


// Register map definition

// Gyro REGISTER MAP
#define WHO_AM_I			0x0F
#define WHO_AM_I_TOKEN      0b01101000  // WHO AM I for Gyro and Accel

#define CTRL_REG1_G			0x10
#define CTRL_REG2_G			0x11
#define CTRL_REG3_G			0x12
#define ORIENT_CFG_G        0x13

// Temperature is part of Gyro readings
#define OUT_TEMP_L  		0x15
#define OUT_TEMP_H  		0x16

#define STATUS_REG          0x17
#define OUT_X_L_G			0x18
#define OUT_X_H_G			0x19
#define OUT_Y_L_G			0x1A
#define OUT_Y_H_G			0x1B
#define OUT_Z_L_G			0x1C
#define OUT_Z_H_G			0x1D

#define CTRL_REG4_G			0x1E

// Accel REGISTER MAP
#define CTRL_REG5_XL		0x1F
#define CTRL_REG6_XL		0x20
#define CTRL_REG7_XL		0x21
#define CTRL_REG8   		0x22
#define CTRL_REG9   		0x23
#define CTRL_REG10   		0x24
#define STATUS_REG_A		0x27
#define OUT_X_L_A			0x28
#define OUT_X_H_A			0x29
#define OUT_Y_L_A			0x2A
#define OUT_Y_H_A			0x2B
#define OUT_Z_L_A			0x2C
#define OUT_Z_H_A			0x2D

#define FIFO_CTRL_REG_G		0x2E


// Magneto REGISTER MAP
#define WHO_AM_I_M			0x0F
#define WHO_AM_I_TOKEN_M    0b00111101  // WHO AM I for Magneto

#define CTRL_REG1_M         0x20
#define CTRL_REG2_M         0x21
#define CTRL_REG3_M         0x22
#define CTRL_REG4_M         0x23
#define CTRL_REG5_M         0x24
#define CTRL_REG6_M         0x25
#define CTRL_REG7_M         0x26

#define STATUS_REG_M        0x27
#define OUT_X_L_M			0x28
#define OUT_X_H_M			0x29
#define OUT_Y_L_M			0x2A
#define OUT_Y_H_M			0x2B
#define OUT_Z_L_M			0x2C
#define OUT_Z_H_M			0x2D


#define READ_AND_AUTOINCREMENT          0b10000000
#define MAG_READ_AND_AUTOINCREMENT      0b11000000
#define JUST_READ                       0b10000000

#define LSM9DS1_READ            1
#define LSM9DS1_WRITE           0

#define LSM_BIAS_TEMPERATURE            25
#define LSM_TEMP_SCALE                  16  // 16 LSB / °C    


// Define of the chip select pins
#define MAG_CS             11
#define ACC_CS             18


void InitLSM9DS1(void);

unsigned char CheckWhoAmI(unsigned char Byte);

void WriteAccLSM9DS1(unsigned char RegAddress, unsigned char Data);
unsigned char ReadAccLSM9DS1(unsigned char RegAddress);

void WriteMagLSM9DS1(unsigned char RegAddress, unsigned char Data);
unsigned char ReadMagLSM9DS1(unsigned char RegAddress);

void RebootLSM9DS1(void);

void ReadAccel(void);
void ReadGyro(void);
void ReadMagneto(void);
void ReadTemperature(void);

#endif
