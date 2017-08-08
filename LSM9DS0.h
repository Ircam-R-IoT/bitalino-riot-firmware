#ifndef	_LSM9DS0_H
#define	_LSM9DS0_H

// LSM9DS0 Driver
// Emmanuel FLETY - March 2014
// ported to arduino and alike : march 2015

#define NORMAL_CONFIG_ACCEL	0b01110111
#define LP_CONFIG_ACCEL		0b00001000

#define SPI_MODE                0x00
#define I2C_MODE                0x01

// I2C addresses
#define LSM9DS0_ADDRESS_ACCELMAG           (0x1D)         // 3B >> 1 = 7bit default
#define LSM9DS0_ADDRESS_GYRO               (0x6B)         // D6 >> 1 = 7bit default


// Register map definition
// Accel / Magneto register map
#define OUT_TEMP_L_XM		0x05
#define OUT_TEMP_H_XM		0x06
#define STATUS_REG_M		0x07
#define OUT_X_L_M		0x08
#define OUT_X_H_M		0x09
#define OUT_Y_L_M		0x0A
#define OUT_Y_H_M		0x0B
#define OUT_Z_L_M		0x0C
#define OUT_Z_H_M		0x0D

#define WHO_AM_I_M		0x0F
#define INT_CFG_M		0x12
#define INT_SOURCE_M		0x13
#define INT_TSH_L_M		0x14
#define INT_TSH_H_M		0x15
#define OFFSET_X_L_M		0x16
#define OFFSET_X_H_M		0x17
#define OFFSET_Y_L_M		0x18
#define OFFSET_Y_H_M		0x19
#define OFFSET_Z_L_M		0x1A
#define OFFSET_Z_H_M		0x1B
#define REFERENCE_X		0x1C
#define REFERENCE_Y		0x1D
#define REFERENCE_Z		0x1E
#define CTRL_REG0_XM		0x1F
#define CTRL_REG1_XM		0x20
#define CTRL_REG2_XM		0x21
#define CTRL_REG3_XM		0x22
#define CTRL_REG4_XM		0x23
#define CTRL_REG5_XM		0x24
#define CTRL_REG6_XM		0x25
#define CTRL_REG7_XM		0x26

#define STATUS_REG_A		0x27
#define OUT_X_L_A		0x28
#define OUT_X_H_A		0x29
#define OUT_Y_L_A		0x2A
#define OUT_Y_H_A		0x2B
#define OUT_Z_L_A		0x2C
#define OUT_Z_H_A		0x2D
#define FIFO_CTRL_REG_A		0x2E
#define FIFO_SRC_REG_A		0x2F
#define INT1_CFG_A		0x30
#define INT1_SOURCE_A		0x31
#define INT1_THS_A		0x32
#define INT1_DURATION_A		0x33
#define INT2_CFG_A		0x34
#define INT2_SOURCE_A		0x35
#define INT2_THS_A		0x36
#define INT2_DURATION_A		0x37
#define CLICK_CFG_A		0x38
#define CLICK_SRC_A		0x39
#define CLICK_THS_A		0x3A
#define TIME_LIMIT_A		0x3B
#define TIME_LATENCY_A		0x3C
#define TIME_WINDOW_A		0x3D
#define ACT_THS			0x3E
#define ACT_DUR			0x3F

// Gyro REGISTER MAP
#define WHO_AM_I_G		0x0F
#define CTRL_REG1_G		0x20
#define CTRL_REG2_G		0x21
#define CTRL_REG3_G		0x22
#define CTRL_REG4_G		0x23
#define CTRL_REG5_G		0x24
#define REFERENCE_G		0x25

#define STATUS_REG_G		0x27
#define OUT_X_L_G		0x28
#define OUT_X_H_G		0x29
#define OUT_Y_L_G		0x2A
#define OUT_Y_H_G		0x2B
#define OUT_Z_L_G		0x2C
#define OUT_Z_H_G		0x2D
#define FIFO_CTRL_REG_G		0x2E
#define FIFO_SRC_REG_G		0x2F
#define INT1_CFG_G		0x30
#define INT1_SOURCE_G		0x31
#define INT1_TSH_XH_G		0x32
#define INT1_TSH_XL_G		0x33
#define INT1_TSH_YH_G		0x34
#define INT1_TSH_YL_G		0x35
#define INT1_TSH_ZH_G		0x36
#define INT1_TSH_ZL_G		0x37
#define INT1_DURATION_G		0x38

#define READ_AND_AUTOINCREMENT	0b11000000
#define JUST_READ		0b10000000

#define LSM9DS0_READ		1
#define LSM9DS0_WRITE		0

#define LSM9DS0_TEMP_OFFSET    

// Define of the chip select pins
#define GYR_CS             11
#define ACC_CS             18

void InitLSM9DS0(void);

void WriteAccLSM9DS0(unsigned char RegAddress, unsigned char Data);
unsigned char ReadAccLSM9DS0(unsigned char RegAddress);

void WriteGyrLSM9DS0(unsigned char RegAddress, unsigned char Data);
unsigned char ReadGyrLSM9DS0(unsigned char RegAddress);

void PowerDownLSM9DS0(void);
void PowerUpLSM9DS0(void);

void ReadAccel(void);
void ReadGyro(void);
void ReadMagneto(void);
void ReadTemperature(void);

#endif
