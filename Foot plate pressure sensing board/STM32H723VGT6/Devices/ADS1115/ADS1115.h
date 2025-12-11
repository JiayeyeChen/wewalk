#include "stm32h7xx_hal.h"

#define ADS1115_ADDR 0x48 // Default I2C address
#define ADS1115_ADDRESS 0x48 // Default I2C address
#define ADS1115_ADDR_GND  0x48  // ADDR pin connected to GND
#define ADS1115_ADDR_VDD  0x49  // ADDR pin connected to VDD
#define ADS1115_ADDR_SDA  0x4A  // ADDR pin connected to SDA
#define ADS1115_ADDR_SCL  0x4B  // ADDR pin connected to SCL

// Register addresses
#define ADS1115_REG_CONVERSION 0x00
#define ADS1115_REG_CONFIG     0x01
#define ADS1115_REG_LO_THRESH  0x02
#define ADS1115_REG_HI_THRESH  0x03
#define ADS1115_OS_SINGLE       0x8000  // Write: Start single conversion
#define ADS1115_OS_BUSY         0x0000  // Read: Conversion in progress
#define ADS1115_OS_NOTBUSY      0x8000  // Read: Conversion complete

// Configuration settings
typedef enum {
    ADS1115_MUX_AIN0_AIN1 = 0x0000, // Differential P=AIN0, N=AIN1
    ADS1115_MUX_AIN0_AIN3 = 0x1000,
    ADS1115_MUX_AIN1_AIN3 = 0x2000,
    ADS1115_MUX_AIN2_AIN3 = 0x3000,
    ADS1115_MUX_AIN0_GND  = 0x4000, // Single-ended
    ADS1115_MUX_AIN1_GND  = 0x5000,
    ADS1115_MUX_AIN2_GND  = 0x6000,
    ADS1115_MUX_AIN3_GND  = 0x7000
} ADS1115_MUX;

typedef enum {
    ADS1115_PGA_6_144V = 0x0000, // ±6.144V range = Gain 2/3
    ADS1115_PGA_4_096V = 0x0200, // ±4.096V range = Gain 1
    ADS1115_PGA_2_048V = 0x0400, // ±2.048V range = Gain 2 (default)
    ADS1115_PGA_1_024V = 0x0600, // ±1.024V range = Gain 4
    ADS1115_PGA_0_512V = 0x0800, // ±0.512V range = Gain 8
    ADS1115_PGA_0_256V = 0x0A00  // ±0.256V range = Gain 16
} ADS1115_PGA;

typedef enum {
    ADS1115_MODE_CONTINUOUS = 0x0000,
    ADS1115_MODE_SINGLE     = 0x0100 // Single-shot mode (default)
} ADS1115_MODE;

typedef enum {
    ADS1115_DR_8SPS   = 0x0000,
    ADS1115_DR_16SPS  = 0x0020,
    ADS1115_DR_32SPS  = 0x0040,
    ADS1115_DR_64SPS  = 0x0060,
    ADS1115_DR_128SPS = 0x0080, // Default
    ADS1115_DR_250SPS = 0x00A0,
    ADS1115_DR_475SPS = 0x00C0,
    ADS1115_DR_860SPS = 0x00E0
} ADS1115_DR;

typedef struct {
    I2C_HandleTypeDef *hi2c;
    volatile uint8_t configComplete;
    volatile uint8_t readComplete;
    int16_t adcValue;
} ADS1115_Handle;
#define ADS1115_COMP_QUE_DISABLE 0x0003 // Disable comparator (default)

/*
#define ADS1115_ADDRESS 0x48
// I2C Addresses (based on ADDR pin connection)
#define ADS1115_ADDR_GND  0x48  // ADDR pin connected to GND
#define ADS1115_ADDR_VDD  0x49  // ADDR pin connected to VDD
#define ADS1115_ADDR_SDA  0x4A  // ADDR pin connected to SDA
#define ADS1115_ADDR_SCL  0x4B  // ADDR pin connected to SCL

// Register Addresses
#define ADS1115_REG_CONVERSION  0x00  // Conversion register
#define ADS1115_REG_CONFIG      0x01  // Configuration register
#define ADS1115_REG_LO_THRESH   0x02  // Low threshold register
#define ADS1115_REG_HI_THRESH   0x03  // High threshold register

// Configuration Register Bit Definitions
// Operational Status (OS)
#define ADS1115_OS_SINGLE       0x8000  // Write: Start single conversion
#define ADS1115_OS_BUSY         0x0000  // Read: Conversion in progress
#define ADS1115_OS_NOTBUSY      0x8000  // Read: Conversion complete

// Input Multiplexer (MUX)
#define ADS1115_MUX_AIN0_AIN1   0x0000  // AIN0 - AIN1 (default)
#define ADS1115_MUX_AIN0_AIN3   0x1000  // AIN0 - AIN3
#define ADS1115_MUX_AIN1_AIN3   0x2000  // AIN1 - AIN3
#define ADS1115_MUX_AIN2_AIN3   0x3000  // AIN2 - AIN3
#define ADS1115_MUX_AIN0_GND    0x4000  // AIN0 - GND
#define ADS1115_MUX_AIN1_GND    0x5000  // AIN1 - GND
#define ADS1115_MUX_AIN2_GND    0x6000  // AIN2 - GND
#define ADS1115_MUX_AIN3_GND    0x7000  // AIN3 - GND

// Programmable Gain Amplifier (PGA)
#define ADS1115_PGA_6_144V      0x0000  // ±6.144V range
#define ADS1115_PGA_4_096V      0x0200  // ±4.096V range
#define ADS1115_PGA_2_048V      0x0400  // ±2.048V range (default)
#define ADS1115_PGA_1_024V      0x0600  // ±1.024V range
#define ADS1115_PGA_0_512V      0x0800  // ±0.512V range
#define ADS1115_PGA_0_256V      0x0A00  // ±0.256V range

// Device Operating Mode (MODE)
#define ADS1115_MODE_CONTIN     0x0000  // Continuous conversion
#define ADS1115_MODE_SINGLE     0x0100  // Single-shot (default)

// Data Rate (DR)
#define ADS1115_DR_8SPS         0x0000  // 8 samples/second
#define ADS1115_DR_16SPS        0x0020  // 16 samples/second
#define ADS1115_DR_32SPS        0x0040  // 32 samples/second
#define ADS1115_DR_64SPS        0x0060  // 64 samples/second
#define ADS1115_DR_128SPS       0x0080  // 128 samples/second (default)
#define ADS1115_DR_250SPS       0x00A0  // 250 samples/second
#define ADS1115_DR_475SPS       0x00C0  // 475 samples/second
#define ADS1115_DR_860SPS       0x00E0  // 860 samples/second

// Comparator Mode (COMP_MODE)
#define ADS1115_COMP_MODE_TRAD  0x0000  // Traditional (default)
#define ADS1115_COMP_MODE_WIND  0x0010  // Window comparator

// Comparator Polarity (COMP_POL)
#define ADS1115_COMP_POL_LOW    0x0000  // Active low (default)
#define ADS1115_COMP_POL_HIGH   0x0008  // Active high

// Latching Comparator (COMP_LAT)
#define ADS1115_COMP_LAT_NON    0x0000  // Non-latching (default)
#define ADS1115_COMP_LAT_LATCH  0x0004  // Latching

// Comparator Queue (COMP_QUE)
#define ADS1115_COMP_QUE_1CONV  0x0000  // Assert after 1 conversion
#define ADS1115_COMP_QUE_2CONV  0x0001  // Assert after 2 conversions
#define ADS1115_COMP_QUE_4CONV  0x0002  // Assert after 4 conversions
#define ADS1115_COMP_QUE_DISABLE 0x0003 // Disable comparator (default)

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t dmaReady;
    float lastVoltage[4]; // Cache for last readings
} ADS1115_HandleTypeDef;


// Register addresses
#define ADS1115_CONVERSION 0x00
#define ADS1115_CONFIG     0x01

// Configuration settings
typedef enum {
    ADS1115_CH0 = 0x4000,
    ADS1115_CH1 = 0x5000,
    ADS1115_CH2 = 0x6000,
    ADS1115_CH3 = 0x7000
} ADS1115_Channel;*/

typedef enum {
    ADS1115_FSR_6144 = 0x0000,
    ADS1115_FSR_4096 = 0x0200,
    ADS1115_FSR_2048 = 0x0400, // Default
    ADS1115_FSR_1024 = 0x0600,
    ADS1115_FSR_0512 = 0x0800,
    ADS1115_FSR_0256 = 0x0A00
} ADS1115_FSR;

void ADS1115_WriteConfig(I2C_HandleTypeDef *hi2c, uint16_t config);
int16_t Read_ADS1115_Channel(uint8_t channel);
int16_t ADS1115_ReadADC_SingleEnded(I2C_HandleTypeDef *hi2c, uint8_t channel, ADS1115_PGA gain);
float ADS1115_ReadVoltage(I2C_HandleTypeDef *hi2c, uint8_t channel, ADS1115_PGA gain);