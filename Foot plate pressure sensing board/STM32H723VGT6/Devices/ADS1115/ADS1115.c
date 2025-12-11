#include "main.h"
#include "ADS1115.h"
//volatile uint8_t dma_complete = 0;
//int16_t adc_results[4]; // Stores results for all 4 channels
extern I2C_HandleTypeDef hi2c1;
extern int count;
int DD=2;

void ADS1115_WriteConfig(I2C_HandleTypeDef *hi2c, uint16_t config)
{
    uint8_t data[3];
    data[0] = ADS1115_REG_CONFIG;
    data[1] = (uint8_t)(config >> 8);   // MSB
    data[2] = (uint8_t)(config & 0xFF); // LSB
    
    HAL_I2C_Master_Transmit(hi2c, ADS1115_ADDR<<1, data, 3, HAL_MAX_DELAY);
}

uint16_t ADS1115_ReadConfig(I2C_HandleTypeDef *hi2c)
{
    uint8_t reg = ADS1115_REG_CONFIG;
    uint8_t data[2];
    
    HAL_I2C_Master_Transmit(hi2c, ADS1115_ADDR<<1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, ADS1115_ADDR<<1, data, 2, HAL_MAX_DELAY);
    return (data[0] << 8) | data[1];
}
int16_t ADS1115_ReadADC_SingleEnded(I2C_HandleTypeDef *hi2c, uint8_t channel, ADS1115_PGA gain)
{
    // Start single conversion
    uint16_t config = ADS1115_MODE_SINGLE;
    
    switch(channel) {
        case 0: config |= ADS1115_MUX_AIN0_GND; break;
        case 1: config |= ADS1115_MUX_AIN1_GND; break;
        case 2: config |= ADS1115_MUX_AIN2_GND; break;
        case 3: config |= ADS1115_MUX_AIN3_GND; break;
        default: return 0;
    }
    
    config |= gain;
    config |= ADS1115_DR_860SPS; // Data rate
    config |= 0x8000; // Start single conversion
    
    ADS1115_WriteConfig(hi2c, config);
    // Read conversion result
    uint8_t reg = ADS1115_REG_CONVERSION;
    uint8_t data[2];
    
    HAL_I2C_Master_Transmit(hi2c, ADS1115_ADDR<<1, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(hi2c, ADS1115_ADDR<<1, data, 2, HAL_MAX_DELAY);
     return (int16_t)((data[0] << 8) | data[1]);
}

float ADS1115_ReadVoltage(I2C_HandleTypeDef *hi2c, uint8_t channel, ADS1115_PGA gain)
{
    int16_t adc_val = ADS1115_ReadADC_SingleEnded(hi2c, channel, gain);
    
    // Calculate voltage based on PGA setting
    float voltage;
    switch(gain) {
        case ADS1115_PGA_6_144V: voltage = adc_val * 6.144f / 32768.0f; break;
        case ADS1115_PGA_4_096V: voltage = adc_val * 4.096f / 32768.0f; break;
        case ADS1115_PGA_2_048V: voltage = adc_val * 2.048f / 32768.0f; break;
        case ADS1115_PGA_1_024V: voltage = adc_val * 1.024f / 32768.0f; break;
        case ADS1115_PGA_0_512V: voltage = adc_val * 0.512f / 32768.0f; break;
        case ADS1115_PGA_0_256V: voltage = adc_val * 0.256f / 32768.0f; break;
        default: voltage = 0.0f;
    }
    
    return voltage;
}

