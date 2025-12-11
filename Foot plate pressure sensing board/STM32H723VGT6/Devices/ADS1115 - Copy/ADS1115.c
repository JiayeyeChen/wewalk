#include "main.h"
#include "ADS1115.h"
volatile uint8_t dma_complete = 0;
int16_t adc_results[4]; // Stores results for all 4 channels
extern I2C_HandleTypeDef hi2c1;


void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    dma_complete = 1;
}

// Configure ADS1115 for a specific channel
void ADS1115_ConfigureChannel(int channel) {
    uint16_t config = ADS1115_OS_SINGLE | 
                     ADS1115_PGA_4_096V |
                     ADS1115_MODE_SINGLE |
                     ADS1115_DR_128SPS |
                     ADS1115_COMP_QUE_DISABLE;
    switch(channel) {
        case 0: config |= 0x4000; break;	//AIN0
        case 1: config |= 0x5000; break; // AIN1
        case 2: config |= 0x6000; break; // AIN2
        case 3: config |= 0x7000; break; // AIN3
    }
    uint8_t config_data[2] = {config >> 8, config & 0xFF};
    HAL_I2C_Mem_Write_DMA(&hi2c1, ADS1115_ADDRESS << 1, ADS1115_REG_CONFIG, 1, config_data, 2);
}
/*			uint8_t channel;
				uint16_t config = ADS1115_CONFIG_OS_SINGLE | 
                     ADS1115_CONFIG_MODE_SINGLE |
                     ADS1115_CONFIG_PGA_4_096V |
                     ADS1115_CONFIG_DR_128SPS |
                     ADS1115_CONFIG_CQUE_NONE;
    
    // Set MUX bits based on channel
    switch(channel) {
        case 0: config |= ADS1115_CONFIG_MUX_SINGLE_0; break;
        case 1: config |= 0x5000; break; // AIN1
        case 2: config |= 0x6000; break; // AIN2
        case 3: config |= 0x7000; break; // AIN3
    }
*/
// Read all 4 channels sequentially using DMA
void ADS1115_ReadAllChannels_DMA(void) {
    uint8_t conversion_reg = ADS1115_REG_CONVERSION;
    
    for (int i = 0; i < 4; i++) {
        // Configure channel
        ADS1115_ConfigureChannel(i);
        HAL_Delay(1); // Small delay for configuration
        
        // Start conversion and wait
        dma_complete = 0;
        HAL_I2C_Mem_Read_DMA(&hi2c1, ADS1115_ADDRESS << 1, conversion_reg, 1, 
                            (uint8_t*)&adc_results[i], 2);
        HAL_Delay(10);
        // Wait for DMA completion (could use interrupt instead)
        while (!dma_complete);
    }
}

// Alternative: Faster method using single configuration
void ADS1115_ReadAllChannels_Fast(void) {
    // Configure for channel 0 first
    ADS1115_ConfigureChannel(0);
    HAL_Delay(1);
    
    // Read all channels in sequence
    for (int i = 0; i < 4; i++) {
        // Start conversion (no need to reconfigure if using continuous mode)
        dma_complete = 0;
        HAL_I2C_Mem_Read_DMA(&hi2c1, ADS1115_ADDRESS << 1, ADS1115_REG_CONVERSION, 1, 
                            (uint8_t*)&adc_results[i], 2);
        while (!dma_complete);
        
        // For next iteration, change MUX setting
        if (i < 3) {
            uint16_t new_config = (ADS1115_CH0 >> ((i+1)*4)) | 
                                (adc_results[0] & 0x8FFF); // Keep other settings
            uint8_t config_data[2] = {new_config >> 8, new_config & 0xFF};
            HAL_I2C_Mem_Write_DMA(&hi2c1, ADS1115_ADDRESS << 1, ADS1115_REG_CONFIG, 1, config_data, 2);
            HAL_Delay(1);
        }
    }
}
// Configure the interrupt pin
void ADS1115_Alert_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    __HAL_RCC_GPIOE_CLK_ENABLE(); // Enable the appropriate GPIO clock
    
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING; // Active low
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    
    HAL_NVIC_SetPriority(EXTI0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

// In your EXTI handler
void EXTI0_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == GPIO_PIN_0) {
        dma_complete = 1; // Signal that conversion is ready
    }
}

void ADS1115_Init(void) {
    for(int i = 0; i < 4; i++) adc_results[i] = 1.0f;
}
/*
// Initialize ADS1115 handler
void ADS1115_Init(ADS1115_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c) {
    hadc->hi2c = hi2c;
    hadc->dmaReady = 1;
    for(int i = 0; i < 4; i++) hadc->lastVoltage[i] = 1.0f;
}*/
/*
// DMA-enabled read function
uint8_t ADS1115_ReadChannel_DMA(ADS1115_HandleTypeDef *hadc, uint8_t channel, ADS1115_FSR fsr) {
    if(!hadc->dmaReady) return 0;
    
    uint16_t config = 0x8000; // Start single conversion
    config |= (channel & 0x03) << 12; // Set channel
    config |= fsr;
    config |= ADS1115_MODE_SINGLESHOT;
    config |= ADS1115_DR_860SPS; // Use fastest rate for DMA
    
    uint8_t data[3] = {
        ADS1115_CONFIG,
        (uint8_t)(config >> 8),
        (uint8_t)(config & 0xFF)
    };
    
    hadc->dmaReady = 0;
    return (HAL_I2C_Master_Transmit_DMA(hadc->hi2c, ADS1115_ADDRESS << 1, data, 3) == HAL_OK);
}

// Call this from I2C Tx Complete callback
void ADS1115_TxCpltCallback(ADS1115_HandleTypeDef *hadc) {
    // After config, set pointer to conversion register
    uint8_t reg = ADS1115_CONVERSION;
    HAL_I2C_Master_Transmit_DMA(hadc->hi2c, ADS1115_ADDRESS << 1, &reg, 1);
}

// Call this from I2C Rx Complete callback
void ADS1115_RxCpltCallback(ADS1115_HandleTypeDef *hadc, uint8_t *rxData) {
    int16_t value = (rxData[0] << 8) | rxData[1];
    uint8_t channel = (hadc->hi2c->Instance->CR2 & I2C_CR2_SADD) >> 12; // Get channel from last config
    
    // Convert to voltage (using proper FSR from last config)
    hadc->lastVoltage[channel] = value * 2.048f / 32768.0f; // Default ±2.048V range
    hadc->dmaReady = 1;
}*/