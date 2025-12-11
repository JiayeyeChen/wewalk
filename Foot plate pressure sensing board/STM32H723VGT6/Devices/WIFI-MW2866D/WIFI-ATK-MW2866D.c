#include "main.h"
#include "stm32h7xx_hal.h"
#include "stdbool.h"
#define WIFI_DMA_BUFFER_SIZE 2048
#define WIFI_RX_BUFFER_SIZE 1024
uint8_t wifi_rx_buffer[WIFI_RX_BUFFER_SIZE];
uint8_t wifi_dma_buffer[WIFI_DMA_BUFFER_SIZE];
volatile uint16_t wifi_dma_index = 0;
volatile uint8_t wifi_response_ready = 0;
extern UART_HandleTypeDef huart9;

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
//    if(huart == &huart9) {
//        wifi_response_ready = 1;
//    }
//}

void WIFI_StartDMAReception(UART_HandleTypeDef *huart) {
    wifi_dma_index = 0;
    wifi_response_ready = 0;
    HAL_UART_Receive_DMA(huart, wifi_dma_buffer, WIFI_DMA_BUFFER_SIZE);
}

// Non-blocking command sending
bool WIFI_SendCommandNB(UART_HandleTypeDef *huart, const char *cmd) {
    WIFI_StartDMAReception(huart);
    return HAL_UART_Transmit(huart, (uint8_t*)cmd, strlen(cmd), HAL_MAX_DELAY) == HAL_OK;
}

// Check for response in main loop
int16_t WIFI_CheckResponse(void) {
    if(!wifi_response_ready) return 0;
    
    // Process received data
    if(strstr((char*)wifi_dma_buffer, "OK") != NULL) {
        return wifi_dma_index;
    }
    if(strstr((char*)wifi_dma_buffer, "ERROR") != NULL) {
        return -1;
    }
    
    return -2; // No OK or ERROR found
}
bool WIFI_StartTCPConnection(UART_HandleTypeDef *huart, const char *ip, uint16_t port) {
    char cmd[64];
		int16_t rr;
    snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%d\r\n", ip, port);
		rr=WIFI_SendATCommand(huart,cmd, 5000);
    return (rr> 0);
}

bool WIFI_SendData(UART_HandleTypeDef *huart, const char *data) {
    char cmd[32];
    uint16_t length = strlen(data);
    
    // Set send length
    snprintf(cmd, sizeof(cmd), "AT+CIPSEND=%d\r\n", length);
    if(WIFI_SendATCommand(huart,cmd, 1000) <= 0) {
        return false;
    }
    
    // Send actual data
    return (WIFI_SendCommand(huart, data, 1000) == HAL_OK);
}
bool WIFI_Connect(UART_HandleTypeDef *huart, const char *ssid, const char *password) {
    char cmd[128];
    
    // Build connection command
    snprintf(cmd, sizeof(cmd), "AT+CWJAP=\"%s\",\"%s\"\r\n", ssid, password);
    
    // Send connection command (may take longer to connect)
    int16_t result = WIFI_SendATCommand(huart, cmd, 10000);
    
    return (result > 0);
}
void WIFI_Init(UART_HandleTypeDef *huart) {
    // Reset module (optional)
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
    HAL_Delay(100);
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_Delay(2000); // Wait for module to start
    
    // Test communication
    int16_t result = WIFI_SendATCommand(huart, "AT\r\n", 1000);
    if(result <= 0) {
        Error_Handler(); // Communication failed
    }
    
    // Disable echo
    WIFI_SendATCommand(huart, "ATE0\r\n", 1000);
    
    // Set station mode
    WIFI_SendATCommand(huart, "AT+CWMODE=1\r\n", 2000);
}

// Send AT command and wait for response
HAL_StatusTypeDef WIFI_SendCommand(UART_HandleTypeDef *huart, const char *cmd, uint32_t timeout) {
    return HAL_UART_Transmit(huart, (uint8_t*)cmd, strlen(cmd), timeout);
}

// Receive response with timeout
int16_t WIFI_ReceiveResponse(UART_HandleTypeDef *huart, uint32_t timeout) {
    HAL_StatusTypeDef status;
    uint32_t start = HAL_GetTick();
    uint16_t index = 0;
    
    memset(wifi_rx_buffer, 0, WIFI_RX_BUFFER_SIZE);
    
    while((HAL_GetTick() - start) < timeout) {
        uint8_t byte;
        status = HAL_UART_Receive(huart, &byte, 1, 10);
        
        if(status == HAL_OK) {
            wifi_rx_buffer[index++] = byte;
            
            // Check for overflow
            if(index >= WIFI_RX_BUFFER_SIZE - 1) {
                break;
            }
            
            // Check for OK response
            if(strstr((char*)wifi_rx_buffer, "OK") != NULL) {
                return index;
            }
            
            // Check for ERROR response
            if(strstr((char*)wifi_rx_buffer, "ERROR") != NULL) {
                return -1;
            }
        }
    }
    
    return index > 0 ? index : -2; // -2 for timeout
}

// Combined send and receive
int16_t WIFI_SendATCommand(UART_HandleTypeDef *huart, const char *cmd, uint32_t timeout) {
    if(WIFI_SendCommand(huart, cmd, timeout) != HAL_OK) {
        return -3; // Transmission error
    }
    return WIFI_ReceiveResponse(huart, timeout);
}
//New below,need change to uart9
void Send_AT_Command(char *cmd, uint32_t timeout)
{
  HAL_UART_Transmit(&huart9, (uint8_t *)cmd, strlen(cmd), timeout);
  HAL_UART_Transmit(&huart9, (uint8_t *)"\r\n", 2, timeout);
}

void Receive_Response(uint8_t *buffer, uint32_t size, uint32_t timeout)
{
  HAL_UART_Receive(&huart9, buffer, size, timeout);
}

void WiFi_Init()
{
  uint8_t response[128];
  
  // 复位模块
  Send_AT_Command("AT+RST", 1000);
  HAL_Delay(2000);
  
  // 设置WiFi模式为Station
  Send_AT_Command("AT+CWMODE=1", 1000);
  Receive_Response(response, sizeof(response), 1000);
  
  // 连接到WiFi网络
  char connect_cmd[128];
  sprintf(connect_cmd, "AT+CWJAP=\"%s\",\"%s\"", "your_SSID", "your_password");
  Send_AT_Command(connect_cmd, 5000);
  Receive_Response(response, sizeof(response), 5000);
  
  // 启用多连接
  Send_AT_Command("AT+CIPMUX=1", 1000);
  Receive_Response(response, sizeof(response), 1000);
}

void HTTP_GET_Request(const char *host, const char *path)
{
  uint8_t response[256];
  
  // 建立TCP连接
  char tcp_cmd[128];
  sprintf(tcp_cmd, "AT+CIPSTART=0,\"TCP\",\"%s\",80", host);
  Send_AT_Command(tcp_cmd, 3000);
  Receive_Response(response, sizeof(response), 3000);
  
  // 准备HTTP请求
  char http_request[256];
  sprintf(http_request, "GET %s HTTP/1.1\r\nHost: %s\r\n\r\n", path, host);
  
  // 发送数据长度
  char send_cmd[32];
  sprintf(send_cmd, "AT+CIPSEND=0,%d", strlen(http_request));
  Send_AT_Command(send_cmd, 1000);
  Receive_Response(response, sizeof(response), 1000);
  
  // 发送HTTP请求
  Send_AT_Command(http_request, 1000);
  Receive_Response(response, sizeof(response), 5000);
  
  // 关闭连接
  Send_AT_Command("AT+CIPCLOSE=0", 1000);
  Receive_Response(response, sizeof(response), 1000);
}

uint8_t rx_buffer[256];
uint32_t rx_index = 0;
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3)
  {
    rx_index++;
    HAL_UART_Receive_IT(&huart9, &rx_buffer[rx_index], 1);
  }
}*/

void Enable_UART_Interrupt()
{
  HAL_UART_Receive_IT(&huart9, &rx_buffer[0], 1);
}

void Process_Received_Data()
{
  if(strstr((char *)rx_buffer, "+IPD"))
  {
    // 处理接收到的网络数据
  }
  memset(rx_buffer, 0, sizeof(rx_buffer));
  rx_index = 0;
}
