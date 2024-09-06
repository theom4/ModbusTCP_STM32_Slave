/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "string.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "mongoose.h"
#include "mongoose_config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define UNIT_ID 0x01
#define COIL_OUT_BLOCK_SIZE 128
#define COIL_IN_BLOCK_SIZE 128
#define REG_IN_BLOCK_SIZE 100
#define REG_OUT_BLOCK_SIZE 100
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x2004c000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x2004c0a0
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x2004c000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x2004c0a0))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */

ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */
#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
struct mg_connection* conn = NULL;
bool connected = false;
uint8_t modbus_coil_out_mem[COIL_OUT_BLOCK_SIZE];
const uint8_t modbus_coil_in_mem[COIL_IN_BLOCK_SIZE] =
{
  0x49,
  0xF1,
  0xAA,
  0x96,
  0xB0,
  0x72,
  0xC9,
  0x15,
  0x47,
  0x01
};
const uint16_t modbus_reg_in_mem[REG_IN_BLOCK_SIZE] =
{
  0xFFDC,
  0x3F1D,
  0xC840,
  0x59E6,
  0x8B0F,
  0xF723,
  0x2CA9,
  0xD15E,
  0x6478,
  0x90F1
};
uint16_t modbus_reg_out_mem[REG_OUT_BLOCK_SIZE] = 
{
  0xA3F2,
  0x5B7C,
  0xC1D4,
  0x8E09,
  0xF4A6,
  0x23E1,
  0x9B7F,
  0x4D92,
  0x6AC8,
  0xEF3B
};

typedef enum 
{
  ILLEGAL_FUNCTION = 0x01,
  ILLEGAL_DATA_ADDRESS = 0x02,
  ILLEGAL_DATA_VALUE = 0x03,
  SLAVE_DEVICE_FAILURE = 0x04,
  SLAVE_DEVICE_BUSY  = 0x06
} ModbusExceptionCode;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void modbus_exception(ModbusExceptionCode error_code)
{
  MG_ERROR(("MODBUS EXCEPTION : %d", error_code));
}
int _write(int file, char *ptr, int len)
{
  (void)file;
  HAL_UART_Transmit(&huart3, (uint8_t*)ptr, len, 1000);
  return len;
}
uint64_t mg_millis(void)
{
  return HAL_GetTick();
}
bool glue_modbus_read_coil(uint8_t func, uint16_t start, uint8_t bit_position, bool *coil_state)
{
  if(func == 1)//coil
  {
      if(start > COIL_OUT_BLOCK_SIZE)
      {
          //MG_ERROR(("INVALID COIL ADDRESS"));
          modbus_exception(ILLEGAL_DATA_ADDRESS);
          return 0;
      }
      *coil_state = (bool)((modbus_coil_in_mem[start] >> bit_position) & 0x01);   
      return 1;
  }
  else if(func == 2)//discrete input
  {
      if(start > COIL_IN_BLOCK_SIZE)
      {
          
          modbus_exception(ILLEGAL_DATA_ADDRESS);
          return 0;
      }
     *coil_state = (bool)((modbus_coil_in_mem[start] >> bit_position) & 0x01);
      return 1;
  }
  else return 0;
}
bool glue_modbus_write_coil(uint16_t start, uint8_t bit_position, bool coil_state)
{
  if(start > COIL_OUT_BLOCK_SIZE)
  {
    MG_ERROR(("INVALID COIL ADDRESS"));
    return 0;
  }
  if(coil_state == 1)
  {
       modbus_coil_out_mem[start] |= (1 << bit_position);
  }
  else
  {
       modbus_coil_out_mem[start] &= ~(1 << bit_position);
  }
  return 1;
}
bool glue_modbus_write_reg(uint16_t start, uint16_t value)
{
    if(start > REG_OUT_BLOCK_SIZE)
    {
      MG_ERROR(("INVALID REG ADDRESS"));
      //modbus_exception(ILLEGAL_DATA_ADDRESS);
      return 0;
    }
    modbus_reg_out_mem[start] = value;
    return 1;

 
}
bool glue_modbus_read_reg(uint8_t func, uint16_t start, uint16_t* value)
{
  
  switch(func)
  {
    case 3://read holding register
      if(start > REG_IN_BLOCK_SIZE )
      {
      modbus_exception(ILLEGAL_DATA_ADDRESS);
      return 0;
      }
      * value = modbus_reg_out_mem[start];
      return 1;
    case 4://read input register
      if(start > REG_OUT_BLOCK_SIZE )
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        return 0;
      }
     * value = modbus_reg_in_mem[start];
      return 1;
    default:
     modbus_exception(ILLEGAL_FUNCTION);
     return 0;
    
  }
}
static void handle_modbus_pdu(struct mg_connection *c, uint8_t *buf,
                              size_t len) {
  MG_DEBUG(("Received PDU %p len %lu, hexdump:", buf, len));
  mg_hexdump(buf, len);
  // size_t hdr_size = 8, max_data_size = sizeof(response) - hdr_size;
  if (len < 12) {
    MG_ERROR(("PDU too small"));
  } else {
    uint8_t func = buf[7];  // Function
    bool success = false;
    size_t response_len = 0;
    uint8_t response[260] = {0};
    memcpy(response, buf, 8);
    uint16_t tid = mg_ntohs(*(uint16_t *) &buf[0]);  // Transaction ID
    uint16_t pid = mg_ntohs(*(uint16_t *) &buf[0]);  // Protocol ID
    uint16_t len = mg_ntohs(*(uint16_t *) &buf[4]);  // PDU length
    uint8_t uid = buf[6];                            // Unit identifier
    if (func == 6) {  // write single holding register
      uint16_t start = mg_ntohs(*(uint16_t *) &buf[8]);
      uint16_t value = mg_ntohs(*(uint16_t *) &buf[10]);
      success = glue_modbus_write_reg(start, value);
      if(success == false)
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        response[8] = ILLEGAL_DATA_ADDRESS;
        goto error_check;

      }
      *(uint16_t *) &response[8] = mg_htons(start);
      *(uint16_t *) &response[10] = mg_htons(value);
      response_len = 12;
      MG_DEBUG(("Glue returned %s", success ? "success" : "failure"));
    } else if (func == 16) {  // Write multiple
      uint16_t start = mg_ntohs(*(uint16_t *) &buf[8]);
      uint16_t num = mg_ntohs(*(uint16_t *) &buf[10]);
      if(num > 123)
      {
        modbus_exception(ILLEGAL_DATA_VALUE);
        success = false;
        goto error_check;
      }
      uint16_t i, *data = (uint16_t *) &buf[13];
      if ((size_t) (num * 2 + 10) < sizeof(response)) {
        for (i = 0; i < num; i++) {
          success =
              glue_modbus_write_reg((uint16_t) (start + i), mg_htons(data[i]));
          if (success == false)
          {
             modbus_exception(ILLEGAL_DATA_ADDRESS);
             response[8] = ILLEGAL_DATA_ADDRESS;
             goto error_check;
          }
        }
        *(uint16_t *) &response[8] = mg_htons(start);
        *(uint16_t *) &response[10] = mg_htons(num);
        response_len = 12;
        MG_DEBUG(("Glue returned %s", success ? "success" : "failure"));
      }
    } else if (func == 3 || func == 4) {  // Read multiple
      uint16_t start = mg_ntohs(*(uint16_t *) &buf[8]);
      uint16_t num = mg_ntohs(*(uint16_t *) &buf[10]);
      if ((size_t) (num * 2 + 9) < sizeof(response)) {
        uint16_t i, val, *data = (uint16_t *) &response[9];
        for (i = 0; i < num; i++) {
          success = glue_modbus_read_reg(func,(uint16_t) (start + i), &val);
          if (success == false) break;
          data[i] = mg_htons(val);
        }
        response[8] = (uint8_t) (num * 2);
        response_len = 9 + response[8];
        MG_DEBUG(("Glue returned %s", success ? "success" : "failure"));
      }
    }
    else if(func == 5)//write single coil
    {
      response_len = 12;
      success = true;
      response[8] = buf[8];//copy the address bytes in the response buffer
      response[9] = buf[9];
      response[10] = buf[10];
      response[11] = buf[11];
     uint16_t start_byte = (uint16_t)(buf[8] << 8 |  buf[9]);
      if(start_byte > COIL_OUT_BLOCK_SIZE)
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        response[8] = ILLEGAL_DATA_ADDRESS;
        success = false;
        goto error_check;
      }

      uint16_t byte_loc = start_byte / 8;
      uint8_t bit_pos = start_byte % 8;
      if(buf[10] == 0xFF && buf[11] == 0x00)
      {
          success &= glue_modbus_write_coil(byte_loc, bit_pos, true);
      }
      else if(buf[10] == 0x00 && buf[11] == 0x00)
      {
          success &= glue_modbus_write_coil(byte_loc, bit_pos, false);
      }
      else 
      {
         MG_INFO(("Invalid coil value, command ignored"));
        
      }
    }
    else if(func == 15)//write multiple coils
    {
      success = true;
      uint16_t start_byte = (uint16_t)(buf[8] << 8 | buf[9]);
      
      uint16_t byte_loc = start_byte / 8;
      uint8_t bit_pos = start_byte % 8;
      uint16_t coils_count = (uint16_t)((buf[10] << 8) | buf[11]);
      uint16_t buffer_start_index = 13;//coils value starts at index 13
      uint8_t shift_index = 0;//for comparing with the buffer;
      response[8] = buf[8];//copy the address bytes in the response buffer
      response[9] = buf[9];
      response[10] = buf[10];
      response[11] = buf[11];
      response_len = 12;
      if((start_byte  + coils_count / 8)> COIL_OUT_BLOCK_SIZE)
      {
        modbus_exception(ILLEGAL_DATA_ADDRESS);
        success = false;
         response[8] = ILLEGAL_DATA_ADDRESS;
        goto error_check;
      }
      if(coils_count > 1968)
      {
        success = false;
        modbus_exception(ILLEGAL_DATA_VALUE);
        response[8] = ILLEGAL_DATA_ADDRESS;
        goto error_check;
      }
      for(uint16_t i = 0; i < coils_count; i++)
      {
        if(((buf[buffer_start_index] >> shift_index) & 0x01) == 1)
        {
            success &= glue_modbus_write_coil(byte_loc, bit_pos, true);
        }
        else 
        {
            success &= glue_modbus_write_coil(byte_loc, bit_pos, false);
        }
        bit_pos ++;
        shift_index++;
        if(bit_pos == 8)
        {
          bit_pos = 0;
          byte_loc++;
        }
        if(shift_index == 8)
        {
          shift_index = 0;
          buffer_start_index++;
        }
      }
     
    }
    else if(func == 1 || func == 2)//read coil or input
    {
        success = true;
        uint16_t coil_addr  = (uint16_t)((buf[8] << 8) | buf[9]);//
        uint8_t bit_pos = coil_addr % 8;
        uint16_t byte_num = coil_addr / 8;
        bool coil_bit_value = false;
        uint16_t coils_count = (uint16_t)((buf[10] << 8) | buf[11]);//
        if(coils_count > 2000)
        {
          modbus_exception(ILLEGAL_DATA_VALUE);
          success = false;
          response[8] = ILLEGAL_DATA_VALUE;
          goto error_check;
        }

        // uint16_t buffer_start_index = 13;
        uint8_t response_bit_index = 0 ;
        uint16_t resp_index = 9;
        response[8] = (coils_count / 8) + (bool)((coils_count % 8) > 0);
        if(response[8] > COIL_IN_BLOCK_SIZE)
        {
          modbus_exception(ILLEGAL_DATA_VALUE);
          success = false;
          response[8] = ILLEGAL_DATA_VALUE;
          goto error_check;
        }
        for(uint16_t i = 0; i < coils_count; i++)
        {
          if(glue_modbus_read_coil(func,byte_num, bit_pos, &coil_bit_value) == true)
          {
            if(coil_bit_value == true)
            {
              response[resp_index] |= (1 << response_bit_index);
            }
          }
          else
          {
            success = false;
            modbus_exception(ILLEGAL_DATA_VALUE);
            response[8] = ILLEGAL_DATA_VALUE;
            goto error_check;
          }
          bit_pos ++;
          response_bit_index++;
          if(bit_pos == 8)
          {
            bit_pos = 0;
            byte_num++;
          }
          if(response_bit_index == 8)
          {
            response_bit_index = 0;
            resp_index++;
          }
        }
         response_len = 9 +resp_index;
    }
error_check : 
    if (success == false) {
      response_len = 9;
      response[7] |= 0x80;
      //response[8] = 4;  // Server Device Failure
    }
    *(uint16_t *) &response[4] = mg_htons((uint16_t) (response_len - 6));
    MG_DEBUG(("Sending PDU response %lu:", response_len));
    mg_hexdump(response, response_len);
    mg_send(c, response, response_len);
  }
}

 void modbus_ev_handler(struct mg_connection *c, int ev, void *ev_data) {
  // if (ev == MG_EV_OPEN) c->is_hexdumping = 1;
  if (ev == MG_EV_READ) {
    uint16_t len;
    if (c->recv.len < 7) return;  // Less than minimum length, buffer more
    len = mg_ntohs(*(uint16_t *) &c->recv.buf[4]);  // PDU length
    MG_INFO(("Got %lu, expecting %lu", c->recv.len, len + 6));
    if (c->recv.len < len + 6U) return;          // Partial frame, buffer more
    if(c->recv.buf[6] != UNIT_ID)
    {
      MG_INFO(("UNIT ID NOT MATCHING"));
      return;
    }
    handle_modbus_pdu(c, c->recv.buf, len + 6);  // Parse PDU and call user
    mg_iobuf_del(&c->recv, 0, len + 6U);         // Delete received PDU
  }
  (void) ev_data;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ETH_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  struct mg_mgr mgr;
  mg_mgr_init(&mgr);
  mg_log_set(MG_LL_DEBUG);
  mg_listen(&mgr, "tcp://0.0.0.0:502", modbus_ev_handler, NULL);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    mg_mgr_poll(&mgr, 100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ETH Initialization Function
  * @param None
  * @retval None
  */
static void MX_ETH_Init(void)
{

  /* USER CODE BEGIN ETH_Init 0 */

  /* USER CODE END ETH_Init 0 */

   static uint8_t MACAddr[6];

  /* USER CODE BEGIN ETH_Init 1 */

  /* USER CODE END ETH_Init 1 */
  heth.Instance = ETH;
  MACAddr[0] = 0x00;
  MACAddr[1] = 0x80;
  MACAddr[2] = 0xE1;
  MACAddr[3] = 0x00;
  MACAddr[4] = 0x00;
  MACAddr[5] = 0x00;
  heth.Init.MACAddr = &MACAddr[0];
  heth.Init.MediaInterface = HAL_ETH_RMII_MODE;
  heth.Init.TxDesc = DMATxDscrTab;
  heth.Init.RxDesc = DMARxDscrTab;
  heth.Init.RxBuffLen = 1524;

  /* USER CODE BEGIN MACADDRESS */

  /* USER CODE END MACADDRESS */

  if (HAL_ETH_Init(&heth) != HAL_OK)
  {
    Error_Handler();
  }

  memset(&TxConfig, 0 , sizeof(ETH_TxPacketConfig));
  TxConfig.Attributes = ETH_TX_PACKETS_FEATURES_CSUM | ETH_TX_PACKETS_FEATURES_CRCPAD;
  TxConfig.ChecksumCtrl = ETH_CHECKSUM_IPHDR_PAYLOAD_INSERT_PHDR_CALC;
  TxConfig.CRCPadCtrl = ETH_CRC_PAD_INSERT;
  /* USER CODE BEGIN ETH_Init 2 */

  /* USER CODE END ETH_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
