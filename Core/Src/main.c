/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 *@file           : main.c
 *@brief          : Main program body
 ******************************************************************************
 *@attention
 *
 *Copyright (c) 2023 STMicroelectronics.
 *All rights reserved.
 *
 *This software is licensed under terms that can be found in the LICENSE file
 *in the root directory of this software component.
 *If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can.h"
#include "dac.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
    HAL_UART_Transmit(&huart2, (uint8_t*) &ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#include <stdbool.h>

CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];

int battery_current, battery_voltage, battery_temperature, battery_capacity, battery_id1, battery_id2, cell_number, battery_percent, battery_voltage_cells_total;
uint16_t battcells[12];
uint8_t charge_status;
bool charging_flag = false;

float dac_charge_value = 80;

#define FULLCHARGE 25600
uint8_t battery_timeout = 0;

bool can_tx_error_flag = false;
bool acquired = false;

void enableBattery()
{
    CAN_TxHeaderTypeDef TxHeader;
    uint32_t TxMailbox;

    TxHeader.DLC = 8;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.TransmitGlobalTime = DISABLE;
    TxHeader.StdId = 0x456;
    uint8_t TxData[] = {0xAA, 0x01, 0x4E, 0x69, 0x44, 0x61, 0x59, 0x65};

    if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    {
        can_tx_error_flag = true;
        HAL_CAN_Stop(&hcan1);
        HAL_CAN_ResetError(&hcan1);
       	// HAL_CAN_MspDeInit(&hcan1);
       	// HAL_CAN_DeInit(&hcan1);
       	// HAL_CAN_MspInit(&hcan1);
        MX_CAN1_Init();
    }
    else can_tx_error_flag = false;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == GPIO_PIN_13) {
        battery_id1 = 0;
        battery_id2 = 0;
        battery_current = 0;
        battery_voltage = 0;
        battery_capacity = 0;
        charge_status = 0;
        battery_temperature = 0;
        charging_flag = false;
        memset(battcells, 0, sizeof(battcells));
        enableBattery();
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, RxData);

   	// printf("id: %ld | ", RxHeader.StdId);
   	// printf("data: %X %X %X %X %X %X %X %X\n", RxData[0], RxData[1], RxData[2], RxData[3], RxData[4], RxData[5], RxData[6], RxData[7]);

    if (RxHeader.StdId >= 0x200 && RxHeader.StdId <= 0x210)
    	acquired = true;
}

#include "../../modbus/mb.h"
#include "../../modbus/mbport.h"

enum
{
    HREG_BATTERY_ID1,
	HREG_BATTERY_ID2,
    HREG_BATTERY_CURRENT,
    HREG_BATTERY_VOLTAGE,
    HREG_BATTERY_CAPACITY,
	HREG_BATTERY_PERCENT,
    HREG_CHARGE_STATUS,
	HREG_BATTERY_TEMPERATURE,
    HREG_CELL_VOLTAGE_1,
    HREG_CELL_VOLTAGE_2,
    HREG_CELL_VOLTAGE_3,
    HREG_CELL_VOLTAGE_4,
    HREG_CELL_VOLTAGE_5,
    HREG_CELL_VOLTAGE_6,
    HREG_CELL_VOLTAGE_7,
    HREG_CELL_VOLTAGE_8,
    HREG_CELL_VOLTAGE_9,
    HREG_CELL_VOLTAGE_10,
    HREG_CELL_VOLTAGE_11,
    HREG_CELL_VOLTAGE_12,
    HREG_DOOR_STATUS,
    HREG_DOOR_OPEN,
	HREG_DAC_VALUE,
    HREG_FINAL_ENTRY
} holdingRegs_t;

#define REG_HOLDING_START 1
#define REG_HOLDING_NREGS HREG_FINAL_ENTRY
static USHORT usRegHoldingStart = REG_HOLDING_START;
static USHORT usRegHoldingBuf[REG_HOLDING_NREGS];

uint32_t millis()
{
    return HAL_GetTick();
}

int map(int x, int in_min, int in_max, int out_min, int out_max) {
    if (x < in_min) return out_min;
    if (x > in_max) return out_max;
    return (x - in_min) * (out_max - out_min + 1) / (in_max - in_min + 1) + out_min;
}

void parsing_can_data()
{
    if (RxHeader.StdId == 0x200)
    {
        battery_capacity = (RxData[0] << 8) | (RxData[1]);
        if (battery_capacity < FULLCHARGE) {
        	// charging_flag = true;
        }
        else {
        	// charging_flag = false;
        }
    }
    else if (RxHeader.StdId == 0x201)
    {
        battery_current = (RxData[0] << 8) | (RxData[1]);
        battery_voltage = ((RxData[4] << 8) | (RxData[5]));
    }
    else if (RxHeader.StdId == 0x202)
    {
        cell_number = RxData[0];
        charge_status = RxData[5];
        battcells[cell_number] = ((RxData[1] << 8) | (RxData[2]));
    }
    else if (RxHeader.StdId == 0x209)
    {
        battery_temperature = ((RxData[1] << 8) | (RxData[2]));
    }
    else if (RxHeader.StdId == 0x210)
    {
       	// enableBattery();
    	// battery_id = ((RxData[0] << 24) | (RxData[1] << 16) | (RxData[2] << 8) | (RxData[3]));
        battery_id1 = ((RxData[0] << 8) | (RxData[1]));
    	battery_id2 = ((RxData[2] << 8) | (RxData[3]));
    }
}

int update_dac_ticks;
int update_dac = 0;

void dac_compare() {
    if(millis() - update_dac_ticks >= 100) {
        update_dac_ticks = millis();
        if(update_dac < dac_charge_value) {
            update_dac += 1;
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ((update_dac * 0.01) * 4096.0) / 3.3);
        } else if (update_dac >= dac_charge_value) {
            update_dac = dac_charge_value;
            HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, ((dac_charge_value * 0.01) * 4096.0) / 3.3);
        }
    }
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
  MX_USART2_UART_Init();
  MX_CAN1_Init();
  MX_DAC_Init();
  MX_TIM7_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

   	// // init
   	// printf("%s\n", "can start");

   	// close door
    HAL_GPIO_WritePin(DOOR_LOCK_GPIO_Port, DOOR_LOCK_Pin, 0);

    HAL_DAC_Start(&hdac, DAC_CHANNEL_1);

    HAL_GPIO_WritePin(EN_CHARGE_GPIO_Port, EN_CHARGE_Pin, 0);
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);

    eMBInit(MB_RTU, 11, 3, huart2.Init.BaudRate, MB_PAR_NONE);
    eMBEnable();

    uint32_t ticks, ticks2;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
    while (1)
    {

        if (millis() - ticks2 >= 50) {
            ticks2 = millis();
           	// printf("%d %d %d %d %d %d\n", battery_id, battery_current,  battery_voltage, battery_capacity, charge_status, cell_voltage);

            if (HAL_GPIO_ReadPin(DOOR_SENS_GPIO_Port, DOOR_SENS_Pin) == 1 && battery_percent < 100) {
            	HAL_GPIO_WritePin(EN_CHARGE_GPIO_Port, EN_CHARGE_Pin, 1);
                if (charging_flag) dac_compare();
            } else {
            	HAL_GPIO_WritePin(EN_CHARGE_GPIO_Port, EN_CHARGE_Pin, 0);
            	if (!charging_flag) {
                    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
                    update_dac = 0;
                }
            }

            if (battery_percent >= 100 && battery_timeout < 10) {
                charging_flag = false;
                HAL_GPIO_WritePin(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin, 0);
                HAL_GPIO_WritePin(LED_FULL_GPIO_Port, LED_FULL_Pin, 1);
            } else if (battery_percent > 98 && battery_timeout < 10) {
            	HAL_GPIO_WritePin(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin, 0);
            	HAL_GPIO_WritePin(LED_FULL_GPIO_Port, LED_FULL_Pin, 1);
            } else if (battery_percent <= 98 && battery_timeout < 10) {
                charging_flag = true;
                HAL_GPIO_WritePin(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin, 1);
                HAL_GPIO_WritePin(LED_FULL_GPIO_Port, LED_FULL_Pin, 0);
            } else {
            	update_dac = 0;
                HAL_GPIO_WritePin(LED_CHARGE_GPIO_Port, LED_CHARGE_Pin, 0);
                HAL_GPIO_WritePin(LED_FULL_GPIO_Port, LED_FULL_Pin, 0);
            }

        }

        if (acquired) {
            acquired = false;
            battery_timeout = 0;
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
            parsing_can_data();
            battery_voltage_cells_total = (battcells[0] + battcells[1] + battcells[2] + battcells[3] + battcells[4] + battcells[5] + battcells[6] + battcells[7] + battcells[8] + battcells[9] + battcells[10] + battcells[11]);
            battery_percent = map(battery_voltage_cells_total, 35600, 49600, 0, 100);
        }

        if (millis() - ticks >= (can_tx_error_flag ? 100 : 500)) {
            ticks = millis();
            enableBattery();
            battery_timeout++;
            if (battery_timeout >= 10) {
                battery_timeout = 10;
                battery_id1 = 0;
                battery_id2 = 0;
                battery_current = 0;
                battery_voltage = 0;
                battery_capacity = 0;
                charge_status = 0;
                battery_temperature = 0;
                battery_voltage_cells_total = 0;
                battery_percent = 0;
                charging_flag = false;
                memset(battcells, 0, sizeof(battcells));
            }
        }

        eMBPoll();
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
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
 *@brief Fetching modbus holding register data.
 *
 *@param iRegIndex
 *@return uint8_t
 */
uint8_t fetchHoldingRegsData(int iRegIndex)
{
    uint8_t numUs = 1;
    switch (iRegIndex)
    {
        case HREG_BATTERY_ID1:
            usRegHoldingBuf[HREG_BATTERY_ID1] = battery_id1;
            break;

        case HREG_BATTERY_ID2:
            usRegHoldingBuf[HREG_BATTERY_ID2] = battery_id2;
            break;

        case HREG_BATTERY_CURRENT:
            usRegHoldingBuf[HREG_BATTERY_CURRENT] = battery_current;
            break;

        case HREG_BATTERY_VOLTAGE:
            usRegHoldingBuf[HREG_BATTERY_VOLTAGE] = battery_voltage_cells_total;
            break;

        case HREG_BATTERY_CAPACITY:
            usRegHoldingBuf[HREG_BATTERY_CAPACITY] = battery_capacity;
            break;

        case HREG_BATTERY_PERCENT:
             usRegHoldingBuf[HREG_BATTERY_PERCENT] = battery_percent;
             break;

        case HREG_CHARGE_STATUS:
            usRegHoldingBuf[HREG_CHARGE_STATUS] = charge_status;
            break;

        case HREG_BATTERY_TEMPERATURE:
        	usRegHoldingBuf[HREG_BATTERY_TEMPERATURE] = battery_temperature;
        	break;

        case HREG_CELL_VOLTAGE_1:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_1] = battcells[0];
            break;

        case HREG_CELL_VOLTAGE_2:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_2] = battcells[1];
            break;

        case HREG_CELL_VOLTAGE_3:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_3] = battcells[2];
            break;

        case HREG_CELL_VOLTAGE_4:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_4] = battcells[3];
            break;

        case HREG_CELL_VOLTAGE_5:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_5] = battcells[4];
            break;

        case HREG_CELL_VOLTAGE_6:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_6] = battcells[5];
            break;

        case HREG_CELL_VOLTAGE_7:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_7] = battcells[6];
            break;

        case HREG_CELL_VOLTAGE_8:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_8] = battcells[7];
            break;

        case HREG_CELL_VOLTAGE_9:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_9] = battcells[8];
            break;

        case HREG_CELL_VOLTAGE_10:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_10] = battcells[9];
            break;

        case HREG_CELL_VOLTAGE_11:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_11] = battcells[10];
            break;

        case HREG_CELL_VOLTAGE_12:
            usRegHoldingBuf[HREG_CELL_VOLTAGE_12] = battcells[11];
            break;

        case HREG_DOOR_STATUS:
            usRegHoldingBuf[HREG_DOOR_STATUS] = !HAL_GPIO_ReadPin(DOOR_SENS_GPIO_Port, DOOR_SENS_Pin);
            break;

        case HREG_DOOR_OPEN:
//        	if(HAL_GPIO_ReadPin(DOOR_SENS_GPIO_Port, DOOR_SENS_Pin) == 1)
//        		usRegHoldingBuf[HREG_DOOR_OPEN] = 0;
            break;

        case HREG_DAC_VALUE:
        	usRegHoldingBuf[HREG_DAC_VALUE] = dac_charge_value;
        	break;

        default:
            numUs = 0;
            break;
    }

    return numUs;
}

/**
 *@brief Writing modbus holding registers.
 *
 *@param iRegIndex
 *@param tempReg
 */
void writeHoldingRegs(int iRegIndex, uint16_t tempReg)
{
    switch (iRegIndex)
    {
        case HREG_DOOR_OPEN:
            if (tempReg > 1) break;
            if (tempReg == 1)
            {
                usRegHoldingBuf[HREG_DOOR_OPEN] = 1;
                HAL_GPIO_WritePin(DOOR_LOCK_GPIO_Port, DOOR_LOCK_Pin, 1);
                HAL_Delay(10);
                HAL_GPIO_WritePin(DOOR_LOCK_GPIO_Port, DOOR_LOCK_Pin, 0);
            }
            break;

        case HREG_DAC_VALUE:
        	usRegHoldingBuf[HREG_DAC_VALUE] = tempReg;
        	dac_charge_value = tempReg;
            break;

        default:
           	// usRegHoldingBuf[iRegIndex] = tempReg;
            break;
    }
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs, eMBRegisterMode eMode)
{
    eMBErrorCode eStatus = MB_ENOERR;
    int iRegIndex;

    if (eMode == MB_REG_READ)
    {
        if ((usAddress >= REG_HOLDING_START) &&
            (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
        {
            iRegIndex = (int)(usAddress - usRegHoldingStart);
            while (usNRegs > 0)
            {
                uint8_t numUs = fetchHoldingRegsData(iRegIndex);
                if (numUs < 1)
                {
                    return MB_ENORES;
                }

                for (size_t i = 0; i < numUs; ++i)
                {
                    if (usNRegs > 0)
                    {
                        *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] >> 8);
                        *pucRegBuffer++ = (unsigned char)(usRegHoldingBuf[iRegIndex] &0xFF);
                        iRegIndex++;
                        usNRegs--;
                    }
                    else
                    {
                        return MB_ENORES;
                    }
                }
            }
        }
        else
        {
            eStatus = MB_ENOREG;
        }
    }

    if (eMode == MB_REG_WRITE)
    {
        if ((usAddress >= REG_HOLDING_START) && (usAddress + usNRegs <= REG_HOLDING_START + REG_HOLDING_NREGS))
        {
            iRegIndex = (int)(usAddress - usRegHoldingStart);

            while (usNRegs > 0)
            {
                writeHoldingRegs(iRegIndex, (USHORT)(((unsigned int) *pucRegBuffer << 8) | ((unsigned int) *(pucRegBuffer + 1))));
                pucRegBuffer += 2;
                usNRegs--;
                iRegIndex++;
            }
        }
        else eStatus = MB_ENOREG;
    }

    return eStatus;
}

eMBErrorCode eMBRegInputCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNRegs)
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegCoilsCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNCoils, eMBRegisterMode eMode)
{
    return MB_ENOREG;
}

eMBErrorCode eMBRegDiscreteCB(UCHAR *pucRegBuffer, USHORT usAddress, USHORT usNDiscrete)
{
    return MB_ENOREG;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
    /*User can add his own implementation to report the HAL error return state */
    __disable_irq();
    while (1) {}

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
    /*User can add his own implementation to report the file name and line number,
       ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
