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
#include "dma.h"
#include "fdcan.h"
#include "usart.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include "stdio.h"

#include "bms_cmdlist.h"
#include "bms_datatypes.h"
#include "bms_utility.h"
#include "bms_mcuWrapper.h"
#include "bms_libWrapper.h"

#include "uartDMA.h"
#include "bms_can.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct {
    volatile uint32_t runtime_sec;
    volatile uint32_t counter_commsError;
    volatile uint32_t counter_commsErrorCumulative;
} ProgramStats;

typedef struct {
    uint32_t freq;
    uint32_t period;
    float duty;
} ImdStatus;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

ProgramStats    programStats = {0};
ImdStatus       imdStatus = {0};

volatile bool initRequired = true;

const uint32_t MAIN_LOOP_DELAY = 500;

static const bool DEBUG_SERIAL_LOOP_TIME = true;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void BMS_WriteFanDuty(float duty);
void BMS_WriteFaultSignal(bool state);
void BMS_FaultHandler(BMS_StatusTypeDef status);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


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
  MX_DMA_Init();
  MX_LPUART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM16_Init();
  MX_FDCAN1_Init();
  MX_FDCAN2_Init();
  MX_TIM15_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


    // Set CS2 Pin to HIGH to disable second SPI on 6822 + MSTR should be high by default
    HAL_GPIO_WritePin(BMS_CS2_GPIO_Port, BMS_CS2_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMS_MSTR_GPIO_Port, BMS_MSTR_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(BMS_MSTR2_GPIO_Port, BMS_MSTR2_Pin, GPIO_PIN_SET);

    // Fault By Default
    BMS_WriteFaultSignal(true);

    // Configure FDCAN
    BMS_CAN_Config();

    // Start Timer16
    HAL_TIM_Base_Start_IT(&htim16);
    HAL_TIM_IC_Start_IT(&htim15, TIM_CHANNEL_1); // Signal Input Channel (Main)
    HAL_TIM_IC_Start(&htim15, TIM_CHANNEL_2);    // Secondary Channel

    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);       // Fan PWM Output
    BMS_WriteFanDuty(0.7);

    uint32_t timeDiff = 0;
    uint32_t timeStart;

    printfDma("\n\n +++++             PROGRAM START             +++++ \n\n");
    bms_softReset();
    HAL_Delay(500);         // Initialisation delay

    bms_wakeupChain();
    bms_softReset();


    BMS_StatusTypeDef bmsStatus;
    while (1)
    {
        timeStart = HAL_GetTick();

        // Init BMS
        if (initRequired == true)
        {
            bmsStatus = bms_init();
            if (bms_init() == BMS_OK)
            {
                initRequired = false;
            }
            else
            {
                BMS_FaultHandler(bmsStatus);
                continue;
            }
        }

        // BMS Program Loop
        bmsStatus = BMS_ProgramLoop();
        if (bmsStatus != BMS_OK)
        {
            BMS_FaultHandler(bmsStatus);
        }

        printfDma("-\n");

        // Calculate single loop runtime
        timeDiff = HAL_GetTick() - timeStart;
        if (DEBUG_SERIAL_LOOP_TIME)
        {
            printfDma("\nRuntime: %ld ms, LoopTime: %ld ms \n\n", HAL_GetTick(), timeDiff);
        }

        HAL_Delay(MAIN_LOOP_DELAY);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV3;
  RCC_OscInitStruct.PLL.PLLN = 32;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/// Timer interrupt callback
// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    uint32_t currentTime = HAL_GetTick(); // Get current time

    // Check which version of the timer triggered this callback and toggle LED
    static uint32_t secondsDiv = 0;

    // CAN Transmit Periodic Timing Variable
    static uint32_t canTiming = 0;

    // Voltage fault timer variables
    const uint32_t VOLTAGE_FAULT_PERIOD = 500 - 10;
    static uint32_t lastVoltageFaultTime = 0;
    static bool voltageFaultDetected = false;

    // Temperature fault Time
    const uint32_t TEMP_FAULT_PERIOD = 1000 - 10;
    static uint32_t lastTempFaultTime = 0;
    static bool tempFaultDetected = false;

    if (htim == &htim16) // This is triggered 10x every second
    {
        // runtime (seconds) timer
        if (++secondsDiv >= 10)
        {
            programStats.runtime_sec += 1;
            secondsDiv = 0;
        }

        // CAN Timing Divider
        if (++canTiming >= 5)
        {
            CanTxMsg *msgArr = NULL;
            uint32_t len = 0;
            BMS_GetCanData(&msgArr, &len);
            BMS_CAN_SendBuffer(msgArr, len);
            canTiming = 0;
        }

        // voltage fault handling
        if (BMS_CheckVoltage() == BMS_OK)       // If OK:
        {
            voltageFaultDetected = false;
        }
        else                                    // If NOT OK
        {
            if (!voltageFaultDetected)          // First time NOT OK
            {
                voltageFaultDetected = true;
                lastVoltageFaultTime = currentTime;
            }
            else if (currentTime - lastVoltageFaultTime > VOLTAGE_FAULT_PERIOD)     // NOT OK for given period
            {
                BMS_WriteFaultSignal(true);
            }
        }

        // temperature fault handling
        if (BMS_CheckTemps() == BMS_OK)     // If OK:
        {
            tempFaultDetected = false;
        }
        else                                // If NOT OK
        {
            if (!tempFaultDetected)         // First time NOT OK
            {
                tempFaultDetected = true;
                lastTempFaultTime = currentTime;
            }
            else if (currentTime - lastTempFaultTime > TEMP_FAULT_PERIOD)     // NOT OK for given period
            {
                BMS_WriteFaultSignal(true);
            }
        };
        //    const float MIN_TEMP = 0;

        // Disable fault signal only if everything is OK
        if (!tempFaultDetected && !voltageFaultDetected && (BMS_CheckCommsFault() == BMS_OK))
        {
            BMS_WriteFaultSignal(false);
        }

        // If charging and SDC is disconnected (LOW):
        if (BMS_IsCharging() && !HAL_GPIO_ReadPin(SDC_IN_GPIO_Port, SDC_IN_Pin))
        {
            printfDma("SDC Disconnected, Disabling Charging \n");
            BMS_EnableCharging(false);
        }
    }
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim == &htim15)   // PWM input
    {
        const uint32_t clockFreq = 128 * 1000 * 1000; // in Hz

        imdStatus.period = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        if (imdStatus.period != 0)
        {
            imdStatus.duty = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2)*100.0) / imdStatus.period;
            imdStatus.freq = (clockFreq/imdStatus.period);
        }
    }
}


void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t currentTime = HAL_GetTick();

    const uint32_t DEBOUNCE_DELAY = 500;
    static uint32_t lastDebounceTime_B1 = 0;
    static uint32_t lastDebounceTime_CHRGR_BTTN = 0;

    switch (GPIO_Pin)
    {
    case B1_Pin:                // Blue onboard button (for debugging mainly)
        // Debounce check
        if (currentTime - lastDebounceTime_B1 < DEBOUNCE_DELAY) break;
        lastDebounceTime_B1 = currentTime;
        printfDma("Blue Button pressed\n");
        break;

    case CHRGR_BTTN_Pin:        // Charger Button
        // Debounce check
        if (currentTime - lastDebounceTime_CHRGR_BTTN < DEBOUNCE_DELAY) break;
        lastDebounceTime_CHRGR_BTTN = currentTime;

        printfDma("Charger Button pressed\n");
        BMS_ChargingButtonLogic();
        break;

    default:
        break;
    }
}


void BMS_FaultHandler(BMS_StatusTypeDef status)
{
    const uint32_t COMMS_RETRY_DELAY = 250;
    const uint32_t COMMS_RETRY_TIMES = 2;

    // Disable discharge in any case
    bms_wakeupChain();
    bms_stopDischarge();

    // if currently charging:
    if (BMS_IsCharging())
    {
        printfDma("Fault Detected, Disabling Charging \n");
        BMS_EnableCharging(false);
    }

    switch (status)
    {
    case BMS_ERR_COMMS:
        BMS_SetCommsFault(true);
        programStats.counter_commsError = 0;
        do
        {
            programStats.counter_commsError++;
            programStats.counter_commsErrorCumulative++;
            if (programStats.counter_commsError >= COMMS_RETRY_TIMES)
            {
                BMS_WriteFaultSignal(true);
            }
            HAL_Delay(COMMS_RETRY_DELAY);
            bms_softReset();
            bms_wakeupChain();
        }
        while (bms_readRegister(REG_SID) != BMS_OK);

        initRequired = true; // After comms is OK, redo init
        break;
    default:
        break;
    }
}

void BMS_WriteFanDuty(float duty)
{
    if (duty > 1 || duty <= 0)
        return;

    duty = 1 - duty;

    TIM3->CCR2 = (uint16_t)(255.0 * duty);
}


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
