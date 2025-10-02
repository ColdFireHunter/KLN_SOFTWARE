#include <Arduino.h>
#include <global.h>
#include <pins.h>
#include <gpio.h>

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
gpio GPIO;

// thresholds in mV und Zeiten in ms
static const int LINE_FAULT_SET_THRESHOLD_MV = 13000;
static const int LINE_FAULT_CLEAR_THRESHOLD_MV = 18000;
static const uint32_t LINE_FAULT_SET_TIME_MS = 30000UL;  // 30 s
static const uint32_t LINE_FAULT_CLEAR_TIME_MS = 2000UL; // 2 s
void handleLineFault();

// --- Battery cycle thresholds & timings ---
static const int BAT_FAULT_SET_THRESHOLD_MV = 8000;    // < 8V -> Fault
static const int BAT_FAULT_CLEAR_THRESHOLD_MV = 10000; // >10V -> Clear
static const uint32_t BAT_CYCLE_ON_MS = 30000UL;       // 30s charger ON
static const uint32_t BAT_CYCLE_OFF_MS = 5000UL;       // 5s charger OFF (measure at the END)
static const uint32_t BAT_CHARGING_CLEAR_MS = 2000UL;  // 2s "charging" -> Fault reset

extern int battery_voltage; // mV
extern int line_voltage;    // mV

enum BatCycleState : uint8_t
{
  BAT_HOLD_CHARGING = 0, // line present & charger says CHARGING -> keep ON, do nothing
  BAT_CYCLE_ON,          // 30s charger ON
  BAT_CYCLE_OFF          // 5s charger OFF, measure at the end
};

static BatCycleState state = BAT_HOLD_CHARGING;
static uint32_t stateStartMs = 0;
static uint32_t chargingHoldStartMs = 0; // neu: Timer für 2s-Clear im Charging-Hold

bool isLinePresent();     // uses 18V like your line logic
bool isChargerCharging(); // HIGH = charging (as you specified)
void handleBatteryFault();

static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
   */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.EOCSelection = EOC_SEQ_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_79CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
   */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
}

void setup()
{
  GPIO.initialize();

  MX_DMA_Init();
  MX_ADC1_Init();
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_buffer, 3);

  DEBUG_SERIAL.begin(115200);
  DEBUG_SERIAL.println("KLN:");
  DEBUG_SERIAL.print("Software Version: ");
  DEBUG_SERIAL.println(SW_VERSION);
  DEBUG_SERIAL.print("Hardware Version: ");
  DEBUG_SERIAL.println(HW_VERSION);
  DEBUG_SERIAL.print("Software Build Date: ");
  DEBUG_SERIAL.println(SW_DATE);
  DEBUG_SERIAL.print("Software Build Time: ");
  DEBUG_SERIAL.println(SW_TIME);
}
void loop()
{
  GPIO.handler();
  GPIO.setLED(LED_CHGR, GPIO.getChargerStatus(), GPIO.getChargerStatus());
  GPIO.setLED(LED_LINE, GPIO.getMuxStatus(), false);
  GPIO.setLED(LED_BAT, !GPIO.getMuxStatus(), false);

  handleLineFault();
  handleBatteryFault();
}

extern "C" void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};
  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspInit 0 */

    /* USER CODE END ADC1_MspInit 0 */

    /** Initializes the peripherals clocks
     */
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
    PeriphClkInit.AdcClockSelection = RCC_ADCCLKSOURCE_SYSCLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
      Error_Handler();
    }

    /* Peripheral clock enable */
    __HAL_RCC_ADC_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* ADC1 DMA Init */
    /* ADC1 Init */
    hdma_adc1.Instance = DMA1_Channel1;
    hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
    hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
    hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
    hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
    hdma_adc1.Init.Mode = DMA_CIRCULAR;
    hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
    if (HAL_DMA_Init(&hdma_adc1) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

    /* USER CODE BEGIN ADC1_MspInit 1 */

    /* USER CODE END ADC1_MspInit 1 */
  }
}
extern "C" void HAL_ADC_MspDeInit(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    /* USER CODE BEGIN ADC1_MspDeInit 0 */

    /* USER CODE END ADC1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_ADC_CLK_DISABLE();

    /**ADC1 GPIO Configuration
    PA0     ------> ADC1_IN0
    PA1     ------> ADC1_IN1
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0 | GPIO_PIN_1);

    /* ADC1 DMA DeInit */
    HAL_DMA_DeInit(hadc->DMA_Handle);
    /* USER CODE BEGIN ADC1_MspDeInit 1 */

    /* USER CODE END ADC1_MspDeInit 1 */
  }
}
extern "C" void DMA1_Channel1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */

  /* USER CODE END DMA1_Channel1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_adc1);
  /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */

  /* USER CODE END DMA1_Channel1_IRQn 1 */
}

void handleLineFault()
{
  // Merker für aktuellen Fehlerzustand und Timer
  static bool lineFaultActive = false;
  static uint32_t belowStartMs = 0;
  static uint32_t aboveStartMs = 0;

  const uint32_t now = millis();
  const int lv = line_voltage; // mV

  if (!lineFaultActive)
  {
    // Fehler setzen, wenn 30 s lang < 13.0 V
    if (lv < LINE_FAULT_SET_THRESHOLD_MV)
    {
      if (belowStartMs == 0)
        belowStartMs = now;
      if ((uint32_t)(now - belowStartMs) >= LINE_FAULT_SET_TIME_MS)
      {
        lineFaultActive = true;
        belowStartMs = 0;
        GPIO.setLineFault(false);
      }
    }
    else
    {
      // Bedingung nicht mehr erfüllt -> Timer zurücksetzen
      belowStartMs = 0;
    }

    // Während kein Fehler aktiv ist, Rücksetz-Timer nicht verwenden
    aboveStartMs = 0;
  }
  else
  {
    // Fehler zurücksetzen, wenn 2 s lang > 18.0 V
    if (lv > LINE_FAULT_CLEAR_THRESHOLD_MV)
    {
      if (aboveStartMs == 0)
        aboveStartMs = now;
      if ((uint32_t)(now - aboveStartMs) >= LINE_FAULT_CLEAR_TIME_MS)
      {
        lineFaultActive = false;
        aboveStartMs = 0;
        GPIO.setLineFault(true);
      }
    }
    else
    {
      // Bedingung nicht mehr erfüllt -> Timer zurücksetzen
      aboveStartMs = 0;
    }

    // Während Fehler aktiv ist, Setz-Timer nicht verwenden
    belowStartMs = 0;
  }
}

bool isLinePresent()
{
  // Same 18V presence criterion you’ve been using
  return line_voltage >= 18000;
}

bool isChargerCharging()
{
  // IMPORTANT: Your spec says HIGH = charging
  return GPIO.getChargerStatus();
}

void handleBatteryFault()
{
  const uint32_t now = millis();
  const bool lineOK = isLinePresent();
  const bool chgHigh = isChargerCharging();

  // Sofortpfad nur, wenn NICHT in der Messphase
  if (state != BAT_CYCLE_OFF)
  {
    if (lineOK && chgHigh)
    {
      // Charger anlassen
      GPIO.setChargerEnable(true);

      // 2s am Stück "charging" -> Battery Fault reset
      if (chargingHoldStartMs == 0)
        chargingHoldStartMs = now;
      if ((uint32_t)(now - chargingHoldStartMs) >= BAT_CHARGING_CLEAR_MS)
      {
        GPIO.setBatteryFault(false);
      }

      // State halten
      if (state != BAT_HOLD_CHARGING)
      {
        state = BAT_HOLD_CHARGING;
        stateStartMs = now;
      }
      return;
    }
  }

  // Nicht (lineOK && chgHigh) ODER Messphase aktiv -> Charging-Clear-Timer zurücksetzen
  chargingHoldStartMs = 0;

  switch (state)
  {
  case BAT_HOLD_CHARGING:
    state = BAT_CYCLE_ON;
    stateStartMs = now;
    [[fallthrough]];

  case BAT_CYCLE_ON:
    GPIO.setChargerEnable(true);
    if ((uint32_t)(now - stateStartMs) >= 30000UL)
    { // 30s
      state = BAT_CYCLE_OFF;
      stateStartMs = now;
    }
    break;

  case BAT_CYCLE_OFF:
    // Messphase: Charger aus, Charger-Status ignorieren
    GPIO.setChargerEnable(false);
    if ((uint32_t)(now - stateStartMs) >= 5000UL)
    {                                 // 5s
      const int bv = battery_voltage; // mV
      if (bv < 8000)
      {
        GPIO.setBatteryFault(true);
      }
      else if (bv > 10000)
      {
        GPIO.setBatteryFault(false);
      }
      // Zurück zu 30s ON
      state = BAT_CYCLE_ON;
      stateStartMs = now;
    }
    break;
  }
}
