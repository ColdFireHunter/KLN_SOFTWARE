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

// STM32G0: Unique ID base address (96-bit UID)
#define UID_BASE (0x1FFF7590UL) // Unique device ID register base

#define BATTERY_VOLTAGE_THRESHOLD 10.0f  // Voltage threshold in volts
#define VOLTAGE_MEASUREMENT_PERIOD 60000 // Measure every 60 seconds
#define VOLTAGE_DELAY_PERIOD 5000        // 5 seconds in milliseconds
#define STARTUP_DELAY 5000               // 5 seconds startup delay

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
gpio GPIO;

bool faultLatched = false;                    // Flag for latched fault state
bool startupComplete = false;                 // Flag for indicating startup delay completion
bool voltageMeasurementInProgress = false;    // To track if the voltage measurement is being processed
unsigned long startupDelayStartTime = 0;      // Store the start time of the delay
unsigned long lastVoltageMeasurementTime = 0; // Last voltage measurement time
unsigned long lastFaultCheckTime = 0;         // Last fault check time
unsigned long voltageDelayStartTime = 0;      // To track when the 5-second delay started

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
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
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

static uint32_t _rng_state = 1;
inline void rngSeed(uint32_t seed)
{
  _rng_state = seed ? seed : 1;
}
inline uint32_t rngNext()
{
  // xorshift32
  _rng_state ^= _rng_state << 13;
  _rng_state ^= _rng_state >> 17;
  _rng_state ^= _rng_state << 5;
  return _rng_state;
}
inline long rngRandom(long min, long max)
{
  // avoid division by zero
  uint32_t span = (uint32_t)(max - min);
  if (span == 0)
    return min;
  return min + (long)(rngNext() % span);
}
static inline uint32_t getDeviceUidSeed()
{
  volatile uint32_t *uid = (uint32_t *)UID_BASE;
  // XOR word0 ^ word1 ^ word2 → 32-bit seed
  return uid[0] ^ uid[1] ^ uid[2];
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

  uint32_t seed = getDeviceUidSeed();
  rngSeed(seed);
  DEBUG_SERIAL.print("Seeding RNG from G0 UID: 0x");
  DEBUG_SERIAL.println(seed, HEX);

  // Start the startup delay timer
  startupDelayStartTime = millis();
}

void loop()
{
  GPIO.handler();
  GPIO.setLED(LED_CHGR, GPIO.getChargerStatus(), GPIO.getChargerStatus());
  GPIO.setLED(LED_LINE, GPIO.getMuxStatus(), false);
  GPIO.setLED(LED_BAT, !GPIO.getMuxStatus(), false);
  GPIO.setLineFault(GPIO.getMuxStatus());

  unsigned long currentMillis = millis();

  // Non-blocking startup delay (5 seconds)
  if (!startupComplete && (currentMillis - startupDelayStartTime >= STARTUP_DELAY))
  {
    startupComplete = true;
    DEBUG_SERIAL.println("Startup delay completed.");

    // Enable charger if no fault is latched
    if (!faultLatched)
    {
      GPIO.setChargerEnable(true);
      DEBUG_SERIAL.println("Charger Enabled at Startup.");
    }
  }

  if (!startupComplete)
  {
    return; // Do nothing if startup delay hasn't finished
  }

  // Modify the voltage measurement section
  if (currentMillis - lastVoltageMeasurementTime >= VOLTAGE_MEASUREMENT_PERIOD)
  {
    lastVoltageMeasurementTime = currentMillis;

    DEBUG_SERIAL.print("Battery Voltage: ");
    DEBUG_SERIAL.println(battery_voltage);

    // Check if voltage drops below threshold and latch fault
    if (battery_voltage < BATTERY_VOLTAGE_THRESHOLD && !faultLatched)
    {
      DEBUG_SERIAL.println("Voltage below threshold! Latching fault...");
      faultLatched = true;
      GPIO.setBatteryFault(true); // Latch the fault and disable the charger
      GPIO.setChargerEnable(false);
    }

    // If voltage recovers above threshold, clear the fault and enable charger
    if (battery_voltage >= BATTERY_VOLTAGE_THRESHOLD && faultLatched)
    {
      DEBUG_SERIAL.println("Voltage recovered! Releasing fault and enabling charger...");
      faultLatched = false;
      GPIO.setBatteryFault(false); // Clear the fault
      GPIO.setChargerEnable(true); // Enable the charger
    }
  }

  // Handle standard deviation fault latching (but not releasing)
  if (currentMillis - lastFaultCheckTime >= 50)
  {
    lastFaultCheckTime = currentMillis;
    DEBUG_SERIAL.print("Voltage Standard Deviation: ");
    DEBUG_SERIAL.println(battery_stddev);

    // If standard deviation exceeds threshold, latch the fault (charger stays off)
    if (battery_stddev > 50.0f && !faultLatched)
    {
      DEBUG_SERIAL.println("High voltage variation detected! Latching fault due to stddev...");
      faultLatched = true;
      GPIO.setBatteryFault(true);
      GPIO.setChargerEnable(false); // Disable the charger when fault is latched
    }
  }
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
