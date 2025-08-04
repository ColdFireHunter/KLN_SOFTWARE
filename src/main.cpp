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

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;
gpio GPIO;

Neotimer battery_fault_timer(50);            // 500 ms fault check
uint32_t last_battery_stddev = 0;            // last reading, mV
uint32_t battery_jump_threshold_stddev = 50; // configurable jump threshold, mV

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

// Seed the generator: pass any nonzero value.
inline void rngSeed(uint32_t seed)
{
  _rng_state = seed ? seed : 1;
}

// Return next 32-bit random word
inline uint32_t rngNext()
{
  // xorshift32
  _rng_state ^= _rng_state << 13;
  _rng_state ^= _rng_state >> 17;
  _rng_state ^= _rng_state << 5;
  return _rng_state;
}

// rnd(min, max): [min … max-1]
inline long rngRandom(long min, long max)
{
  // avoid division by zero
  uint32_t span = (uint32_t)(max - min);
  if (span == 0)
    return min;
  return min + (long)(rngNext() % span);
}

// Overload to mimic Arduino’s random(max)
inline long rngRandom(long max)
{
  return rngRandom(0, max);
}

// Read the 96-bit UID and fold to 32 bits
static inline uint32_t getDeviceUidSeed()
{
  volatile uint32_t *uid = (uint32_t *)UID_BASE;
  // XOR word0 ^ word1 ^ word2 → 32-bit seed
  return uid[0] ^ uid[1] ^ uid[2];
}

//------------------------------------------------------------------------------
// Common latch function: if `cond` true and no fault yet, latch with `cause`:
//   cause=1 → 5 s min‐off + stddev/voltage recovery
//   cause=2 → 10 s min‐off only
//------------------------------------------------------------------------------
static bool batteryFaultLatched = false;
static uint8_t batteryFaultCause = 0;
static uint32_t faultDisableTimeMs = 0;
static uint32_t faultMinOffMs = 0;

void clearFault()
{
  batteryFaultLatched = false;
  batteryFaultCause = 0;
  DEBUG_SERIAL.println("Battery‐fault cleared, re‐enabling charger.");
  GPIO.setChargerEnable(true);
}
void tryLatchFault(bool cond, uint8_t cause, uint32_t minOffMs)
{
  if (!batteryFaultLatched && cond)
  {
    batteryFaultLatched = true;
    batteryFaultCause = cause;
    faultDisableTimeMs = millis();
    faultMinOffMs = minOffMs;

    const char *why = (cause == 1
                           ? "stddev spike/open‐fuse"
                           : "random low‐voltage check");
    DEBUG_SERIAL.print("Battery‐fault latched: ");
    DEBUG_SERIAL.print(why);
    DEBUG_SERIAL.print(", min‐off=");
    DEBUG_SERIAL.print(minOffMs);
    DEBUG_SERIAL.println(" ms.");
    // disable charger
    GPIO.setChargerEnable(false);
  }
}
//------------------------------------------------------------------------------
// Recovery: once the min‐off time has elapsed, clear the latch if:
//  • cause==1: stddev spike cleared AND voltage >10 000 mV
//  • cause==2: (no other condition)
//------------------------------------------------------------------------------
void handleRecovery()
{
  if (!batteryFaultLatched)
    return;

  uint32_t now = millis();
  if (now - faultDisableTimeMs < faultMinOffMs)
    return;

  if (batteryFaultCause == 1)
  {
    // re‐read conditions
    uint32_t curr_std = static_cast<uint32_t>(battery_stddev);
    uint32_t curr_mv = battery_voltage;
    bool open_or_fuse_fault =
        (abs((int32_t)curr_std - (int32_t)last_battery_stddev) > static_cast<int32_t>(battery_jump_threshold_stddev));

    if (!open_or_fuse_fault && curr_mv > 10000)
    {
      clearFault();
    }
  }
  else // cause==2
  {
    clearFault();
  }
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
}

void loop()
{
  GPIO.handler();
  GPIO.setLED(LED_CHGR, GPIO.getChargerStatus(), GPIO.getChargerStatus());
  GPIO.setLED(LED_LINE, GPIO.getMuxStatus(), false);
  GPIO.setLED(LED_BAT, !GPIO.getMuxStatus(), false);

  // --- enable jump/fuse & random logic 1 s after boot ---
  static bool faultLogicEnabled = false;
  static uint32_t nextRandomCheckMs = 0;
  if (!faultLogicEnabled && millis() >= 1000)
  {
    faultLogicEnabled = true;
    last_battery_stddev = static_cast<uint32_t>(battery_stddev);
    // schedule first random check in [10,30] min
    nextRandomCheckMs = millis() + rngRandom(600000, 1800001);
    DEBUG_SERIAL.println("Battery‐fault logic enabled.");
  }

  if (faultLogicEnabled)
  {
    uint32_t now = millis();

    // —— periodic ADC/DMA stddev‐based check ——
    if (battery_fault_timer.repeat())
    {
      uint32_t curr_std = static_cast<uint32_t>(battery_stddev);
      bool open_or_fuse_fault =
          (last_battery_stddev > 0) &&
          (abs((int32_t)curr_std - (int32_t)last_battery_stddev) > static_cast<int32_t>(battery_jump_threshold_stddev));
      last_battery_stddev = curr_std;

      tryLatchFault(open_or_fuse_fault, 1 /*cause*/, 5000);
    }

    // —— random “health” check every 10–30 min ——
    if ((int32_t)(now - nextRandomCheckMs) >= 0)
    {
      // schedule next
      nextRandomCheckMs = now + rngRandom(600000, 1800001);
      DEBUG_SERIAL.println("Random battery health check...");
      // only if charger is off
      if (!GPIO.getChargerStatus())
      {
        uint32_t curr_mv = battery_voltage;
        bool lowVoltageFault = (curr_mv < 10000);
        tryLatchFault(lowVoltageFault, 2 /*cause*/, 10000);
      }
    }

    // —— recovery logic ——
    handleRecovery();

    // always alarm immediately on low SOC
    bool low_percent = (GPIO.getBatteryPercent() < 15.0f);
    GPIO.setBatteryFault(batteryFaultLatched || low_percent);
  }

  GPIO.setLineFault(!GPIO.getMuxStatus());
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
