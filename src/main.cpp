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

// ===================== Battery fault & charger manager =====================
static const int BAT_OK_THRESHOLD_MV = 10000;     // 10.0V
static const int STDDEV_FAULT_THRESHOLD_MV = 125; // Method 1 trigger
static const uint32_t STARTUP_DELAY_MS = 10000;   // 10 s startup hold-off
static const uint32_t BLANKING_MS = 5000;         // 5 s blanking
static const uint32_t CHECK_MIN_MS = 30000;       // 30 s between checks (random)
static const uint32_t CHECK_MAX_MS = 120000;      // 120 s (exclusive)
static const uint32_t SETTLE_MS = 2000;           // 2 s settle after upward jump
static const uint32_t M2_ANTIRESP_MS = 10000;     // don't reschedule within 10 s
static const int LINE_OK_THRESHOLD_MV = 20000;    // 20.0V line threshold for enabling Method-1
static const int LINE_HYST = 1000;                // 1.0V hysteresis for line threshold

static bool m1Suppressed = false; // track suppression state for low-spam debug

static bool batteryFaultLatched = false;
static bool chargerEnabled = false;

static uint32_t t_startup = 0;

// recovery blanking
static bool blankingActive = false;
static uint32_t t_blank_start = 0;

// Method 2 scheduler: random time *between* checks
static bool finishedPrev = false;
static uint32_t nextCheckDue = 0;
static bool checkInProgress = false;
static uint32_t t_check_start = 0;
static uint32_t lastScheduleAt = 0; // anti-reschedule guard

// Settle handling for upward jumps
static int prev_batt_mv = 0;
static bool settleActive = false;
static uint32_t t_settle_start = 0;
static uint32_t lastRiseAt = 0;

// --- helpers ---
static inline void setCharger(bool enable)
{
  if (chargerEnabled != enable)
  {
    chargerEnabled = enable;
    GPIO.setChargerEnable(enable);
    DEBUG_SERIAL.print("[CHARGER] ");
    DEBUG_SERIAL.println(enable ? "ENABLED" : "DISABLED");
  }
}

// Assumption: gpio.getChargerStatus() == true while CHARGING (pin LOW). So "finished" is !getChargerStatus().
static inline bool isChargeFinished() { return !GPIO.getChargerStatus(); }
static inline bool batOverOk() { return battery_voltage > BAT_OK_THRESHOLD_MV; }

static void startBlanking()
{
  blankingActive = true;
  t_blank_start = millis();
  DEBUG_SERIAL.println("[BAT] Recovery blanking started");
}
static bool blankingDone() { return blankingActive && (millis() - t_blank_start >= BLANKING_MS); }
static void clearBlanking()
{
  if (blankingActive)
  {
    blankingActive = false;
    DEBUG_SERIAL.println("[BAT] Recovery blanking cleared");
  }
}

static inline uint32_t randBetween(uint32_t a, uint32_t b_exclusive)
{
  return (uint32_t)rngRandom((long)a, (long)b_exclusive);
}

static void scheduleNextCheck(uint32_t now)
{
  // Anti-reschedule: only allow if >10 s since the last schedule
  if ((now - lastScheduleAt) < M2_ANTIRESP_MS)
  {
    return;
  }
  uint32_t delay = randBetween(CHECK_MIN_MS, CHECK_MAX_MS);
  nextCheckDue = now + delay;
  lastScheduleAt = now;
  DEBUG_SERIAL.print("[M2] Next check in ");
  DEBUG_SERIAL.print(delay / 1000);
  DEBUG_SERIAL.println(" s");
}

// --- Method 2: random interval BETWEEN checks; charger off only for 5s per check ---
static void method2_fallback_tick()
{
  const uint32_t now = millis();
  const bool finished = isChargeFinished();

  // On rising edge to "finished", (re)start the between-check timer (with anti-reschedule guard)
  if (finished && !finishedPrev)
  {
    scheduleNextCheck(now);
  }
  finishedPrev = finished;

  // If a 5s check is currently running, finish it
  if (checkInProgress)
  {
    if (now - t_check_start >= BLANKING_MS)
    {
      checkInProgress = false; // measurement point

      DEBUG_SERIAL.print("[M2] Voltage check: ");
      DEBUG_SERIAL.print(battery_voltage);
      DEBUG_SERIAL.println(" mV");

      // If we very recently had an upward jump, don't create/clear faults yet
      bool inSettleGuard = (now - lastRiseAt) < SETTLE_MS;

      if (!inSettleGuard && (battery_voltage < BAT_OK_THRESHOLD_MV))
      {
        // Below 10.0V -> latch fault, keep charger disabled
        batteryFaultLatched = true;
        GPIO.setBatteryFault(true);
        startBlanking();
        DEBUG_SERIAL.println("[M2] Fault latched due to low voltage");
      }
      else if (battery_voltage >= BAT_OK_THRESHOLD_MV)
      {
        // OK -> re-enable charger (this does not 'lift' a fault; it's just ending a check)
        setCharger(true);
        GPIO.setBatteryFault(false);
        // If a fault was active, its clearing is handled in the main tick with settle + blanking
        DEBUG_SERIAL.println(inSettleGuard ? "[M2] Voltage OK (within settle guard), charger re-enabled"
                                           : "[M2] Voltage OK, charger re-enabled");
      }

      // schedule next check (time between checks is random)
      scheduleNextCheck(now);
    }
    return; // don't start another check while one is active
  }

  if (batteryFaultLatched)
    return;

  // Start a new brief check if charger reports "finished" and random delay elapsed
  if (finished && (nextCheckDue != 0) && ((int32_t)(now - nextCheckDue) >= 0))
  {
    setCharger(false); // disable only for blanking + measurement
    t_check_start = now;
    checkInProgress = true;
    DEBUG_SERIAL.println("[M2] Starting 5s blanking before measurement");
  }
}

// --- Main battery manager tick ---
static void battery_manager_tick()
{
  const uint32_t now = millis();

  // ----- Upward crossing & 2s settle handling -----
  if (prev_batt_mv <= BAT_OK_THRESHOLD_MV && battery_voltage > BAT_OK_THRESHOLD_MV)
  {
    lastRiseAt = now;
    settleActive = true;
    t_settle_start = now;
    DEBUG_SERIAL.println("[BAT] Upward crossing detected, starting 2s settle");
  }
  // Cancel settle if it drops back below threshold
  if (settleActive && battery_voltage <= BAT_OK_THRESHOLD_MV)
  {
    settleActive = false;
  }
  bool settleDone = settleActive && ((now - t_settle_start) >= SETTLE_MS);

  // ----- Startup delay (10s hold-off) -----
  if (t_startup == 0)
  {
    t_startup = now;
    return;
  }
  if (now - t_startup < STARTUP_DELAY_MS)
  {
    prev_batt_mv = battery_voltage;
    return;
  }

  // ----- First voltage check (one-time) -----
  static bool firstCheckDone = false;
  if (!firstCheckDone)
  {
    DEBUG_SERIAL.print("[BAT] Startup voltage: ");
    DEBUG_SERIAL.print(battery_voltage);
    DEBUG_SERIAL.println(" mV");

    if (batOverOk())
    {
      setCharger(true);
      GPIO.setBatteryFault(false);
    }
    else
    {
      setCharger(false);
      batteryFaultLatched = true;
      GPIO.setBatteryFault(true);
      startBlanking();
      DEBUG_SERIAL.println("[BAT] Fault latched at startup (low voltage)");
    }
    firstCheckDone = true;
  }

  static bool m1Suppressed = false;
  bool wantSuppress = m1Suppressed
                          ? (line_voltage < (LINE_OK_THRESHOLD_MV + LINE_HYST / 2))  // bleib suppressed bis klar drüber
                          : (line_voltage < (LINE_OK_THRESHOLD_MV - LINE_HYST / 2)); // suppress erst wenn klar drunter

  if (wantSuppress != m1Suppressed)
  {
    m1Suppressed = wantSuppress;
    if (m1Suppressed)
    {
      GPIO.setLineFault(true);
      DEBUG_SERIAL.printf("[M1] Disabled: line low (%d mV < %d)\n", line_voltage, LINE_OK_THRESHOLD_MV);
    }
    else
    {
      GPIO.setLineFault(false);
      DEBUG_SERIAL.printf("[M1] Enabled: line OK (%d mV >= %d)\n", line_voltage, LINE_OK_THRESHOLD_MV);
    }
  }

  // ----- Method-1 (instant stddev) with 2s settle guard and line gate -----
  bool inSettleGuard = (now - lastRiseAt) < SETTLE_MS;
  if (!batteryFaultLatched && !inSettleGuard && !m1Suppressed &&
      (battery_stddev > STDDEV_FAULT_THRESHOLD_MV))
  {
    batteryFaultLatched = true;
    setCharger(false);
    GPIO.setBatteryFault(true);
    startBlanking();
    DEBUG_SERIAL.print("[M1] Stddev fault triggered (");
    DEBUG_SERIAL.print(battery_stddev);
    DEBUG_SERIAL.println(" mV)");
  }

  // ----- Method-2 (random interval checks; 5s blanking per check) -----
  method2_fallback_tick();

  // ----- Recovery from latched fault: needs blanking(5s) + settle(2s) + V > 10.0V -----
  if (batteryFaultLatched)
  {
    if (blankingDone() && settleDone && batOverOk())
    {
      batteryFaultLatched = false;
      GPIO.setBatteryFault(false);
      setCharger(true);
      clearBlanking();
      settleActive = false; // consume settle
      DEBUG_SERIAL.println("[BAT] Fault cleared after recovery (+settle)");
    }
  }

  prev_batt_mv = battery_voltage;
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

  t_startup = millis(); // start the 10s startup hold-off
  setCharger(false);    // charger disabled during startup hold-off
  GPIO.setBatteryFault(false);
}
void loop()
{
  GPIO.handler();
  GPIO.setLED(LED_CHGR, GPIO.getChargerStatus(), GPIO.getChargerStatus());
  GPIO.setLED(LED_LINE, GPIO.getMuxStatus(), false);
  GPIO.setLED(LED_BAT, !GPIO.getMuxStatus(), false);
  GPIO.setLineFault(GPIO.getMuxStatus());

  // Battery fault/charge management
  battery_manager_tick();
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
