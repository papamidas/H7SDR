/* USER CODE BEGIN Header */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "fftwindows.h"
#include "base64_encode.h"
#include "arm_math.h"
#include <stdint.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ERRHND      {err_File = __FILE__; err_Line = __LINE__; Error_Handler();}
#define SET_TRIG1   {HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_SET);}
#define RESET_TRIG1 {HAL_GPIO_WritePin(TRIG1_GPIO_Port, TRIG1_Pin, GPIO_PIN_RESET);}
#define SET_FLAG1   {HAL_GPIO_WritePin(FLAG1_GPIO_Port, FLAG1_Pin, GPIO_PIN_SET);}
#define RESET_FLAG1 {HAL_GPIO_WritePin(FLAG1_GPIO_Port, FLAG1_Pin, GPIO_PIN_RESET);}
#define SET_FLAG2   {HAL_GPIO_WritePin(FLAG2_GPIO_Port, FLAG2_Pin, GPIO_PIN_SET);}
#define RESET_FLAG2 {HAL_GPIO_WritePin(FLAG2_GPIO_Port, FLAG2_Pin, GPIO_PIN_RESET);}
// Frequency of high speed external clock in Hz
#define F_HSE 8000000
// PLL2 N divisor, DIVM2 = 1 assumed
#define DIVN2 26
// PLL2 fracn2
#define FRACN2 8010
// PLL2 DIVP2
#define DIVP2 8
#define ADC_CLOCK (F_HSE * (26.0 + FRACN2/8192) / DIVP2)
// ADC in 16bit mode needs 10 adc_ker_clk-cycles per conversion
#define CYCLES_PER_ADC_CONVERSION 10
// Time between ADC Samples in s:
#define ADC_PERIOD (CYCLES_PER_ADC_CONVERSION/ADC_CLOCK)
// first decimation filter oversampling ratio
#define FOSR_first 15
// second decimation filter oversampling ratio
#define FOSR_second 8
// Time between DAC samples in s:
#define DAC_PERIOD (ADC_PERIOD * FOSR_first*FOSR_second)

#define DMA_IDLE -1
#define DMA_INPROGRESS 0
#define DMA_HALFCOMPLETE 1
#define DMA_COMPLETE 2

#define DAC_BUFLEN 64
#define FILTER2_BUFLEN (DAC_BUFLEN)
#define FILTER0_BUFLEN (FOSR_second * DAC_BUFLEN)
// ADC_BUFLEN has to be a multiple of 8 (I forgot why...):
#define ADC_BUFLEN (FOSR_first * FOSR_second * DAC_BUFLEN)
// Taps for CIC compensation filter
#define NUM_TAPS 81

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch1;

DFSDM_Filter_HandleTypeDef hdfsdm1_filter0;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter1;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter2;
DFSDM_Filter_HandleTypeDef hdfsdm1_filter3;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel0;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel1;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel2;
DFSDM_Channel_HandleTypeDef hdfsdm1_channel3;
DMA_HandleTypeDef hdma_dfsdm1_flt0;
DMA_HandleTypeDef hdma_dfsdm1_flt1;
DMA_HandleTypeDef hdma_dfsdm1_flt2;
DMA_HandleTypeDef hdma_dfsdm1_flt3;

LPTIM_HandleTypeDef hlptim1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static uint16_t adc_buf[ADC_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static float32_t f32_ibuf[FILTER0_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static float32_t f32_qbuf[FILTER0_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static q31_t q31_filter0buf[FILTER0_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static q31_t q31_filter1buf[FILTER0_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static q31_t q31_filter2buf[FILTER2_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static q31_t q31_filter3buf[FILTER2_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static float32_t cplx_buf[2*FILTER2_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static float32_t mag_buf[FILTER2_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static float32_t filteredMag_buf[FILTER2_BUFLEN]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static float32_t firStateF32[FILTER2_BUFLEN/2 + NUM_TAPS -1]);
__attribute__((section(".ram_sdr.ADCBUF"))) ALIGN_32BYTES (static uint16_t dac_buf[FILTER2_BUFLEN]);

/*

float32_t cic_compensation_fir_coeff[31] = {
		+0.0069887819, +0.0041257750, -0.0048042590, -0.0173970575,	-0.0203086182, +0.0024378493, +0.0458612471,
		+0.0710893703, +0.0300397219, -0.0805266040, -0.1842281187, -0.1572475188, +0.0788135148, +0.4727037367,
		+0.8464181000, +1.0000000000, +0.8464181000, +0.4727037367, +0.0788135148, -0.1572475188, -0.1842281187,
		-0.0805266040, +0.0300397219, +0.0710893703, +0.0458612471, +0.0024378493, -0.0203086182, -0.0173970575,
		-0.0048042590, +0.0041257750, +0.0069887819};
 */
/*
float32_t cic_compensation_fir_coeff[81] = {
		+0.0002662589, -0.0019258547, -0.0027314644, -0.0012491348, +0.0017764407, +0.0040306125, +0.0031138979,
		-0.0012103384, -0.0059117608, -0.0064405278, -0.0007524619, +0.0076810727, +0.0114169766, +0.0051884959,
		-0.0080865832, -0.0176107450, -0.0129870200, +0.0054343633, +0.0238248253, +0.0245654785, +0.0022224689,
		-0.0280433507, -0.0396466601, -0.0169173794, +0.0273996878, +0.0571376961, +0.0408084327, -0.0179408090,
		-0.0750885233, -0.0768359047, -0.0065270418, +0.0905038921, +0.1313586058, +0.0594465897, -0.0976344743,
		-0.2255574379, -0.1894809324, +0.0697413479, +0.4769489071, +0.8495355583, +1.0000000000, +0.8495355583,
		+0.4769489071, +0.0697413479, -0.1894809324, -0.2255574379, -0.0976344743, +0.0594465897, +0.1313586058,
		+0.0905038921, -0.0065270418, -0.0768359047, -0.0750885233, -0.0179408090, +0.0408084327, +0.0571376961,
		+0.0273996878, -0.0169173794, -0.0396466601, -0.0280433507, +0.0022224689, +0.0245654785, +0.0238248253,
		+0.0054343633, -0.0129870200, -0.0176107450, -0.0080865832, +0.0051884959, +0.0114169766, +0.0076810727,
		-0.0007524619, -0.0064405278, -0.0059117608, -0.0012103384, +0.0031138979, +0.0040306125, +0.0017764407,
		-0.0012491348, -0.0027314644, -0.0019258547, +0.0002662589};
*/
// mit Hochpass:
float32_t cic_compensation_fir_coeff[81] = {
		+0.0009506491, -0.0016149139, -0.0026729986, -0.0012594242, +0.0017881922, +0.0039142256, +0.0024694202,
		-0.0028309208, -0.0086990718, -0.0101849145, -0.0050763894, +0.0028101508, +0.0053185214, -0.0033244777,
		-0.0199392501, -0.0327262471, -0.0303191978, -0.0130977853, +0.0038263100, +0.0012023183, -0.0268923326,
		-0.0638570754, -0.0805951016, -0.0599397519, -0.0157889142, +0.0123535948, -0.0098975128, -0.0785083560,
		-0.1452986453, -0.1513968665, -0.0785432385, +0.0237417624, +0.0648637581, -0.0170092890, -0.1916618459,
		-0.3340529012, -0.2967158273, -0.0149534091, +0.4290760698, +0.8357281697, +1.0000000000, +0.8357281697,
		+0.4290760698, -0.0149534091, -0.2967158273, -0.3340529012, -0.1916618459, -0.0170092890, +0.0648637581,
		+0.0237417624, -0.0785432385, -0.1513968665, -0.1452986453, -0.0785083560, -0.0098975128, +0.0123535948,
		-0.0157889142, -0.0599397519, -0.0805951016, -0.0638570754, -0.0268923326, +0.0012023183, +0.0038263100,
		-0.0130977853, -0.0303191978, -0.0327262471, -0.0199392501, -0.0033244777, +0.0053185214, +0.0028101508,
		-0.0050763894, -0.0101849145, -0.0086990718, -0.0028309208, +0.0024694202, +0.0039142256, +0.0017881922,
		-0.0012594242, -0.0026729986, -0.0016149139, +0.0009506491};

/* ------------------------------------------------------------------
 * Global variables for FIR filter
 * ------------------------------------------------------------------- */
arm_fir_instance_f32 fir_instance;
arm_status armStatus;

// DFSDM channel 0 input register (unfortunately not found in HAL, so declared here:)
volatile uint32_t *DFSDM_CH0DATINR = (uint32_t *) (DFSDM1_BASE + 0x10);
volatile uint32_t dfsdm_chydatinr = 0;
// DFSDM channel 2 input register
volatile uint32_t *DFSDM_CH2DATINR = (uint32_t *) (DFSDM1_BASE + 0x10 + 2*0x20);
//
volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
volatile uint32_t *DEMCR = (uint32_t *)0xE000EDFC;
//
char *err_File;
int err_Line;

volatile int8_t adcBufDMAState = DMA_IDLE;
volatile int8_t filter0BufDMAState = DMA_IDLE;
volatile int8_t filter1BufDMAState = DMA_IDLE;
volatile int8_t filter2BufDMAState = DMA_IDLE;
volatile int8_t filter3BufDMAState = DMA_IDLE;
volatile int8_t dacBufDMAState = DMA_IDLE;
volatile int8_t txBufDMAState = DMA_IDLE;

volatile uint8_t dacDMAInitialized = 0;

volatile int16_t dac_gain = 40;

volatile uint8_t offsetCalibrationDone = 0;
uint32_t lastTick;
uint32_t curTick;
const float32_t a_fgmean = 1.0e-5;
float32_t fgmean = 32767.0;
int32_t gMean = 32767;

volatile int dacDMADelay_ms = 1;
const uint8_t ifftFlag = 0; // forward FFT

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_DAC1_Init(void);
static void MX_ADC1_Init(void);
static void MX_DFSDM1_Init(void);
static void MX_LPTIM1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void feedDFSDMinput(uint16_t startIndex);
void feedSecondFilter(uint16_t startindex);
void feedDACBuf(uint16_t startindex);
void delay_ns (uint16_t ns);
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

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_DAC1_Init();
  MX_ADC1_Init();
  MX_DFSDM1_Init();
  MX_LPTIM1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  arm_fir_init_f32(&fir_instance, NUM_TAPS, cic_compensation_fir_coeff, firStateF32, FILTER2_BUFLEN/2);

  //kaiser_f32(win_buf, 38, ADC_BUFLEN);

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, q31_filter0buf, FILTER0_BUFLEN) != HAL_OK) {
	  ERRHND;
  }
  filter0BufDMAState = DMA_INPROGRESS;

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter1, q31_filter1buf, FILTER0_BUFLEN) != HAL_OK) {
	  ERRHND;
  }
  filter1BufDMAState = DMA_INPROGRESS;

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter2, q31_filter2buf, FILTER2_BUFLEN) != HAL_OK) {
	  ERRHND;
  }
  filter2BufDMAState = DMA_INPROGRESS;

  if (HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter3, q31_filter3buf, FILTER2_BUFLEN) != HAL_OK) {
	  ERRHND;
  }
  filter3BufDMAState = DMA_INPROGRESS;

  //ADC_ConfigureBoostMode(&hadc1); // recommended for ADCCLK>20M, see datasheet

  if (HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUFLEN) != HAL_OK) {
	  ERRHND;
   }
  adcBufDMAState = DMA_INPROGRESS;

  // First ADC block will be fed to DFSDM filter input beginning after xxx
  // Start of DAC DMA should be delayed
  // HAL_Delay(dacDMADelay_ms);

  // TIM1 only needed for short delays:
  if(HAL_OK != HAL_TIM_Base_Start(&htim1)) {
	  ERRHND;
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)  {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin); // blue LED LD2 on NUCLEO board

      if (adcBufDMAState > DMA_INPROGRESS) {
		  if (adcBufDMAState == DMA_HALFCOMPLETE) {
			  SET_FLAG2; // dunkelblau
			  feedDFSDMinput(0);
			  adcBufDMAState = DMA_INPROGRESS;
		  }
          else if (adcBufDMAState == DMA_COMPLETE) {
			  RESET_FLAG2; // dunkelblau
			  feedDFSDMinput(ADC_BUFLEN/2);
			  adcBufDMAState = DMA_INPROGRESS;
		  }
      }

      if (filter0BufDMAState > DMA_INPROGRESS) { // filters 0+1 are ready at the same time
    	  if (filter0BufDMAState == DMA_HALFCOMPLETE) {
    		  SET_FLAG1; //magenta
              /* Feed first halves of filter-output 0,1 to filter inputs 2,3 */
    	      feedSecondFilter(0);
    	      filter0BufDMAState = DMA_INPROGRESS;
    	      filter1BufDMAState = DMA_INPROGRESS;
    	 }
    	 else if (filter0BufDMAState == DMA_COMPLETE) {
     		  RESET_FLAG1; //magenta
              /* Feed second halves of filter-output 0,1 to filter inputs 2,3 */
    	      feedSecondFilter(FILTER0_BUFLEN/2);
    	      filter0BufDMAState = DMA_INPROGRESS;
    	      filter1BufDMAState = DMA_INPROGRESS;
    	 }
      }

      if (filter2BufDMAState > DMA_INPROGRESS) { // filters 2+3 are ready at the same time
    	  if (filter2BufDMAState == DMA_HALFCOMPLETE) {
              /* Store filtered values in first half of DAC buf */
    	      feedDACBuf(0);
    	      // jetzt DAC-DMA starten, falls nicht schon geschehen:
    	      if(dacDMAInitialized == 0) {
    	    	  if (HAL_OK != HAL_LPTIM_Counter_Start(&hlptim1,
    	    			  FOSR_first*FOSR_second * CYCLES_PER_ADC_CONVERSION - 1 )) {
    	    		  ERRHND;
    	    	  }
    	    	  if(HAL_OK != HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_1, (uint32_t*)dac_buf,
    	    	 		   FILTER2_BUFLEN, DAC_ALIGN_12B_L)) {
    	    		  ERRHND;
    	    	  }
    	    	  dacDMAInitialized = 1;
    	      }
    	      filter2BufDMAState = DMA_INPROGRESS;
    	      filter3BufDMAState = DMA_INPROGRESS;
    	 }
    	 else if (filter2BufDMAState == DMA_COMPLETE) {
              /* Store filtered values on second half of DAC buf */
              feedDACBuf(FILTER2_BUFLEN/2);
    	      filter2BufDMAState = DMA_INPROGRESS;
    	      filter3BufDMAState = DMA_INPROGRESS;
    	 }
      }

      if (dacBufDMAState > DMA_INPROGRESS) {
          if (dacBufDMAState == DMA_HALFCOMPLETE) {
              SET_TRIG1; //hellblau
              dacBufDMAState = DMA_INPROGRESS;
          }
          else if (dacBufDMAState == DMA_COMPLETE) {
              RESET_TRIG1; //hellblau
              dacBufDMAState = DMA_INPROGRESS;
          }
      }

      if ((curTick = HAL_GetTick()) != lastTick) {
	      fgmean = fgmean * (1.0 - a_fgmean) + a_fgmean * adc_buf[0];
	      gMean = (int)(fgmean + 0.5);
	      lastTick = curTick;
      }
  }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}
  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 200;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_LPTIM1;
  PeriphClkInitStruct.PLL2.PLL2M = 1;
  PeriphClkInitStruct.PLL2.PLL2N = 26;
  PeriphClkInitStruct.PLL2.PLL2P = 8;
  PeriphClkInitStruct.PLL2.PLL2Q = 8;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_3;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 8010;
  PeriphClkInitStruct.Lptim1ClockSelection = RCC_LPTIM1CLKSOURCE_PLL2;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_16B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */
  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }
  /** DAC channel OUT1 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_LPTIM1_OUT;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief DFSDM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DFSDM1_Init(void)
{

  /* USER CODE BEGIN DFSDM1_Init 0 */

  /* USER CODE END DFSDM1_Init 0 */

  /* USER CODE BEGIN DFSDM1_Init 1 */

  /* USER CODE END DFSDM1_Init 1 */
  hdfsdm1_filter0.Instance = DFSDM1_Filter0;
  hdfsdm1_filter0.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter0.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter0.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter0.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter0.Init.FilterParam.Oversampling = 15;
  hdfsdm1_filter0.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter1.Instance = DFSDM1_Filter1;
  hdfsdm1_filter1.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter1.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter1.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter1.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC4_ORDER;
  hdfsdm1_filter1.Init.FilterParam.Oversampling = 15;
  hdfsdm1_filter1.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter2.Instance = DFSDM1_Filter2;
  hdfsdm1_filter2.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter2.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter2.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter2.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter2.Init.FilterParam.Oversampling = 8;
  hdfsdm1_filter2.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter2) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_filter3.Instance = DFSDM1_Filter3;
  hdfsdm1_filter3.Init.RegularParam.Trigger = DFSDM_FILTER_SW_TRIGGER;
  hdfsdm1_filter3.Init.RegularParam.FastMode = ENABLE;
  hdfsdm1_filter3.Init.RegularParam.DmaMode = ENABLE;
  hdfsdm1_filter3.Init.FilterParam.SincOrder = DFSDM_FILTER_SINC5_ORDER;
  hdfsdm1_filter3.Init.FilterParam.Oversampling = 8;
  hdfsdm1_filter3.Init.FilterParam.IntOversampling = 1;
  if (HAL_DFSDM_FilterInit(&hdfsdm1_filter3) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel0.Instance = DFSDM1_Channel0;
  hdfsdm1_channel0.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel0.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel0.Init.OutputClock.Divider = 2;
  hdfsdm1_channel0.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel0.Init.Input.DataPacking = DFSDM_CHANNEL_DUAL_MODE;
  hdfsdm1_channel0.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel0.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel0.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel0.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel0.Init.Awd.Oversampling = 1;
  hdfsdm1_channel0.Init.Offset = 0x00;
  hdfsdm1_channel0.Init.RightBitShift = 8;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel0) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel1.Instance = DFSDM1_Channel1;
  hdfsdm1_channel1.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel1.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel1.Init.OutputClock.Divider = 2;
  hdfsdm1_channel1.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel1.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel1.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel1.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel1.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel1.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel1.Init.Awd.Oversampling = 1;
  hdfsdm1_channel1.Init.Offset = 0x00;
  hdfsdm1_channel1.Init.RightBitShift = 8;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel1) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel2.Instance = DFSDM1_Channel2;
  hdfsdm1_channel2.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel2.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel2.Init.OutputClock.Divider = 2;
  hdfsdm1_channel2.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel2.Init.Input.DataPacking = DFSDM_CHANNEL_DUAL_MODE;
  hdfsdm1_channel2.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel2.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel2.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel2.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel2.Init.Awd.Oversampling = 1;
  hdfsdm1_channel2.Init.Offset = 0x00;
  hdfsdm1_channel2.Init.RightBitShift = 7;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel2) != HAL_OK)
  {
    Error_Handler();
  }
  hdfsdm1_channel3.Instance = DFSDM1_Channel3;
  hdfsdm1_channel3.Init.OutputClock.Activation = DISABLE;
  hdfsdm1_channel3.Init.OutputClock.Selection = DFSDM_CHANNEL_OUTPUT_CLOCK_SYSTEM;
  hdfsdm1_channel3.Init.OutputClock.Divider = 2;
  hdfsdm1_channel3.Init.Input.Multiplexer = DFSDM_CHANNEL_INTERNAL_REGISTER;
  hdfsdm1_channel3.Init.Input.DataPacking = DFSDM_CHANNEL_STANDARD_MODE;
  hdfsdm1_channel3.Init.Input.Pins = DFSDM_CHANNEL_SAME_CHANNEL_PINS;
  hdfsdm1_channel3.Init.SerialInterface.Type = DFSDM_CHANNEL_SPI_RISING;
  hdfsdm1_channel3.Init.SerialInterface.SpiClock = DFSDM_CHANNEL_SPI_CLOCK_EXTERNAL;
  hdfsdm1_channel3.Init.Awd.FilterOrder = DFSDM_CHANNEL_FASTSINC_ORDER;
  hdfsdm1_channel3.Init.Awd.Oversampling = 1;
  hdfsdm1_channel3.Init.Offset = 0x00;
  hdfsdm1_channel3.Init.RightBitShift = 7;
  if (HAL_DFSDM_ChannelInit(&hdfsdm1_channel3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter0, DFSDM_CHANNEL_0, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter1, DFSDM_CHANNEL_1, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter2, DFSDM_CHANNEL_2, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_DFSDM_FilterConfigRegChannel(&hdfsdm1_filter3, DFSDM_CHANNEL_3, DFSDM_CONTINUOUS_CONV_ON) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DFSDM1_Init 2 */

  /* USER CODE END DFSDM1_Init 2 */

}

/**
  * @brief LPTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPTIM1_Init(void)
{

  /* USER CODE BEGIN LPTIM1_Init 0 */

  /* USER CODE END LPTIM1_Init 0 */

  /* USER CODE BEGIN LPTIM1_Init 1 */

  /* USER CODE END LPTIM1_Init 1 */
  hlptim1.Instance = LPTIM1;
  hlptim1.Init.Clock.Source = LPTIM_CLOCKSOURCE_APBCLOCK_LPOSC;
  hlptim1.Init.Clock.Prescaler = LPTIM_PRESCALER_DIV1;
  hlptim1.Init.Trigger.Source = LPTIM_TRIGSOURCE_SOFTWARE;
  hlptim1.Init.OutputPolarity = LPTIM_OUTPUTPOLARITY_HIGH;
  hlptim1.Init.UpdateMode = LPTIM_UPDATE_IMMEDIATE;
  hlptim1.Init.CounterSource = LPTIM_COUNTERSOURCE_INTERNAL;
  hlptim1.Init.Input1Source = LPTIM_INPUT1SOURCE_GPIO;
  hlptim1.Init.Input2Source = LPTIM_INPUT2SOURCE_GPIO;
  if (HAL_LPTIM_Init(&hlptim1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPTIM1_Init 2 */

  /* USER CODE END LPTIM1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_8_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA1_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream3_IRQn);
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, FLAG1_Pin|FLAG2_Pin|TRIG1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : FLAG1_Pin FLAG2_Pin TRIG1_Pin */
  GPIO_InitStruct.Pin = FLAG1_Pin|FLAG2_Pin|TRIG1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

// Frequency translation by -fs/4 without multiplication
// by using cos(n/2 * Pi) = 1,0,-1,0,... and -sin(n/2 * Pi) = 0,-1,0,1,...
// hi word: Q, lo word: I
void feedDFSDMinput(uint16_t startIndex)
{
  int32_t dcFree_ADCSample;
  for(uint16_t i=startIndex; i < startIndex+ADC_BUFLEN/2; i+=4) {
	  dcFree_ADCSample = (adc_buf[i] - gMean) & 0xffff;
	  dfsdm_chydatinr =  dcFree_ADCSample;   // Phase 0: x[n] + j0
	  *DFSDM_CH0DATINR = dfsdm_chydatinr; // load and trigger DFSDM by 32bit write

	  dcFree_ADCSample = (adc_buf[i + 1] - gMean) & 0xffff;
	  dfsdm_chydatinr =  ((-dcFree_ADCSample) & 0xffff) << 16; // Phase 90: 0 + -jx[n]
	  *DFSDM_CH0DATINR = dfsdm_chydatinr; // load and trigger DFSDM by 32bit write

	  dcFree_ADCSample = (adc_buf[i + 2] - gMean) & 0xffff;
	  dfsdm_chydatinr =  (-dcFree_ADCSample) & 0xffff; // Phase 180: -x[n] + j0
	  *DFSDM_CH0DATINR = dfsdm_chydatinr; // load and trigger DFSDM by 32bit write

	  dcFree_ADCSample = (adc_buf[i + 3] - gMean) & 0xffff;
	  dfsdm_chydatinr =  dcFree_ADCSample << 16; // Phase 270: 0 + jx[n]
	  *DFSDM_CH0DATINR = dfsdm_chydatinr; // load and trigger DFSDM by 32bit write
  }
}

void feedSecondFilter(uint16_t startindex) {
	uint32_t secondCICin;

	for(uint16_t i=startindex; i< startindex+FILTER0_BUFLEN/2; i++) {
		secondCICin = (q31_filter0buf[i] & 0xffff0000) | (((uint32_t)q31_filter1buf[i] >> 16) & 0xffff);
		*DFSDM_CH2DATINR = secondCICin;
    }

}

void feedDACBuf(uint16_t startindex)
{
    static float32_t meanMag;
    static float32_t meanI;
    static float32_t meanQ;
    float32_t curMag;
    float32_t curMeanI;
    float32_t curMeanQ;
    const float32_t a = 1e-2;

    // determining mean I and Q values in output data of filter2:

	arm_q31_to_float(q31_filter2buf+startindex, f32_ibuf+startindex, FILTER2_BUFLEN/2);
	arm_q31_to_float(q31_filter3buf+startindex, f32_qbuf+startindex, FILTER2_BUFLEN/2);
	arm_mean_f32(f32_ibuf+startindex, FILTER2_BUFLEN/2, &curMeanI);
	arm_mean_f32(f32_qbuf+startindex, FILTER2_BUFLEN/2, &curMeanQ);

    meanI = (1-a) * meanI + a * curMeanI;
    meanQ = (1-a) * meanQ + a * curMeanQ;

	curMag = 0.0;

	// fill a complex data array in an interleaved fashion: (real, imag, real, imag, ...)
	for(uint16_t i=startindex; i< startindex+FILTER2_BUFLEN/2; i++) {
		cplx_buf[2*i] = f32_ibuf[i] - meanI;
		cplx_buf[2*i+1] = f32_qbuf[i] - meanQ;
	}
	arm_cmplx_mag_f32(cplx_buf+2*startindex, mag_buf+startindex, FILTER2_BUFLEN/2);

	//arm_offset_f32(mag_buf+startindex, -meanMag, mag_buf+startindex, FILTER2_BUFLEN/2);
	//arm_scale_f32(mag_buf+startindex, dac_gain * 32768.0, mag_buf+startindex, FILTER2_BUFLEN/2);

	arm_fir_f32(&fir_instance, mag_buf + startindex, filteredMag_buf + startindex, FILTER2_BUFLEN/2);

	arm_mean_f32(filteredMag_buf+startindex, FILTER2_BUFLEN/2, &curMag);
	meanMag = (1-a) * meanMag + a * curMag;

	for(uint16_t i=startindex; i< startindex+FILTER2_BUFLEN/2; i++) {
        dac_buf[i] = (filteredMag_buf[i] - meanMag) * dac_gain * 32768.0 + 32767;
//        dac_buf[i] = (mag_buf[i] - meanMag) * dac_gain * 32768.0 + 32767;
	}

}


/* ADC DMA interrupts ************************************************/
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
     adcBufDMAState = DMA_HALFCOMPLETE;
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
     adcBufDMAState = DMA_COMPLETE;
}

void HAL_ADC_ErrorCallback(ADC_HandleTypeDef *hadc)
{
     if (hadc->ErrorCode) {
	     ERRHND;
     }
}

/* DAC DMA interrupts ************************************************/
void HAL_DAC_ConvHalfCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
     dacBufDMAState = DMA_HALFCOMPLETE;
}

void HAL_DAC_ConvCpltCallbackCh1(DAC_HandleTypeDef *hdac)
{
     dacBufDMAState = DMA_COMPLETE;
}

/* DFSDM DMA interrupts ************************************************/
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	if (hdfsdm_filter == &hdfsdm1_filter0) {
		filter0BufDMAState = DMA_HALFCOMPLETE;
		return;
	}
	if (hdfsdm_filter == &hdfsdm1_filter1) {
		filter1BufDMAState = DMA_HALFCOMPLETE;
		return;
	}
	if (hdfsdm_filter == &hdfsdm1_filter2) {
		filter2BufDMAState = DMA_HALFCOMPLETE;
		return;
	}
	if (hdfsdm_filter == &hdfsdm1_filter3) {
		filter3BufDMAState = DMA_HALFCOMPLETE;
		return;
	}
	ERRHND;
}

void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
	if (hdfsdm_filter == &hdfsdm1_filter0) {
		filter0BufDMAState = DMA_COMPLETE;
		return;
	}
	if (hdfsdm_filter == &hdfsdm1_filter1) {
		filter1BufDMAState = DMA_COMPLETE;
		return;
	}
	if (hdfsdm_filter == &hdfsdm1_filter2) {
		filter2BufDMAState = DMA_COMPLETE;
		return;
	}
	if (hdfsdm_filter == &hdfsdm1_filter3) {
		filter3BufDMAState = DMA_COMPLETE;
		return;
	}
	ERRHND;
}


/* UART DMA interrupts ************************************************/
void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3) {
    	txBufDMAState = DMA_HALFCOMPLETE;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart3) {
    	txBufDMAState = DMA_COMPLETE;
    }
}

void delay_ns (uint16_t ns)  // TIM1 runs on 200MHz, so multiples of 5ns are possible
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < (ns/5));  // wait for the counter to reach the us input in the parameter
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
      while (1)
	  {}
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
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
