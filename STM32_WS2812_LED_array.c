/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union{
  struct{
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NEOPIXEL_ZERO 26 // (ARR+1)0.32 rounded. Changing pulse register (CCRx)
#define NEOPIXEL_ONE 51 // (ARR+1)0.64 rounded. Changing pulse register (CCRx)
#define NUM_PIXELS 256
#define BITS_PER_PIXEL 24 // 8 per colour
#define DMA_BUFF_SIZE (NUM_PIXELS * BITS_PER_PIXEL) + 1 // +1 for reset

#define wavelength 2 //cm
#define separation 1 //cm
#define brightness 25 //max 255
#define variance 200 //variance of Gaussian used for array tapering

#define steps 180
#define theta_initial 15
#define theta_final 15
#define theta_step (theta_final-theta_initial)/(steps-1)
#define phi_initial 0
#define phi_final 360
#define phi_step (phi_final-phi_initial)/(steps-1)
#define rest 50 //delay between updates in milliseconds

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
#if defined ( __ICCARM__ ) /*!< IAR Compiler */
#pragma location=0x30000000
ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
#pragma location=0x30000200
ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __CC_ARM )  /* MDK ARM Compiler */

__attribute__((at(0x30000000))) ETH_DMADescTypeDef  DMARxDscrTab[ETH_RX_DESC_CNT]; /* Ethernet Rx DMA Descriptors */
__attribute__((at(0x30000200))) ETH_DMADescTypeDef  DMATxDscrTab[ETH_TX_DESC_CNT]; /* Ethernet Tx DMA Descriptors */

#elif defined ( __GNUC__ ) /* GNU Compiler */
ETH_DMADescTypeDef DMARxDscrTab[ETH_RX_DESC_CNT] __attribute__((section(".RxDecripSection"))); /* Ethernet Rx DMA Descriptors */
ETH_DMADescTypeDef DMATxDscrTab[ETH_TX_DESC_CNT] __attribute__((section(".TxDecripSection")));   /* Ethernet Tx DMA Descriptors */

#endif

ETH_TxPacketConfig TxConfig;

ETH_HandleTypeDef heth;

TIM_HandleTypeDef htim3;
DMA_HandleTypeDef hdma_tim3_ch3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ETH_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_DMA_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim){
  HAL_TIM_PWM_Stop_DMA(&htim3, TIM_CHANNEL_3);}

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
  MX_USB_OTG_FS_PCD_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  int i,j,k=0;
  float angles[steps][2]={0};
  for (i=0; i<steps; i++){
      angles[i][0]=theta_initial+i*theta_step;}
  for (j=0; j<steps; j++){
      angles[j][1]=phi_initial+j*phi_step;}

  void pathDifference(float phaseStep[2],float theta,float phi){
	  float alpha=atan2(1/tanf(theta*(M_PI/180.0f)),cosf(phi*(M_PI/180.0f))); //angle with x-axis after projection in x-z plane
	  float beta=atan2(1/tanf(theta*(M_PI/180.0f)),sinf(phi*(M_PI/180.0f))); //angle with y-axis after projection in y-z plane
      float dx=(separation*cosf(alpha)/wavelength); //phase step between adjacent elements in the x direction
      float dy=(separation*cosf(beta)/wavelength); //phase step between adjacent elements in the y direction
      phaseStep[0]=dx;
      phaseStep[1]=dy;}

    void fillArray(float data[16][16],float theta,float phi){
    	float phaseStep[2]={0};
    	pathDifference(phaseStep,theta,phi);
    	float dx=phaseStep[0];
    	float dy=phaseStep[1];
    	for (i=0; i<16; i++){ //loop for x coordinate
    		for (j=0; j<16; j++){ //loop for y coordinate
    			float phase=-(i-7.5f)*dx-(j-7.5f)*dy; //calculate phase relative to the phase center at (7.5,7.5)
                phase=fmod(phase,1); //scale between -1 and 1 (corresponding to -360 -> 360)
                if (phase>0.5){phase--;} //scale between -0.5 and 0.5 (corresponding to -180 -> 180)
                if (phase<-0.5){phase++;}
                data[i][j]=phase + 0.5f;}}}; //scale between 0 and 1 (corresponding to -180 -> 180)

    void fillList(float scaledPhase[], float scaledGain[], float theta, float phi){ //scale the phase of each pixel to be between 0 and 1
    	float data[16][16]={0};
        fillArray(data,theta,phi);
        for (i=0; i<256; i++){
           int x=i%16; //translate coordinates
           int y=(i-x)/16;
           if (y%2==1){x=15-x;} //pixel indexing snakes back and forth so x values must be flipped every other row
           scaledGain[i]=expf(-(pow(x-7.5,2)+pow(y-7.5,2))/(2*variance)); //Taper array using Gaussian distribution
           scaledPhase[i]=data[x][y];}}

    void fillBuffer(PixelRGB_t pixel[], uint32_t dmaBuffer[],float theta,float phi){ //fill the dmaBuffer with data
    	float scaledGain[NUM_PIXELS] = {0};
    	float scaledPhase[NUM_PIXELS] = {0};
    	fillList(scaledPhase,scaledGain,theta,phi);
        for (i=0; i<NUM_PIXELS; i++){ //loop through 256 pixels
        	float A=scaledPhase[i];
            if (A<0.5){ //colour goes from green to red as phase gets more negative (down to -180 degrees)
            	pixel[i].color.g=brightness*scaledGain[i]*2*A;
            	pixel[i].color.r=brightness*scaledGain[i]*(1-2*A);;
            	pixel[i].color.b=0;}
            else{ //colour goes from green to blue as phase gets more positive(up to 180 degrees)
            	pixel[i].color.g=brightness*scaledGain[i]*(2-2*A);
            	pixel[i].color.r=0;
            	pixel[i].color.b=brightness*scaledGain[i]*(2*A-1);}
    		pixel[i].data=( pixel[i].color.g<<16u | pixel[i].color.r<<8u | pixel[i].color.b ); //combine 8 bit data for each colour into 24 bit data for each pixel
    		for (j = 23; j >= 0; j--){ //loop 24 bits per LED
    			if ((pixel[i].data >> j) & 1u){ //check if data is zero or one
    				dmaBuffer[(i*24)+(23-j)] = NEOPIXEL_ONE;} //if data is one, fill buffer with one code
    	        else{
    	        	dmaBuffer[(i*24)+(23-j)] = NEOPIXEL_ZERO;}}} //if data is zero, fill buffer with zero code
        dmaBuffer[DMA_BUFF_SIZE - 1] = 0;} //very important that last element is 0

    void light(float theta, float phi){ //send data
    	PixelRGB_t pixel[NUM_PIXELS] = {0}; //array of 256 pixels
        uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0}; //array of 6145 bits
        fillBuffer(pixel,dmaBuffer,theta,phi);
  	    HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t)dmaBuffer, (uint16_t)DMA_BUFF_SIZE);}

    //light(10,45); //static display

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  light(angles[k][0],angles[k][1]); //updating display
	  k++;
	  k=k%(steps-1);
	  HAL_Delay(rest);

  }
  /* USER CODE END 3 */
}
