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

#define DATA_SIZE 16
#define SIG_SIZE (DATA_SIZE * NUM_PIXELS)

#define brightness 25 //max 255

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

	uint32_t signal[SIG_SIZE]={0};
	uint32_t signal1[SIG_SIZE]={0};
	//1st 16 bits of signal are for the 0th element of the 0th chip. Next 16 bits are for the 1st element of the 0th chip. etc etc

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

	int i,j,k,x,y;
	int l=0;

	for (i=0;i<4096;i++){
	    if (i%2==0){signal[i]=0;}
	    else{signal[i]=1;}} //binary signal intended for renesas chip

	for (i=0;i<4096;i++){
		signal1[i]=1;}

	// _p for phase and _g for gain

	void sig2chip(float renesas_chip_p[64][4], float renesas_chip_g[64][4], uint32_t signal[SIG_SIZE]){        //raw signal to renesas chip indexing
	    for (i=0;i<256;i++){
	        int index=i%4; //find the element of the chip from 0 to 3
	        int chip=(i-index)/4; //find the chip it is from 0 to 63
	        float data_p[6]={0}; //6 bits of phase data
	        float data_g[7]={0}; //7 bits of gain data (VGA stage)
	        for (x=0;x<6;x++){
	        	data_p[x]=signal[16*i+x];} //fill a sub array with phase data
	        for (y=0;y<7;y++){
	        	data_g[y]=signal[16*i+8+y];} //fill a sub array with gain data
	        float phase=0;
	        float gain=0;
	        for (j=0;j<6;j++){
	            phase=phase+(data_p[5-j])*pow(2,j);} //binary -> decimal
	        phase=(phase/63); //scale between 0 and 1 (corresponding to 0 -> 360)
	        if (phase>0.5){phase--;} //scale between -0.5 and 0.5 (corresponding to -180 -> 180)
	        renesas_chip_p[chip][index]=phase+0.5; //scale between 0 and 1 (corresponding to -180 -> 180)
	        for (k=0;k<7;k++){
	            gain=gain+data_g[6-k]*pow(2,k);} //binary -> decimal
	        gain=gain/127; //scale between 0 and 1 (corresponding to 1.6dB -> 27dB)
	        renesas_chip_g[chip][index]=gain;}}

	void chip2grid(float renesas_grid_p[16][16], float renesas_grid_g[16][16], uint32_t signal[SIG_SIZE]){     //translate from renesas chip indexing to x/y coordinate labels
	      float renesas_chip_p[64][4]; //64x4 array (64 chips each with 4 elements)
	      float renesas_chip_g[64][4]; //64x4 array (64 chips each with 4 elements)
	      sig2chip(renesas_chip_p,renesas_chip_g,signal);
		  for (x=0; x<16; x++){
			  for (y=0; y<16; y++){
				  int a=(x - x%2)/2 + (y - y%2)/2; //find the chip it is from 0 to 63
				  int b;
				  if (x%2==0 && y%2==0){b=0;}
				  if (x%2==0 && y%2==1){b=1;}
				  if (x%2==1 && y%2==1){b=2;}
				  if (x%2==1 && y%2==0){b=3;} //find the element of the chip from 0 to 3
				  renesas_grid_p[x][y]=renesas_chip_p[a][b]; //map to new indexing
	              renesas_grid_g[x][y]=renesas_chip_g[a][b];}}} //map to new indexing

	void grid2list(float renesas_list_p[256], float renesas_list_g[256], uint32_t signal[SIG_SIZE]){ //translate from x/y coordinates to 1d list (that snakes along)
	    float renesas_grid_p[16][16]; //16x16 array (16 rows and 16 columns)
	    float renesas_grid_g[16][16]; //16x16 array (16 rows and 16 columns)
	    chip2grid(renesas_grid_p, renesas_grid_g, signal);
	    for (i=0; i<256; i++){
	        int x=i%16; //find the x coordinate of the pixel
	        int y=(i-x)/16; //find the y coordinate of the pixel
	        if (y%2==1){x=15-x;} //account for the pixels snaking back and forth
	        renesas_list_p[i]=renesas_grid_p[x][y];
	        renesas_list_g[i]=renesas_grid_g[x][y];}}

  void fillBuffer(PixelRGB_t pixel[], uint32_t dmaBuffer[], uint32_t signal[SIG_SIZE]){ //fill the dmaBuffer with data
  	float renesas_list_p[256] = {0}; //256x1 list (256 pixels)
  	float renesas_list_g[256] = {0}; //256x1 list (256 pixels)
  	grid2list(renesas_list_p,renesas_list_g, signal);
      for (i=0; i<256; i++){ //loop through 256 pixels
      	float A=renesas_list_p[i];
          if (A<0.5){ //colour goes from green to red as phase gets more negative (down to -180 degrees)
          	pixel[i].color.g=brightness*renesas_list_g[i]*2*A;
          	pixel[i].color.r=brightness*renesas_list_g[i]*(1-2*A);;
          	pixel[i].color.b=0;}
          else{ //colour goes from green to blue as phase gets more positive(up to 180 degrees)
          	pixel[i].color.g=brightness*renesas_list_g[i]*(2-2*A);
          	pixel[i].color.r=0;
          	pixel[i].color.b=brightness*renesas_list_g[i]*(2*A-1);}
  		pixel[i].data=( pixel[i].color.g<<16u | pixel[i].color.r<<8u | pixel[i].color.b ); //combine 8 bit data for each colour into 24 bit data for each pixel
  		for (j = 23; j >= 0; j--){ //loop 24 bits per LED
  			if ((pixel[i].data >> j) & 1u){ //check if data is zero or one
  				dmaBuffer[(i*24)+(23-j)] = NEOPIXEL_ONE;} //if data is one, fill buffer with one code
  	        else{
  	        	dmaBuffer[(i*24)+(23-j)] = NEOPIXEL_ZERO;}}} //if data is zero, fill buffer with zero code
      dmaBuffer[DMA_BUFF_SIZE - 1] = 0;} //very important that last element is 0

  void light(uint32_t signal[SIG_SIZE]){ //send data
  	PixelRGB_t pixel[NUM_PIXELS] = {0}; //array of 256 pixels
    uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0}; //array of 6145 bits
    fillBuffer(pixel,dmaBuffer,signal);
	HAL_TIM_PWM_Start_DMA(&htim3, TIM_CHANNEL_3, (uint32_t)dmaBuffer, (uint16_t)DMA_BUFF_SIZE);}

  //light(signal);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  if (l%2==0){light(signal);}
	  else{light(signal1);}
	  HAL_Delay(1000);
	  l++;

  }
  /* USER CODE END 3 */
}