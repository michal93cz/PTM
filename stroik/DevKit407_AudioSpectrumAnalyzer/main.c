/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include "arm_math.h"               // Required to use float32_t type
#include "main.h"
#include "pdm_filter.h"
#include "stm32f4_discovery.h"
#include "stm32f4xx.h"
//inne
#include "defines.h"
#include "stm32f4xx.h"
#include "tm_stm32f4_pcd8544.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
uint8_t  PDM_Input_Buffer[PDM_Input_Buffer_SIZE];
uint16_t PCM_Output_Buffer[PCM_Output_Buffer_SIZE];

float32_t buffer_input[BUFFER_size];
float32_t buffer_input_copy[QUATER_Buffer];
float32_t buffer_output[BUFFER_size];
float32_t buffer_output_mag[BUFFER_size];
float32_t buffer_output_mag_copy[QUATER_Buffer];
float32_t maxvalue;
float32_t czestotliwosc;
uint32_t  maxvalueindex;


char      text[100];

arm_rfft_instance_f32 S;
arm_cfft_radix4_instance_f32 S_CFFT;
PDMFilter_InitStruct Filter;

/* Private function prototypes -----------------------------------------------*/
static void GPIO_Configure(void);
static void I2S_Configure(void);
static void NVIC_Configure(void);
static void RCC_Configure(void);
static void led_init();

void E();
void C();
void D();
void F();
void G();
void H();
void reset_lines();
/* Functions -----------------------------------------------------------------*/
int main(void)
{
	//180MHz
		  	SystemInit();

		  	//Initialize LCD with 0x38 software contrast
		  	PCD8544_Init(0x38);

		  	//Go to x=14, y=3 position
		  	PCD8544_GotoXY(14, 3);

		  	//PCD8544_Puts("E",PCD8544_Pixel_Set, PCD8544_FontSize_5x7);

  extern uint32_t Data_Status;

  unsigned int freq, i, z;
  float dt;

  RCC_Configure();
  NVIC_Configure();

  GPIO_Configure();
  PCD8544_Refresh();
  I2S_Configure();

  led_init();

  // Initialize PDM filter
  Filter.Fs = OUT_FREQ;
  Filter.HP_HZ = 10;
  Filter.LP_HZ = 16000;
  Filter.In_MicChannels = 1;
  Filter.Out_MicChannels = 1;
  PDM_Filter_Init(&Filter);

  arm_rfft_init_f32(&S, &S_CFFT, HALF_Buffer, 0, 1);


  while(1){



    // Clear arrays
    for(i=0; i<QUATER_Buffer; ++i){
      buffer_input_copy[i] = 0;
      buffer_output_mag_copy[i] = 0;
    }

    z = 0;



    I2S_Cmd(SPI2, ENABLE);

    int zamek = 1;
    while(1){


      if(Data_Status){
        for(i=0; i<(OUT_FREQ/1000); i++){
          buffer_input[i+(OUT_FREQ/1000)*z] = (float32_t) PCM_Output_Buffer[i];
        }

        ++z;
        if(z > HALF_Buffer/(OUT_FREQ/1000)){
          z = 0;

          // ************************************************************


          // ************************************************************
          // Calculate Real FFT
          arm_rfft_f32(&S, buffer_input, buffer_output);
          // Calculate magnitude
          arm_cmplx_mag_f32(buffer_output, buffer_output_mag, HALF_Buffer);
          // Get maximum value of magnitude
          arm_max_f32(&(buffer_output_mag[1]), HALF_Buffer, &maxvalue, &maxvalueindex);
          // Scale magnitude values
          for(i=0; i<HALF_Buffer; ++i){
            //buffer_output_mag[i] = 100*buffer_output_mag[i]/maxvalue;
            buffer_output_mag[i+1] = 140*buffer_output_mag[i+1]/20000000;
          }

          // ************************************************************

        }
        Data_Status = 0;
      }

      czestotliwosc=(maxvalueindex+1)*OUT_FREQ/HALF_Buffer;

      if(czestotliwosc>327 && czestotliwosc<329)
      {
    	  GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    	  GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    	  GPIO_SetBits(GPIOD, GPIO_Pin_12);
    	  E();
    	  PCD8544_Refresh();
      }
      else
      {
    	  GPIO_ResetBits(GPIOD, GPIO_Pin_13);
    	  GPIO_ResetBits(GPIOD, GPIO_Pin_15);
    	  GPIO_ResetBits(GPIOD, GPIO_Pin_12);
    	  if(czestotliwosc<=327) GPIO_SetBits(GPIOD, GPIO_Pin_15);
    	  if(czestotliwosc>=329) GPIO_SetBits(GPIOD, GPIO_Pin_13);
    	  reset_lines();
    	  PCD8544_Refresh();
      }
      //PCD8544_Refresh();

      switch((int)czestotliwosc)
          	{
          		case 328:	// E
          		{
          			//PCD8544_Puts("E",PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
          			//PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Set);
          			//E();
          			break;
          		}
          		case 2: // D
          		{
          			break;
          		}
          		case 3: // E
          		{
          			break;
          		}
          		case 4:	// F
          		{
          			break;
          		}
          		case 5: // G
          		{
          			break;
          		}
          		case 6: // H
          		{
          			break;
          		}
          		default:
          		{
          			//PCD8544_Puts("brak sygnalu", PCD8544_Pixel_Set, PCD8544_FontSize_5x7);
          			zamek = 1;
          			break;
          		}

          	}

      // Wait some time
      //for(i=0; i<0x10000; ++i);
    }

    I2S_Cmd(SPI2, DISABLE);
  }
}
//funkcje do obslugi wyswietlacza
void reset_lines()
{
	PCD8544_DrawLine(6,18,14,18,PCD8544_Pixel_Clear); //srodek poziom
	PCD8544_DrawLine(5,25,15,25,PCD8544_Pixel_Clear); //dol poziom
	PCD8544_DrawLine(5,11,15,11,PCD8544_Pixel_Clear); //gora poziom
	PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Clear); //lewa cala pion
	PCD8544_DrawLine(15,18,15,24,PCD8544_Pixel_Clear); //prawa dol pion
	PCD8544_DrawLine(15,12,15,24,PCD8544_Pixel_Clear); //prawa cala pion
}
//litery
void C()
{
	PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Set); //lewa cala pion
	PCD8544_DrawLine(5,25,15,25,PCD8544_Pixel_Set); //dol poziom
	PCD8544_DrawLine(5,11,15,11,PCD8544_Pixel_Set); //gora poziom
}
void D()
{
	PCD8544_DrawLine(15,12,15,24,PCD8544_Pixel_Set); //prawa cala pion
	PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Set); //lewa cala pion
	PCD8544_DrawLine(5,25,15,25,PCD8544_Pixel_Set); //dol poziom
	PCD8544_DrawLine(5,11,15,11,PCD8544_Pixel_Set); //gora poziom
}
void E()
{
	PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Set); //lewa cala pion
	PCD8544_DrawLine(6,18,14,18,PCD8544_Pixel_Set); //srodek poziom
	PCD8544_DrawLine(5,25,15,25,PCD8544_Pixel_Set); //dol poziom
	PCD8544_DrawLine(5,11,15,11,PCD8544_Pixel_Set); //gora poziom
}
void F()
{
	PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Set); //lewa cala pion
	PCD8544_DrawLine(6,18,14,18,PCD8544_Pixel_Set); //srodek poziom
	PCD8544_DrawLine(5,11,15,11,PCD8544_Pixel_Set); //gora poziom
}
void G()
{
	PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Set); //lewa cala pion
	PCD8544_DrawLine(15,18,15,24,PCD8544_Pixel_Set); //prawa dol pion
	PCD8544_DrawLine(5,25,15,25,PCD8544_Pixel_Set); //dol poziom
	PCD8544_DrawLine(5,11,15,11,PCD8544_Pixel_Set); //gora poziom
}
void H()
{
	PCD8544_DrawLine(5,12,5,24,PCD8544_Pixel_Set); //lewa cala pion
	PCD8544_DrawLine(15,12,15,24,PCD8544_Pixel_Set); //prawa cala pion
	PCD8544_DrawLine(6,18,14,18,PCD8544_Pixel_Set); //srodek poziom
}


/* Private functions ---------------------------------------------------------*/
static void GPIO_Configure(void){
  GPIO_InitTypeDef GPIO_InitStructure;

  // Configure MP45DT02's CLK / I2S2_CLK (PB10) line
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_10;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  // Configure MP45DT02's DOUT / I2S2_DATA (PC3) line
  GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_3;
  GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
  GPIO_Init(GPIOC, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOB, GPIO_PinSource10, GPIO_AF_SPI2);  // Connect pin 10 of port B to the SPI2 peripheral
  GPIO_PinAFConfig(GPIOC, GPIO_PinSource3, GPIO_AF_SPI2);   // Connect pin 3 of port C to the SPI2 peripheral
}

static void led_init()
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

	GPIO_InitTypeDef GPIO_InitStructure;
	/* Configure PD12, PD13, PD14 and PD15 in output pushpull mode */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13| GPIO_Pin_14| GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOD, &GPIO_InitStructure);
}

static void I2S_Configure(void){
  I2S_InitTypeDef I2S_InitStructure;

  SPI_I2S_DeInit(SPI2);
  I2S_InitStructure.I2S_AudioFreq = OUT_FREQ*2;
  I2S_InitStructure.I2S_Standard = I2S_Standard_LSB;
  I2S_InitStructure.I2S_DataFormat = I2S_DataFormat_16b;
  I2S_InitStructure.I2S_CPOL = I2S_CPOL_High;
  I2S_InitStructure.I2S_Mode = I2S_Mode_MasterRx;
  I2S_InitStructure.I2S_MCLKOutput = I2S_MCLKOutput_Disable;
  I2S_Init(SPI2, &I2S_InitStructure);

  // Enable the Rx buffer not empty interrupt
  SPI_I2S_ITConfig(SPI2, SPI_I2S_IT_RXNE, ENABLE);
}

static void NVIC_Configure(void){
  NVIC_InitTypeDef NVIC_InitStructure;

  // Configure the interrupt priority grouping
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);

  // Configure the SPI2 interrupt channel
  NVIC_InitStructure.NVIC_IRQChannel = SPI2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_Init(&NVIC_InitStructure);
}

static void RCC_Configure(void){
  // Enable CRC module - required by PDM Library
  //RCC->AHB1ENR |= RCC_AHB1ENR_CRCEN;

  /********/
  /* AHB1 */
  /********/
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_GPIOC |
                         RCC_AHB1Periph_CRC, ENABLE);

  /********/
  /* APB1 */
  /********/
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);

  RCC_PLLI2SCmd(ENABLE);
}
