
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_hal.h"

/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
int i = 0;
int curr_data[16];
int prev_data[16];
int button_data[6][16];
int disp_data[16];
int cursor_data[16];
int read;
int tempo = 100;//beats per minute
int count;
int target;
int next_beat = 0;
int cursor_pos = 16;
int beat_num = 16;
int dispStringLen;
uint8_t commandBuf[2];
int clear_prev = 1;
int clear_curr = 1;
int pause_prev = 1;
int pause_curr = 1;
int pause = 0;
int clear = 0;
uint8_t commandBufparam[3];
char tempoStr[8];
int up_prev = 1;
int up_curr = 1;
int down_prev = 1;
int down_curr = 1;
int up = 0;
int down = 0;
int state = 1; //sounds
int right_prev = 1;
int right_curr = 1;
int left_prev = 1;
int left_curr = 1;
int right = 0;
int left = 0;

// Audio
int sample_audio = 0; //flag to indicate to main that TIM14 was triggered
int outputting_audio = 0; //flag to indicate to ISR that audio is currently being output
int audio_idx = 0; //current position in audio output array
extern uint8_t sound1[], sound2[], sound3[], sound4[], sound5[], sound6[];
int dac_value = 0;
int lengths[] = {5288, 9911, 3233, 9623, 11910, 6821};//{2821, 4926, 5006, 5743, 7485, 3828};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI1_Init(void);
static void MX_DAC1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void GPIOBtoggle(uint16_t pinNum);
void displayString(char dispStr[]);
void clearDisplay(void);
void setCursor(uint8_t pos);
void cursorReturn(void);
/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
void GPIOBtoggle(uint16_t pinNum)
{
	HAL_GPIO_WritePin(GPIOB, pinNum, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOB, pinNum, GPIO_PIN_RESET);
}

void displayString(char dispStr[])
{

	dispStringLen = strlen(dispStr);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);		//Drive slave select low (select)
	HAL_SPI_Transmit(&hspi1, (uint8_t *) dispStr, dispStringLen, 100);				//transmit character in dispCharBuf
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);			//Drive slave select high (deselect)
}

void clearDisplay(void)
{
	commandBuf[0] = 0xFE;	//set prefix
	commandBuf[1] = 0x51;	//command value
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);		//Drive slave select low (select)
	HAL_SPI_Transmit(&hspi1, commandBuf, 2, 100);				//transmit character in dispCharBuf
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);			//Drive slave select high (deselect)
}

void setCursor(uint8_t pos)
{
	commandBufparam[0] = 0xFE;	//set prefix
	commandBufparam[1] = 0x45;	//command value
	commandBufparam[2] = pos;	//load position value
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);		//Drive slave select low (select)
	HAL_SPI_Transmit(&hspi1, commandBufparam, 3, 100);				//transmit character in dispCharBuf
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);			//Drive slave select high (deselect)
}

void cursorReturn(void)
{
	commandBufparam[0] = 0xFE;	//set prefix
	commandBufparam[1] = 0x45;	//command value
	commandBufparam[2] = 0x00;	//load home position value
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_RESET);		//Drive slave select low (select)
	HAL_SPI_Transmit(&hspi1, commandBufparam, 3, 100);				//transmit character in dispCharBuf
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);			//Drive slave select high (deselect)
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim->Instance == TIM2) {
		read = 1;
		if(!pause)
		{
			count++;
			if(count == target)
			{
				count = 0;
				next_beat = 1;
			}
		}
	} else if (htim->Instance == TIM14) {
		sample_audio = 1;
		if (outputting_audio) { //Changed
			dac_value = 0; //TODO - default output 85?
			for (int i = 0; i < 6; i++) {
				int current;
				if (cursor_pos == beat_num) {
					current = 0;
				} else {
					current = cursor_pos;
				}
				if (button_data[i][current] == 1) {
					switch (i) {
					case 0:
						if (audio_idx < lengths[0]) {
							dac_value += sound1[audio_idx] * 4;
						} else { //TODO
							outputting_audio = 0;
						}
						break;
					case 1:
						if (audio_idx < lengths[1]) {
							dac_value += sound2[audio_idx] * 4;
						} else {
							outputting_audio = 0;
						}
						break;
					case 2:
						if (audio_idx < lengths[2]) {
							dac_value += sound3[audio_idx] * 4;
						} else {
							outputting_audio = 0;
						}
						break;
					case 3:
						if (audio_idx < lengths[3]) {
							dac_value += sound4[audio_idx] * 4;
						} else {
							outputting_audio = 0;
						}
						break;
					case 4:
						if (audio_idx < lengths[4]) {
							dac_value += sound5[audio_idx] * 4;
						} else {
							outputting_audio = 0;
						}
						break;
					case 5:
						if (audio_idx < lengths[5]) {
							dac_value += sound6[audio_idx] * 4;
						} else {
							outputting_audio = 0;
						}
						break;
					}
				}
			}
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_value);
			audio_idx++;
		} else {
			HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_TIM2_Init();
  MX_SPI1_Init();
  MX_DAC1_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim14);
  HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
  target = 1000 / (2 * tempo) * 60;
  /*HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);//make sure ldbar is high
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);//make sure srclear bar is high
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);//make sure ser is low, timing diagram said this was correct
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);//make sure clk is low
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);//clear shift register
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);*/
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3, GPIO_PIN_SET);			//Drive slave select high (deselect)
  GPIOBtoggle(GPIO_PIN_2);//output clear
  for(i = 0; i < 16; i++)
  {
	  prev_data[i] = 0;
	  for (int j = 0; j < 6; j++) {
		  button_data[j][i] = 0;
	  }
	  cursor_data[i] = 0;
	  disp_data[i] = 0;
  }



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  clearDisplay();
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  target = 1000 / (2 * tempo) * 60;
	  cursorReturn();
	  displayString("Sound: ");
	  if(state == 1)
	  {
		  displayString("Tom");
	  }
	  else if(state == 2)
	  {
		  displayString("Snare");
	  }
	  else if(state == 3)
	  {
		  displayString("Bass");
	  }
	  else if(state == 4)
	  {
		  displayString("Kick");
	  }
	  else if(state == 5)
	  {
		  displayString("Hi-Hat");
	  }
	  else if(state == 6)
	  {
		  displayString("Clap");
	  }
	  displayString("   ");
	  setCursor(0x40);
	  displayString("Tempo: ");
	  displayString(itoa(tempo, tempoStr, 10));
	  displayString(" ");
	  if(read)
	  {
		  read = 0;				//clear flag
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);	//load in current data values
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);		//Back high again to not load
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_RESET);		//not inhibiting serial data transfer
		  for(i = 0; i < 8; i++)
	  	  {
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);	//clk to shift value to output
			  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_RESET);	//back low again
			  curr_data[i] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9); //question - why array?
			  if (prev_data[i] == 0 && curr_data[i] == 1) { //if pressed
				  if (button_data[state-1][i] == 0) {
					  button_data[state-1][i] = 1;
				  } else {
					  button_data[state-1][i] = 0;
				  }
			  }
			  prev_data[i] = curr_data[i];
	  	  }
		  for(i = 8; i < 16; i++)
	  	  {
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);	//clk to shift value to output
			  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);	//back low again
			  curr_data[i] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_9); //question - why array?
			  if (prev_data[i] == 0 && curr_data[i] == 1) { //if pressed
				  if (button_data[state-1][i] == 0) {
					  button_data[state-1][i] = 1;
				  } else {
					  button_data[state-1][i] = 0;
				  }
			  }
			  prev_data[i] = curr_data[i];
	  	  }
		  //Auxiliary buttons
		  pause_curr = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);
		  if(pause_curr == 1 && pause_prev == 0)
		  {
			  pause = !pause;
		  }
		  pause_prev = pause_curr;
		  clear_curr = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_0);
		  if(clear_curr == 1 && clear_prev == 0)
		  {
			  clear = 1;
		  }
		  clear_prev = clear_curr;
		  up_curr = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_1);
		  if(up_curr == 1 && up_prev == 0)
		  {
			  up = 1;
		  }
		  up_prev = up_curr;
		  down_curr = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
		  if(down_curr == 1 && down_prev == 0)
		  {
			  down = 1;
		  }
		  down_prev = down_curr;
		  right_curr = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_4);
		  if(right_curr == 1 && right_prev == 0)
		  {
			  right = 1;
		  }
		  right_prev = right_curr;
		  left_curr = HAL_GPIO_ReadPin(GPIOF, GPIO_PIN_5);
		  if(left_curr == 1 && left_prev == 0)
		  {
			  left = 1;
		  }
		  left_prev = left_curr;
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_6, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	  }
	  if(next_beat)
	  {
		  // note: current position is cursor_pos - 1
		  next_beat = 0;
		  if(cursor_pos == beat_num) //special case: cursor #16
		  {

			  cursor_data[0] = 0; //previous (0) off
		  }
		  else
		  {
			  cursor_data[cursor_pos] = 0; //previous (cursor - 1 + 1) off
		  }
		  cursor_data[cursor_pos-1] = 1; //turn on current one
		  cursor_pos--; //decrement position
		  if(cursor_pos < 1)
		  {
			  cursor_pos = 16;
		  }
		  // go thru all 6 states. if ANY are 1, outputting is on and also make array to show which are on
		  outputting_audio = 0; //TODO - is this ok? just want it 0 when none of them are on.
		  for (int j = 0; j < 6; j++) { //SUSPECT LINES
			  if (cursor_pos == beat_num) {
				  if (button_data[j][0] == 1) {
					  outputting_audio = 1;
				  }
			  } else {
				  if (button_data[j][cursor_pos] == 1) {
					  outputting_audio = 1;
				  }
			  }
		  }
		  audio_idx = 0; //restart audio - Changed, minimally
	  }
	  if(up)
	  {
		  up = 0;
		  tempo++;
	  }
	  if(down)
	  {
		  down = 0;
		  tempo--;
	  }
	  if(right)
	  {
		  right = 0;
		  if(state == 6)
		  {
			  state = 1;
		  }
		  else
		  {
			  state++;
		  }
	  }
	  if(left)
	  {
		  left = 0;
		  if(state == 1)
		  {
			  state = 6;
		  }
		  else
		  {
			  state--;
		  }
	  }
	  if(clear)
	  {
		  clear = 0;
		  for(i = 0; i < 16; i++)
		    {
		  	  prev_data[i] = 0;
		  	  for (int j = 0; j < 6; j++) {
		  		  button_data[j][i] = 0;
		  	  }
		  	  cursor_data[i] = 0;
		  	  disp_data[i] = 0;
		    }
		  cursor_pos = 16;
	  }
	  for(i = 0; i < 8; i++)
	  {
		  disp_data[i] =  button_data[state-1][i] || cursor_data[i];
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, disp_data[i]);
		  GPIOBtoggle(GPIO_PIN_1);
	  }
	  for(i = 8; i < 16; i++)
	  {
		  disp_data[i] =  button_data[state-1][i] || cursor_data[i];
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, disp_data[i]);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, GPIO_PIN_RESET);
	  }
	  GPIOBtoggle(GPIO_PIN_2);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL12;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DAC1 init function */
static void MX_DAC1_Init(void)
{

  DAC_ChannelConfTypeDef sConfig;

    /**DAC Initialization 
    */
  hdac1.Instance = DAC;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**DAC channel OUT1 config 
    */
  sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24000;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 0;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM14 init function */
static void MX_TIM14_Init(void)
{

  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 546;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 1;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC0 PC1 PC2 
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2 
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC4 PC5 PC6 
                           PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PF4 PF5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 PB10 
                           PB11 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10 
                          |GPIO_PIN_11|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
