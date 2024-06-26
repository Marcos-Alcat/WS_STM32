/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * Ref 1: https://microcontrollerslab.com/stm32-blue-pill-adc-polling-interrupt-dma/
  * Ref 2: https://github.com/fabimass/stm32f103-freertos/tree/main/adc/adc_isr
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
QueueHandle_t queue_ADC, queue_PA, queue_CP, queue_AM;
SemaphoreHandle_t ADC_semph;

//para cambio de prioridades:
TaskHandle_t xTarea_Config_Handle = NULL, xTarea_Pantalla_Handle=NULL, xTarea_Medir_Handle = NULL;

//var globales para usar live expretion//
int add_tiempo = 0;
int tarea = 0;
int cola_max = 10;
int cola_min = 0;
uint16_t max = 70, min = 20; //xq pinto asi....
//var globales para usar live expretion//


#define THRESHOLD_VALUE 2048

#define BTN_USER_UP 2750	  //presiono boton UP valor ADC aprox: 2750.
#define BTN_USER_DOWN 2570    //presiono boton DOWN valor ADC aprox: 2574.
//	BTN_USER_UP - BTN_USER_DOWN = DIDERENCIA(180). Es la diferencia entre ellos.
//  Damos margen de 60 y queda: DIDERENCIA(180) -60 --> 120.. dividido 2 --> 60... asi que.
//  BTN_USER_UP - 60   = 2690     y   BTN_USER_UP + 60   = 2810
//  BTN_USER_DOWN + 60 = 2630     y   BTN_USER_DOWN - 60 = 2510
#define BTN_USER_UP_INF  BTN_USER_UP - 60
#define BTN_USER_UP_SUP  BTN_USER_UP + 60
#define BTN_USER_DOWN_INF  BTN_USER_DOWN - 60
#define BTN_USER_DOWN_SUP  BTN_USER_DOWN + 60
#define BTN_USER_OK 2100      //presiono dos botones valor ADC aprox: 1981.


static void Pantalla(void *pvParameters){
	uint16_t display_value;
	unsigned portBASE_TYPE uxPriority;
	uxPriority = NULL;
	while (1){
		//Consulta la prioridad a la que esta tarea está en ejecución.
		uxPriority = uxTaskPriorityGet( NULL );
		if(uxPriority==3)
		xQueuePeek(queue_CP,&display_value,portMAX_DELAY);//recibe valor que esta configurando tarea Config. Usa Peek para no eliminar y terminar en bloqueo.
		else if(uxPriority==2){
		xQueueReceive(queue_PA,&display_value,portMAX_DELAY);//recibe valor medido por sensor.
		}
		tm1637_ShowNumber(display_value);
		/*
		 * La función HAL-ADC-Start-IT() es responsable de permitir la interrupción y inicio de la conversión de ADC de los canales regulares.
		 * Toma en un solo parámetro que es el puntero de la estructura ADC-HandleTypeDef que contiene los parámetros de configuración para el
		 * ADC especificado. En nuestro caso es "&hadc1.
		 */
		HAL_ADC_Start_IT(&hadc1);
	}
}


static void Medir(void *pvParameters){
	uint32_t distancia = 0, pMillis, val1 = 0, val2 = 0;
	while(1){
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_SET);
		__HAL_TIM_SET_COUNTER(&htim2, 0);
		while (__HAL_TIM_GET_COUNTER (&htim2) < 10);  // wait for 10 us
		HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

		pMillis = HAL_GetTick();
		while (!(HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 10 >  HAL_GetTick());
		val1 = __HAL_TIM_GET_COUNTER (&htim2);

		pMillis = HAL_GetTick();
		while ((HAL_GPIO_ReadPin (ECHO_GPIO_Port, ECHO_Pin)) && pMillis + 50 > HAL_GetTick());
		val2 = __HAL_TIM_GET_COUNTER (&htim2);
		distancia = (val2-val1)* 0.036/2;
		//xQueueSend(queue_AM, &distancia, portMAX_DELAY);
		xQueueSend(queue_PA, &distancia, portMAX_DELAY);
		vTaskDelay(60/portTICK_PERIOD_MS);
	}
}

static void Config(void *pvParameters){
	//unsigned portBASE_TYPE uxPriority;
	//uxPriority = uxTaskPriorityGet( NULL );
	uint16_t ADC_value, display_value=0;
	char init_estate = 1;
	char Param_Config = 2;
	char E_Confir = 0;
	//char max = 70, min = 20; //xq pinto asi.... Las puse globales solo para verlas... van locales.
	while (1){

		xSemaphoreTake(ADC_semph, portMAX_DELAY);
		if (init_estate){                 //si init_estate es 1, significa que esta iniciando el sistema y debe verificar la cola.
			//aca deberia ir una funcion que verifique mejor los valores.
			if(!cola_max && !cola_max){   //solo si lee la cola y los valores son correctos
				init_estate = 0;          //si hay valores correctos ya no es necesaio verificar cola.
				//sube valores a cola CM. Es una especie de confirmacion de que estan bien.
				//sube prioridad de memoria.
				//vuelve a tomar el semaforo y asi se bloquea. CREO....
				xSemaphoreTake(ADC_semph, portMAX_DELAY);
			}
		}
		//Si llego a esta etapa significa que debe configurar valores, para eso fuerza interrupcion ADC para actualizar cola.
		HAL_ADC_Start_IT(&hadc1);
		xQueueReceive(queue_ADC,&ADC_value,portMAX_DELAY);

		//la tarea pantalla debe poder leer su propia prioridad para saber si lee la cola CP(config-pantalla) o cola PA(Pantalla-Alarma)
		vTaskPrioritySet( xTarea_Pantalla_Handle, 3); //si aun no hay datos en la cola CP pantalla si bloquea hasat que primero aparezca valor de max. CREO.


		//es muy probable que los if que tienen en su condicion un Param_Config se reemplacen por Switch CASE.
		if(Param_Config == 2){
			if ((ADC_value > BTN_USER_UP_INF) && (ADC_value < BTN_USER_UP_SUP)){  //condicion que responde a boton UP.
				max++;
				E_Confir = 1;
			}
			else if ((ADC_value > BTN_USER_DOWN_INF) && (ADC_value < BTN_USER_DOWN_SUP)){  //condicion que responde a boton DOWN.
				max--;
				E_Confir = 1;
			}
			//aca se manda el valor de max a la cola CP para verlo en el display.
			xQueueReceive(queue_CP,&display_value,portMAX_DELAY);  //lo usa para eliminar el dato.
			xQueueSend(queue_CP,&max,portMAX_DELAY); //CP              //escribe el nuevo dato.
			vTaskDelay(50/portTICK_PERIOD_MS); //demora para que no incremente tan rapido, sale de la tarea y retora.
		}

		else if(Param_Config == 1){
			if ((ADC_value > BTN_USER_UP_INF) && (ADC_value < BTN_USER_UP_SUP)){  //condicion que responde a boton UP.
				min++;
				E_Confir = 1;
			}
			else if ((ADC_value > BTN_USER_DOWN_INF) && (ADC_value < BTN_USER_DOWN_SUP)){  //condicion que responde a boton DOWN.
				min--;
				E_Confir = 1;
			}
			//aca se manda el valor de max a la cola CP para verlo en el display.
			xQueueReceive(queue_CP,&display_value,portMAX_DELAY);
			xQueueSend(queue_CP,&min,portMAX_DELAY);
			vTaskDelay(50/portTICK_PERIOD_MS); //demora para que no incremente tan rapido, sale de la tarea y retora.
		}

		//este es el if de confirmacion de parametro:
		if((BTN_USER_OK > ADC_value)&&(E_Confir)){ //esto significa que se presionaron los dos botones y que prebiamente se configiro un parametro.
			Param_Config--;
			E_Confir=0;
			vTaskDelay(250/portTICK_PERIOD_MS); //demora para que no incremente tan rapido.
		}


		if(Param_Config>0) xSemaphoreGive(ADC_semph);  //mientras no se hayan configurado maximo y minimo sigue dando semaforo.
		else{											//si ya configuro ambos parametros debe finalizar.
			Param_Config = 2;
			vTaskPrioritySet( xTarea_Medir_Handle, 2); //si aun no hay datos en la cola CP pantalla si bloquea hasat que primero aparezca valor de max
			vTaskPrioritySet( xTarea_Pantalla_Handle, 2);
			xSemaphoreTake(ADC_semph, portMAX_DELAY);
			//va a una funcion de finalizacion que:
			/*
		    -copia valores a CM.
			-sube prioridad a memoria.
			-analizar si es mejor la toma del semaforo aca.
			 */
		}

	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
	static uint16_t adc_value = 0;

	// Obtains the conversion result
	adc_value = HAL_ADC_GetValue(hadc);

	// Sends the value to the queue
	xQueueOverwriteFromISR(queue_ADC, &adc_value, &xHigherPriorityTaskWoken); //en la cinfig de interrup: ADC1 y ADC2 global poner una prioridad de 5, sino queda trabado ahí.

	//En caso de que se presionen dos botones debe entregar semaforo para config.
	if(adc_value<BTN_USER_OK)
		xSemaphoreGiveFromISR(ADC_semph, &xHigherPriorityTaskWoken);
	portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);

	//dos botones: max: 1981
	//boton up: 2750
	//boton min: 2574
}

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
  MX_ADC1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
  HAL_TIM_Base_Start(&htim2);

  //Creacion de Colas:
  queue_ADC = xQueueCreate(1,sizeof(uint16_t));
  queue_PA = xQueueCreate(1,sizeof(uint16_t));
  queue_CP = xQueueCreate(1,sizeof(uint16_t));
  queue_AM = xQueueCreate(1,sizeof(uint32_t));
  int display_value = 0;
  xQueueSend(queue_CP,&display_value,portMAX_DELAY);

  //Creacion de semaforos:
  vSemaphoreCreateBinary(ADC_semph);
  xSemaphoreTake(ADC_semph, portMAX_DELAY);

  //Creacion de tareas:
  xTaskCreate(Pantalla, "Pantalla task", configMINIMAL_STACK_SIZE, NULL, 2, &xTarea_Pantalla_Handle);
  xTaskCreate(Config, "Config task", 200, NULL, 3, &xTarea_Config_Handle);
  xTaskCreate(Medir, "Medir task", 100, NULL, 2, &xTarea_Medir_Handle);

  //inicializa display:
  tm1637_SetBrightness(3);//Set max brightness
  tm1637_DisplayUpdate(0,0,0,0);//Clear display (all segments off)
  tm1637_ShowNumber(122);
  vTaskStartScheduler();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIG_GPIO_Port, TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TM1637_DIO_Pin|TM1637_CLK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIG_Pin */
  GPIO_InitStruct.Pin = TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIG_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ECHO_Pin */
  GPIO_InitStruct.Pin = ECHO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ECHO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TM1637_DIO_Pin */
  GPIO_InitStruct.Pin = TM1637_DIO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TM1637_DIO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TM1637_CLK_Pin */
  GPIO_InitStruct.Pin = TM1637_CLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(TM1637_CLK_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
