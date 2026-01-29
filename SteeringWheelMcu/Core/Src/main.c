/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "adc.h"
#include "dma.h"
#include "fdcan.h"
#include "gpio.h"
#include <stdio.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_FDCAN1_Init();
  /* USER CODE BEGIN 2 */


  
    //GPIO_PIN_SET == 1 (presset ned)
    //GPIO_PIN_RESET == 0 (ikke presset ned)


    //Initializer for debouncing
      static GPIO_PinState last_raw = GPIO_PIN_RESET;
      static GPIO_PinState stable = GPIO_PIN_RESET;
      static uint32_t last_change_ms = 0; 

     
    //Initializer for Press and hold
      static uint32_t press_start_ms = 0;
      static uint8_t hold_fired = 0; 

      //pressed armed er ikke nødvendig men gjør det ekstra sikkert at det ikke 
      //kan gå noe feil med press and hold 

      static uint8_t pressed_armed = 0;


    //initializerer variabler for rotary debouncing 
      static int pos_last = -1; 
      static int pos_stable = -1;
      static uint32_t pos_last_change_ms = 0;


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


    
      //leser av verdier hver loop
      GPIO_PinState raw = HAL_GPIO_ReadPin(Button1_GPIO_Port, Button1_Pin);
      uint32_t now_ms = HAL_GetTick();


      //snapshot for å sjekke for debouncing

      //Vil kjøre hvergang det er en change
      if (raw != last_raw) {

        last_raw = raw;

        //now_ms blir lest av vær loop, last_change oppdateres for hver endring i input verdi
        last_change_ms = now_ms;

      }


      //om forrige if statement ikke har kjørt på 20ms og den nye raw lesingen er forskjellig fra stable (den forrige stabile verdien)
      //så vil if statementen kjøre. 
      //for debouncing (20ms)
      if  ((now_ms - last_change_ms) >= 20 && raw != stable)  {


          //lagrer den nye verdien som skal være stable (faktisk brukt)
          GPIO_PinState prev_stable = stable;
          stable = raw;


          //bare aktiver om går fra ikke trykk til trykk. Ikke vis den går fra trykk til ikke trykk.
          if(prev_stable == GPIO_PIN_RESET && stable == GPIO_PIN_SET){

            printf("KNAPP1! TRYKK\n");
            press_start_ms = now_ms;
            hold_fired = 0;
            pressed_armed = 1;

          }

          if (prev_stable == GPIO_PIN_SET && stable == GPIO_PIN_RESET){


            //ikke nødvendig men gjør det ekstra sikkert at press and hold if statement ikke kjører tilfelle en teknisk feil
            pressed_armed = 0;
            hold_fired = 0;
            

            //om vi vil ha funksjonalitet for når knappen blir sluppet

          }

      }

      
          //må ligge utenfor debounce sjekken fordi debounce sjekker kjører bare når raw er forskjellig fra den forriige stabile verdien
          //Hold må kjøres når det ikke har vært endring i den stabile verdien
          //den får ikke bounce fordi stable blir bare SET om en endring har vært stabil i 20 ms (i debounce if statement)
          
          if (pressed_armed && stable == GPIO_PIN_SET && !hold_fired){

            if((now_ms - press_start_ms) >= 4000){
                printf("KNAPP1 HOLD\n");
                hold_fired = 1;
            }
            
          }

          // Rotary switch 


          //Leser av verdier til rotary inputs (4 per en rotary switch)
          GPIO_PinState r1 = HAL_GPIO_ReadPin(Rotary11_GPIO_Port, Rotary11_Pin);
          GPIO_PinState r2 = HAL_GPIO_ReadPin(Rotary12_GPIO_Port, Rotary12_Pin);
          GPIO_PinState r3 = HAL_GPIO_ReadPin(Rotary13_GPIO_Port, Rotary13_Pin);
          GPIO_PinState r4 = HAL_GPIO_ReadPin(Rotary14_GPIO_Port, Rotary14_Pin);


          //gir b1 - b4 true eller false avhengig om gpio_pin_set eller gpio_pin_reset
          uint8_t b1 = (r1 == GPIO_PIN_SET); //b1 = 1 om r1 er GPIO_PIN_SET osv
          uint8_t b2 = (r2 == GPIO_PIN_SET);
          uint8_t b3 = (r3 == GPIO_PIN_SET);
          uint8_t b4 = (r4 == GPIO_PIN_SET);



          //definerer ugyldig posisjons verdi for sikkerhet
          //antar posisjonen er ygyldig til vi har gjort sjekker 
          int pos = -1;


          uint8_t sum = b1 + b2 + b3 + b4;

          //bare om 1 av 4 inputs er aktiv er den gyldig
          //alt annet er feil eller støy eller overgang (midt mellom to hakk)
          if (sum == 1) {

            //sjekker hvilke input som er true og setter posisjonsvariablen til å representere den inputen
            if (b1) pos = 1; 
            else if (b2) pos = 2;
            else if (b3) pos = 3;
            else pos = 4;

          } else {

            pos = -1;

          }



          
          
          // om endring i posisjon oppdaterer den pos i pos_last og gjør lagrer tidspunktet
          if (pos != pos_last) {
              pos_last = pos;
              pos_last_change_ms = now_ms;
          }

          //om endring i posisjon ikke har skjedd på 20ms og den nye posisjonen (lagret i pos_last) ikke er lik den forrige stable pos
          if ((now_ms - pos_last_change_ms) >= 20 && pos_last != pos_stable){
              
              //lagrer den nye stable posisjonen
              pos_stable = pos_last;

              //hvis posisjonen er valid vil koden kjøre
              if (pos_stable != -1) {

                printf("ROTARY1 STABLE %d\n", pos_stable);

                //kode for hva som skal skje for mode/posisjons bytte

              }


          }



          /* UTEN DEBOUNCE (KANSKJE NICE FOR TESTING)

                  //-2 betyr ikke initialisert enda (-1 ugyldig og 1-4 gyldig)
                  static int last_pos = -2;


                  //if statement for å si ifra hver gang en endring i rotary input skjer (snur fra et hakk/posisjon til et annet)
                  //if statement skjører om den nye posisjonen er gyldig og ikke lik forrige posisjon
                  if (pos != -1 && pos != last_pos) {
                      last_pos = pos;
                      printf("ROTARY1 POS; %d\n", pos);
                  }


          */
          

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
