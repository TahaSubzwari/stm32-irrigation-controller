/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    main.c
 * @brief   Main program body
 ******************************************************************************
 *
 * STM32 IRRIGATION CONTROLLER
 * ---------------------------
 * Embedded controller for an irrigation system. Manages a single pump
 * and servo-driven distributor to fill a reservoir from an underground spring
 * and irrigate three terraced zones at different elevations over a 24-hour
 * cycle.
 *
 * Target: STM32 Nucleo-F401RE (ARM Cortex-M4, 84 MHz)
 *
 *
 * SYSTEM OVERVIEW
 * ---------------
 * The system runs as a two-mode state machine:
 *
 *   SETUP MODE:  Triggered by reset (black button). User configures pipeline
 *                assignments, PWM speed options, and start/stop hours for
 *                each of 4 pipelines (1 inlet + 3 zones) over UART. Mode LED
 *                flashes; system waits for B1 (blue button) to advance.
 *
 *   RUN MODE:    Triggered by B1. Scaled wall clock starts at hour 0 and
 *                advances 300x faster than real time. Each hour, the active pipeline
 *                is selected from the schedule, the servo and RGB LED update, the motor
 *                drives at the configured PWM, and reservoir depth + RPM
 *                are sampled and logged to UART. Halts at hour 24 or on a
 *                reservoir-empty fault.
 * 
 * PERIPHERAL MAP
 * --------------
 *   TIM2 CH1   - Servo PWM (50 Hz, 1-2 ms pulse width)
 *   TIM3 CH1   - DC motor PWM, forward direction (via L9110)
 *   TIM3 CH3   - DC motor PWM, reverse direction (via L9110, inlet only)
 *   TIM4 CH4   - HC-SR04 echo input capture (both edges)
 *   TIM5       - Scaled wall-clock tick (12 s real = 1 simulated hour)
 *   ADC1 CH9   - Potentiometer (8-bit) for manual inlet speed control
 *   USART2     - Terminal I/O at 115200 baud, 8N1
 *   EXTI       - RPM_TICK pin: optical encoder pulse counter
 *   GPIO       - 7-segment digit drive (PA + PB), RGB LED (PC + PD),
 *                LD2 mode LED, B1 user button, HC-SR04 trigger
 *
 ******************************************************************************
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
#define LED_RED_ON()     HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_SET)
#define LED_RED_OFF()    HAL_GPIO_WritePin(RED_GPIO_Port, RED_Pin, GPIO_PIN_RESET)
#define LED_GREEN_ON()   HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_SET)
#define LED_GREEN_OFF()  HAL_GPIO_WritePin(GREEN_GPIO_Port, GREEN_Pin, GPIO_PIN_RESET)
#define LED_BLUE_ON()    HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_SET)
#define LED_BLUE_OFF()   HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET)
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

//Single byte to store UART input
uint8_t byte = 0;

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
char msg[100];          // UART message buffer

//Pipeline Variables
int pipeA = 0;
int pipeB = 0;
int pipeC = 0;
int pipeD = 0;

//Pump PWN Variables
int pwmA = 0;
int pwmB = 0;
int pwmC = 0;
int pwmD = 0;

//First hour variables
int firstA = 0;
int firstB = 0;
int firstC = 0;
int firstD = 0;

//Last hour variables
int lastA = 0;
int lastB = 0;
int lastC = 0;
int lastD = 0;

char dig10 = 0;
char dig1 = 0;

uint8_t txd_msg_buffer[64] = {0};

volatile uint8_t clock_hours = 0;
volatile uint8_t wall_clock_hr_update_flag = 0;

uint16_t servo_angle_0 = 500; // Pipeline A - 0 degrees
uint16_t servo_angle_1 = 1000; // Pipeline B - 45 degrees
uint16_t servo_angle_2 = 1500; // Pipeline C - 90 degrees
uint16_t servo_angle_3 = 2000; // Pipeline D - 135 degrees

uint16_t motor_pwm_values[4] = {0, 1400, 1700, 2000}; // Adjust for motor speed levels
volatile uint32_t rpm_tick_count = 0;
volatile uint32_t measured_rpm = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
void Enter_Setup_Mode(void);
void Enter_Run_Mode(void);
int Measure_Distance(void);
void HCSR04_Trigger(void);
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B);
void RGB_SetColor(uint8_t pipeline);
static void UpdateMotor(uint8_t current_pipeline, uint8_t active_pwm);
void Resevoir_Empty(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rcv_intv_flag = 0;

volatile uint8_t hcsr04_Rx_flag = 0;
volatile uint8_t first_edge = 0;
volatile uint16_t time_edge1 = 0;
volatile uint16_t time_edge2 = 0;

float distance_cm = 0;
float distance_percent = 0;

//ENTERING SETUP MODE
void Enter_Setup_Mode(void)
{
    LED_BLUE_OFF();
    LED_GREEN_OFF();
    LED_RED_OFF();

    DIGITS_Display(0,0);

    TIM3 -> CCR1 = 0;
	TIM3 -> CCR3 = 0;

    HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n=== SETUP MODE ===\r\n", 24, 1000);
    HAL_UART_Transmit(&huart2, (uint8_t*)"Enter SETUP Parameters:\r\n", 26, 1000);

    // Ask for Pipeline and PWM inputs ----------------------------------------
    sprintf((char*)txd_msg_buffer,"\r\nPipeline A (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pipeA = byte - '0';

    sprintf((char*)txd_msg_buffer,"\r\nPump pwm A (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pwmA = byte - '0';

    sprintf((char*)txd_msg_buffer,"\r\nPipeline B (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pipeB = byte - '0';

    sprintf((char*)txd_msg_buffer,"\r\nPump pwm B (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pwmB = byte - '0';

    sprintf((char*)txd_msg_buffer,"\r\nPipeline C (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pipeC = byte - '0';

    sprintf((char*)txd_msg_buffer,"\r\nPump pwm C (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pwmC = byte - '0';

    sprintf((char*)txd_msg_buffer,"\r\nPipeline D (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pipeD = byte - '0';

    sprintf((char*)txd_msg_buffer,"\r\nPump pwm D (options 0 to 3): ");
    rcv_intv_flag = 0;
    HAL_UART_Receive_IT(&huart2, &byte, 1);
    HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
    while (rcv_intv_flag == 0) {}
    pwmD = byte - '0';

    //--------------------------------------------------------------------------------------

    //Ask for first and last hour of each pipeline
    sprintf((char*)txd_msg_buffer, "\r\nPipeline 0 Pump FIRST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	firstA = (dig10-'0')*10 + (dig1 - '0');

    sprintf((char*)txd_msg_buffer, "\r\nPipeline 0 Pump LAST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	lastA = (dig10-'0')*10 + (dig1 - '0');

    sprintf((char*)txd_msg_buffer, "\r\nPipeline 1 Pump FIRST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	firstB = (dig10-'0')*10 + (dig1 - '0');

    sprintf((char*)txd_msg_buffer, "\r\nPipeline 1 Pump LAST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	lastB = (dig10-'0')*10 + (dig1 - '0');

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 2 Pump FIRST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	firstC = (dig10-'0')*10 + (dig1 - '0');

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 2 Pump LAST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	lastC = (dig10-'0')*10 + (dig1 - '0');

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 3 Pump FIRST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	firstD = (dig10-'0')*10 + (dig1 - '0');

	sprintf((char*)txd_msg_buffer, "\r\nPipeline 3 Pump LAST HOUR (options: 00 to 23): ");
    rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
	while (rcv_intv_flag == 0) {}
	dig10 = byte;
	rcv_intv_flag = 0;
	rcv_intv_flag = 0;
	HAL_UART_Receive_IT(&huart2, &byte, 1);
	while (rcv_intv_flag == 0) {}
	dig1 = byte;
	rcv_intv_flag = 0;

	lastD = (dig10-'0')*10 + (dig1 - '0');

	//------------------------------------------------------------------------
	//Output all the information and wait for user to press blue button
	sprintf((char*)txd_msg_buffer, "\t\n\nPrinting SETUP Parameters\n\n");
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "\n\rCURRENT WALL CLOCK HOUR 0\n");
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "\n\rPipeline: %d ", pipeA);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump PWM: %d ", pwmA);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump FIRST Hour: %d Pump LAST Hour %d\n", firstA, lastA);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


	sprintf((char*)txd_msg_buffer, "\n\rPipeline: %d ", pipeB);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump PWM: %d ", pwmB);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump FIRST Hour: %d Pump LAST Hour %d\n", firstB, lastB);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


	sprintf((char*)txd_msg_buffer, "\n\rPipeline: %d ", pipeC);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump PWM: %d ", pwmC);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump FIRST Hour: %d Pump LAST Hour %d\n", firstC, lastC);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


	sprintf((char*)txd_msg_buffer, "\n\rPipeline: %d ", pipeD);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump PWM: %d ", pwmD);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "Pump FIRST Hour: %d Pump LAST Hour %d\n", firstD, lastD);
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


	sprintf((char*)txd_msg_buffer, "\n\n\rSETUP is done. Press Blue Button for RUN MODE");
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


	while(1){
		GPIO_PinState buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13);

		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);		//LED is ON
		HAL_Delay(100);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);	//LED is OFF
		HAL_Delay(100);

		if(buttonState == GPIO_PIN_RESET){
			break;
		}
	}

}

//Send pulse
void HCSR04_Trigger(void)
{
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_SET);
    for (int j = 0; j < 15; j++) {};  // 10µs pulse
    HAL_GPIO_WritePin(HCSR04_TRIG_GPIO_Port, HCSR04_TRIG_Pin, GPIO_PIN_RESET);
}



//------------------MEASURE DISTANCE AS PERCENTAGE-----------------------
int Measure_Distance(void) {

	uint16_t time_diff = 0;
	hcsr04_Rx_flag = 0;
	first_edge = 0;
	time_edge1 = 0;
	time_edge2 = 0;

	HCSR04_Trigger();

	//Wait for measurement
	while (hcsr04_Rx_flag == 0) {}

	//Compute pulse difference
	if (time_edge2 > time_edge1)
	time_diff = time_edge2 - time_edge1;

	//Convert to cm (based on datasheet: time/58)
	distance_cm = 88 - (time_diff / 58.0f);
	if(distance_cm < 0.0f){
		distance_cm = 0.0f;
	}

	//Compute percentage from max distance
	distance_percent = (distance_cm / 88.0f) * 100.0f;

	return (uint16_t)distance_percent;
}

void Set_Servo_Angle(uint16_t pulse_width)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, pulse_width);
}


//-------------------DISPLAY DISTANCE PERCENTAGE ON 7 SEG----------------
void DIGITS_Display(uint8_t DIGIT_A, uint8_t DIGIT_B) {
    DIGIT_A &= 0x0F;
    DIGIT_B &= 0x0F;

    // Pipeline A (tens digit) - Port A
    HAL_GPIO_WritePin(GPIOA, DIGIT_A0_Pin, (DIGIT_A & 0x1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, DIGIT_A1_Pin, (DIGIT_A & 0x2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, DIGIT_A2_Pin, (DIGIT_A & 0x4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, DIGIT_A3_Pin, (DIGIT_A & 0x8) ? GPIO_PIN_SET : GPIO_PIN_RESET);

    // Pipeline B (ones digit) - Port B
    HAL_GPIO_WritePin(GPIOB, DIGIT_B0_Pin, (DIGIT_B & 0x1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin, (DIGIT_B & 0x2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DIGIT_B2_Pin, (DIGIT_B & 0x4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, DIGIT_B3_Pin, (DIGIT_B & 0x8) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}


//---------------------SET RBG LED COLOR---------------------------------
void RGB_SetColor(uint8_t pipeline)
{
    // Turn all LEDs OFF first
    LED_RED_OFF();
    LED_GREEN_OFF();
    LED_BLUE_OFF();

    if (pipeline == 0)
    {
        LED_RED_ON();                   // Pipeline 0 PURPLE
        LED_BLUE_ON();
    }
    else if (pipeline == 1)
    {
        LED_RED_ON();                 // Pipeline 1 RED
    }
    else if (pipeline == 2)
    {
        LED_GREEN_ON();                  // Pipeline 2 GREEN
    }
    else if (pipeline == 3)
    {
        LED_BLUE_ON();                 // Pipeline 3 BLUE
    }
    else
    {
        LED_RED_OFF();
        LED_GREEN_OFF();
        LED_BLUE_OFF();

    }
}


//----------------------UPDATE MOTOR CONTINUALLY-------------------------
static void UpdateMotor(uint8_t current_pipeline, uint8_t active_pwm){
	if (current_pipeline == 0 && active_pwm == 0)
		{
		// Start conversion, wait briefly, read value
		HAL_ADC_Start(&hadc1);

		if (HAL_ADC_PollForConversion(&hadc1, 10) == HAL_OK) {
			uint32_t pot_value = HAL_ADC_GetValue(&hadc1); // 0-255

			// Map to TIM3 CCR range (TIM3 period = 2000)
			uint16_t scaled_pwm = (uint16_t)((pot_value * 2000UL) / 255UL);

			// Apply to TIM3 channel 3
			TIM3->CCR3 = scaled_pwm;

			// ensure other channel is off
			TIM3->CCR1 = 0;
		}
		HAL_ADC_Stop(&hadc1);
	}
}


//---------------------SPECIAL EVENT------------------------------------
void Reservoir_Empty(void) {
		//Stop motion
	    TIM3->CCR1 = 0;
	    TIM3->CCR3 = 0;

	    //Board LED off
	    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

		sprintf((char*)txd_msg_buffer, "\r\n Reservoir is EMPTY! \r\n");
		HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	    //Flash white
	    while (1)
	    {
	        //Digits display 00
	        DIGITS_Display(0,0);

	        LED_RED_ON();
	        LED_GREEN_ON();
	        LED_BLUE_ON();
	        HAL_Delay(250);
	        LED_RED_OFF();
	        LED_GREEN_OFF();
	        LED_BLUE_OFF();
	        HAL_Delay(250);


	    }
}


//-------------------ENTER RUN MODE--------------------------------------
void Enter_Run_Mode(void){
	HAL_TIM_Base_Start(&htim4);         // Start timing base
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);  // Enable input capture

	HAL_TIM_Base_Start_IT(&htim5);

	uint8_t current_pipeline = 0;  // Track active pipeline
	uint8_t active_pwm = pwmA;

	uint8_t current_fill = Measure_Distance();	//Get the fill level
	uint8_t tens = current_fill / 10;
	uint8_t ones = current_fill % 10;
	DIGITS_Display(tens, ones);

	sprintf((char*)txd_msg_buffer, "\r\n\nCLOCK : PIPE : PWM : RPM : DEPTH :\r\n");
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	sprintf((char*)txd_msg_buffer, "---------------------------------");
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);


	if (clock_hours >= firstA && clock_hours <= lastA) {
		current_pipeline = pipeA;
		active_pwm = pwmA;
	}
	else if (clock_hours >= firstB && clock_hours <= lastB) {
		current_pipeline = pipeB;
		active_pwm = pwmB;
	}
	else if (clock_hours >= firstC && clock_hours <= lastC) {
		current_pipeline = pipeC;
		active_pwm = pwmC;
	}
	else if (clock_hours >= firstD && clock_hours <= lastD) {
		current_pipeline = pipeD;
		active_pwm = pwmD;
	}
	else {
		current_pipeline = 255;
		active_pwm = 0;
	}

	RGB_SetColor(current_pipeline);

	//Control motor & servo for hour 0
	if(current_pipeline == 255){
		TIM3->CCR1 = 0;
		TIM3->CCR3 = 0;
	}
	else if(active_pwm != 0 && current_pipeline != 0){
		TIM3->CCR1 = motor_pwm_values[active_pwm];
		TIM3->CCR3 = 0;
	}
	else if (current_pipeline == 0){
		if(active_pwm == 0){
			UpdateMotor(current_pipeline, active_pwm);
		}
		else{
			TIM3->CCR1 = 0;
			TIM3->CCR3 = motor_pwm_values[active_pwm];
		}
	}

	//Calculate RPM after short delay for stable reading
	HAL_Delay(1000);
	measured_rpm = (rpm_tick_count / 4);
	rpm_tick_count = 0;

	if(current_pipeline == 255){
		sprintf((char*)txd_msg_buffer, "\r %d    :      :     : 0    :  %d  :\r\n",
				clock_hours, current_fill);
	} else {
		sprintf((char*)txd_msg_buffer, "\r %d    :  %d   :  %d   :  %d   :   %d  :\r\n",
				clock_hours, current_pipeline, active_pwm, measured_rpm, current_fill);
	}
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);

	//Check if reservoir is empty
	if (current_fill <= 0)
	{
		Reservoir_Empty();
	}

	//Set servo for hour 0
	if(current_pipeline == 0){Set_Servo_Angle(servo_angle_0);}
	else if(current_pipeline == 1){Set_Servo_Angle(servo_angle_1);}
	else if(current_pipeline == 2){Set_Servo_Angle(servo_angle_2);}
	else if(current_pipeline == 3){Set_Servo_Angle(servo_angle_3);}


	while (clock_hours < 24)
	{
		if(active_pwm == 0 && current_pipeline == 0){
			UpdateMotor(current_pipeline, active_pwm);
		}

		if (wall_clock_hr_update_flag){
			wall_clock_hr_update_flag = 0;

			uint8_t current_fill = Measure_Distance();

			uint8_t tens = current_fill / 10;
			uint8_t ones = current_fill % 10;
			DIGITS_Display(tens, ones);


			if (clock_hours >= firstA && clock_hours <= lastA) {
				current_pipeline = pipeA;
				active_pwm = pwmA;
			}
			else if (clock_hours >= firstB && clock_hours <= lastB) {
				current_pipeline = pipeB;
				active_pwm = pwmB;
			}
			else if (clock_hours >= firstC && clock_hours <= lastC) {
				current_pipeline = pipeC;
				active_pwm = pwmC;
			}
			else if (clock_hours >= firstD && clock_hours <= lastD) {
				current_pipeline = pipeD;
				active_pwm = pwmD;
			}
			else {
				current_pipeline = 255;
				active_pwm = 0;
			}

			RGB_SetColor(current_pipeline);

			if(current_pipeline == 255){
				TIM3 -> CCR1 = 0;
				TIM3 -> CCR3 = 0;
			}
			if(active_pwm != 0 && current_pipeline != 0){
				TIM3 -> CCR1 = motor_pwm_values[active_pwm];
				TIM3 -> CCR3 = 0;
			}
			else if (current_pipeline == 0 && active_pwm != 0){
				TIM3 -> CCR1 = 0;
				TIM3 -> CCR3 = motor_pwm_values[active_pwm];
			}

			HAL_Delay(1000);

			measured_rpm = (rpm_tick_count / 4);
			rpm_tick_count = 0;

			if(current_pipeline == 255){
				sprintf((char*)txd_msg_buffer, "\r %d    :      :     : 0    :  %d  :\r\n", clock_hours, current_fill);
				HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			} else {
				sprintf((char*)txd_msg_buffer, "\r %d    :  %d   :  %d   :  %d   :   %d  :\r\n", clock_hours, current_pipeline, active_pwm, measured_rpm, current_fill);
				HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
			}

			//Check if reservoir is empty
			if (current_fill <= 0)
			{
				Reservoir_Empty();
			}

			if(current_pipeline == 0){Set_Servo_Angle(servo_angle_0);}
			else if(current_pipeline == 1){Set_Servo_Angle(servo_angle_1);}
			else if(current_pipeline == 2){Set_Servo_Angle(servo_angle_2);}
			else if(current_pipeline == 3){Set_Servo_Angle(servo_angle_3);}
		}

	}
	sprintf((char*)txd_msg_buffer, "\r\n irrigation complete!\r\n");
	HAL_UART_Transmit(&huart2, txd_msg_buffer, strlen((char*)txd_msg_buffer), 1000);
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
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Init(&htim4);
  HAL_TIM_Base_Start(&htim4);
  HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);


  Enter_Setup_Mode();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);		//LED is ON
  Enter_Run_Mode();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

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

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 16-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 20000-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 16-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1200-1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 16-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65536-1;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 16000-1;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 12000-1;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|DIGIT_A3_Pin|DIGIT_A2_Pin|DIGIT_A1_Pin
                          |DIGIT_A0_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, DIGIT_B1_Pin|DIGIT_B2_Pin|DIGIT_B3_Pin|DIGIT_B0_Pin
                          |HCSR04_TRIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, RED_Pin|GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUE_GPIO_Port, BLUE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin DIGIT_A3_Pin DIGIT_A2_Pin DIGIT_A1_Pin
                           DIGIT_A0_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|DIGIT_A3_Pin|DIGIT_A2_Pin|DIGIT_A1_Pin
                          |DIGIT_A0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : RPM_TICK_Pin */
  GPIO_InitStruct.Pin = RPM_TICK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(RPM_TICK_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIGIT_B1_Pin DIGIT_B2_Pin DIGIT_B3_Pin DIGIT_B0_Pin
                           HCSR04_TRIG_Pin */
  GPIO_InitStruct.Pin = DIGIT_B1_Pin|DIGIT_B2_Pin|DIGIT_B3_Pin|DIGIT_B0_Pin
                          |HCSR04_TRIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : RED_Pin GREEN_Pin */
  GPIO_InitStruct.Pin = RED_Pin|GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUE_Pin */
  GPIO_InitStruct.Pin = BLUE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUE_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart -> Instance == USART2) {
		HAL_UART_Transmit(&huart2, &byte, 1, 100);
		rcv_intv_flag = 1;
	}
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)
        {
            if (first_edge == 0)
            {
                time_edge1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                first_edge = 1;
            }
            else
            {
                time_edge2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
                __HAL_TIM_SET_COUNTER(htim, 0);
                hcsr04_Rx_flag = 1;
            }
        }
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
    	clock_hours++;
        wall_clock_hr_update_flag = 1; // trigger display update

    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == RPM_TICK_Pin)
    {
        rpm_tick_count++;
    }
}


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
