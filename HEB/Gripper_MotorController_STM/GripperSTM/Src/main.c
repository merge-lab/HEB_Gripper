/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "gripper_protocol.h"   // shared binary wire protocol with the ESP32
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum {
	CONSTANT,
	SPRING
} ControlType;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TX_BUFFER_SIZE 256
#define RX_BUFFER_SIZE 64
const int DEADBAND_WIDTH = 3;
const int RAMP_WIDTH = 8;

#define COUNTS_PER_REV      65536.0f
#define DEGREES_PER_REV     360.0f
#define COUNTS_TO_DEG       (DEGREES_PER_REV / COUNTS_PER_REV)
#define SPEED_SCALE_DEG     (1000.0f * COUNTS_TO_DEG)

// Fixed torque sign for this motor's wiring/encoder convention.
// Deterministic replacement for the old detect_motor_direction() auto-cal.
// If the spring pushes the wrong way, flip this between 1 and -1 and reflash.
#define MOTOR_DIRECTION     (1)

// Fail-safe: if no command is received within this window, relax the motor.
// The ESP32 re-sends its active command ~every 200 ms, so this also acts as a
// heartbeat: lose the link (or the ESP32) and the motor goes limp on its own.
#define HEARTBEAT_TIMEOUT_MS  500
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

COMP_HandleTypeDef hcomp1;
COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp4;

CORDIC_HandleTypeDef hcordic;

DAC_HandleTypeDef hdac3;

OPAMP_HandleTypeDef hopamp1;
OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
TIM_HandleTypeDef htim6;

uint8_t tx_buffer[TX_BUFFER_SIZE];

volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t rx_index = 0;
uint8_t rx_char;
volatile bool command_ready = false;

// Binary protocol: latest fully-framed command packet, plus a dedicated TX
// buffer for the response (kept separate from tx_buffer used by printf).
volatile Command_Packet pending_command;
uint8_t response_tx_buffer[sizeof(Response_Packet)];
volatile int32_t position_offset;
volatile float torque_setpoint;          // spring stiffness (kp) in SPRING mode
volatile float constant_force_setpoint;  // commanded torque in CONSTANT mode
volatile ControlType control_type;

// Vibration overlay (added on top of the active base mode). 0 amplitude = off.
volatile float vib_freq_hz = 0.0f;
volatile float vib_amplitude = 0.0f;

// Relax / heartbeat state. Boot relaxed (no torque) until commanded.
volatile bool motor_relaxed = true;
volatile uint32_t last_command_ms = 0;
int16_t direction_correction = 1;

volatile float debug_position = 0;
volatile float debug_speed = 0;
volatile bool debug_ready = false;
volatile float debug_torque = 0;
volatile float debug_motor_torque = 0;  // measured Iq (A): actual motor torque -> trigger
volatile float debug_user_torque  = 0;  // estimated user torque on trigger (A-equivalent)
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_COMP1_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP4_Init(void);
static void MX_CORDIC_Init(void);
static void MX_DAC3_Init(void);
static void MX_OPAMP1_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_OPAMP3_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
static void MX_TIM6_Init(void);
void handle_command(void);
void send_response(Device_Responses response);
uint32_t get_motor_enc(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// overwrite the _write so that printf can be used to print to the console
int _write(int file, char *ptr, int len) {
	if (huart2.gState != HAL_UART_STATE_READY) {
		return 0;
	}
	int cpy_len = len; // restrict the size of the message to the size of the transmit buffer size
	if (cpy_len > TX_BUFFER_SIZE) {
		cpy_len = TX_BUFFER_SIZE;
	}
	memcpy(tx_buffer, ptr, cpy_len); // copy the message to the global transmit buffer
	if (HAL_UART_Transmit_DMA(&huart2, tx_buffer, cpy_len) == HAL_OK) { // non-blocking send over uart
		return len; // Successfully started
	} else {
		return 0; // Failed to start
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        HAL_UART_Receive_IT(&huart2, &rx_char, 1);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	// Check that the interrupt was generated by the correct UART peripheral
	if (huart->Instance == USART2) {
		bool echo = false;
		if (echo) {
			if (huart2.gState == HAL_UART_STATE_READY) {
				// Echo the character using a NON-BLOCKING (Interrupt) transmit
				// We must use a static variable here, because rx_char (global) could
				// be overwritten by the *next* RX interrupt before this TX completes.
				static uint8_t echo_char;
				echo_char = rx_char;
				HAL_UART_Transmit_IT(&huart2, &echo_char, 1);
			}
		}
		// ----- Binary frame assembler (resync-safe) -----
		// A frame is a fixed-size Command_Packet bracketed by START_BYTE/END_BYTE.
		uint8_t b = rx_char;

		if (rx_index == 0) {
			// Hunting for the start of a packet; ignore any noise until START_BYTE.
			if (b == (uint8_t)START_BYTE) {
				rx_buffer[rx_index++] = b;
			}
		} else {
			rx_buffer[rx_index++] = b;

			if (rx_index == sizeof(Command_Packet)) {
				// Full frame collected; only accept it if the trailing byte frames it.
				if (rx_buffer[sizeof(Command_Packet) - 1] == (uint8_t)END_BYTE) {
					memcpy((void *)&pending_command,
					       (const void *)rx_buffer,
					       sizeof(Command_Packet));
					command_ready = true;
				}
				rx_index = 0; // reset for the next frame (also resyncs on a bad frame)
			} else if (rx_index >= RX_BUFFER_SIZE) {
				rx_index = 0; // overflow guard, should never hit for a valid packet
			}
		}

		// Re-arm the UART receive interrupt to wait for the next byte
		HAL_UART_Receive_IT(&huart2, &rx_char, 1);
	}
}

void send_response(Device_Responses response) {
	Response_Packet resp;
	resp.startByte     = START_BYTE;
	resp.response      = response;
	// DIAGNOSTIC: report position-relative-to-center (deg) and commanded torque,
	// both bit-cast floats, so we can see the spring's behavior as the handle moves.
	memcpy(&resp.encoder_value,   (const void *)&debug_position,     sizeof(resp.encoder_value));
	memcpy(&resp.motor_value,     (const void *)&debug_torque,       sizeof(resp.motor_value));
	memcpy(&resp.measured_torque, (const void *)&debug_motor_torque, sizeof(resp.measured_torque));
	memcpy(&resp.user_torque,     (const void *)&debug_user_torque,  sizeof(resp.user_torque));
	resp.endByte       = END_BYTE;

	memcpy(response_tx_buffer, &resp, sizeof(Response_Packet));
	// Fire-and-forget DMA send; if a previous TX (e.g. printf) is still in
	// flight this returns busy and the response is dropped. Fine for bring-up.
	HAL_UART_Transmit_DMA(&huart2, response_tx_buffer, sizeof(Response_Packet));
}

void handle_command(void) {
	command_ready = false; // Reset the flag

	// Take a local copy so the ISR can keep filling pending_command.
	Command_Packet cmd;
	memcpy(&cmd, (const void *)&pending_command, sizeof(Command_Packet));

	// Framing is checked in the ISR, but guard anyway.
	if (cmd.startByte != START_BYTE || cmd.endByte != END_BYTE) {
		send_response(MALFORMED);
		return;
	}

	// Any valid command counts as a heartbeat (keeps the fail-safe from firing).
	last_command_ms = HAL_GetTick();

	switch (cmd.command) {
	case PING:
		send_response(PONG);
		break;

	case SPRING_FORCE: {
		// param_1 carries the spring constant as a bit-cast float.
		float k;
		memcpy(&k, &cmd.param_1, sizeof(k));

		// Reject nonsense / unsafe stiffness (negative = positive feedback).
		if (isnan(k) || isinf(k) || k < 0.0f) {
			send_response(WRONG_PARAMS);
			break;
		}

		// On (re)engagement - from relaxed or another mode - zero the spring
		// center on the CURRENT handle position so it holds where you turn it on.
		// Don't re-zero on stiffness changes / periodic re-sends, or the center
		// would track the handle and the spring would collapse to no force.
		if (motor_relaxed || control_type != SPRING) {
			position_offset = (int32_t)ENCODER_M1._Super.hMecAngle;
		}

		control_type    = SPRING;
		torque_setpoint = k;       // used as kp by the 1 kHz spring loop
		motor_relaxed   = false;   // engage
		send_response(ACK);
		break;
	}

	case CONSTANT_FORCE: {
		// param_1 carries the commanded torque as a bit-cast float.
		// Sign selects direction; magnitude is clamped by the loop's saturation.
		float f;
		memcpy(&f, &cmd.param_1, sizeof(f));

		if (isnan(f) || isinf(f)) {
			send_response(WRONG_PARAMS);
			break;
		}

		control_type            = CONSTANT;
		constant_force_setpoint = f;
		motor_relaxed           = false;   // engage
		send_response(ACK);
		break;
	}

	case VIBRATE: {
		// param_1 = frequency in Hz (integer); param_2 = amplitude (bit-cast float).
		// Overlay only: does not change the active base mode (spring/constant).
		uint32_t freq_u = cmd.param_1;
		float amp;
		memcpy(&amp, &cmd.param_2, sizeof(amp));

		// Keep well under the 1 kHz loop's Nyquist limit; reject bad amplitude.
		if (isnan(amp) || isinf(amp) || amp < 0.0f || freq_u > 400) {
			send_response(WRONG_PARAMS);
			break;
		}

		vib_freq_hz   = (float)freq_u;
		vib_amplitude = amp;        // 0 disables the overlay
		send_response(ACK);
		break;
	}

	case RELAX:
		// End all intentional force. The control loop ramps torque to 0 and
		// the vibration overlay is disabled.
		motor_relaxed = true;
		vib_amplitude = 0.0f;
		send_response(ACK);
		break;

	default:
		send_response(NOT_RECOGNIZED);
		break;
	}
}

uint32_t get_motor_enc() {
	return htim4.Instance->CNT;
}

int32_t get_motor_pos() {
	return (int32_t)ENCODER_M1._Super.hMecAngle - position_offset;
}

float get_motor_speed() {
    // Called once per 1 kHz control tick, so the sample period is a fixed 1 ms.
    // Returning the per-tick position delta (counts/ms) avoids the HAL_GetTick()
    // 1 ms quantization, which made the old estimate alternate between 0 and
    // spikes (delta_time_ms was 0 or 1 at this rate) - a major jitter source.
    static int32_t previous_pos = 0;
    static bool first_call = true;

    int32_t current_pos = get_motor_pos();

    if (first_call) {
        previous_pos = current_pos;
        first_call = false;
        return 0.0f;
    }

    int32_t delta_position = current_pos - previous_pos;
    previous_pos = current_pos;

    return (float)delta_position;   // counts per 1 ms tick
}

void loop_1ms()
{
    if (IDLE == MC_GetSTMStateMotor1()) {
        MC_ProgramTorqueRampMotor1(0, 0);
        MC_StartMotor1();
        return;
    }

    if (MC_GetSTMStateMotor1() != RUN) {
        return;
    }

    /* ----- Position ----- */

    float pos_deg = (float)get_motor_pos() * COUNTS_TO_DEG;

    /* ----- Velocity ----- */

    static float speed_filt = 0.0f;

    float raw_speed = get_motor_speed() * SPEED_SCALE_DEG;

    // 20 Hz-ish low pass at 1 kHz loop rate
    speed_filt =
        0.98f * speed_filt +
        0.02f * raw_speed;

    /* ----- Tunable gains ----- */

    float kp = torque_setpoint;     // spring stiffness
    float kd = 0.004f;             // damping

    /* ----- Deadband ----- */

    const float pos_deadband = 0.10f;   // degrees
    const float vel_deadband = 0.50f;   // deg/s

    float spring_error = pos_deg;

    if (fabsf(pos_deg) < pos_deadband) {
        spring_error = 0.0f;
    }
    else {
        // smooth transition out of deadband
        if (pos_deg > 0)
            spring_error -= pos_deadband;
        else
            spring_error += pos_deadband;
    }

    /* ----- Relax / heartbeat gate ----- */
    // Motor produces force only when engaged AND we've heard from the ESP32
    // recently. Otherwise torque_command is 0 and the LPF ramps output to zero.
    bool active = !motor_relaxed &&
                  ((HAL_GetTick() - last_command_ms) < HEARTBEAT_TIMEOUT_MS);

    /* ----- Control law (depends on mode) ----- */

    float torque_command;

    if (!active)
    {
        torque_command = 0.0f;
    }
    else if (control_type == CONSTANT)
    {
        // Open-loop constant torque, independent of position.
        torque_command = constant_force_setpoint;
    }
    else // SPRING
    {
        if (fabsf(pos_deg) < pos_deadband &&
            fabsf(speed_filt) < vel_deadband)
        {
            torque_command = 0.0f;
        }
        else
        {
            torque_command =
                -kp * spring_error   // restoring: torque opposes displacement
                -kd * speed_filt;    // damping: torque opposes velocity
        }
    }

    /* ----- Torque LPF (base force only) ----- */

    const float TORQUE_MAX = 1.5f;

    static float torque_filt = 0.0f;

    torque_filt =
        0.90f * torque_filt +
        0.10f * torque_command;

    // Anti-windup: clamp the LPF state itself (not just the output) so a stiff
    // spring at large displacement can't accumulate torque far beyond the usable
    // range and then take many cycles to unwind (which causes overshoot/jitter).
    // Also leaves headroom for the vibration overlay so it isn't clipped.
    float base_limit = TORQUE_MAX - vib_amplitude;
    if (base_limit < 0.0f)
        base_limit = 0.0f;

    if (torque_filt > base_limit)
        torque_filt = base_limit;
    if (torque_filt < -base_limit)
        torque_filt = -base_limit;

    /* ----- Vibration overlay -----
       Added AFTER the LPF so it isn't attenuated by the ~17 Hz filter. */

    static float vib_phase = 0.0f;
    const float TWO_PI = 6.28318530718f;

    float vib = 0.0f;
    if (active && vib_amplitude > 0.0f && vib_freq_hz > 0.0f) {
        vib_phase += TWO_PI * vib_freq_hz / 1000.0f;   // 1 kHz loop rate
        if (vib_phase > TWO_PI)
            vib_phase -= TWO_PI;
        vib = vib_amplitude * sinf(vib_phase);
    }

    float torque_out = torque_filt + vib;

    /* ----- Saturation (final safety clamp) ----- */

    if (torque_out > TORQUE_MAX)
        torque_out = TORQUE_MAX;

    if (torque_out < -TORQUE_MAX)
        torque_out = -TORQUE_MAX;

    MC_ProgramTorqueRampMotor1_F(
        torque_out * (float)direction_correction,
        0);

    /* ----- Measured torque & user-torque estimate -----
       Iq (quadrature current) is what the motor actually produced this cycle, in
       Amperes and proportional to torque. Bring it back into the trigger sign
       convention used by the command (see direction_correction above). */
    qd_f_t iqd = MC_GetIqdMotor1_F();
    float motor_torque = iqd.q * (float)direction_correction;   // A) actual motor torque

    /* User torque from the trigger's equation of motion:
         J*alpha = tau_motor + tau_user - b*omega - friction
       => tau_user ~= -tau_motor + J*alpha + b*omega
       Quasi-static (holding steady) this reduces to tau_user ~= -tau_motor, i.e. the
       user pushes equal-and-opposite to the motor. J_EST/B_EST default to 0; raise
       them (via force-sensor calibration) to correct for inertia/damping during motion. */
    static float prev_speed = 0.0f;
    float alpha = (speed_filt - prev_speed) * 1000.0f;          // deg/s^2 at the 1 kHz rate
    prev_speed = speed_filt;
    const float J_EST = 0.0f;   // inertia term (tune later)
    const float B_EST = 0.0f;   // viscous damping term (tune later)
    float user_torque = -motor_torque + J_EST * alpha + B_EST * speed_filt;   // B) user torque

    /* ----- Debug ----- */

    static uint32_t counter = 0;

    if (++counter >= 20) {
        counter = 0;

        debug_position = pos_deg;
        debug_speed = speed_filt;
        debug_torque = torque_filt;
        debug_motor_torque = motor_torque;
        debug_user_torque  = user_torque;
        debug_ready = true;
    }
// OLD VERSION OF THE SPRING LOOP
//	if (MC_GetSTMStateMotor1() == RUN) {
//		torque_command = 0;
//		if(position_error > RAMP_WIDTH) {
//			if (control_type == CONSTANT) {
//				torque_command = -torque_setpoint;
//			} else {
//				torque_command = -torque_setpoint * position_error;
//			}
//		} else if (position_error < -RAMP_WIDTH) {
//			if (control_type == CONSTANT) {
//				torque_command = torque_setpoint;
//			} else {
//				torque_command = -torque_setpoint * position_error;
//			}
//		} else {
//			if (position_error > DEADBAND_WIDTH) {
//				torque_command = ((float)(position_error-DEADBAND_WIDTH)/(RAMP_WIDTH-DEADBAND_WIDTH)) * -torque_setpoint;
//			} else if (position_error < -DEADBAND_WIDTH) {
//				torque_command = ((float)(fabsf(position_error)-DEADBAND_WIDTH)/(RAMP_WIDTH-DEADBAND_WIDTH)) * torque_setpoint;
//			} else {
//				float speed_threshold = 3.0f; // Ticks per second - TUNE THIS
//				if (fabsf(curr_speed) < speed_threshold && abs(position_error) < (DEADBAND_WIDTH / 4)) {
//				   // If speed is very low AND we are very close to the center, do nothing.
//				   torque_command = 0.0f;
//				} else {
//				  // Otherwise, apply damping
//				  torque_command = -0.005 * curr_speed; // Use your damping gain Kd
//				}
//			}
//		}
//		if (torque_command > 1.5) {
//			torque_command = 1.5;
//		} else if (torque_command < -1.5) {
//			torque_command = -1.5;
//		}
//		if (++counter >= 20) {
//			counter = 0;
//			debug_position = position_error;
//			debug_speed = curr_speed;
//			debug_torque = torque_command;
//			debug_ready = true;
//		}
//		MC_ProgramTorqueRampMotor1_F(torque_command * (double)direction_correction, 0);
//	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM6) {
		loop_1ms();
	}
}

int16_t detect_motor_direction(void) {
	float test_torque = 0.5;
	uint32_t test_duration_ms = 50;
	uint32_t timeout = 5000;
	int32_t threshhold = 500;
	while (MC_GetSTMStateMotor1() != RUN && timeout > 0) {
		HAL_Delay(1);
		timeout--;
	}
	if (MC_GetSTMStateMotor1() != RUN) {
		return 1; // no change
	}
	int32_t initial_pos = get_motor_pos();
	MC_ProgramTorqueRampMotor1_F(test_torque, 0);
	HAL_Delay(test_duration_ms);
	int32_t final_pos = get_motor_pos();
	MC_ProgramTorqueRampMotor1_F(0, 0);
	HAL_Delay(50);
	int32_t delta = final_pos - initial_pos;
	if (delta <-threshhold) {
		return -1;
	}
	else if (delta > threshhold) {
		return 1;
	} else {
		return 1;
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	HAL_Init();
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_COMP1_Init();
	MX_COMP2_Init();
	MX_COMP4_Init();
	MX_CORDIC_Init();
	MX_DAC3_Init();
	MX_OPAMP1_Init();
	MX_OPAMP2_Init();
	MX_OPAMP3_Init();
	MX_TIM1_Init();
	MX_TIM4_Init();
	MX_TIM6_Init();
	MX_USART2_UART_Init();
	MX_MotorControl_Init();

	/* Initialize interrupts */
	MX_NVIC_Init();
	HAL_UART_Receive_IT(&huart2, &rx_char, 1);
	torque_setpoint = 0;
	control_type = SPRING;
	if (IDLE == MC_GetSTMStateMotor1()) {
		MC_ProgramTorqueRampMotor1(0.3, 0);
		MC_StartMotor1();
	}
	direction_correction = MOTOR_DIRECTION;   // fixed sign; see MOTOR_DIRECTION define

	/* Set current handle position as spring center */
	position_offset = (int32_t)ENCODER_M1._Super.hMecAngle;

	HAL_TIM_Base_Start_IT(&htim6);
	while (1) {
		if (command_ready) {
			handle_command();
		}
		if (debug_ready) {
			debug_ready = false;
//			printf("Pos:%f, Sp:%f, Tor:%f, MotorState:%d \n", debug_position, debug_speed, debug_torque, MC_GetSTMStateMotor1());
		}
	}
//	while (1) {
//	    printf("%ld\n", TIM4->CNT);
//	    HAL_Delay(5);
//	}

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
	RCC_OscInitStruct.PLL.PLLN = 85;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
	RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
	RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}

	/** Enables the Clock Security System
	 */
	HAL_RCC_EnableCSS();
}

/**
 * @brief NVIC Configuration.
 * @retval None
 */
static void MX_NVIC_Init(void) {
	/* USART2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(USART2_IRQn, 3, 1);
	HAL_NVIC_EnableIRQ(USART2_IRQn);
	/* DMA1_Channel1_IRQn interrupt configuration (FOR USART2_RX) */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
	/* DMA1_Channel2_IRQn interrupt configuration (FOR USART2_TX) */
	HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
	/* TIM1_BRK_TIM15_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 4, 1);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
	/* TIM1_UP_TIM16_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM1_UP_TIM16_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_UP_TIM16_IRQn);
	/* ADC1_2_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(ADC1_2_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
	/* TIM4_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM4_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(TIM4_IRQn);
	/* EXTI15_10_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	/* TIM6_DAC_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

/**
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_MultiModeTypeDef multimode = { 0 };
	ADC_InjectionConfTypeDef sConfigInjected = { 0 };
	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Common config
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_LEFT;
	hadc1.Init.GainCompensation = 0;
	hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.NbrOfConversion = 3;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure the ADC multi-mode
	 */
	multimode.Mode = ADC_MODE_INDEPENDENT;
	if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
	sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
	sConfigInjected.InjectedOffset = 0;
	sConfigInjected.InjectedNbrOfConversion = 2;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.QueueInjectedContext = DISABLE;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
	sConfigInjected.ExternalTrigInjecConvEdge =
			ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
	sConfigInjected.InjecOversamplingMode = DISABLE;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_12;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_47CYCLES_5;
	sConfig.SingleDiff = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	sConfig.Offset = 0;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = ADC_REGULAR_RANK_2;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = ADC_REGULAR_RANK_3;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

}

/**
 * @brief ADC2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC2_Init(void) {

	/* USER CODE BEGIN ADC2_Init 0 */

	/* USER CODE END ADC2_Init 0 */

	ADC_InjectionConfTypeDef sConfigInjected = { 0 };

	/* USER CODE BEGIN ADC2_Init 1 */

	/* USER CODE END ADC2_Init 1 */

	/** Common config
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc2.Init.Resolution = ADC_RESOLUTION_12B;
	hadc2.Init.DataAlign = ADC_DATAALIGN_LEFT;
	hadc2.Init.GainCompensation = 0;
	hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
	hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc2.Init.LowPowerAutoWait = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc2.Init.OversamplingMode = DISABLE;
	if (HAL_ADC_Init(&hadc2) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_VOPAMP3_ADC2;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
	sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_6CYCLES_5;
	sConfigInjected.InjectedSingleDiff = ADC_SINGLE_ENDED;
	sConfigInjected.InjectedOffsetNumber = ADC_OFFSET_NONE;
	sConfigInjected.InjectedOffset = 0;
	sConfigInjected.InjectedNbrOfConversion = 2;
	sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
	sConfigInjected.AutoInjectedConv = DISABLE;
	sConfigInjected.QueueInjectedContext = DISABLE;
	sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_CC4;
	sConfigInjected.ExternalTrigInjecConvEdge =
			ADC_EXTERNALTRIGINJECCONV_EDGE_RISING;
	sConfigInjected.InjecOversamplingMode = DISABLE;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Injected Channel
	 */
	sConfigInjected.InjectedChannel = ADC_CHANNEL_3;
	sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
	if (HAL_ADCEx_InjectedConfigChannel(&hadc2, &sConfigInjected) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC2_Init 2 */

	/* USER CODE END ADC2_Init 2 */

}

/**
 * @brief COMP1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP1_Init(void) {

	/* USER CODE BEGIN COMP1_Init 0 */

	/* USER CODE END COMP1_Init 0 */

	/* USER CODE BEGIN COMP1_Init 1 */

	/* USER CODE END COMP1_Init 1 */
	hcomp1.Instance = COMP1;
	hcomp1.Init.InputPlus = COMP_INPUT_PLUS_IO1;
	hcomp1.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH1;
	hcomp1.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp1.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp1.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp1.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	if (HAL_COMP_Init(&hcomp1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN COMP1_Init 2 */

	/* USER CODE END COMP1_Init 2 */

}

/**
 * @brief COMP2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP2_Init(void) {

	/* USER CODE BEGIN COMP2_Init 0 */

	/* USER CODE END COMP2_Init 0 */

	/* USER CODE BEGIN COMP2_Init 1 */

	/* USER CODE END COMP2_Init 1 */
	hcomp2.Instance = COMP2;
	hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
	hcomp2.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
	hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	if (HAL_COMP_Init(&hcomp2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN COMP2_Init 2 */

	/* USER CODE END COMP2_Init 2 */

}

/**
 * @brief COMP4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_COMP4_Init(void) {

	/* USER CODE BEGIN COMP4_Init 0 */

	/* USER CODE END COMP4_Init 0 */

	/* USER CODE BEGIN COMP4_Init 1 */

	/* USER CODE END COMP4_Init 1 */
	hcomp4.Instance = COMP4;
	hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
	hcomp4.Init.InputMinus = COMP_INPUT_MINUS_DAC3_CH2;
	hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
	hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
	hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
	hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
	if (HAL_COMP_Init(&hcomp4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN COMP4_Init 2 */

	/* USER CODE END COMP4_Init 2 */

}

/**
 * @brief CORDIC Initialization Function
 * @param None
 * @retval None
 */
static void MX_CORDIC_Init(void) {

	/* USER CODE BEGIN CORDIC_Init 0 */

	/* USER CODE END CORDIC_Init 0 */

	/* USER CODE BEGIN CORDIC_Init 1 */

	/* USER CODE END CORDIC_Init 1 */
	hcordic.Instance = CORDIC;
	if (HAL_CORDIC_Init(&hcordic) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN CORDIC_Init 2 */

	/* USER CODE END CORDIC_Init 2 */

}

/**
 * @brief DAC3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_DAC3_Init(void) {

	/* USER CODE BEGIN DAC3_Init 0 */

	/* USER CODE END DAC3_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC3_Init 1 */

	/* USER CODE END DAC3_Init 1 */

	/** DAC Initialization
	 */
	hdac3.Instance = DAC3;
	if (HAL_DAC_Init(&hdac3) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT1 config
	 */
	sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_AUTOMATIC;
	sConfig.DAC_DMADoubleDataMode = DISABLE;
	sConfig.DAC_SignedFormat = DISABLE;
	sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
	sConfig.DAC_Trigger = DAC_TRIGGER_NONE;
	sConfig.DAC_Trigger2 = DAC_TRIGGER_NONE;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_DISABLE;
	sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_INTERNAL;
	sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
	if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}

	/** DAC channel OUT2 config
	 */
	if (HAL_DAC_ConfigChannel(&hdac3, &sConfig, DAC_CHANNEL_2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN DAC3_Init 2 */

	/* USER CODE END DAC3_Init 2 */

}

/**
 * @brief OPAMP1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OPAMP1_Init(void) {

	/* USER CODE BEGIN OPAMP1_Init 0 */

	/* USER CODE END OPAMP1_Init 0 */

	/* USER CODE BEGIN OPAMP1_Init 1 */

	/* USER CODE END OPAMP1_Init 1 */
	hopamp1.Instance = OPAMP1;
	hopamp1.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	hopamp1.Init.Mode = OPAMP_PGA_MODE;
	hopamp1.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	hopamp1.Init.InternalOutput = DISABLE;
	hopamp1.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp1.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
	hopamp1.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
	hopamp1.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	if (HAL_OPAMP_Init(&hopamp1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN OPAMP1_Init 2 */

	/* USER CODE END OPAMP1_Init 2 */

}

/**
 * @brief OPAMP2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OPAMP2_Init(void) {

	/* USER CODE BEGIN OPAMP2_Init 0 */

	/* USER CODE END OPAMP2_Init 0 */

	/* USER CODE BEGIN OPAMP2_Init 1 */

	/* USER CODE END OPAMP2_Init 1 */
	hopamp2.Instance = OPAMP2;
	hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	hopamp2.Init.Mode = OPAMP_PGA_MODE;
	hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	hopamp2.Init.InternalOutput = DISABLE;
	hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp2.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
	hopamp2.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
	hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	if (HAL_OPAMP_Init(&hopamp2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN OPAMP2_Init 2 */

	/* USER CODE END OPAMP2_Init 2 */

}

/**
 * @brief OPAMP3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_OPAMP3_Init(void) {

	/* USER CODE BEGIN OPAMP3_Init 0 */

	/* USER CODE END OPAMP3_Init 0 */

	/* USER CODE BEGIN OPAMP3_Init 1 */

	/* USER CODE END OPAMP3_Init 1 */
	hopamp3.Instance = OPAMP3;
	hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMALSPEED;
	hopamp3.Init.Mode = OPAMP_PGA_MODE;
	hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
	hopamp3.Init.InternalOutput = ENABLE;
	hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
	hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0_BIAS;
	hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_16_OR_MINUS_15;
	hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
	if (HAL_OPAMP_Init(&hopamp3) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN OPAMP3_Init 2 */

	/* USER CODE END OPAMP3_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	TIMEx_BreakInputConfigTypeDef sBreakInputConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };
	TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = ((TIM_CLOCK_DIVIDER) - 1);
	htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1;
	htim1.Init.Period = ((PWM_PERIOD_CYCLES ) / 2);
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV2;
	htim1.Init.RepetitionCounter = (REP_COUNTER );
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_OC4REF;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP1;
	sBreakInputConfig.Enable = TIM_BREAKINPUTSOURCE_ENABLE;
	sBreakInputConfig.Polarity = TIM_BREAKINPUTSOURCE_POLARITY_HIGH;
	if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK,
			&sBreakInputConfig) != HAL_OK) {
		Error_Handler();
	}
	sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP2;
	if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK,
			&sBreakInputConfig) != HAL_OK) {
		Error_Handler();
	}
	sBreakInputConfig.Source = TIM_BREAKINPUTSOURCE_COMP4;
	if (HAL_TIMEx_ConfigBreakInput(&htim1, TIM_BREAKINPUT_BRK,
			&sBreakInputConfig) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = ((PWM_PERIOD_CYCLES ) / 4);
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
	sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM2;
	sConfigOC.Pulse = (((PWM_PERIOD_CYCLES ) / 2) - (HTMIN));
	if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4)
			!= HAL_OK) {
		Error_Handler();
	}
	sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
	sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
	sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
	sBreakDeadTimeConfig.DeadTime = ((DEAD_TIME_COUNTS ) / 2);
	sBreakDeadTimeConfig.BreakState = TIM_BREAK_ENABLE;
	sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
	sBreakDeadTimeConfig.BreakFilter = 3;
	sBreakDeadTimeConfig.BreakAFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
	sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
	sBreakDeadTimeConfig.Break2Filter = 3;
	sBreakDeadTimeConfig.Break2AFMode = TIM_BREAK_AFMODE_INPUT;
	sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
	if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */
	HAL_TIM_MspPostInit(&htim1);

}

/**
 * @brief TIM4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM4_Init(void) {

	/* USER CODE BEGIN TIM4_Init 0 */

	/* USER CODE END TIM4_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM4_Init 1 */

	/* USER CODE END TIM4_Init 1 */
	htim4.Instance = TIM4;
	htim4.Init.Prescaler = 0;
	htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim4.Init.Period = M1_PULSE_NBR;
	htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = M1_ENC_IC_FILTER;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = M1_ENC_IC_FILTER;
	if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM4_Init 2 */

	/* USER CODE END TIM4_Init 2 */

}

/**
 * @brief USART2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART2_UART_Init(void) {

	/* USER CODE BEGIN USART2_Init 0 */

	/* USER CODE END USART2_Init 0 */

	/* USER CODE BEGIN USART2_Init 1 */

	/* USER CODE END USART2_Init 1 */
	huart2.Instance = USART2;
	//huart2.Init.BaudRate = 1843200;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart2) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART2_Init 2 */
	/* USER CODE END USART2_Init 2 */

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMAMUX1_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */

	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : Start_Stop_Pin */
	GPIO_InitStruct.Pin = Start_Stop_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Start_Stop_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */

	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
static void MX_TIM6_Init() {

	__HAL_RCC_TIM6_CLK_ENABLE();
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 170-1;
	htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6.Init.Period = 1000-1;
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
