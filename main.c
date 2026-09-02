#include "main.h"

#include "NRF24.h"
#include "NRF24_reg_addresses.h"
#include "mpu6050.h"
#include <string.h>
#include <stdbool.h>
#include <math.h>

typedef struct __attribute__((packed)) {
    uint16_t channels[8];
    bool toggles[4];
    bool buttons[8];
    uint32_t servoMapPacked;
} TransmitterData_t;

typedef struct __attribute__((packed)) {
    uint16_t batteryMv;
    uint8_t batteryPercent;
    uint8_t flags;
    uint32_t packetCount;
    uint16_t currentCentiAmp;
    uint16_t consumedMah;
    uint8_t linkQuality;
    uint8_t state;
    int8_t rollDeg;
    int8_t pitchDeg;
    uint8_t calibrationProgress;
} TelemetryData_t;

typedef char TelemetryData_Size_Must_Be_17[(sizeof(TelemetryData_t) == 17) ? 1 : -1];

typedef char TransmitterData_Size_Must_Be_32[(sizeof(TransmitterData_t) == 32) ? 1 : -1];

#define PLD_S sizeof(TransmitterData_t)
#define TLM_S sizeof(TelemetryData_t)
#define R_RX_PL_WID 0x60U

#define SERVO_MIN 1000
#define SERVO_MAX 2000
#define SERVO_CENTER 1500
#define SERVO_JITTER_DEADBAND_US 20
#define NORMAL_CENTER_DEADBAND_CH 60U

#define OUT_SERVO1 0
#define OUT_SERVO2 1
#define OUT_SERVO3 2
#define OUT_SERVO4 3
#define OUT_SERVO5 4
#define OUT_SERVO6 5
#define OUT_MOTOR  6
#define OUT_NONE   7

#define PLANE_WING 0
#define PLANE_NORMAL 1
#define PLANE_VTAIL 2

#define SERVO1_CH TIM_CHANNEL_1
#define SERVO2_CH TIM_CHANNEL_2
#define SERVO3_CH TIM_CHANNEL_3
#define SERVO4_CH TIM_CHANNEL_1
#define SERVO5_CH TIM_CHANNEL_3
#define SERVO6_CH TIM_CHANNEL_4
#define MOTOR_CH  TIM_CHANNEL_3

#define BATTERY_CELLS 3
#define BATTERY_FULL_VOLTAGE 12.6f
#define BATTERY_EMPTY_VOLTAGE 10.5f
#define BATTERY_LOW_WARNING 10.8f
#define BATTERY_CRITICAL 10.2f
#define VOLTAGE_DIVIDER_RATIO 4.8605f
#define ADC_REFERENCE_VOLTAGE 3.300f
#define VOLTAGE_CALIBRATION_MULTIPLIER 0.9471f  // calibrated: 12.20V indicated -> 11.555V DMM

#define CURRENT_SENSOR_ZERO_V 0.005f
#define CURRENT_SENSOR_VOLTS_PER_AMP 0.0831f   // calibrated from 5mV zero, 56.5mV @ 0.62A
#define CURRENT_NOISE_FLOOR_A 0.03f
#define BATTERY_CAPACITY_MAH 2000.0f
#define FC_LED_PORT GPIOB
#define FC_LED_PIN GPIO_PIN_5

typedef struct {
    float Kp, Ki, Kd;
    float integral;
    float prev_error;
    float output;
    float integral_limit;
} PID_t;

#define ANGLE_ROLL_KP  4.0f
#define ANGLE_PITCH_KP 4.0f
#define RATE_ROLL_KP   1.60f
#define RATE_ROLL_KI   0.25f
#define RATE_ROLL_KD   0.015f
#define RATE_PITCH_KP  1.60f
#define RATE_PITCH_KI  0.25f
#define RATE_PITCH_KD  0.015f
#define RATE_YAW_KP    1.00f
#define RATE_YAW_KI    0.10f
#define RATE_YAW_KD    0.000f

#define MPU6050_ADDR MPU6050_I2C_ADDR_68
#define ACC_LSB_PER_G 16384.0f
#define GYRO_LSB_PER_DPS 131.0f
#define ALPHA 0.95f

#define STAB_ERROR_DEADBAND_DEG 0.20f
#define ACRO_GYRO_NOISE_DPS      1.20f
#define IRQ_INT_BUILD 1
#define CONTROL_LOOP_PERIOD_MS 5U
#define FAILSAFE_HOLD_MS 200U
#define FAILSAFE_TRIGGER_MS 400U
#define ARM_THROTTLE_MAX_CH 120U
#define ARM_LINK_MIN_PACKETS 25U
#define IMU_CALIBRATION_SAMPLES 400U
#define MAX_ROLL_ANGLE_DEG 35.0f
#define MAX_PITCH_ANGLE_DEG 25.0f
#define MAX_ROLL_RATE_DPS 120.0f
#define MAX_PITCH_RATE_DPS 100.0f
#define MAX_YAW_RATE_DPS 90.0f
#define MAX_STABILIZATION_US 350.0f

typedef struct {
    uint16_t min_us;
    uint16_t center_us;
    uint16_t max_us;
    bool reversed;
} OutputConfig_t;

#define FC_NRF_IRQ_PIN   GPIO_PIN_1
#define FC_NRF_IRQ_PORT  GPIOB
#define FC_MPU_INT_PIN   GPIO_PIN_4
#define FC_MPU_INT_PORT  GPIOA

SPI_HandleTypeDef hspi1;
I2C_HandleTypeDef hi2c1;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
ADC_HandleTypeDef hadc1;

const uint8_t stm32_rx_pipe0[5] = {0x58, 0x70, 0x40, 0x32, 0x37};
const uint8_t stm32_tx_pipe1[5] = {0x58, 0x70, 0x40, 0x32, 0x37};

TransmitterData_t rx_data;
TelemetryData_t tx_telemetry;
uint8_t dataR[PLD_S];
uint32_t packet_count = 0;

float radio_roll = 0.0f;
float radio_pitch = 0.0f;
float radio_throttle = 0.0f;
float radio_yaw = 0.0f;

mpu6050_si_t mpu_data;
float pitch = 0.0f, roll = 0.0f, yaw = 0.0f;
uint8_t mpu_ready = 0;

PID_t pid_rate_roll, pid_rate_pitch, pid_rate_yaw;

uint16_t servo_roll = SERVO_CENTER;
uint16_t servo_pitch = SERVO_CENTER;
uint16_t servo_throttle = SERVO_MIN;
uint16_t servo_yaw = SERVO_CENTER;
uint16_t servo_aux1 = SERVO_CENTER;
uint16_t servo_aux2 = SERVO_CENTER;
uint16_t servo_motor = SERVO_MIN;
uint16_t last_physical_pulse[7] = {1500,1500,1500,1500,1500,1500,1000};

uint16_t adc_current = 0;
uint16_t adc_voltage = 0;
float current_amps = 0.0f;
float consumed_mah = 0.0f;
float minimum_battery_voltage = 99.0f;
float voltage_raw = 0.0f;
float battery_voltage = 0.0f;
float battery_percent = 0.0f;
float battery_start_percent = 0.0f;
bool battery_soc_initialized = false;
uint8_t battery_warning = 0;

bool armed = false;
bool auto_level = true;
bool failsafe = false;
bool failsafe_latched = false;
bool imu_calibrated = false;
bool imu_calibrating = false;
bool imu_calibration_started = false;
uint8_t imu_calibration_progress = 0U;
int8_t telemetry_roll_deg = 0;
int8_t telemetry_pitch_deg = 0;
uint32_t last_packet_time = 0;
uint32_t valid_packets_since_boot = 0;

uint8_t plane_type = 0;

uint8_t servo_map[8] = {OUT_NONE, OUT_NONE, OUT_NONE, OUT_NONE, OUT_NONE, OUT_NONE, OUT_NONE, OUT_NONE};
uint8_t flight_mode = 0;

float dt = 0.01f;
float gyro_bias_x = 0.0f, gyro_bias_y = 0.0f, gyro_bias_z = 0.0f;
uint32_t last_imu_cycle = 0U;
uint32_t last_capacity_update_ms = 0U;
uint32_t link_window_start_ms = 0U;
uint16_t link_window_packets = 0U;
uint8_t receiver_link_quality = 0U;
uint8_t radio_initialized = 0;
bool ack_payload_enabled = false;
uint32_t ack_payload_queued = 0;
uint32_t ack_payload_queue_full = 0;
uint32_t ack_bad_rx_width = 0;
uint32_t last_sensor_update = 0;
uint32_t last_battery_update = 0;
uint32_t last_servo_map_packed = 0xFFFFFFFFUL;
bool output_map_changed = true;

uint32_t ack_payload_pending = 0;

volatile uint8_t nrf_irq_pending = 0;
volatile uint8_t mpu_irq_pending = 0;
volatile uint32_t nrf_irq_count = 0;
volatile uint32_t mpu_irq_count = 0;
uint32_t last_mpu_service_ms = 0;
uint32_t last_radio_guard_ms = 0;
uint32_t last_control_update_ms = 0;
uint16_t desired_physical_pulse[7] = {1500,1500,1500,1500,1500,1500,1000};

OutputConfig_t output_config[7] = {
    {1000,1500,2000,false}, {1000,1500,2000,false},
    {1000,1500,2000,false}, {1000,1500,2000,false},
    {1000,1500,2000,false}, {1000,1500,2000,false},
    {1000,1000,2000,false}
};

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
void Error_Handler(void);

void delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

static void micros_fast_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t fc_nrf_xfer(uint8_t tx) {
    uint8_t rx_byte = 0U;
    if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx_byte, 1U, 2U) != HAL_OK) {
        return 0xFFU;
    }
    return rx_byte;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == FC_NRF_IRQ_PIN) {
        nrf_irq_pending = 1U;
        nrf_irq_count++;
    }

    if (GPIO_Pin == FC_MPU_INT_PIN) {
        mpu_irq_pending = 1U;
        mpu_irq_count++;
    }
}

__attribute__((weak)) void EXTI1_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(FC_NRF_IRQ_PIN);
}

__attribute__((weak)) void EXTI4_IRQHandler(void) {
    HAL_GPIO_EXTI_IRQHandler(FC_MPU_INT_PIN);
}

void pid_init(PID_t *pid, float Kp, float Ki, float Kd) {
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->integral_limit = 120.0f;
}

float pid_update(PID_t *pid, float error, float measured_rate, float dt_local,
                 float output_limit) {
    if (dt_local < 0.002f || dt_local > 0.03f) dt_local = 0.005f;

    float derivative = -measured_rate;
    float candidate_integral = pid->integral + error * dt_local;
    pid->integral = candidate_integral;
    if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
    if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;

    float output = (pid->Kp * error) +
                   (pid->Ki * pid->integral) +
                   (pid->Kd * derivative);

    if (output > output_limit) {
        output = output_limit;
        if (error > 0.0f) pid->integral -= error * dt_local;
    }
    if (output < -output_limit) {
        output = -output_limit;
        if (error < 0.0f) pid->integral -= error * dt_local;
    }

    pid->prev_error = error;
    pid->output = output;
    return output;
}

void pid_reset_all(void) {
    pid_init(&pid_rate_roll, RATE_ROLL_KP, RATE_ROLL_KI, RATE_ROLL_KD);
    pid_init(&pid_rate_pitch, RATE_PITCH_KP, RATE_PITCH_KI, RATE_PITCH_KD);
    pid_init(&pid_rate_yaw, RATE_YAW_KP, RATE_YAW_KI, RATE_YAW_KD);
}

static void pwm_enable_preload_and_safe_start(void) {

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, SERVO_CENTER);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, SERVO_CENTER);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, SERVO_CENTER);

    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, SERVO_CENTER);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, SERVO_MIN);

    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, SERVO_CENTER);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, SERVO_CENTER);

    htim1.Instance->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
    htim1.Instance->CCMR2 |= TIM_CCMR2_OC3PE;

    htim3.Instance->CCMR1 |= TIM_CCMR1_OC1PE;
    htim3.Instance->CCMR2 |= TIM_CCMR2_OC3PE;

    htim4.Instance->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;

    htim1.Instance->EGR = TIM_EGR_UG;
    htim3.Instance->EGR = TIM_EGR_UG;
    htim4.Instance->EGR = TIM_EGR_UG;

    for (uint8_t i = 0; i < 6; i++) last_physical_pulse[i] = SERVO_CENTER;
    last_physical_pulse[OUT_MOTOR] = SERVO_MIN;
}

static void pwm_write_us(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t pulse_us) {
    if (pulse_us < SERVO_MIN) pulse_us = SERVO_MIN;
    if (pulse_us > SERVO_MAX) pulse_us = SERVO_MAX;

    if ((uint16_t)__HAL_TIM_GET_COMPARE(htim, channel) != pulse_us) {
        __HAL_TIM_SET_COMPARE(htim, channel, pulse_us);
    }
}

void write_physical_output(uint8_t output, uint16_t pulse) {
    if (output > OUT_MOTOR) return;

    OutputConfig_t *cfg = &output_config[output];
    int32_t configured = pulse;
    if (cfg->reversed && output != OUT_MOTOR) {
        configured = (int32_t)cfg->center_us * 2L - configured;
    }
    if (configured < cfg->min_us) configured = cfg->min_us;
    if (configured > cfg->max_us) configured = cfg->max_us;

    desired_physical_pulse[output] = (uint16_t)configured;
}

void write_all_safe(void) {
    for (uint8_t i = 0U; i < 6U; i++) {
        desired_physical_pulse[i] = SERVO_CENTER;
    }
    desired_physical_pulse[OUT_MOTOR] = SERVO_MIN;
}

static void pwm_commit_targets(void) {
    pwm_write_us(&htim1, SERVO1_CH, desired_physical_pulse[OUT_SERVO1]);
    pwm_write_us(&htim1, SERVO2_CH, desired_physical_pulse[OUT_SERVO2]);
    pwm_write_us(&htim1, SERVO3_CH, desired_physical_pulse[OUT_SERVO3]);
    pwm_write_us(&htim3, SERVO4_CH, desired_physical_pulse[OUT_SERVO4]);
    pwm_write_us(&htim4, SERVO5_CH, desired_physical_pulse[OUT_SERVO5]);
    pwm_write_us(&htim4, SERVO6_CH, desired_physical_pulse[OUT_SERVO6]);
    pwm_write_us(&htim3, MOTOR_CH, desired_physical_pulse[OUT_MOTOR]);
}

void decode_servo_map(uint32_t packed) {
    // Packed TX configuration:
    //   bits  0..23 = logical function -> physical output map
    //   bits 24..25 = plane type
    //   bits 26..31 = physical servo reverse flags S1..S6
    //
    // IMPORTANT: compare the COMPLETE 32-bit word.  The previous calibrated
    // FC masked the word to 26 bits before the comparison, so changing only a
    // reverse flag looked like "nothing changed" and was ignored.
    if (packed == last_servo_map_packed) return;
    last_servo_map_packed = packed;

    uint32_t mapOnly = packed & 0x03FFFFFFUL;

    for (uint8_t i = 0; i < 8; i++) {
        servo_map[i] = (mapOnly >> (i * 3)) & 0x07U;
    }

    plane_type = (uint8_t)((mapOnly >> 24) & 0x03U);
    if (plane_type > 2U) plane_type = 0U;

    // Reverse is defined by PHYSICAL output, not by logical function.
    // Bit 26=S1, 27=S2, ... 31=S6. Motor is never reversed here.
    for (uint8_t i = 0; i < 6U; i++) {
        output_config[i].reversed =
            ((packed >> (26U + i)) & 0x01U) ? true : false;
    }
    output_config[OUT_MOTOR].reversed = false;

    // Forces the output layer to recognize the new configuration immediately.
    output_map_changed = true;
}

uint16_t channel_to_servo(uint16_t value) {
    return SERVO_MIN + (uint16_t)(((uint32_t)value * 1000UL) / 4095UL);
}

uint16_t stick_to_servo(uint16_t value) {
    int32_t pulse = SERVO_CENTER + ((int32_t)value - 2048L) * 500L / 2048L;
    if (pulse < SERVO_MIN) pulse = SERVO_MIN;
    if (pulse > SERVO_MAX) pulse = SERVO_MAX;
    return (uint16_t)pulse;
}

static uint16_t normal_stick_to_servo(uint16_t value) {
    return stick_to_servo(value);
}

void apply_function(uint8_t function, uint16_t pulse) {
    if (function >= 8) return;
    if (servo_map[function] == OUT_NONE) return;
    write_physical_output(servo_map[function], pulse);
}

void update_outputs(uint16_t aileron, uint16_t elevator, uint16_t throttle,
                    uint16_t rudder, uint16_t flaps, uint16_t gear,
                    uint16_t aux1, uint16_t aux2) {
    for (uint8_t i = 0U; i < 6U; i++) {
        desired_physical_pulse[i] = SERVO_CENTER;
    }
    desired_physical_pulse[OUT_MOTOR] = SERVO_MIN;

    output_map_changed = false;

    apply_function(0, aileron);
    apply_function(1, elevator);
    apply_function(2, throttle);
    apply_function(3, rudder);
    apply_function(4, flaps);
    apply_function(5, gear);
    apply_function(6, aux1);
    apply_function(7, aux2);
}

static float battery_voltage_to_percent(float v) {
    float p;
    if (v >= 12.60f) p = 100.0f;
    else if (v >= 12.50f) p = 97.0f + (v - 12.50f) * 30.0f;
    else if (v >= 12.30f) p = 90.0f + (v - 12.30f) * 35.0f;
    else if (v >= 12.00f) p = 80.0f + (v - 12.00f) * 33.3333f;
    else if (v >= 11.70f) p = 60.0f + (v - 11.70f) * 66.6667f;
    else if (v >= 11.40f) p = 35.0f + (v - 11.40f) * 83.3333f;
    else if (v >= 11.10f) p = 15.0f + (v - 11.10f) * 66.6667f;
    else if (v >= 10.80f) p = 5.0f + (v - 10.80f) * 33.3333f;
    else if (v > 10.50f) p = (v - 10.50f) * 16.6667f;
    else p = 0.0f;
    if (p < 0.0f) p = 0.0f;
    if (p > 100.0f) p = 100.0f;
    return p;
}

static void led_pb5_init(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    __HAL_RCC_GPIOB_CLK_ENABLE();
    HAL_GPIO_WritePin(FC_LED_PORT, FC_LED_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = FC_LED_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(FC_LED_PORT, &GPIO_InitStruct);
}

static uint16_t adc_read_average(uint32_t channel, uint8_t samples) {
    ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &sConfig);

    // Discard one conversion after switching ADC channels. This avoids the
    // previous channel's sample/hold charge slightly biasing the new reading.
    HAL_ADC_Start(&hadc1);
    (void)HAL_ADC_PollForConversion(&hadc1, 2);
    (void)HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);

    uint32_t sum = 0U;
    uint8_t valid = 0U;
    for (uint8_t i = 0U; i < samples; i++) {
        HAL_ADC_Start(&hadc1);
        if (HAL_ADC_PollForConversion(&hadc1, 2) == HAL_OK) {
            sum += HAL_ADC_GetValue(&hadc1);
            valid++;
        }
        HAL_ADC_Stop(&hadc1);
    }
    return valid ? (uint16_t)(sum / valid) : 0U;
}

void calculate_battery(void) {
    // 32-sample averaging gives a steadier voltage/current display without
    // changing the radio or control loop timing (battery task is only 4 Hz).
    adc_voltage = adc_read_average(ADC_CHANNEL_1, 32U);
    voltage_raw = ((float)adc_voltage * ADC_REFERENCE_VOLTAGE) / 4095.0f;

    // Divider ratio is the hardware ratio. The multiplier is the measured
    // system calibration, covering resistor tolerance + ADC/reference error.
    float measured_voltage = voltage_raw * VOLTAGE_DIVIDER_RATIO * VOLTAGE_CALIBRATION_MULTIPLIER;
    if (measured_voltage < 0.0f) measured_voltage = 0.0f;

    battery_voltage = (battery_voltage <= 0.1f)
        ? measured_voltage
        : battery_voltage * 0.88f + measured_voltage * 0.12f;

    if (battery_voltage > 1.0f && battery_voltage < minimum_battery_voltage) {
        minimum_battery_voltage = battery_voltage;
    }

    adc_current = adc_read_average(ADC_CHANNEL_0, 32U);
    float current_sensor_voltage = ((float)adc_current * ADC_REFERENCE_VOLTAGE) / 4095.0f;
    float current_signal = current_sensor_voltage - CURRENT_SENSOR_ZERO_V;
    if (current_signal < 0.0f) current_signal = 0.0f;

    float measured_current = current_signal / CURRENT_SENSOR_VOLTS_PER_AMP;
    if (measured_current < CURRENT_NOISE_FLOOR_A) measured_current = 0.0f;

    // Smooth enough for a readable TX display while still responding quickly
    // to motor load changes.
    if (current_amps <= 0.001f) current_amps = measured_current;
    else current_amps = current_amps * 0.82f + measured_current * 0.18f;

    uint32_t now_ms = HAL_GetTick();
    if (last_capacity_update_ms != 0U) {
        uint32_t elapsed_ms = now_ms - last_capacity_update_ms;
        if (elapsed_ms <= 1500U) {
            // A * milliseconds / 3600 = mAh.
            consumed_mah += current_amps * ((float)elapsed_ms / 3600.0f);
        }
    }
    last_capacity_update_ms = now_ms;

    float voltage_percent = battery_voltage_to_percent(battery_voltage);

    if (!battery_soc_initialized && battery_voltage > 9.0f) {
        battery_start_percent = voltage_percent;
        battery_percent = voltage_percent;
        consumed_mah = 0.0f;
        battery_soc_initialized = true;
    } else if (battery_soc_initialized) {
        float used_percent = (consumed_mah / BATTERY_CAPACITY_MAH) * 100.0f;
        float coulomb_percent = battery_start_percent - used_percent;
        if (coulomb_percent < 0.0f) coulomb_percent = 0.0f;
        if (coulomb_percent > 100.0f) coulomb_percent = 100.0f;

        // Voltage safety clamps prevent an optimistic mAh estimate from
        // reporting a healthy pack when cell voltage is already too low.
        if (battery_voltage <= 10.50f) coulomb_percent = 0.0f;
        else if (battery_voltage <= 10.80f && coulomb_percent > 5.0f) coulomb_percent = 5.0f;
        else if (battery_voltage <= 11.10f && coulomb_percent > 15.0f) coulomb_percent = 15.0f;

        battery_percent = coulomb_percent;
    }

    if (battery_percent < 0.0f) battery_percent = 0.0f;
    if (battery_percent > 100.0f) battery_percent = 100.0f;

    if (battery_warning == 2) {
        if (battery_voltage > (BATTERY_CRITICAL + 0.35f)) {
            battery_warning = (battery_voltage <= BATTERY_LOW_WARNING) ? 1 : 0;
        }
    } else if (battery_warning == 1) {
        if (battery_voltage <= BATTERY_CRITICAL) {
            battery_warning = 2;
        } else if (battery_voltage > (BATTERY_LOW_WARNING + 0.30f)) {
            battery_warning = 0;
        }
    } else {
        if (battery_voltage <= BATTERY_CRITICAL) battery_warning = 2;
        else if (battery_voltage <= BATTERY_LOW_WARNING) battery_warning = 1;
    }
}

void process_radio(TransmitterData_t *data) {
    radio_roll = ((float)data->channels[0] - 2048.0f) / 2048.0f * 100.0f;
    radio_pitch = ((float)data->channels[1] - 2048.0f) / 2048.0f * 100.0f;
    radio_throttle = ((float)data->channels[2]) / 4095.0f * 100.0f;
    radio_yaw = ((float)data->channels[3] - 2048.0f) / 2048.0f * 100.0f;

    if (fabsf(radio_roll) < 0.5f) radio_roll = 0.0f;
    if (fabsf(radio_pitch) < 0.5f) radio_pitch = 0.0f;
    if (fabsf(radio_yaw) < 0.5f) radio_yaw = 0.0f;

    decode_servo_map(data->servoMapPacked);

    flight_mode = (uint8_t)(((uint32_t)data->channels[4] * 3UL) / 4096UL);
    if (flight_mode > 2U) flight_mode = 2U;
    auto_level = (flight_mode == 1U);

    HAL_GPIO_WritePin(FC_LED_PORT, FC_LED_PIN, data->toggles[3] ? GPIO_PIN_SET : GPIO_PIN_RESET);

    bool requested_arm = data->toggles[0] ? true : false;
    if (!requested_arm) {
        if (armed) pid_reset_all();
        armed = false;
        failsafe_latched = false;
    } else if (!armed && !failsafe_latched && imu_calibrated &&
               data->channels[2] <= ARM_THROTTLE_MAX_CH &&
               valid_packets_since_boot >= ARM_LINK_MIN_PACKETS) {
        armed = true;
        pid_reset_all();
    }

    last_packet_time = HAL_GetTick();

    valid_packets_since_boot++;
    link_window_packets++;

    if (failsafe && !failsafe_latched) {
        failsafe = false;
        pid_reset_all();
    }
}

static uint16_t throttle_channel_to_esc_us(uint16_t channel) {

    if (channel > 4095U) channel = 4095U;

    if (channel <= 20U) return 1000U;
    if (channel >= 4075U) return 2000U;

    return (uint16_t)(1000UL +
           (((uint32_t)channel * 1000UL + 2047UL) / 4095UL));
}

void flight_controller(void) {
    if (failsafe) {
        write_all_safe();
        return;
    }

    uint16_t aileron = normal_stick_to_servo(rx_data.channels[0]);
    uint16_t elevator = normal_stick_to_servo(rx_data.channels[1]);
    uint16_t rudder = normal_stick_to_servo(rx_data.channels[3]);

    uint16_t throttle_pulse = armed
        ? throttle_channel_to_esc_us(rx_data.channels[2])
        : SERVO_MIN;

    uint16_t flaps = channel_to_servo(rx_data.channels[7]);
    uint16_t gear = (rx_data.channels[5] >= 2048U) ? SERVO_MAX : SERVO_MIN;
    uint16_t aux1 = channel_to_servo(rx_data.channels[6]);
    uint16_t aux2 = channel_to_servo(rx_data.channels[7]);

    if (flight_mode != 0U && mpu_ready && imu_calibrated) {
        float target_roll_rate;
        float target_pitch_rate;

        if (flight_mode == 1U) {
            float target_roll = radio_roll * MAX_ROLL_ANGLE_DEG / 100.0f;
            float target_pitch = radio_pitch * MAX_PITCH_ANGLE_DEG / 100.0f;
            float roll_error = target_roll - roll;
            float pitch_error = target_pitch - pitch;
            if (fabsf(roll_error) < STAB_ERROR_DEADBAND_DEG) roll_error = 0.0f;
            if (fabsf(pitch_error) < STAB_ERROR_DEADBAND_DEG) pitch_error = 0.0f;
            target_roll_rate = roll_error * ANGLE_ROLL_KP;
            target_pitch_rate = pitch_error * ANGLE_PITCH_KP;
            if (target_roll_rate > MAX_ROLL_RATE_DPS) target_roll_rate = MAX_ROLL_RATE_DPS;
            if (target_roll_rate < -MAX_ROLL_RATE_DPS) target_roll_rate = -MAX_ROLL_RATE_DPS;
            if (target_pitch_rate > MAX_PITCH_RATE_DPS) target_pitch_rate = MAX_PITCH_RATE_DPS;
            if (target_pitch_rate < -MAX_PITCH_RATE_DPS) target_pitch_rate = -MAX_PITCH_RATE_DPS;
        } else {
            target_roll_rate = radio_roll * MAX_ROLL_RATE_DPS / 100.0f;
            target_pitch_rate = radio_pitch * MAX_PITCH_RATE_DPS / 100.0f;
        }

        float target_yaw_rate = radio_yaw * MAX_YAW_RATE_DPS / 100.0f;
        float roll_correction = pid_update(&pid_rate_roll,
            target_roll_rate - mpu_data.gx_dps, mpu_data.gx_dps, dt, MAX_STABILIZATION_US);
        float pitch_correction = pid_update(&pid_rate_pitch,
            target_pitch_rate - mpu_data.gy_dps, mpu_data.gy_dps, dt, MAX_STABILIZATION_US);
        float yaw_correction = pid_update(&pid_rate_yaw,
            target_yaw_rate - mpu_data.gz_dps, mpu_data.gz_dps, dt, 250.0f);

        aileron = (uint16_t)fminf(SERVO_MAX, fmaxf(SERVO_MIN,
            (float)aileron + roll_correction));
        elevator = (uint16_t)fminf(SERVO_MAX, fmaxf(SERVO_MIN,
            (float)elevator + pitch_correction));
        rudder = (uint16_t)fminf(SERVO_MAX, fmaxf(SERVO_MIN,
            (float)rudder + yaw_correction));
    } else if (flight_mode == 0U) {
        pid_reset_all();
    }

    if (plane_type == PLANE_VTAIL) {
        int32_t p = (int32_t)elevator - SERVO_CENTER;
        int32_t y = (int32_t)rudder - SERVO_CENTER;

        int32_t left = SERVO_CENTER + (int32_t)(0.707f * (float)(p + y));
        int32_t right = SERVO_CENTER + (int32_t)(0.707f * (float)(p - y));

        if (left < SERVO_MIN) left = SERVO_MIN;
        if (left > SERVO_MAX) left = SERVO_MAX;
        if (right < SERVO_MIN) right = SERVO_MIN;
        if (right > SERVO_MAX) right = SERVO_MAX;

        update_outputs(aileron, (uint16_t)left, throttle_pulse,
                       (uint16_t)right, flaps, gear, aux1, aux2);
    } else {
        update_outputs(aileron, elevator, throttle_pulse, rudder,
                       flaps, gear, aux1, aux2);
    }
}

static void nrf_activate_features(void) {
    csn(0);
    fc_nrf_xfer(0x50U);
    fc_nrf_xfer(0x73U);
    csn(1);
}

static bool nrf_enable_ack_payload(void) {
    uint8_t feature = read_register(FEATURE);

    feature |= 0x06U;
    write_register(FEATURE, feature);

    if ((read_register(FEATURE) & 0x06U) != 0x06U) {
        nrf_activate_features();
        feature = read_register(FEATURE) | 0x06U;
        write_register(FEATURE, feature);
    }

    write_register(DYNPD, read_register(DYNPD) | 0x03U);

    return ((read_register(FEATURE) & 0x06U) == 0x06U) &&
           ((read_register(DYNPD) & 0x03U) == 0x03U);
}

static bool queue_ack_telemetry(void) {
    uint16_t mv = 0U;

    if (battery_voltage > 0.0f) {
        float scaled = battery_voltage * 1000.0f;
        if (scaled > 65535.0f) scaled = 65535.0f;
        mv = (uint16_t)(scaled + 0.5f);
    }

    tx_telemetry.batteryMv = mv;

    float bp = battery_percent;
    if (bp < 0.0f) bp = 0.0f;
    if (bp > 100.0f) bp = 100.0f;
    tx_telemetry.batteryPercent = (uint8_t)(bp + 0.5f);

    tx_telemetry.flags =
        (failsafe ? 0U : 0x01U) |
        ((battery_warning & 0x03U) << 1);

    tx_telemetry.packetCount = packet_count;
    float centi_amp = current_amps * 100.0f;
    if (centi_amp > 65535.0f) centi_amp = 65535.0f;
    tx_telemetry.currentCentiAmp = (uint16_t)(centi_amp + 0.5f);
    tx_telemetry.consumedMah = (consumed_mah > 65535.0f)
        ? 65535U : (uint16_t)(consumed_mah + 0.5f);
    tx_telemetry.linkQuality = receiver_link_quality;
    tx_telemetry.state = (armed ? 0x01U : 0U) |
                         (imu_calibrated ? 0x02U : 0U) |
                         (failsafe_latched ? 0x04U : 0U) |
                         (imu_calibrating ? 0x08U : 0U);
    tx_telemetry.rollDeg = telemetry_roll_deg;
    tx_telemetry.pitchDeg = telemetry_pitch_deg;
    tx_telemetry.calibrationProgress = imu_calibration_progress;

    uint8_t fifo = read_register(FIFO_STATUS);

    if (fifo & 0x20U) {
        ack_payload_queue_full++;
        return false;
    }

    if ((fifo & 0x10U) == 0U) {
        ack_payload_pending++;
    }

    csn(0);
    fc_nrf_xfer(0xA8U);

    const uint8_t *p = (const uint8_t *)&tx_telemetry;
    for (uint8_t i = 0U; i < TLM_S; i++) {
        fc_nrf_xfer(p[i]);
    }

    csn(1);

    ack_payload_queued++;
    return true;
}

static uint8_t nrf_read_payload_width(void) {
    uint8_t width;

    csn(0);
    fc_nrf_xfer(R_RX_PL_WID);
    width = fc_nrf_xfer(NOP_CMD);
    csn(1);

    return width;
}

void radio_init(void) {
    ce(0);
    csn(1);

    write_register(CONFIG, 0x00);

    if (read_register(CONFIG) == 0xFFU) {
        radio_initialized = 0;
        return;
    }

    write_register(EN_AA, 0x01);
    write_register(EN_RXADDR, 0x01);
    write_register(SETUP_AW, 0x03);
    write_register(SETUP_RETR, 0x12);
    write_register(RF_CH, 100);
    write_register(RF_SETUP, 0x06);

    write_register_buf(RX_ADDR_P0, stm32_rx_pipe0, 5);
    write_register_buf(TX_ADDR, stm32_rx_pipe0, 5);

    write_register(RX_PW_P0, 0);

    ack_payload_enabled = nrf_enable_ack_payload();

    write_register(STATUS, 0x70);
    flush_rx();
    flush_tx();

    tx_telemetry.batteryMv = 0;
    tx_telemetry.batteryPercent = 0;
    tx_telemetry.flags = 0;
    tx_telemetry.packetCount = 0;
    tx_telemetry.currentCentiAmp = 0;
    tx_telemetry.consumedMah = 0;
    tx_telemetry.linkQuality = 0;
    tx_telemetry.state = 0;
    tx_telemetry.rollDeg = 0;
    tx_telemetry.pitchDeg = 0;
    tx_telemetry.calibrationProgress = 0;

    write_register(CONFIG, 0x0F);
    delay_us(5000U);

    if (ack_payload_enabled) {
        queue_ack_telemetry();
    }

    ce(1);
    radio_initialized = 1;
}

void handle_incoming_packet(void) {
    uint8_t width = nrf_read_payload_width();

    if (width == 0U || width > 32U) {
        ack_bad_rx_width++;
        flush_rx();
        write_register(STATUS, 0x70);
        return;
    }

    if (width != PLD_S) {
        ack_bad_rx_width++;

        csn(0);
        fc_nrf_xfer(R_RX_PAYLOAD);
        for (uint8_t i = 0U; i < width; i++) {
            (void)fc_nrf_xfer(NOP_CMD);
        }
        csn(1);

        write_register(STATUS, 0x70);
        return;
    }

    csn(0);
    fc_nrf_xfer(R_RX_PAYLOAD);

    for (uint8_t i = 0U; i < PLD_S; i++) {
        dataR[i] = fc_nrf_xfer(NOP_CMD);
    }

    csn(1);

    memcpy(&rx_data, dataR, PLD_S);
    packet_count++;

    process_radio(&rx_data);

    if (ack_payload_enabled) {
        queue_ack_telemetry();
    }

    write_register(STATUS, 0x70);
}

static void service_nrf24_irq(void);

static bool calibrate_imu_stationary(void) {
    float sx = 0.0f, sy = 0.0f, sz = 0.0f;
    uint16_t accepted = 0U;

    write_all_safe();
    pwm_commit_targets();

    imu_calibrating = true;
    imu_calibration_progress = 0U;
    telemetry_roll_deg = 0;
    telemetry_pitch_deg = 0;

    for (uint16_t i = 0U; i < IMU_CALIBRATION_SAMPLES; i++) {
        mpu6050_raw_t raw;
        if (mpu6050_read_raw(&hi2c1, MPU6050_ADDR, &raw)) {
            mpu6050_si_t sample;
            mpu6050_convert(&raw, &sample, ACC_LSB_PER_G, GYRO_LSB_PER_DPS);
            sx += sample.gx_dps;
            sy += sample.gy_dps;
            sz += sample.gz_dps;
            accepted++;

            float live_roll = atan2f(sample.ay_g, sample.az_g) * 57.29578f;
            float live_pitch = atan2f(-sample.ax_g,
                                      sqrtf(sample.ay_g * sample.ay_g +
                                            sample.az_g * sample.az_g)) * 57.29578f;

            if (live_roll > 90.0f) live_roll = 90.0f;
            if (live_roll < -90.0f) live_roll = -90.0f;
            if (live_pitch > 90.0f) live_pitch = 90.0f;
            if (live_pitch < -90.0f) live_pitch = -90.0f;

            telemetry_roll_deg = (int8_t)live_roll;
            telemetry_pitch_deg = (int8_t)live_pitch;
        }

        imu_calibration_progress = (uint8_t)(((uint32_t)(i + 1U) * 100UL) /
                                             IMU_CALIBRATION_SAMPLES);

        if (radio_initialized &&
            HAL_GPIO_ReadPin(FC_NRF_IRQ_PORT, FC_NRF_IRQ_PIN) == GPIO_PIN_RESET) {
            service_nrf24_irq();
        }

        if ((i & 0x1FU) == 0U) {
              }

        HAL_Delay(3U);
    }

    imu_calibrating = false;
    imu_calibration_progress = 100U;

    if (accepted < (IMU_CALIBRATION_SAMPLES * 9U) / 10U) return false;
    gyro_bias_x = sx / accepted;
    gyro_bias_y = sy / accepted;
    gyro_bias_z = sz / accepted;
    pitch = 0.0f;
    roll = 0.0f;
    telemetry_roll_deg = 0;
    telemetry_pitch_deg = 0;
    last_imu_cycle = DWT->CYCCNT;
    return true;
}

static void service_mpu_data_ready(void) {
    if (!mpu_ready) return;

    mpu6050_raw_t raw;

    if (mpu6050_read_raw(&hi2c1, MPU6050_ADDR, &raw)) {
        uint32_t cycle_now = DWT->CYCCNT;
        uint32_t elapsed_cycles = cycle_now - last_imu_cycle;
        last_imu_cycle = cycle_now;
        dt = (float)elapsed_cycles / (float)SystemCoreClock;
        if (dt < 0.002f || dt > 0.020f) dt = 0.005f;
        mpu6050_convert(&raw, &mpu_data, ACC_LSB_PER_G, GYRO_LSB_PER_DPS);

        mpu_data.gx_dps -= gyro_bias_x;
        mpu_data.gy_dps -= gyro_bias_y;
        mpu_data.gz_dps -= gyro_bias_z;

        if (fabsf(mpu_data.gx_dps) < ACRO_GYRO_NOISE_DPS) mpu_data.gx_dps = 0.0f;
        if (fabsf(mpu_data.gy_dps) < ACRO_GYRO_NOISE_DPS) mpu_data.gy_dps = 0.0f;
        if (fabsf(mpu_data.gz_dps) < ACRO_GYRO_NOISE_DPS) mpu_data.gz_dps = 0.0f;

        mpu6050_complementary(&mpu_data, dt, &pitch, &roll, ALPHA);

        float ui_roll = roll;
        float ui_pitch = pitch;
        if (ui_roll > 90.0f) ui_roll = 90.0f;
        if (ui_roll < -90.0f) ui_roll = -90.0f;
        if (ui_pitch > 90.0f) ui_pitch = 90.0f;
        if (ui_pitch < -90.0f) ui_pitch = -90.0f;
        telemetry_roll_deg = (int8_t)ui_roll;
        telemetry_pitch_deg = (int8_t)ui_pitch;
    }

    last_mpu_service_ms = HAL_GetTick();
}

static void service_nrf24_irq(void) {
    if (!radio_initialized) return;

    uint8_t guard = 0U;

    do {
        uint8_t status = get_status();
        uint8_t fifo = read_register(FIFO_STATUS);

        if ((status & (1U << RX_DR)) || ((fifo & 0x01U) == 0U)) {
            uint8_t rx_guard = 0U;

            while (((read_register(FIFO_STATUS) & 0x01U) == 0U) &&
                   (rx_guard < 3U)) {
                handle_incoming_packet();
                rx_guard++;
            }
        }

        status = get_status();
        if (status & 0x70U) {
            write_register(STATUS, status & 0x70U);
        }

        guard++;
    } while ((HAL_GPIO_ReadPin(FC_NRF_IRQ_PORT, FC_NRF_IRQ_PIN) == GPIO_PIN_RESET) &&
             (guard < 4U));
}

int main(void)
{
  HAL_Init();
  SystemClock_Config();
  micros_fast_init();

  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  led_pb5_init();

  HAL_ADCEx_Calibration_Start(&hadc1);

  pid_reset_all();

  pwm_enable_preload_and_safe_start();

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3, MOTOR_CH);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  write_all_safe();
  pwm_commit_targets();

  radio_init();

  mpu_ready = mpu6050_init(&hi2c1, MPU6050_ADDR) ? 1U : 0U;

  imu_calibrated = false;
  imu_calibrating = false;
  imu_calibration_started = false;
  imu_calibration_progress = 0U;


  uint32_t now = HAL_GetTick();

  last_mpu_service_ms = now;
  last_radio_guard_ms = now;
  last_control_update_ms = now;
  last_packet_time = now;
  last_sensor_update = now;
  last_battery_update = now;
  last_capacity_update_ms = now;
  link_window_start_ms = now;

  if (radio_initialized &&
      HAL_GPIO_ReadPin(FC_NRF_IRQ_PORT, FC_NRF_IRQ_PIN) == GPIO_PIN_RESET) {
      nrf_irq_pending = 1U;
  }

  if (mpu_ready &&
      HAL_GPIO_ReadPin(FC_MPU_INT_PORT, FC_MPU_INT_PIN) == GPIO_PIN_SET) {
      mpu_irq_pending = 1U;
  }

  while (1)
  {
      now = HAL_GetTick();


      if (radio_initialized &&
          (nrf_irq_pending ||
           HAL_GPIO_ReadPin(FC_NRF_IRQ_PORT, FC_NRF_IRQ_PIN) == GPIO_PIN_RESET ||
           (uint32_t)(now - last_radio_guard_ms) >= 5U)) {
          nrf_irq_pending = 0U;
          last_radio_guard_ms = now;
          service_nrf24_irq();
      }

      now = HAL_GetTick();

      if (mpu_ready && !imu_calibration_started &&
          valid_packets_since_boot >= 220U) {
          imu_calibration_started = true;
          imu_calibrated = calibrate_imu_stationary();
          now = HAL_GetTick();
          last_mpu_service_ms = now;
          last_control_update_ms = now;
          last_packet_time = now;
      }

      if (mpu_ready && imu_calibrated &&
          (mpu_irq_pending ||
           (uint32_t)(now - last_mpu_service_ms) >= 5U)) {
          mpu_irq_pending = 0U;
          service_mpu_data_ready();
      }

      now = HAL_GetTick();

      uint32_t link_age_ms = now - last_packet_time;

      if (valid_packets_since_boot > 0U &&
          !failsafe && link_age_ms > FAILSAFE_TRIGGER_MS) {
          failsafe = true;
          failsafe_latched = true;
          armed = false;
          write_all_safe();
          pwm_commit_targets();
          pid_reset_all();
      }

      if ((uint32_t)(now - link_window_start_ms) >= 500U) {
          uint16_t expected = (uint16_t)((now - link_window_start_ms) / 10U);
          uint32_t quality = expected ? ((uint32_t)link_window_packets * 100U) / expected : 0U;
          if (quality > 100U) quality = 100U;
          receiver_link_quality = (uint8_t)quality;
          link_window_packets = 0U;
          link_window_start_ms = now;
      }

      if ((uint32_t)(now - last_control_update_ms) >= CONTROL_LOOP_PERIOD_MS) {
          last_control_update_ms = now;
          flight_controller();
          pwm_commit_targets();
      }

      if ((uint32_t)(now - last_battery_update) >= 250U) {
          last_battery_update = now;
          calculate_battery();
      }

  }
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK |
                                RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 |
                                RCC_CLOCKTYPE_PCLK2;

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

static void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_I2C1_Init(void)
{
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
}

static void MX_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

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

  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_13CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


static void MX_TIM1_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim1);
}

static void MX_TIM3_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 19999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = SERVO_CENTER;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim3);
}

static void MX_TIM4_Init(void)
{
  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 71;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 19999;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = SERVO_CENTER;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_TIM_MspPostInit(&htim4);
}

static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_AFIO_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_AFIO_REMAP_SWJ_NOJTAG();
  __HAL_AFIO_REMAP_TIM3_PARTIAL();

  HAL_GPIO_WritePin(FC_LED_PORT, FC_LED_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, NRF_CE_Pin|NRF_CSN_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = FC_LED_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(FC_LED_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FC_NRF_IRQ_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(FC_NRF_IRQ_PORT, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = FC_MPU_INT_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(FC_MPU_INT_PORT, &GPIO_InitStruct);

  __HAL_GPIO_EXTI_CLEAR_IT(FC_NRF_IRQ_PIN);
  __HAL_GPIO_EXTI_CLEAR_IT(FC_MPU_INT_PIN);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif
