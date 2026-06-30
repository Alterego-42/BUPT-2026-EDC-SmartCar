/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : K230 UART tracking + PCA9685 dual stepper control
  ******************************************************************************
  */
/* USER CODE END Header */

#include "main.h"
#include "gpio.h"
#include <stdint.h>

/* Hardware wiring */
#define PCA9685_ADDR_7BIT             0x40U
#define PCA9685_MODE1                 0x00U
#define PCA9685_MODE2                 0x01U
#define PCA9685_LED0_ON_L             0x06U

#define H_AXIS_BASE_CH                0U     /* PCA9685 CH0~3: yaw / horizontal */
#define V_AXIS_BASE_CH                4U     /* PCA9685 CH4~7: pitch / vertical */

#define LASER_GPIO_PORT               GPIOB
#define LASER_GPIO_PIN                GPIO_PIN_0
#define LASER_ACTIVE_HIGH             1U

#define STATUS_LED_PORT               GPIOC
#define STATUS_LED_PIN                GPIO_PIN_13

/* Tunable tracking parameters */
#define UART_BAUDRATE                 115200U
#define VISION_TIMEOUT_MS             120U

#define YAW_DEADBAND_CDEG             6       /* 0.06 deg: closed-loop laser error */
#define PITCH_DEADBAND_CDEG           6       /* 0.06 deg */
#define YAW_MAX_ERROR_CDEG            300     /* K230 clamps commands to about 3 deg */
#define PITCH_MAX_ERROR_CDEG          300

#define H_STEP_MIN_MS                 1U      /* CH0~3 is intentionally faster */
#define H_STEP_MAX_MS                 4U
#define V_STEP_MIN_MS                 1U
#define V_STEP_MAX_MS                 4U
#define H_STEPS_PER_FRAME             24U     /* enough headroom for about 60 deg/s on geared 28BYJ-class axes */
#define V_STEPS_PER_FRAME             24U

#define COMPONENT_YAW                 0U
#define COMPONENT_PITCH               1U
#define AXIS_AUTO_LEARN_ENABLED       0U
#define AXIS_RESPONSE_MIN_CDEG        8       /* ignore tiny/noisy response under 0.08 deg */
#define AXIS_LEARN_SAMPLES            3U

#define YAW_MOTOR_DIR                 1       /* fixed mapping: flip sign here if the axis runs away */
#define PITCH_MOTOR_DIR               1       /* fixed mapping: flip sign here if the axis runs away */

#define RELEASE_COILS_WHEN_IDLE       0U
#define LASER_DEFAULT_ON              1U      /* keep a separate laser entry; currently always on */
#define TELEMETRY_PERIOD_MS           60000U
#define ACK_PERIOD_FRAMES             5U      /* report STM32 RX state without flooding the UART */
#define I2C_DELAY_LOOPS               6U      /* faster software I2C; raise if PCA9685 ACK becomes unstable */
#define DEBUG_ECHO_LINES              0U      /* ACK already carries the received valid/yaw/pitch */
#define UART_DEBUG_ENABLED            0U      /* keep control UART quiet: K230 -> STM32 only */
#define UART_RX_RING_SIZE             256U

typedef struct {
  uint8_t valid;
  int16_t yaw_cdeg;
  int16_t pitch_cdeg;
  uint32_t last_ms;
  uint32_t frames;
} VisionFrame;

typedef struct {
  uint8_t base_ch;
  uint8_t component;
  uint8_t step_index;
  uint8_t energized;
  int8_t response_sign;
  uint16_t deadband_cdeg;
  uint16_t max_error_cdeg;
  uint16_t min_step_ms;
  uint16_t max_step_ms;
  uint32_t next_step_ms;
  uint16_t last_interval_ms;
  uint8_t pending_eval;
  int8_t last_step_dir;
  int16_t last_yaw_before;
  int16_t last_pitch_before;
  uint32_t last_step_frame;
  uint32_t learn_count;
  uint32_t step_frame_id;
  uint8_t steps_this_frame;
  uint8_t max_steps_per_frame;
} AxisControl;

void SystemClock_Config(void);

static void Board_IO_Init(void);
static void USART1_InitRaw(void);
static void USART1_PollRx(void);
static uint8_t USART1_ReadBufferedByte(char *out);
static void USART1_SendByte(uint8_t ch);
static void USART1_SendString(const char *s);
static void USART1_SendU32(uint32_t value);
static void USART1_SendI32(int32_t value);
static void USART1_SendCentiDeg(int16_t value);
static void Debug_PrintAck(uint32_t frame_id, uint8_t valid, int16_t yaw, int16_t pitch);

static void I2C1_InitFast(void);
static void I2C1_ResetBus(void);
static uint8_t I2C1_WriteRegData(uint8_t reg, const uint8_t *data, uint8_t len);
static uint8_t I2C1_WriteReg(uint8_t reg, uint8_t value);
static void I2C_Delay(void);
static void I2C_SCL(uint8_t high);
static void I2C_SDA(uint8_t high);
static uint8_t I2C_ReadSDA(void);
static void I2C_Start(void);
static void I2C_Stop(void);
static uint8_t I2C_WriteByte(uint8_t data);

static uint8_t PCA9685_Init(void);
static uint8_t PCA9685_AllOff(void);
static uint8_t PCA9685_SetMotor4(uint8_t base_ch, const uint8_t pattern[4]);

static void Axis_Update(AxisControl *axis, int16_t yaw_cdeg, int16_t pitch_cdeg,
                        uint8_t tracking_active, uint32_t now, uint32_t frame_id);
static uint16_t Axis_ComputeInterval(const AxisControl *axis, int16_t error_cdeg);
static void Axis_EvaluateResponse(AxisControl *axis, int16_t yaw_cdeg, int16_t pitch_cdeg, uint32_t frame_id);
static void Axis_Release(AxisControl *axis);
static int16_t Axis_SelectedError(const AxisControl *axis, int16_t yaw_cdeg, int16_t pitch_cdeg);
static int8_t Sign16(int16_t value);
static uint16_t Abs16(int16_t value);

static void Laser_Set(uint8_t on);
static void Laser_Update(uint32_t now);
static void Status_LED_Set(uint8_t on);
static uint8_t TimeDue(uint32_t now, uint32_t deadline);

static uint8_t ParseLine(const char *line);
static uint8_t ParseInt(const char **p, int16_t *out);
static uint8_t ParseCentiDeg(const char **p, int16_t *out);
static uint8_t MatchPrefix(const char *s, const char *prefix);

static const uint8_t step_seq[8][4] = {
  {1,0,0,0},
  {1,1,0,0},
  {0,1,0,0},
  {0,1,1,0},
  {0,0,1,0},
  {0,0,1,1},
  {0,0,0,1},
  {1,0,0,1}
};

static volatile VisionFrame vision = {0, 0, 0, 0, 0};
static char rx_line[64];
static uint8_t rx_len = 0;
static uint8_t laser_enabled = LASER_DEFAULT_ON;
static volatile uint8_t uart_rx_ring[UART_RX_RING_SIZE];
static volatile uint16_t uart_rx_head = 0;
static volatile uint16_t uart_rx_tail = 0;
static volatile uint32_t rx_bytes = 0;
static volatile uint32_t rx_ring_overflow = 0;
static volatile uint32_t rx_hw_errors = 0;
static uint32_t rx_lines = 0;
static uint32_t parse_ok_count = 0;
static uint32_t parse_fail_count = 0;
static uint32_t pca_fail_count = 0;
static uint32_t h_step_count = 0;
static uint32_t v_step_count = 0;
static const char *last_parse_status = "NONE";

int main(void)
{
  uint32_t now;
  uint32_t next_telemetry_ms = 0;
  uint8_t pca_ok;
  uint8_t tracking_active;

  AxisControl h_axis = {
    H_AXIS_BASE_CH, COMPONENT_YAW, 0, 0, YAW_MOTOR_DIR,
    YAW_DEADBAND_CDEG, YAW_MAX_ERROR_CDEG,
    H_STEP_MIN_MS, H_STEP_MAX_MS, 0, H_STEP_MAX_MS,
    0, 0, 0, 0, 0, 0, 0, 0, H_STEPS_PER_FRAME
  };
  AxisControl v_axis = {
    V_AXIS_BASE_CH, COMPONENT_PITCH, 0, 0, PITCH_MOTOR_DIR,
    PITCH_DEADBAND_CDEG, PITCH_MAX_ERROR_CDEG,
    V_STEP_MIN_MS, V_STEP_MAX_MS, 0, V_STEP_MAX_MS,
    0, 0, 0, 0, 0, 0, 0, 0, V_STEPS_PER_FRAME
  };

  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  Board_IO_Init();
  USART1_InitRaw();
  I2C1_InitFast();

  Laser_Set(laser_enabled);
  Status_LED_Set(0);

  USART1_SendString("\r\nSTM32F103C8 K230 tracker run\r\n");
  USART1_SendString("WAIT $K230,valid,yaw,pitch | SW-I2C PCA9685 | laser ON\r\n");

  pca_ok = PCA9685_Init();
  USART1_SendString("PCA9685 init: ");
  USART1_SendString(pca_ok ? "ACK\r\n" : "NO_ACK\r\n");

  while (1)
  {
    now = HAL_GetTick();
    USART1_PollRx();
    Laser_Update(now);

    tracking_active = (vision.valid != 0U) &&
                      ((uint32_t)(now - vision.last_ms) <= VISION_TIMEOUT_MS);

    if (pca_ok)
    {
      Axis_Update(&h_axis, vision.yaw_cdeg, vision.pitch_cdeg, tracking_active, now, vision.frames);
      Axis_Update(&v_axis, vision.yaw_cdeg, vision.pitch_cdeg, tracking_active, now, vision.frames);
    }
    else
    {
      pca_ok = PCA9685_Init();
      if (pca_ok == 0U)
      {
        pca_fail_count++;
      }
    }

    Status_LED_Set(tracking_active);

    if (TimeDue(now, next_telemetry_ms))
    {
      USART1_SendString("STAT valid=");
      USART1_SendU32(tracking_active ? 1U : 0U);
      USART1_SendString(" yaw=");
      USART1_SendCentiDeg(vision.yaw_cdeg);
      USART1_SendString(" pitch=");
      USART1_SendCentiDeg(vision.pitch_cdeg);
      USART1_SendString(" h_steps=");
      USART1_SendU32(h_step_count);
      USART1_SendString(" v_steps=");
      USART1_SendU32(v_step_count);
      USART1_SendString(" h_comp=");
      USART1_SendString((h_axis.component == COMPONENT_YAW) ? "Y" : "P");
      USART1_SendString((h_axis.response_sign > 0) ? "+" : "-");
      USART1_SendString(" h_lrn=");
      USART1_SendU32(h_axis.learn_count);
      USART1_SendString(" v_comp=");
      USART1_SendString((v_axis.component == COMPONENT_YAW) ? "Y" : "P");
      USART1_SendString((v_axis.response_sign > 0) ? "+" : "-");
      USART1_SendString(" v_lrn=");
      USART1_SendU32(v_axis.learn_count);
      USART1_SendString(" ovr=");
      USART1_SendU32(rx_ring_overflow);
      USART1_SendString(" hwerr=");
      USART1_SendU32(rx_hw_errors);
      USART1_SendString(" pca=");
      USART1_SendString(pca_ok ? "ACK\r\n" : "NO_ACK\r\n");
      next_telemetry_ms = now + TELEMETRY_PERIOD_MS;
    }

    HAL_Delay(1);
  }
}

static void Board_IO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_AFIO_CLK_ENABLE();

  HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_GPIO_PIN,
                    LASER_ACTIVE_HIGH ? GPIO_PIN_RESET : GPIO_PIN_SET);
  GPIO_InitStruct.Pin = LASER_GPIO_PIN;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LASER_GPIO_PORT, &GPIO_InitStruct);

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_SET);
  GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

#if UART_DEBUG_ENABLED
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#else
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
#endif

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void USART1_InitRaw(void)
{
  uint32_t pclk = HAL_RCC_GetPCLK2Freq();

  __HAL_RCC_USART1_CLK_ENABLE();
  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  USART1->BRR = (pclk + (UART_BAUDRATE / 2U)) / UART_BAUDRATE;
#if UART_DEBUG_ENABLED
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
#else
  USART1->CR1 = USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;
#endif
  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
}

static void USART1_PollRx(void)
{
  char ch;

  while (USART1_ReadBufferedByte(&ch))
  {
    if (ch == '$')
    {
      rx_len = 0;
      rx_line[rx_len++] = ch;
    }
    else if ((ch == '\r') || (ch == '\n'))
    {
      if (rx_len > 0U)
      {
        rx_line[rx_len] = '\0';
        rx_lines++;
#if DEBUG_ECHO_LINES
        USART1_SendString("RX ");
        USART1_SendString(rx_line);
        USART1_SendString("\r\n");
#endif
        ParseLine(rx_line);
        rx_len = 0;
      }
    }
    else if (rx_len < (uint8_t)(sizeof(rx_line) - 1U))
    {
      rx_line[rx_len++] = ch;
    }
    else
    {
      rx_len = 0;
    }
  }
}

static uint8_t USART1_ReadBufferedByte(char *out)
{
  uint16_t tail = uart_rx_tail;

  if (tail == uart_rx_head)
  {
    return 0U;
  }

  *out = (char)uart_rx_ring[tail];
  uart_rx_tail = (uint16_t)((tail + 1U) & (UART_RX_RING_SIZE - 1U));
  return 1U;
}

static void USART1_SendByte(uint8_t ch)
{
#if UART_DEBUG_ENABLED
  while ((USART1->SR & USART_SR_TXE) == 0U)
  {
  }
  USART1->DR = ch;
#else
  (void)ch;
#endif
}

void USART1_IRQHandler(void)
{
  uint32_t sr = USART1->SR;
  uint8_t ch;
  uint16_t next_head;

  if ((sr & (USART_SR_RXNE | USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
  {
    ch = (uint8_t)(USART1->DR & 0xFFU);

    if ((sr & (USART_SR_ORE | USART_SR_NE | USART_SR_FE | USART_SR_PE)) != 0U)
    {
      rx_hw_errors++;
    }

    if ((sr & USART_SR_RXNE) != 0U)
    {
      next_head = (uint16_t)((uart_rx_head + 1U) & (UART_RX_RING_SIZE - 1U));
      if (next_head != uart_rx_tail)
      {
        uart_rx_ring[uart_rx_head] = ch;
        uart_rx_head = next_head;
        rx_bytes++;
      }
      else
      {
        rx_ring_overflow++;
      }
    }
  }
}

static void USART1_SendString(const char *s)
{
  while (*s != '\0')
  {
    USART1_SendByte((uint8_t)*s++);
  }
}

static void USART1_SendU32(uint32_t value)
{
  char buf[11];
  int i = 10;
  buf[i] = '\0';
  do
  {
    buf[--i] = (char)('0' + (value % 10U));
    value /= 10U;
  } while (value != 0U);
  USART1_SendString(&buf[i]);
}

static void USART1_SendI32(int32_t value)
{
  if (value < 0)
  {
    USART1_SendByte('-');
    value = -value;
  }
  USART1_SendU32((uint32_t)value);
}

static void USART1_SendCentiDeg(int16_t value)
{
  int32_t v = value;
  int32_t whole;
  int32_t frac;

  if (v < 0)
  {
    USART1_SendByte('-');
    v = -v;
  }

  whole = v / 100;
  frac = v % 100;
  USART1_SendI32(whole);
  USART1_SendByte('.');
  USART1_SendByte((uint8_t)('0' + (frac / 10)));
  USART1_SendByte((uint8_t)('0' + (frac % 10)));
}

static void Debug_PrintAck(uint32_t frame_id, uint8_t valid, int16_t yaw, int16_t pitch)
{
  int8_t h_dir = 0;
  int8_t v_dir = 0;

  if ((valid != 0U) && (Abs16(yaw) > YAW_DEADBAND_CDEG))
  {
    h_dir = (int8_t)(-Sign16(yaw) * YAW_MOTOR_DIR);
  }
  if ((valid != 0U) && (Abs16(pitch) > PITCH_DEADBAND_CDEG))
  {
    v_dir = (int8_t)(-Sign16(pitch) * PITCH_MOTOR_DIR);
  }

  USART1_SendString("ACK f=");
  USART1_SendU32(frame_id);
  USART1_SendString(" v=");
  USART1_SendU32(valid);
  USART1_SendString(" y=");
  USART1_SendCentiDeg(yaw);
  USART1_SendString(" p=");
  USART1_SendCentiDeg(pitch);
  USART1_SendString(" hd=");
  USART1_SendI32(h_dir);
  USART1_SendString(" vd=");
  USART1_SendI32(v_dir);
  USART1_SendString(" hs=");
  USART1_SendU32(h_step_count);
  USART1_SendString(" vs=");
  USART1_SendU32(v_step_count);
  USART1_SendString("\r\n");
}

static void I2C1_InitFast(void)
{
  I2C1_ResetBus();
}

static void I2C1_ResetBus(void)
{
  uint8_t i;

  I2C_SDA(1);
  I2C_SCL(1);

  for (i = 0; i < 9U; i++)
  {
    I2C_SCL(0);
    I2C_SCL(1);
  }

  I2C_Stop();
}

static uint8_t I2C1_WriteRegData(uint8_t reg, const uint8_t *data, uint8_t len)
{
  uint8_t i;
  uint8_t ok;

  I2C_Start();
  ok = I2C_WriteByte((uint8_t)(PCA9685_ADDR_7BIT << 1));
  ok &= I2C_WriteByte(reg);
  for (i = 0; i < len; i++)
  {
    ok &= I2C_WriteByte(data[i]);
  }
  I2C_Stop();
  return ok;
}

static uint8_t I2C1_WriteReg(uint8_t reg, uint8_t value)
{
  return I2C1_WriteRegData(reg, &value, 1U);
}

static void I2C_Delay(void)
{
  volatile uint32_t i;
  for (i = 0; i < I2C_DELAY_LOOPS; i++)
  {
    __NOP();
  }
}

static void I2C_SCL(uint8_t high)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  I2C_Delay();
}

static void I2C_SDA(uint8_t high)
{
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, high ? GPIO_PIN_SET : GPIO_PIN_RESET);
  I2C_Delay();
}

static uint8_t I2C_ReadSDA(void)
{
  return (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_SET) ? 1U : 0U;
}

static void I2C_Start(void)
{
  I2C_SDA(1);
  I2C_SCL(1);
  I2C_SDA(0);
  I2C_SCL(0);
}

static void I2C_Stop(void)
{
  I2C_SDA(0);
  I2C_SCL(1);
  I2C_SDA(1);
}

static uint8_t I2C_WriteByte(uint8_t data)
{
  uint8_t bit;
  uint8_t ack;

  for (bit = 0; bit < 8U; bit++)
  {
    I2C_SDA((data & 0x80U) != 0U);
    I2C_SCL(1);
    I2C_SCL(0);
    data <<= 1;
  }

  I2C_SDA(1);
  I2C_SCL(1);
  ack = (I2C_ReadSDA() == 0U) ? 1U : 0U;
  I2C_SCL(0);
  return ack;
}

static uint8_t PCA9685_Init(void)
{
  uint8_t ok = 1U;

  ok &= I2C1_WriteReg(PCA9685_MODE1, 0x00U);
  HAL_Delay(5);
  ok &= I2C1_WriteReg(PCA9685_MODE2, 0x04U);
  ok &= I2C1_WriteReg(PCA9685_MODE1, 0x20U);
  ok &= PCA9685_AllOff();
  return ok;
}

static uint8_t PCA9685_AllOff(void)
{
  uint8_t data[64];
  uint8_t i;
  for (i = 0; i < 16U; i++)
  {
    data[(uint8_t)(i * 4U + 0U)] = 0x00U;
    data[(uint8_t)(i * 4U + 1U)] = 0x00U;
    data[(uint8_t)(i * 4U + 2U)] = 0x00U;
    data[(uint8_t)(i * 4U + 3U)] = 0x10U;
  }
  return I2C1_WriteRegData(PCA9685_LED0_ON_L, data, sizeof(data));
}

static uint8_t PCA9685_SetMotor4(uint8_t base_ch, const uint8_t pattern[4])
{
  uint8_t data[16];
  uint8_t i;

  for (i = 0; i < 4U; i++)
  {
    if (pattern[i] != 0U)
    {
      data[(uint8_t)(i * 4U + 0U)] = 0x00U;
      data[(uint8_t)(i * 4U + 1U)] = 0x10U;
      data[(uint8_t)(i * 4U + 2U)] = 0x00U;
      data[(uint8_t)(i * 4U + 3U)] = 0x00U;
    }
    else
    {
      data[(uint8_t)(i * 4U + 0U)] = 0x00U;
      data[(uint8_t)(i * 4U + 1U)] = 0x00U;
      data[(uint8_t)(i * 4U + 2U)] = 0x00U;
      data[(uint8_t)(i * 4U + 3U)] = 0x10U;
    }
  }

  return I2C1_WriteRegData((uint8_t)(PCA9685_LED0_ON_L + (base_ch * 4U)), data, sizeof(data));
}

static void Axis_Update(AxisControl *axis, int16_t yaw_cdeg, int16_t pitch_cdeg,
                        uint8_t tracking_active, uint32_t now, uint32_t frame_id)
{
  int8_t step_dir;
  int8_t error_sign;
  int16_t error_cdeg;
  uint16_t interval;
  uint16_t mag;

#if AXIS_AUTO_LEARN_ENABLED
  Axis_EvaluateResponse(axis, yaw_cdeg, pitch_cdeg, frame_id);
#else
  (void)yaw_cdeg;
  (void)pitch_cdeg;
#endif
  if (axis->step_frame_id != frame_id)
  {
    axis->step_frame_id = frame_id;
    axis->steps_this_frame = 0U;
    axis->next_step_ms = now;
  }

  error_cdeg = Axis_SelectedError(axis, yaw_cdeg, pitch_cdeg);
  mag = Abs16(error_cdeg);

  if ((tracking_active == 0U) || (mag <= axis->deadband_cdeg))
  {
#if RELEASE_COILS_WHEN_IDLE
    Axis_Release(axis);
#endif
    axis->last_interval_ms = axis->max_step_ms;
    axis->next_step_ms = now + axis->max_step_ms;
    return;
  }

#if AXIS_AUTO_LEARN_ENABLED
  if ((axis->pending_eval != 0U) && (axis->learn_count < 4U))
  {
    return;
  }
#endif

  if (axis->steps_this_frame >= axis->max_steps_per_frame)
  {
    return;
  }

  interval = Axis_ComputeInterval(axis, error_cdeg);
  axis->last_interval_ms = interval;

  if (!TimeDue(now, axis->next_step_ms))
  {
    return;
  }

  error_sign = Sign16(error_cdeg);
  step_dir = (int8_t)(-error_sign * axis->response_sign);

  if (step_dir > 0)
  {
    axis->step_index = (uint8_t)((axis->step_index + 1U) & 0x07U);
  }
  else
  {
    axis->step_index = (uint8_t)((axis->step_index + 7U) & 0x07U);
  }

  if (PCA9685_SetMotor4(axis->base_ch, step_seq[axis->step_index]) != 0U)
  {
    axis->energized = 1U;
    if (axis->base_ch == H_AXIS_BASE_CH)
    {
      h_step_count++;
    }
    else if (axis->base_ch == V_AXIS_BASE_CH)
    {
      v_step_count++;
    }

#if AXIS_AUTO_LEARN_ENABLED
    axis->pending_eval = 1U;
    axis->last_step_dir = step_dir;
    axis->last_yaw_before = yaw_cdeg;
    axis->last_pitch_before = pitch_cdeg;
#endif
    axis->last_step_frame = frame_id;
    axis->steps_this_frame++;
  }

  axis->next_step_ms = now + interval;
}

static uint16_t Axis_ComputeInterval(const AxisControl *axis, int16_t error_cdeg)
{
  uint16_t mag = Abs16(error_cdeg);
  uint32_t span_ms;
  uint32_t scaled_ms;

  if (mag >= axis->max_error_cdeg)
  {
    return axis->min_step_ms;
  }

  if (mag <= axis->deadband_cdeg)
  {
    return axis->max_step_ms;
  }

  span_ms = (uint32_t)(axis->max_step_ms - axis->min_step_ms);
  scaled_ms = ((uint32_t)(mag - axis->deadband_cdeg) * span_ms) /
              (uint32_t)(axis->max_error_cdeg - axis->deadband_cdeg);
  return (uint16_t)((uint32_t)axis->max_step_ms - scaled_ms);
}

static void Axis_EvaluateResponse(AxisControl *axis, int16_t yaw_cdeg, int16_t pitch_cdeg, uint32_t frame_id)
{
  int16_t dy;
  int16_t dp;
  uint16_t ady;
  uint16_t adp;
  int8_t observed_sign;

  if ((axis->pending_eval == 0U) || (frame_id == axis->last_step_frame))
  {
    return;
  }

  dy = (int16_t)(yaw_cdeg - axis->last_yaw_before);
  dp = (int16_t)(pitch_cdeg - axis->last_pitch_before);
  ady = Abs16(dy);
  adp = Abs16(dp);

  if ((ady < AXIS_RESPONSE_MIN_CDEG) && (adp < AXIS_RESPONSE_MIN_CDEG))
  {
    axis->pending_eval = 0U;
    return;
  }

  if (ady >= adp)
  {
    axis->component = COMPONENT_YAW;
    observed_sign = Sign16(dy);
  }
  else
  {
    axis->component = COMPONENT_PITCH;
    observed_sign = Sign16(dp);
  }

  axis->response_sign = (int8_t)(observed_sign * axis->last_step_dir);
  axis->learn_count++;
  axis->pending_eval = 0U;
}

static void Axis_Release(AxisControl *axis)
{
  static const uint8_t off_pattern[4] = {0, 0, 0, 0};
  if (axis->energized != 0U)
  {
    PCA9685_SetMotor4(axis->base_ch, off_pattern);
    axis->energized = 0U;
  }
}

static int16_t Axis_SelectedError(const AxisControl *axis, int16_t yaw_cdeg, int16_t pitch_cdeg)
{
  return (axis->component == COMPONENT_YAW) ? yaw_cdeg : pitch_cdeg;
}

static int8_t Sign16(int16_t value)
{
  return (value >= 0) ? 1 : -1;
}

static uint16_t Abs16(int16_t value)
{
  return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
}

static void Laser_Set(uint8_t on)
{
  uint8_t level = (LASER_ACTIVE_HIGH != 0U) ? on : (uint8_t)!on;
  HAL_GPIO_WritePin(LASER_GPIO_PORT, LASER_GPIO_PIN, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void Laser_Update(uint32_t now)
{
  (void)now;
  Laser_Set(laser_enabled);
}

static void Status_LED_Set(uint8_t on)
{
  HAL_GPIO_WritePin(STATUS_LED_PORT, STATUS_LED_PIN, on ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static uint8_t TimeDue(uint32_t now, uint32_t deadline)
{
  return ((int32_t)(now - deadline) >= 0) ? 1U : 0U;
}

static uint8_t ParseLine(const char *line)
{
  const char *p;
  int16_t valid;
  int16_t yaw;
  int16_t pitch;

  if (!MatchPrefix(line, "$K230,"))
  {
    last_parse_status = "BAD_PREFIX";
    parse_fail_count++;
    USART1_SendString("ERR BAD_PREFIX\r\n");
    return 0U;
  }

  p = line + 6;
  if (!ParseInt(&p, &valid) || (*p != ','))
  {
    last_parse_status = "BAD_VALID";
    parse_fail_count++;
    USART1_SendString("ERR BAD_VALID\r\n");
    return 0U;
  }
  p++;

  if (!ParseCentiDeg(&p, &yaw) || (*p != ','))
  {
    last_parse_status = "BAD_YAW";
    parse_fail_count++;
    USART1_SendString("ERR BAD_YAW\r\n");
    return 0U;
  }
  p++;

  if (!ParseCentiDeg(&p, &pitch))
  {
    last_parse_status = "BAD_PITCH";
    parse_fail_count++;
    USART1_SendString("ERR BAD_PITCH\r\n");
    return 0U;
  }

  vision.valid = (valid != 0) ? 1U : 0U;
  vision.yaw_cdeg = yaw;
  vision.pitch_cdeg = pitch;
  vision.last_ms = HAL_GetTick();
  vision.frames++;
  parse_ok_count++;
  last_parse_status = "OK";
  if ((vision.frames % ACK_PERIOD_FRAMES) == 0U)
  {
    Debug_PrintAck(vision.frames, vision.valid, vision.yaw_cdeg, vision.pitch_cdeg);
  }
  return 1U;
}

static uint8_t ParseInt(const char **p, int16_t *out)
{
  int16_t value = 0;
  int8_t sign = 1;
  uint8_t digits = 0;

  if (**p == '-')
  {
    sign = -1;
    (*p)++;
  }

  while ((**p >= '0') && (**p <= '9'))
  {
    value = (int16_t)((value * 10) + (**p - '0'));
    (*p)++;
    digits++;
  }

  if (digits == 0U)
  {
    return 0U;
  }

  *out = (int16_t)(value * sign);
  return 1U;
}

static uint8_t ParseCentiDeg(const char **p, int16_t *out)
{
  int16_t whole = 0;
  int16_t frac = 0;
  int16_t parsed;
  int8_t sign = 1;
  uint8_t frac_digits = 0;

  if (**p == '-')
  {
    sign = -1;
    (*p)++;
  }
  else if (**p == '+')
  {
    (*p)++;
  }

  if (!ParseInt(p, &whole))
  {
    return 0U;
  }

  if (**p == '.')
  {
    (*p)++;
    while ((**p >= '0') && (**p <= '9'))
    {
      if (frac_digits < 2U)
      {
        frac = (int16_t)((frac * 10) + (**p - '0'));
        frac_digits++;
      }
      (*p)++;
    }
  }

  while (frac_digits < 2U)
  {
    frac = (int16_t)(frac * 10);
    frac_digits++;
  }

  parsed = (int16_t)((whole * 100) + frac);
  *out = (int16_t)(parsed * sign);
  return 1U;
}

static uint8_t MatchPrefix(const char *s, const char *prefix)
{
  while (*prefix != '\0')
  {
    if (*s++ != *prefix++)
    {
      return 0U;
    }
  }
  return 1U;
}

void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

void Error_Handler(void)
{
  __disable_irq();
  while (1)
  {
    Status_LED_Set(1);
    HAL_Delay(80);
    Status_LED_Set(0);
    HAL_Delay(80);
  }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
  (void)file;
  (void)line;
}
#endif
