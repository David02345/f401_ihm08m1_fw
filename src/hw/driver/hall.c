/*
 * hall.c
 *
 *  Created on: 2026. 7. 14.
 *      Author: YDG
 */

#include "hall.h"


static volatile uint32_t hall_last_cycle = 0;

static volatile uint8_t hall_state = 0;
static volatile int8_t hall_sector = -1;
static volatile int8_t hall_prev_sector = -1;
static volatile int8_t hall_dir = 0;

static volatile uint32_t hall_last_edge_us = 0;

static volatile float hall_speed_e = 0.0f;
static volatile float hall_speed_m = 0.0f;
static volatile float hall_angle_e = 0.0f;
static volatile float hall_angle_m = 0.0f;

static volatile bool hall_valid = false;

static uint8_t hallReadState(void);
static int8_t hallGetSector(uint8_t state);
static float hallSectorToElectricalAngle(int8_t sector);

bool hallInit(void)
{
  hall_state = hallReadState();
  hall_sector = hallGetSector(hall_state);

  hall_speed_e = 0.0f;
  hall_speed_m = 0.0f;
  hall_dir = 0;

  if (hall_sector < 0)
  {
    hall_valid = false;
    return false;
  }

  hall_valid = true;
  hall_prev_sector = hall_sector;
  hall_angle_e = hallSectorToElectricalAngle(hall_sector);
  hall_last_cycle = cycleGet();

  return hall_valid;
}

static uint8_t hallReadState(void)
{
  uint8_t state = 0;

  if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) == GPIO_PIN_SET)
  {
    state |= (1U << 0);   // H1
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_3) == GPIO_PIN_SET)
  {
    state |= (1U << 1);   // H2
  }

  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_10) == GPIO_PIN_SET)
  {
    state |= (1U << 2);   // H3
  }

  return state;
}

static int8_t hallGetSector(uint8_t hall_state)
{
  switch (hall_state)
  {
    case 0b001: return 0;
    case 0b101: return 1;
    case 0b100: return 2;
    case 0b110: return 3;
    case 0b010: return 4;
    case 0b011: return 5;
    default:    return -1;
  }
}

void hallUpdate(void)
{
  uint8_t state;
  int8_t sector;
  int8_t diff;
  int8_t new_dir;
  uint32_t now_cycle;
  uint32_t dt_cycles;
  float speed_e_raw;
  float speed_m_raw;
  float center_angle;

  state = hallReadState();
  hall_state = state;

  sector = hallGetSector(state);

  if (sector < 0)
  {
    hall_sector = -1;
    hall_valid = false;
    hall_speed_e = 0.0f;
    hall_speed_m = 0.0f;
    hall_dir = 0;
    return;
  }

  if (sector == hall_sector)
  {
    return;
  }

  if (sector == hall_prev_sector)
  {
    hall_sector = sector;
    hall_angle_e = hallSectorToElectricalAngle(sector);
    hall_valid = true;
    return;
  }

  new_dir = 0;

  if (hall_prev_sector >= 0)
  {
    diff = sector - hall_prev_sector;

    if ((diff == -1) || (diff == 5))
    {
      new_dir = 1;
    }
    else if ((diff == 1) || (diff == -5))
    {
      new_dir = -1;
    }
    else
    {
      hall_sector = -1;
      hall_valid = false;
      hall_dir = 0;
      hall_speed_e = 0.0f;
      hall_speed_m = 0.0f;
      return;
    }
  }

  now_cycle = cycleGet();
  dt_cycles = now_cycle - hall_last_cycle;

  hall_sector = sector;
  hall_dir = new_dir;
  hall_angle_e = hallSectorToElectricalAngle(sector);
  center_angle = hallSectorToElectricalAngle(sector);

  if (hall_dir > 0)
  {
    hall_angle_e = center_angle - 0.5f * HALL_SECTOR_ANGLE_E;
  }
  else if (hall_dir < 0)
  {
    hall_angle_e = center_angle + 0.5f * HALL_SECTOR_ANGLE_E;
  }
  else
  {
    hall_angle_e = center_angle;
  }

  hall_angle_e = wrapFloat(hall_angle_e,
                           0.0f,
                           2.0f * PI);

  if ((new_dir != 0) && (dt_cycles > 0U))
  {
    speed_e_raw = (float)new_dir * HALL_SECTOR_ANGLE_E * (float)cycleGetFreq() / (float)dt_cycles;

    speed_m_raw = speed_e_raw / (float)MOTOR_POLE_PAIRS;

    hall_speed_e += HALL_SPEED_LPF_ALPHA * (speed_e_raw - hall_speed_e);

    hall_speed_m += HALL_SPEED_LPF_ALPHA * (speed_m_raw - hall_speed_m);
  }

  hall_prev_sector = sector;
  hall_last_cycle = now_cycle;
  hall_valid = true;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if ((GPIO_Pin != GPIO_PIN_15) && (GPIO_Pin != GPIO_PIN_3) && (GPIO_Pin != GPIO_PIN_10))
  {
    return;
  }

  hallUpdate();
}

void hallUpdateTimeout(void)
{
  uint32_t now_cycle = cycleGet();
  uint32_t timeout_cycles = (cycleGetFreq() / 1000000U) * HALL_STOP_TIMEOUT_US;

  if ((now_cycle - hall_last_cycle) > timeout_cycles)
  {
    hall_speed_e = 0.0f;
    hall_speed_m = 0.0f;
    hall_dir = 0;

    if (hall_sector >= 0)
    {
      hall_angle_e = hallSectorToElectricalAngle(hall_sector);
    }
  }
}
uint8_t hallGetState(void)
{
  return hall_state;
}

int8_t hallGetSectorIndex(void)
{
  return hall_sector;
}

bool hallIsValid(void)
{
  return hall_valid;
}

float hallGetElectricalSpeed(void)
{
  return hall_speed_e;
}

float hallGetMechanicalSpeed(void)
{
  return hall_speed_m;
}

float hallGetElectricalAngle(void)
{
  uint32_t elapsed_cycle;
  float elapsed_sec;
  float theta_e;

  elapsed_cycle = cycleGet() - hall_last_cycle;
  elapsed_sec = (float)elapsed_cycle / (float)cycleGetFreq();

  theta_e = hall_angle_e + hall_speed_e * elapsed_sec;

  return wrapFloat(theta_e, 0, 2 * PI);
}

float hallGetMechanicalAngle(void)
{
  hall_angle_m += (hall_speed_m * SPD_DT);

  return hall_angle_m;
}

static float hallSectorToElectricalAngle(int8_t sector)
{
  float theta;

  theta = -(float)sector * HALL_SECTOR_ANGLE_E + HALL_ELEC_OFFSET;

  return wrapFloat(theta, 0.0f, 2.0f * PI);
}

int8_t hallGetDirection(void)
{
  return hall_dir;
}
