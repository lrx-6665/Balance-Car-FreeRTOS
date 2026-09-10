/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "OLED.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "pid.h"
#include "car_data.h"
#include "usart.h"
#include "gpio.h"
#include <string.h>
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*
 * ========== 7H55：先动的杆无效/像另一向，只放宽超速冻余量 ==========
 *
 * 7H54 事实：先动哪边，那边没效果，看起来像后动那一边；
 * 例：先动前进杆，滑动前进，车在后退。
 *
 * 调参链（有对照，不翻符号）：
 *  7H51 翻 PID_SPEED_SIGN → 松手加速倒下 → 否决，符号保持 +1
 *  7H53 超速冻写成 Sp>目标（余量 0）→ 反方向不再冲，但目标只有 0.08~0.32，
 *       站立 Sp 或轻推就会冻 T，先动的杆根本没进速度环，车跟着 G 原方向
 *  7H54 只撤换向清角度 I → 先动的杆仍可能被余量 0 冻死
 *
 * 7F10 站立冻 T 用的是 |Sp|>0.4，不是 0。本版 F/B 冻 T 改为 |Sp|>|目标|+0.4。
 * 换向仍只清速度 I。不改增益/ofs/符号。
 */
#define TEST_STAGE  7
#define STAGE_H     1

#define H_SPEED_MAX     8
#define H_TURN_MAX      15
#define BLE_STALE_MS    600U
#define STICK_FULL      100

/* 编码器：前进为正（步骤C已确认）；电机不再二次取反 */
#define ENC_L_DIR   (-1)
#define ENC_R_DIR   (-1)
#define MOTOR_L_DIR (1)
#define MOTOR_R_DIR (1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern osMutexId_t I2cMutexHandle;
extern osMessageQueueId_t BleRxQueueHandle;
extern uint8_t g_ble_rx;
extern UART_HandleTypeDef huart2;
volatile uint16_t g_ble_rx_cnt = 0;
extern volatile uint16_t g_ble_drop_cnt;
#if (TEST_STAGE >= 7)
static uint8_t s_slow_cnt = 0;
static int32_t s_enc_sum_l = 0;
static int32_t s_enc_sum_r = 0;
#if STAGE_H
static int16_t scale_stick(int v, int16_t max_out)
{
    if (v > STICK_FULL) v = STICK_FULL;
    if (v < -STICK_FULL) v = -STICK_FULL;
    if (v > -4 && v < 4) return 0;
    return (int16_t)((v * (int)max_out) / STICK_FULL);
}
#endif
#endif
#if (TEST_STAGE >= 6)
static uint16_t s_arm_ticks = 0;
static uint16_t s_recover_cnt = 0;
static uint16_t s_prearm_cnt = 0;
static uint8_t s_prearm_ok = 0U;
static uint8_t s_filt_rst = 1U;
static uint8_t s_reset_flags = 0U;

#define ARM_TICKS_MAX      150U
#define ARM_HOLD_TICKS     50U
#define ARM_NO_TURN_TICKS  150U
#define ARM_ANGLE_MAX      8.0f
#define PREARM_ANGLE       4.0f
#define PREARM_NEED        100U
#define FALL_ANGLE         22.0f
#define RECOVER_ANGLE      6.0f
#define RECOVER_NEED       60U
#define ERR_SOFT_PWM       12.0f
#define ERR_SOFT_CAP       60
#endif

/* USER CODE END Variables */
/*
 * F103C8 只有 20KB RAM。原先 Control/Display 各 4KB 栈 + 默认堆 10KB，
 * 任务创建会失败 → 永远停在 main 的 "RTOS..."。
 * 栈按字节计（CMSIS-RTOS2），合计约 5.5KB，留给堆足够。
 */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ControlTask */
osThreadId_t ControlTaskHandle;
const osThreadAttr_t ControlTask_attributes = {
  .name = "ControlTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for CommTask */
osThreadId_t CommTaskHandle;
const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;
const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for BleRxQueue */
osMessageQueueId_t BleRxQueueHandle;
const osMessageQueueAttr_t BleRxQueue_attributes = {
  .name = "BleRxQueue"
};
/* Definitions for I2cMutex */
osMutexId_t I2cMutexHandle;
const osMutexAttr_t I2cMutex_attributes = {
  .name = "I2cMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartControlTask(void *argument);
void StartCommTask(void *argument);
void StartDisplayTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  I2cMutexHandle = osMutexNew(&I2cMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  BleRxQueueHandle = osMessageQueueNew(64, sizeof(uint8_t), &BleRxQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* USER CODE BEGIN RTOS_THREADS */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  ControlTaskHandle = osThreadNew(StartControlTask, NULL, &ControlTask_attributes);
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  if (I2cMutexHandle == NULL || BleRxQueueHandle == NULL) {
    Error_Handler();
  }
  if (defaultTaskHandle == NULL || ControlTaskHandle == NULL ||
      CommTaskHandle == NULL || DisplayTaskHandle == NULL) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
* @brief Function implementing the defaultTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  for(;;)
  {
    osDelay(1000);
  }
}

/* USER CODE BEGIN Header_StartControlTask */
/**
* @brief Function implementing the ControlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControlTask */
void StartControlTask(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(10);

    /* 记录复位原因：掉电/BOR 常表现为 POR；便于确认“倒下→掉电” */
    s_reset_flags = (uint8_t)((RCC->CSR >> 24) & 0xFFU);
    g_snap.reset_flags = s_reset_flags;
    __HAL_RCC_CLEAR_RESET_FLAGS();

#if (TEST_STAGE >= 3)
    Motor_Init();
    Encoder_Init();
    Motor_Stop();
#elif (TEST_STAGE == 1)
#else
    Motor_Init();
    Encoder_Init();
    Motor_Stop();
#endif

#if (TEST_STAGE >= 2) && (TEST_STAGE != 4)
    osMutexAcquire(I2cMutexHandle, osWaitForever);
    if (MPU6050_Init() != 0) {
        osMutexRelease(I2cMutexHandle);
        g_car_state = CAR_STOP;
        for (;;) {
            g_snap.pwm_l = (int16_t)MPU6050_GetWhoAmI();
            g_snap.pwm_r = -1;
            osDelay(200);
        }
    }
    MPU6050_Calibrate();
    osMutexRelease(I2cMutexHandle);
    osDelay(300);
#endif

    g_car_state = CAR_RUNNING;
    PID_ResetAll();
    g_target_speed = 0;
    g_target_turn = 0;
#if (TEST_STAGE >= 7)
    s_slow_cnt = 0;
    s_enc_sum_l = 0;
    s_enc_sum_r = 0;
#endif

#if (TEST_STAGE >= 6)
    /*
     * 两段零偏（7F23/7F25 已验证可用；不要 7F24 的过严 C2/C3）
     * C1 扶正 → 松手等待 → C2 采样 Mid
     */
    {
        float sum = 0.0f;
        float p = 0.0f, gy = 0.0f, gz = 0.0f;
        int i;
        int n = 0;
        Motor_Stop();
        g_snap.pwm_l = 0;
        g_snap.pwm_r = 0;
        g_snap.arm_phase = 0U;

        /* C1：扶正稳定 */
        for (i = 0; i < 100; i++) {
            if (osMutexAcquire(I2cMutexHandle, osWaitForever) == osOK) {
                MPU6050_Update(0.01f, &p, &gy, &gz);
                osMutexRelease(I2cMutexHandle);
            }
            g_snap.pitch = p;
            osDelay(20);
        }

        /* 松手等待 */
        for (i = 0; i < 75; i++) {
            if (osMutexAcquire(I2cMutexHandle, osWaitForever) == osOK) {
                MPU6050_Update(0.01f, &p, &gy, &gz);
                osMutexRelease(I2cMutexHandle);
            }
            g_snap.pitch = p;
            osDelay(20);
        }

        /* C2：松手后、陀螺仪接近静止时采样 Mid */
        sum = 0.0f;
        n = 0;
        for (i = 0; i < 100; i++) {
            if (osMutexAcquire(I2cMutexHandle, osWaitForever) == osOK) {
                MPU6050_Update(0.01f, &p, &gy, &gz);
                osMutexRelease(I2cMutexHandle);
            }
            g_snap.pitch = p;
            if (gy > -5.0f && gy < 5.0f && gz > -5.0f && gz < 5.0f) {
                sum += p;
                n++;
            }
            osDelay(20);
        }
        if (n < 30) {
            g_mid_angle = p;
        } else {
            g_mid_angle = sum / (float)n;
        }
        MPU6050_SetPitch(g_mid_angle);
        PID_ResetAll();
        s_arm_ticks = 0U;
        s_recover_cnt = 0U;
        s_prearm_cnt = 0U;
        s_prearm_ok = 0U;
        s_filt_rst = 1U;
        g_snap.arm_phase = 1U;
        osDelay(200);
    }
#endif

#if (TEST_STAGE == 4)
    for (;;)
    {
        g_snap.pwm_l = 60;
        g_snap.pwm_r = 60;
        Motor_SetSpeed(60 * MOTOR_L_DIR, 60 * MOTOR_R_DIR);
        osDelay(3000);
        Motor_Stop();
        g_snap.pwm_l = 0;
        g_snap.pwm_r = 0;
        osDelay(2000);
    }
#endif

    /*
     * 关键：零偏采集用了数秒 osDelay，若仍用任务开头的 last_wake，
     * vTaskDelayUntil 会连续空转追赶几百个周期 → 电机狂刷 → 掉电复位，
     * 或速度环积分爆掉 → 稳定 1～2s 后一股劲前倒。
     */
    last_wake = xTaskGetTickCount();
#if (TEST_STAGE >= 7)
    s_slow_cnt = 0;
    s_enc_sum_l = 0;
    s_enc_sum_r = 0;
#endif
    PID_ResetAll();
    Motor_Stop();
    s_filt_rst = 1U;
    s_prearm_ok = 0U;
    s_prearm_cnt = 0U;
    /* 掉电后再启：多等供电回稳 */
    osDelay(800);
    last_wake = xTaskGetTickCount();

    for (;;)
    {
        vTaskDelayUntil(&last_wake, period);

        float pitch = 0.0f;
#if (TEST_STAGE >= 2)
        float gyro_y = 0.0f, gyro_z = 0.0f;
#endif
        int16_t enc_l = 0, enc_r = 0;

#if (TEST_STAGE >= 2)
        if (osMutexAcquire(I2cMutexHandle, osWaitForever) == osOK) {
            MPU6050_Update(0.01f, &pitch, &gyro_y, &gyro_z);
            osMutexRelease(I2cMutexHandle);
        } else {
            pitch = g_snap.pitch;
        }
#endif

#if (TEST_STAGE >= 3)
        enc_l = (int16_t)(Encoder_ReadLeft()  * ENC_L_DIR);
        enc_r = (int16_t)(Encoder_ReadRight() * ENC_R_DIR);
#if (TEST_STAGE >= 7)
        s_enc_sum_l += enc_l;
        s_enc_sum_r += enc_r;
#endif
#endif

        g_snap.pitch = pitch;
        g_snap.enc_l = enc_l;
        g_snap.enc_r = enc_r;

#if (TEST_STAGE == 1) || (TEST_STAGE == 2) || (TEST_STAGE == 3) || (TEST_STAGE == 5)
        Motor_Stop();
        g_snap.pwm_l = 0;
        g_snap.pwm_r = 0;
        continue;
#endif

#if (TEST_STAGE == 4)
        continue;
#endif

#if (TEST_STAGE >= 6)
        float err_ang = pitch - g_mid_angle - g_mid_trim;
        g_snap.err_ang = err_ang;
        g_snap.auto_mid = 0.0f;

        /* 更早停转：到 ±28° 已失控，再打到 ±45° 只会大电流掉电复位 */
        if (err_ang > FALL_ANGLE || err_ang < -FALL_ANGLE) {
            Motor_Stop();
            PID_ResetAll();
            g_car_state = CAR_FALLEN;
            s_arm_ticks = 0U;
            s_recover_cnt = 0U;
            s_prearm_cnt = 0U;
            s_prearm_ok = 0U;
            s_filt_rst = 1U;
            g_snap.arm_phase = 1U;
            g_snap.pwm_l = 0;
            g_snap.pwm_r = 0;
#if (TEST_STAGE >= 7)
            s_enc_sum_l = 0;
            s_enc_sum_r = 0;
            s_slow_cnt = 0;
#endif
            continue;
        }
        if (g_car_state == CAR_FALLEN) {
            /* 不改写 Mid：7F32/33 精炼会把零点写歪，再启秒倒 */
            if (err_ang > -RECOVER_ANGLE && err_ang < RECOVER_ANGLE) {
                s_recover_cnt++;
            } else {
                s_recover_cnt = 0U;
            }
            if (s_recover_cnt >= RECOVER_NEED) {
                g_car_state = CAR_RUNNING;
                PID_ResetAll();
                s_arm_ticks = 0U;
                s_recover_cnt = 0U;
                s_prearm_cnt = 0U;
                s_prearm_ok = 0U;
                s_filt_rst = 1U;
                g_snap.arm_phase = 1U;
#if (TEST_STAGE >= 7)
                s_enc_sum_l = 0;
                s_enc_sum_r = 0;
                s_slow_cnt = 0;
#endif
            } else {
                Motor_Stop();
                g_snap.pwm_l = 0;
                g_snap.pwm_r = 0;
                continue;
            }
        }

        {
            int16_t dif_pwm = 0;
            int16_t angle_out;
            int16_t ave_pwm;
            int16_t pl;
            int16_t pr;
            int16_t pwm_cap = 100;

#if (TEST_STAGE >= 6)
            /* R：只扶稳，不改写 Mid（7F35）；等到 G 再松手 */
            if (s_prearm_ok == 0U) {
                if (err_ang > -PREARM_ANGLE && err_ang < PREARM_ANGLE) {
                    s_prearm_cnt++;
                } else {
                    s_prearm_cnt = 0U;
                }
                Motor_Stop();
                PID_SetArmHold(1U);
                s_filt_rst = 1U;
                g_snap.pwm_l = 0;
                g_snap.pwm_r = 0;
                g_snap.arm_phase = 1U;
                if (s_prearm_cnt >= PREARM_NEED) {
                    s_prearm_ok = 1U;
                    s_arm_ticks = 0U;
                    s_filt_rst = 1U;
                }
                continue;
            }

            if (s_arm_ticks < ARM_TICKS_MAX) {
                if (err_ang > ARM_ANGLE_MAX || err_ang < -ARM_ANGLE_MAX) {
                    Motor_Stop();
                    PID_SetArmHold(1U);
                    s_prearm_ok = 0U;
                    s_prearm_cnt = 0U;
                    s_arm_ticks = 0U;
                    s_filt_rst = 1U;
                    g_snap.pwm_l = 0;
                    g_snap.pwm_r = 0;
                    g_snap.arm_phase = 1U;
                    continue;
                }
                {
                    float t = ((float)s_arm_ticks + 1.0f) / (float)ARM_TICKS_MAX;
                    s_arm_ticks++;
                    pwm_cap = (int16_t)(60.0f + 40.0f * t);
                    g_snap.arm_phase = 1U;
                }
            } else {
                g_snap.arm_phase = 2U;
            }
            PID_SetArmHold(s_arm_ticks < ARM_HOLD_TICKS ? 1U : 0U);
#endif

#if (TEST_STAGE >= 7)
            s_slow_cnt++;
            if (s_slow_cnt >= 5U) {
                s_slow_cnt = 0U;
#if STAGE_H
                PID_BalanceSlow((int16_t)s_enc_sum_l, (int16_t)s_enc_sum_r,
                                (float)g_target_speed / 25.0f,
                                (float)g_target_turn / 25.0f);
#else
                g_target_speed = 0;
                g_target_turn = 0;
                PID_BalanceSlow((int16_t)s_enc_sum_l, (int16_t)s_enc_sum_r, 0.0f, 0.0f);
#endif
                s_enc_sum_l = 0;
                s_enc_sum_r = 0;
            }
#else
            PID_GetAngle()->Target = 0.0f;
#endif
            angle_out = PID_BalanceAngle(err_ang, &dif_pwm);
            ave_pwm = (int16_t)(-angle_out);

            /* 7F10/15 阻尼 ×28/±32：站立是 (Sp-0)。7H47：遥控也作用在 (Sp-目标) 上，
             * 不再在 hold=0 时整段关掉（7H45/46 一推杆就前冲的对照）。 */
            {
                static float s_damp_f = 0.0f;
                int16_t damp_i = 0;
                if (s_filt_rst) {
                    s_damp_f = 0.0f;
                }
                if (!PID_IsArmHold()) {
                    float spd = PID_GetAveSpeed();
                    float tgt = (float)g_target_speed / 25.0f;
                    float damp_f = (spd - tgt) * 28.0f;
                    if (damp_f > 32.0f) damp_f = 32.0f;
                    if (damp_f < -32.0f) damp_f = -32.0f;
                    s_damp_f = 0.75f * s_damp_f + 0.25f * damp_f;
                    damp_i = (int16_t)s_damp_f;
                    ave_pwm = (int16_t)(ave_pwm - damp_i);
                } else {
                    s_damp_f = 0.0f;
                }
                g_snap.damp_pwm = damp_i;
            }

#if (TEST_STAGE >= 6)
            if (s_arm_ticks < ARM_NO_TURN_TICKS) {
                dif_pwm = 0;
            }
#endif

            pl = ave_pwm + dif_pwm / 2;
            pr = ave_pwm - dif_pwm / 2;
            pl = (int16_t)(pl * MOTOR_L_DIR) + g_motor_trim;
            pr = (int16_t)(pr * MOTOR_R_DIR) - g_motor_trim;
            /* 7F44：取消隔拍，整数补偿；只加右轮、跟总 PWM 同向 */
            if (ave_pwm > 0) {
                pr = (int16_t)(pr + g_pwm_r_ofs);
            } else if (ave_pwm < 0) {
                pr = (int16_t)(pr - g_pwm_r_ofs);
            }

            /* 7F10/7F15：斜率 ±10 */
            {
                static int16_t s_pl_f = 0;
                static int16_t s_pr_f = 0;
                if (s_filt_rst) {
                    s_pl_f = 0;
                    s_pr_f = 0;
                    s_filt_rst = 0U;
                }
                if (pl > s_pl_f + 10) pl = (int16_t)(s_pl_f + 10);
                if (pl < s_pl_f - 10) pl = (int16_t)(s_pl_f - 10);
                if (pr > s_pr_f + 10) pr = (int16_t)(s_pr_f + 10);
                if (pr < s_pr_f - 10) pr = (int16_t)(s_pr_f - 10);
                s_pl_f = pl;
                s_pr_f = pr;
            }

#if (TEST_STAGE >= 6)
            if (pl > pwm_cap) pl = pwm_cap;
            if (pl < -pwm_cap) pl = -pwm_cap;
            if (pr > pwm_cap) pr = pwm_cap;
            if (pr < -pwm_cap) pr = -pwm_cap;
            /* 大倾角少满打：多次弱电后挣扎易掉电 */
            if (err_ang > ERR_SOFT_PWM || err_ang < -ERR_SOFT_PWM) {
                if (pl > ERR_SOFT_CAP) pl = ERR_SOFT_CAP;
                if (pl < -ERR_SOFT_CAP) pl = -ERR_SOFT_CAP;
                if (pr > ERR_SOFT_CAP) pr = ERR_SOFT_CAP;
                if (pr < -ERR_SOFT_CAP) pr = -ERR_SOFT_CAP;
            }
#endif

            if (pl > 100) pl = 100;
            if (pl < -100) pl = -100;
            if (pr > 100) pr = 100;
            if (pr < -100) pr = -100;

            Motor_SetSpeed(pl, pr);
            HAL_GPIO_WritePin(TB6612_STBY_GPIO_Port, TB6612_STBY_Pin, GPIO_PIN_SET);
            g_snap.pwm_l = pl;
            g_snap.pwm_r = pr;
            g_snap.angle_tgt = PID_GetAngleTarget();
            g_snap.ave_spd = PID_GetAveSpeed();
        }
#endif
    }
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
    uint8_t ch;
    char pkt[64];
    uint8_t pkt_len = 0;
    uint8_t in_pkt = 0;
    uint32_t last_ctrl_tick = osKernelGetTickCount();

    BleUart_RestartRx();

    for (;;)
    {
        if (osMessageQueueGet(BleRxQueueHandle, &ch, NULL, 50) != osOK)
        {
            uint32_t now = osKernelGetTickCount();
            if ((now - last_ctrl_tick) > BLE_STALE_MS) {
                g_target_speed = 0;
                g_target_turn  = 0;
            }
            BleUart_RestartRx();
            continue;
        }
        last_ctrl_tick = osKernelGetTickCount();
        g_ble_rx_cnt++;

        if (ch == '[')
        {
            in_pkt = 1;
            pkt_len = 0;
            continue;
        }
        if (in_pkt)
        {
            if (ch == ']')
            {
                pkt[pkt_len] = '\0';
                in_pkt = 0;
                {
                    char *tag = strtok(pkt, ",");
                    if (tag == NULL)
                    {
                        continue;
                    }
                    if (strcmp(tag, "joystick") == 0)
                    {
                        int lh = 0, lv = 0, rh = 0;
                        {
                            char *p = strtok(NULL, ",");
                            if (p) lh = atoi(p);
                        }
                        {
                            char *p = strtok(NULL, ",");
                            if (p) lv = atoi(p);
                        }
                        {
                            char *p = strtok(NULL, ",");
                            if (p) rh = atoi(p);
                        }
#if STAGE_H
                        g_target_speed = scale_stick(lv, H_SPEED_MAX);
                        {
                            int turn_raw = rh;
                            if (turn_raw > -8 && turn_raw < 8) {
                                /* 右杆回中才用左杆左右；左杆在前后时不借用，避免拉满后退转圈 */
                                if (lv > -8 && lv < 8) {
                                    turn_raw = lh;
                                } else {
                                    turn_raw = 0;
                                }
                            }
                            g_target_turn = scale_stick(turn_raw, H_TURN_MAX);
                        }
#else
                        g_target_speed = (int16_t)lv;
                        g_target_turn  = (int16_t)rh;
#endif
                    }
                    else if (strcmp(tag, "key") == 0)
                    {
                        char *name = strtok(NULL, ",");
                        char *act  = strtok(NULL, ",");
                        if (name && act && strcmp(act, "down") == 0)
                        {
                            if (strcmp(name, "1") == 0 || strcmp(name, "S") == 0)
                            {
                                g_target_speed = 0;
                                g_target_turn  = 0;
                            }
                        }
                    }
                    else if (strcmp(tag, "F") == 0)
                    {
                        g_target_speed = H_SPEED_MAX;
                    }
                    else if (strcmp(tag, "B") == 0)
                    {
                        g_target_speed = -H_SPEED_MAX;
                    }
                    else if (strcmp(tag, "L") == 0)
                    {
                        g_target_turn = -H_TURN_MAX;
                    }
                    else if (strcmp(tag, "R") == 0)
                    {
                        g_target_turn = H_TURN_MAX;
                    }
                    else if (strcmp(tag, "S") == 0)
                    {
                        g_target_speed = 0;
                        g_target_turn  = 0;
                    }
                }
            }
            else if (pkt_len < sizeof(pkt) - 1U)
            {
                pkt[pkt_len++] = (char)ch;
            }
            else
            {
                in_pkt = 0;
            }
            continue;
        }

        switch (ch)
        {
            case 'F': g_target_speed =  H_SPEED_MAX; break;
            case 'B': g_target_speed = -H_SPEED_MAX; break;
            case 'L': g_target_turn  = -H_TURN_MAX; break;
            case 'R': g_target_turn  =  H_TURN_MAX; break;
            case 'S':
                g_target_speed = 0;
                g_target_turn  = 0;
                break;
            default: break;
        }
    }
}
void StartDisplayTask(void *argument)
{
    OLED_Clear();
#if (TEST_STAGE >= 7)
    OLED_ShowString(0, 0, "7H55", OLED_8X16);
#else
    OLED_ShowString(0, 0, "ST:", OLED_8X16);
    OLED_ShowNum(24, 0, TEST_STAGE, 1, OLED_8X16);
#endif
#if (TEST_STAGE == 1)
    OLED_ShowString(0, 16, "OLED OK", OLED_8X16);
    OLED_ShowString(0, 32, "PB8/PB9", OLED_8X16);
#endif
    OLED_Update();

    for (;;)
    {
#if (TEST_STAGE == 1)
        OLED_Clear();
        OLED_ShowString(0, 0, "ST:1", OLED_8X16);
        OLED_ShowString(0, 16, "OLED OK", OLED_8X16);
        OLED_ShowString(0, 32, "PB8/PB9", OLED_8X16);
        OLED_ShowString(0, 48, "Alive", OLED_8X16);
        OLED_Update();
        osDelay(200);
#else
        CarSnapshot_t snap = g_snap;

        OLED_Clear();
#if (TEST_STAGE >= 7)
        OLED_ShowString(0, 0, "7H55", OLED_8X16);
        if (snap.arm_phase == 0U) {
            OLED_ShowString(56, 0, "C", OLED_8X16);
        } else if (snap.arm_phase == 1U) {
            OLED_ShowString(56, 0, "R", OLED_8X16);
        } else if (g_target_speed != 0 || g_target_turn != 0) {
            OLED_ShowString(56, 0, "H", OLED_8X16);
        } else {
            OLED_ShowString(56, 0, "G", OLED_8X16);
        }
#else
        OLED_ShowString(0, 0, "ST:", OLED_8X16);
        OLED_ShowNum(24, 0, TEST_STAGE, 1, OLED_8X16);
#endif

#if (TEST_STAGE == 2)
        if (snap.pwm_r == -1) {
            OLED_ShowString(0, 16, "MPU FAIL", OLED_8X16);
            OLED_ShowString(0, 32, "ID:", OLED_8X16);
            OLED_ShowNum(24, 32, (uint32_t)(snap.pwm_l & 0xFF), 3, OLED_8X16);
            OLED_ShowString(0, 48, "chk PB10/11", OLED_6X8);
        } else {
            OLED_ShowString(0, 16, "P:", OLED_8X16);
            OLED_ShowFloatNum(16, 16, snap.pitch, 3, 1, OLED_8X16);
            OLED_ShowString(0, 32, "Tilt car", OLED_8X16);
        }
#elif (TEST_STAGE == 3)
        OLED_ShowString(0, 16, "EL:", OLED_8X16);
        OLED_ShowSignedNum(24, 16, snap.enc_l, 5, OLED_8X16);
        OLED_ShowString(0, 32, "ER:", OLED_8X16);
        OLED_ShowSignedNum(24, 32, snap.enc_r, 5, OLED_8X16);
#elif (TEST_STAGE == 4)
        if (snap.pwm_l != 0) {
            OLED_ShowString(0, 16, "SPINNING", OLED_8X16);
        } else {
            OLED_ShowString(0, 16, "IDLE 2s", OLED_8X16);
        }
        OLED_ShowString(0, 32, "LIFT WHEELS", OLED_8X16);
        OLED_ShowString(0, 48, "need BAT VM", OLED_6X8);
#elif (TEST_STAGE == 5)
        OLED_ShowString(0, 16, "Tspd:", OLED_8X16);
        OLED_ShowSignedNum(48, 16, g_target_speed, 4, OLED_8X16);
        OLED_ShowString(0, 32, "Ttrn:", OLED_8X16);
        OLED_ShowSignedNum(48, 32, g_target_turn, 4, OLED_8X16);
        OLED_ShowString(0, 48, "Rx:", OLED_6X8);
        OLED_ShowNum(24, 48, g_ble_rx_cnt, 5, OLED_6X8);
#elif (TEST_STAGE == 6)
        OLED_ShowString(0, 16, "P:", OLED_8X16);
        OLED_ShowFloatNum(16, 16, snap.pitch - g_mid_angle, 3, 1, OLED_8X16);
        OLED_ShowString(0, 32, "W:", OLED_8X16);
        OLED_ShowSignedNum(16, 32, snap.pwm_l, 4, OLED_8X16);
        OLED_ShowString(0, 48, "Mid:", OLED_6X8);
        OLED_ShowFloatNum(32, 48, g_mid_angle, 3, 1, OLED_6X8);
#else
        /* 7H49：S/U 用大字，确认前后杆是否进了速度通道 */
        OLED_ShowString(0, 16, "S:", OLED_8X16);
        OLED_ShowSignedNum(24, 16, g_target_speed, 3, OLED_8X16);
        OLED_ShowString(64, 16, "U:", OLED_8X16);
        OLED_ShowSignedNum(88, 16, g_target_turn, 3, OLED_8X16);
        OLED_ShowString(0, 32, "T:", OLED_8X16);
        OLED_ShowFloatNum(16, 32, snap.angle_tgt, 3, 1, OLED_8X16);
        OLED_ShowString(64, 32, "P:", OLED_8X16);
        OLED_ShowFloatNum(80, 32, snap.err_ang, 3, 1, OLED_8X16);
        OLED_ShowString(0, 48, "Sp:", OLED_6X8);
        OLED_ShowSignedNum(24, 48, (int32_t)(snap.ave_spd * 100.0f), 3, OLED_6X8);
        OLED_ShowString(52, 48, "L:", OLED_6X8);
        OLED_ShowSignedNum(64, 48, snap.pwm_l, 3, OLED_6X8);
        OLED_ShowString(92, 48, "R:", OLED_6X8);
        OLED_ShowSignedNum(104, 48, snap.pwm_r, 3, OLED_6X8);
#endif
        OLED_Update();
        osDelay(100);
#endif
    }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

