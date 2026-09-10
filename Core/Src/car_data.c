#include "car_data.h"

volatile int16_t g_target_speed = 0;
volatile int16_t g_target_turn  = 0;
volatile CarState_t g_car_state = CAR_INIT;
CarSnapshot_t g_snap = {0};
volatile float g_mid_angle = 0.0f;
/* 7F39：对拧 trim 已否决，必须回到 0（=7F35） */
volatile int16_t g_motor_trim = 0;
volatile int16_t g_enc_bias_l = 0;
volatile int16_t g_turn_trim = 0;
/*
 * 只给右轮加同向 PWM（不扣左轮）。禁止分数/隔拍（7F43 否决）。
 *  2 → 7F41：有时原地慢摆，有时偏左 15~20°（本档，撤回 7F43）
 *  1 → 7F42：偏右 15~20°
 *  1.5 隔拍 → 7F43：新问题（前冲加速 / 先右偏 45° 再 1/4 圈）
 */
volatile int16_t g_pwm_r_ofs = 2;
/* 俯仰阶梯已过关：0.00 后倒 / 0.02 均衡 / ≥0.05 前倒。禁止再拧 */
volatile float g_mid_trim = 0.02f;
