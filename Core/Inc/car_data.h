#ifndef CAR_DATA_H
#define CAR_DATA_H

#include <stdint.h>

typedef enum {
    CAR_INIT = 0,
    CAR_RUNNING,
    CAR_FALLEN,
    CAR_STOP
} CarState_t;

typedef struct {
    float pitch;
    float angle_tgt;
    float ave_spd;
    float err_ang;
    float auto_mid;
    int16_t damp_pwm;
    int16_t enc_l;
    int16_t enc_r;
    int16_t pwm_l;
    int16_t pwm_r;
    uint8_t arm_phase;
    uint8_t reset_flags; /* RCC_CSR 高8位，用于区分掉电/按键复位 */
} CarSnapshot_t;

extern volatile int16_t g_target_speed;
extern volatile int16_t g_target_turn;
extern volatile CarState_t g_car_state;
extern CarSnapshot_t g_snap;
/* 机械竖直对应的俯仰零偏（上电扶正时自动采集） */
extern volatile float g_mid_angle;
/* 机械微调：对拧。7F37(-3)/7F38(+3) 都比 trim=0 更转，禁止再用 */
extern volatile int16_t g_motor_trim;
/* 转向环固定补偿：站立时 7F35 不使用（dif=0） */
extern volatile int16_t g_turn_trim;
/* 右轮同向死区补偿，正值；7F39 起使用 */
extern volatile int16_t g_pwm_r_ofs;
/* 俯仰零偏手动补偿，一般保持 0 */
extern volatile float g_mid_trim;

#endif
