#ifndef PID_H
#define PID_H

#include <stdint.h>
typedef struct {
    float Target;
    float Actual;
    float Actual1;
    float Out;
    float Kp;
    float Ki;
    float Kd;
    float Error0;
    float Error1;
    float ErrorInt;
    float ErrorIntMax;
    float ErrorIntMin;
    float OutMax;
    float OutMin;
    float OutOffset;
} PID_t;

void PID_Init(PID_t *p);
void PID_Update(PID_t *p);
void PID_ResetAll(void);

PID_t *PID_GetAngle(void);
PID_t *PID_GetSpeed(void);
PID_t *PID_GetTurn(void);

/* 50ms 累计编码器增量 → rev/s */
float PID_EncToSpeed50ms(int16_t enc_sum);

void PID_BalanceSlow(int16_t enc_l_sum, int16_t enc_r_sum, float speed_tgt, float turn_tgt);
int16_t PID_BalanceAngle(float angle, int16_t *dif_pwm);
void PID_SetArmHold(uint8_t hold);
uint8_t PID_IsArmHold(void);
float PID_GetAngleTarget(void);
float PID_GetAveSpeed(void);
uint8_t PID_IsStandHold(void);
float PID_EncToSpeed10ms(int16_t enc_l, int16_t enc_r);
float PID_GetAutoMid(void);

#endif
