#include "pid.h"
#include "car_data.h"
#include <stddef.h>

/*
 * ========== 7F35 = 收回 7F15/7F10 控制（有对照，不是新结构）==========
 *
 * 【为何会反复「<1s / 一边倒 / 再启秒倒」】
 * 不是软启动没调好，是把已验证能晃约 2s 的控制拆掉了。
 *
 * 事实排序（你的实测）：
 *  7F10/15：不复位；慢晃约 2s；Sp 慢爬；后倒；右偏 15~20°  ← F 系最好控制
 *  7F14：去掉阻尼、改串级 → 松手直接往一边倒
 *  7F18：恢复 7F15 力矩后，不复位，松手 1~2s 后倒
 *  7F28：mid_trim=0.02 → 前/后倒大致均衡，冷启 2~3s
 *  7F30：满电冷启 3~4s（最好时长）；再启仍约 1s
 *  7F31~33：拧软启动 / Mid 精炼 → 冷启掉到 1~2s，再启秒倒/掉电
 *  7F34：取消冻 T + 阻尼降到 ×18 → 又变成 <1s 往一边倒（同 7F14）
 *
 * 【本版只改回有对照的控制，不发明】
 *  A) 弱速度 ±1.5°；|Sp|>0.4 冻 T=0（7F10：交给阻尼刹 Sp，不让串级泵轮）
 *  B) 速度环保持 7F10：Kp=0.5 / Ki=0.005（7F11/34 加强速度都更差）
 *  C) 原地关转向环（7F21 turn_trim=12 原地右拧，已否决）
 */
#define PID_SPEED_SIGN  (+1.0f)

static PID_t s_angle = {
    .Kp = 5.0f, .Ki = 0.06f, .Kd = 5.5f,
    .OutMax = 100.0f, .OutMin = -100.0f,
    .OutOffset = 1.5f,
    .ErrorIntMax = 150.0f, .ErrorIntMin = -150.0f,
};

static PID_t s_speed = {
    .Kp = 0.5f, .Ki = 0.005f, .Kd = 0.0f,
    .OutMax = 1.5f, .OutMin = -1.5f,
    .OutOffset = 0.0f,
    .ErrorIntMax = 15.0f, .ErrorIntMin = -15.0f,
};

static PID_t s_turn = {
    .Kp = 3.5f, .Ki = 1.5f, .Kd = 0.0f,
    .OutMax = 22.0f, .OutMin = -22.0f,
    .OutOffset = 0.0f,
    .ErrorIntMax = 12.0f, .ErrorIntMin = -12.0f,
};

static float s_ave_spd = 0.0f;
static int16_t s_dif_pwm = 0;
static uint8_t s_arm_hold = 0;
static uint8_t s_stand_hold = 0;
/* 上次非零前后指令：+1 前进 / -1 后退，用于换向沿清积分 */
static int8_t s_last_fb = 0;

void PID_SetArmHold(uint8_t hold)
{
    s_arm_hold = hold ? 1U : 0U;
}

uint8_t PID_IsArmHold(void)
{
    return s_arm_hold;
}

uint8_t PID_IsStandHold(void)
{
    return s_stand_hold;
}

void PID_Init(PID_t *p)
{
    p->Target = 0.0f;
    p->Actual = 0.0f;
    p->Actual1 = 0.0f;
    p->Out = 0.0f;
    p->Error0 = 0.0f;
    p->Error1 = 0.0f;
    p->ErrorInt = 0.0f;
}

void PID_Update(PID_t *p)
{
    p->Error1 = p->Error0;
    p->Error0 = p->Target - p->Actual;

    if (p->Ki != 0.0f) {
        p->ErrorInt += p->Error0;
        if (p->ErrorInt > p->ErrorIntMax) p->ErrorInt = p->ErrorIntMax;
        if (p->ErrorInt < p->ErrorIntMin) p->ErrorInt = p->ErrorIntMin;
    } else {
        p->ErrorInt = 0.0f;
    }

    p->Out = p->Kp * p->Error0
           + p->Ki * p->ErrorInt
           - p->Kd * (p->Actual - p->Actual1);

    if (p->Out > 0.0f) p->Out += p->OutOffset;
    if (p->Out < 0.0f) p->Out -= p->OutOffset;

    if (p->Out > p->OutMax) p->Out = p->OutMax;
    if (p->Out < p->OutMin) p->Out = p->OutMin;

    p->Actual1 = p->Actual;
}

void PID_ResetAll(void)
{
    PID_Init(&s_angle);
    PID_Init(&s_speed);
    PID_Init(&s_turn);
    s_dif_pwm = 0;
    s_ave_spd = 0.0f;
    s_arm_hold = 0U;
    s_stand_hold = 0U;
    s_last_fb = 0;
}

PID_t *PID_GetAngle(void) { return &s_angle; }
PID_t *PID_GetSpeed(void) { return &s_speed; }
PID_t *PID_GetTurn(void)  { return &s_turn; }

float PID_EncToSpeed50ms(int16_t enc_sum)
{
    return (float)enc_sum / 44.0f / 0.05f / 9.27666f;
}

float PID_EncToSpeed10ms(int16_t enc_l, int16_t enc_r)
{
    float ave = ((float)enc_l + (float)enc_r) * 0.5f;
    return ave / 44.0f / 0.01f / 9.27666f * PID_SPEED_SIGN;
}

float PID_GetAutoMid(void)
{
    return 0.0f;
}

void PID_BalanceSlow(int16_t enc_l_sum, int16_t enc_r_sum, float speed_tgt, float turn_tgt)
{
    float left_spd = PID_EncToSpeed50ms(enc_l_sum);
    float right_spd = PID_EncToSpeed50ms(enc_r_sum);
    float dif_spd = left_spd - right_spd;
    /* 7H48：门槛 0.05 会把 S=1（1/25=0.04）当成站立，滑动前后没反应 */
    uint8_t hold = (speed_tgt > -0.02f && speed_tgt < 0.02f
                    && turn_tgt > -0.02f && turn_tgt < 0.02f) ? 1U : 0U;

    s_stand_hold = hold;
    s_ave_spd = (left_spd + right_spd) * 0.5f * PID_SPEED_SIGN;

    if (hold && s_ave_spd > -0.12f && s_ave_spd < 0.12f) {
        s_ave_spd = 0.0f;
    }

    /* 7H49：有前后目标才用 G23 速度环；站立/纯左右仍 7F10（7H47 左右已恢复） */
    {
        uint8_t fb = (speed_tgt <= -0.02f || speed_tgt >= 0.02f) ? 1U : 0U;
        if (fb) {
            s_speed.Kp = 1.5f;
            s_speed.Ki = 0.03f;
            s_speed.OutMax = 4.0f;
            s_speed.OutMin = -4.0f;
            s_speed.ErrorIntMax = 50.0f;
            s_speed.ErrorIntMin = -50.0f;
        } else {
            s_speed.Kp = 0.5f;
            s_speed.Ki = 0.005f;
            s_speed.OutMax = 1.5f;
            s_speed.OutMin = -1.5f;
            s_speed.ErrorIntMax = 15.0f;
            s_speed.ErrorIntMin = -15.0f;
            if (s_speed.ErrorInt > 15.0f) s_speed.ErrorInt = 15.0f;
            if (s_speed.ErrorInt < -15.0f) s_speed.ErrorInt = -15.0f;
        }
    }

    /* 7H53：换向沿才清 I（过零也算）。禁止按「目标与 Sp 异号」每周期清角度 I（7H50）。 */
    {
        int8_t fb_now = 0;
        if (speed_tgt >= 0.02f) {
            fb_now = 1;
        } else if (speed_tgt <= -0.02f) {
            fb_now = -1;
        }
        if (fb_now != 0 && s_last_fb != 0 && fb_now != s_last_fb) {
            s_speed.ErrorInt = 0.0f;
            s_speed.Out = 0.0f;
            /* 7H54：不清算角度 I。7H53 换向清角度 I 后，第二下前后无效且会左右偏 */
        }
        if (fb_now != 0) {
            s_last_fb = fb_now;
        }
    }

    /* 7F10：冻 T 只在 hold。 */
    if (hold && (s_ave_spd > 0.40f || s_ave_spd < -0.40f)) {
        s_speed.ErrorInt = 0.0f;
        s_speed.Out = 0.0f;
        s_angle.Target = 0.0f;
        s_dif_pwm = 0;
        if (s_arm_hold) {
            return;
        }
        (void)dif_spd;
        (void)turn_tgt;
        return;
    }
    /* 7H55：超速冻必须留 7F10 的 0.4 余量。7H53 用 Sp>目标，满杆目标只有 0.32，
     * G 稍有速度或轻推 S=2（目标 0.08）立刻冻 T，先动的杆等于没接到速度环，
     * 车仍按 G 原方向走（先推前进却看到后退）。 */
    if ((speed_tgt >= 0.02f && s_ave_spd > speed_tgt + 0.40f)
        || (speed_tgt <= -0.02f && s_ave_spd < speed_tgt - 0.40f)) {
        s_speed.ErrorInt = 0.0f;
        s_speed.Out = 0.0f;
        s_angle.Target = 0.0f;
        s_dif_pwm = 0;
        (void)dif_spd;
        (void)turn_tgt;
        return;
    }

    s_speed.Target = speed_tgt;
    s_speed.Actual = s_ave_spd;
    if (hold) {
        s_speed.ErrorInt *= 0.85f;
    }
    PID_Update(&s_speed);

    if (s_arm_hold) {
        s_speed.ErrorInt = 0.0f;
        s_angle.Target = 0.0f;
        s_turn.ErrorInt = 0.0f;
        s_dif_pwm = 0;
        return;
    }

    s_angle.Target = s_speed.Out;
    if (hold) {
        if (s_angle.Target > 1.5f) s_angle.Target = 1.5f;
        if (s_angle.Target < -1.5f) s_angle.Target = -1.5f;
        s_turn.ErrorInt = 0.0f;
        s_dif_pwm = 0;
        (void)dif_spd;
        (void)turn_tgt;
        return;
    }

    if (speed_tgt <= -0.02f || speed_tgt >= 0.02f) {
        if (s_angle.Target > 4.0f) s_angle.Target = 4.0f;
        if (s_angle.Target < -4.0f) s_angle.Target = -4.0f;
    } else {
        if (s_angle.Target > 1.5f) s_angle.Target = 1.5f;
        if (s_angle.Target < -1.5f) s_angle.Target = -1.5f;
    }

    /* 纯前进/后退仍关转向（7H45 后退转圈仍成立）；有左右才开转向 */
    if (turn_tgt > -0.05f && turn_tgt < 0.05f) {
        s_turn.ErrorInt = 0.0f;
        s_dif_pwm = 0;
        (void)dif_spd;
        return;
    }

    s_turn.Target = turn_tgt;
    s_turn.Actual = dif_spd;
    PID_Update(&s_turn);
    s_dif_pwm = (int16_t)(s_turn.Out + (float)g_turn_trim);
}

int16_t PID_BalanceAngle(float angle, int16_t *dif_pwm)
{
    if (angle > 6.0f || angle < -6.0f) {
        s_angle.ErrorInt *= 0.5f;
    }
    if (angle > 12.0f || angle < -12.0f) {
        s_angle.ErrorInt = 0.0f;
        s_speed.ErrorInt = 0.0f;
        s_angle.Target = 0.0f;
    }

    if (s_arm_hold) {
        s_angle.ErrorInt = 0.0f;
        s_angle.Target = 0.0f;
    }

    s_angle.Actual = angle;
    PID_Update(&s_angle);
    if (dif_pwm != NULL) {
        *dif_pwm = s_dif_pwm;
    }
    return (int16_t)s_angle.Out;
}

float PID_GetAngleTarget(void)
{
    return s_angle.Target;
}

float PID_GetAveSpeed(void)
{
    return s_ave_spd;
}
