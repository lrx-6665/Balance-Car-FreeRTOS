#include "mpu6050.h"
#include "i2c.h"
#include "main.h"
#include <math.h>

/*
 * ?????????MPU6050 ????? I2C??PB10=SCL, PB11=SDA?????? OLED(PB8/PB9) ?????
 * F103 ??? I2C ??????????????? MyI2C ????????
 */
#define MPU_ADDR          0xD0U
#define MPU_SCL_Port      GPIOB
#define MPU_SCL_Pin       GPIO_PIN_10
#define MPU_SDA_Port      GPIOB
#define MPU_SDA_Pin       GPIO_PIN_11

extern I2C_HandleTypeDef hi2c2;

static float s_pitch = 0.0f;
static float s_gyro_offset_y = 0.0f;
static float s_gyro_offset_z = 0.0f;
static uint8_t s_whoami = 0;
static const float ALPHA = 0.99f;
static const float ACC_OFFSET = 0.5f;

uint8_t MPU6050_GetWhoAmI(void)
{
    return s_whoami;
}

static void mpu_w_scl(uint8_t v)
{
    HAL_GPIO_WritePin(MPU_SCL_Port, MPU_SCL_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void mpu_w_sda(uint8_t v)
{
    HAL_GPIO_WritePin(MPU_SDA_Port, MPU_SDA_Pin, v ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static uint8_t mpu_r_sda(void)
{
    return (HAL_GPIO_ReadPin(MPU_SDA_Port, MPU_SDA_Pin) == GPIO_PIN_SET) ? 1U : 0U;
}

static void mpu_i2c_gpio_init(void)
{
    GPIO_InitTypeDef g = {0};

    /* ?????? I2C2???? PB10/PB11 ???? GPIO */
    (void)HAL_I2C_DeInit(&hi2c2);
    __HAL_RCC_I2C2_CLK_DISABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin = MPU_SCL_Pin | MPU_SDA_Pin;
    g.Mode = GPIO_MODE_OUTPUT_OD;
    g.Pull = GPIO_NOPULL;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &g);

    mpu_w_scl(1);
    mpu_w_sda(1);
}

static void mpu_start(void)
{
    mpu_w_sda(1);
    mpu_w_scl(1);
    mpu_w_sda(0);
    mpu_w_scl(0);
}

static void mpu_stop(void)
{
    mpu_w_sda(0);
    mpu_w_scl(1);
    mpu_w_sda(1);
}

static void mpu_send_byte(uint8_t byte)
{
    uint8_t i;
    for (i = 0; i < 8U; i++) {
        mpu_w_sda((byte & (0x80U >> i)) ? 1U : 0U);
        mpu_w_scl(1);
        mpu_w_scl(0);
    }
    /* ????????? ACK????????? */
    mpu_w_sda(1);
    mpu_w_scl(1);
    mpu_w_scl(0);
}

static uint8_t mpu_recv_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t byte = 0;
    mpu_w_sda(1);
    for (i = 0; i < 8U; i++) {
        mpu_w_scl(1);
        if (mpu_r_sda()) {
            byte |= (uint8_t)(0x80U >> i);
        }
        mpu_w_scl(0);
    }
    mpu_w_sda(ack ? 1U : 0U);
    mpu_w_scl(1);
    mpu_w_scl(0);
    return byte;
}

static void mpu_write(uint8_t reg, uint8_t val)
{
    mpu_start();
    mpu_send_byte(MPU_ADDR);
    mpu_send_byte(reg);
    mpu_send_byte(val);
    mpu_stop();
}

static void mpu_read(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t i;
    mpu_start();
    mpu_send_byte(MPU_ADDR);
    mpu_send_byte(reg);
    mpu_start();
    mpu_send_byte((uint8_t)(MPU_ADDR | 0x01U));
    for (i = 0; i < len; i++) {
        buf[i] = mpu_recv_byte((i + 1U) >= len ? 1U : 0U);
    }
    mpu_stop();
}

uint8_t MPU6050_Init(void)
{
    uint8_t id = 0;

    mpu_i2c_gpio_init();
    HAL_Delay(50);

    /* ???????????????? */
    mpu_write(0x6B, 0x01); /* ??????????=PLL X */
    mpu_write(0x6C, 0x00);
    mpu_write(0x19, 0x07);
    mpu_write(0x1A, 0x00);
    mpu_write(0x1B, 0x18); /* ???? ??2000??/s */
    mpu_write(0x1C, 0x18); /* ????? ??16g */
    HAL_Delay(50);

    mpu_read(0x75, &id, 1);
    s_whoami = id;
    /* ??? 0x68?????????/?????????????? 0x98/0x70 */
    if (id != 0x68U && id != 0x98U && id != 0x70U) {
        return 1;
    }
    return 0;
}

void MPU6050_Calibrate(void)
{
    int32_t sum_y = 0, sum_z = 0;
    uint8_t buf[6];
    int i;

    for (i = 0; i < 200; i++) {
        mpu_read(0x43, buf, 6);
        sum_y += (int16_t)((buf[2] << 8) | buf[3]);
        sum_z += (int16_t)((buf[4] << 8) | buf[5]);
        HAL_Delay(5);
    }
    s_gyro_offset_y = (float)sum_y / 200.0f;
    s_gyro_offset_z = (float)sum_z / 200.0f;

    /* ??????????????????????? 0 ????????????? */
    {
        uint8_t abuf[6];
        int16_t ax, az;
        mpu_read(0x3B, abuf, 6);
        ax = (int16_t)((abuf[0] << 8) | abuf[1]);
        az = (int16_t)((abuf[4] << 8) | abuf[5]);
        s_pitch = -atan2f((float)ax, (float)az) * 57.2958f + ACC_OFFSET;
    }
}

void MPU6050_SetPitch(float pitch)
{
    s_pitch = pitch;
}

void MPU6050_Update(float dt, float *pitch, float *gyro_y, float *gyro_z)
{
    uint8_t buf[14];
    int16_t ax, az, gy, gz;
    float acc_angle;
    float gyro_y_dps;
    float gyro_z_dps;

    /* ??????? Acc+Temp+Gyro?????? GetData ??? */
    mpu_read(0x3B, buf, 14);
    ax = (int16_t)((buf[0] << 8) | buf[1]);
    az = (int16_t)((buf[4] << 8) | buf[5]);
    gy = (int16_t)((buf[10] << 8) | buf[11]) - (int16_t)s_gyro_offset_y;
    gz = (int16_t)((buf[12] << 8) | buf[13]) - (int16_t)s_gyro_offset_z;

    /* ??????AngleAcc = -atan2(AX, AZ) * 180/pi */
    acc_angle = -atan2f((float)ax, (float)az) * 57.2958f + ACC_OFFSET;
    gyro_y_dps = (float)gy / 16.4f;
    gyro_z_dps = (float)gz / 16.4f;

    /* ??��??Alpha=0.01 ???��?????0.99 ??????????? */
    s_pitch = (1.0f - ALPHA) * acc_angle + ALPHA * (s_pitch + gyro_y_dps * dt);

    *pitch  = s_pitch;
    *gyro_y = gyro_y_dps;
    *gyro_z = gyro_z_dps;
}
