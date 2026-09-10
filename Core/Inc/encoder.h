#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

void Encoder_Init(void);
int16_t Encoder_ReadLeft(void);
int16_t Encoder_ReadRight(void);

#endif
