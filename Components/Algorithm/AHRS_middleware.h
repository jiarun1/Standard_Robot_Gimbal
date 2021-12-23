
/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       AHRS_MiddleWare.c/h
  * @brief      ��̬�����м�㣬Ϊ��̬�����ṩ��غ���
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef AHRS_MIDDLEWARE_H
#define AHRS_MIDDLEWARE_H

#include "arm_math.h"
#include "struct_typedef.h"

//���� NULL
#ifndef NULL
#define NULL 0
#endif

//����PI ֵ
#ifndef PI
#define PI 3.14159265358979f
#endif

//���� �Ƕ�(��)ת���� ���ȵı���
#ifndef ANGLE_TO_RAD
#define ANGLE_TO_RAD 0.01745329251994329576923690768489f
#endif

//���� ���� ת���� �Ƕȵı���
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

extern void AHRS_get_height(float32_t *high);
extern void AHRS_get_latitude(float32_t *latitude);
extern float32_t AHRS_invSqrt(float32_t num);
extern float32_t AHRS_sinf(float32_t angle);
extern float32_t AHRS_cosf(float32_t angle);
extern float32_t AHRS_tanf(float32_t angle);
extern float32_t AHRS_asinf(float32_t sin);
extern float32_t AHRS_acosf(float32_t cos);
extern float32_t AHRS_atan2f(float32_t y, float32_t x);
#endif
