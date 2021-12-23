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

#include "arm_math.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"
#include "main.h"

/**
 * @brief          ���ڻ�ȡ��ǰ�߶�
 * @author         RM
 * @param[in]      �߶ȵ�ָ�룬float32_t
 * @retval         ���ؿ�
 */
void AHRS_get_height(float32_t* high)
{
    if (high != NULL)
    {
        *high = 0.0f;
    }
}

/**
 * @brief          ���ڻ�ȡ��ǰγ��
 * @author         RM
 * @param[in]      γ�ȵ�ָ�룬float32_t
 * @retval         ���ؿ�
 */

void AHRS_get_latitude(float32_t* latitude)
{
    if (latitude != NULL)
    {
        *latitude = 22.0f;
    }
}

/**
 * @brief          ���ٿ���������
 * @author         RM
 * @param[in]      ������Ҫ�����ĸ�������float32_t
 * @retval         ����1/sqrt ������ĵ���
 */

float32_t AHRS_invSqrt(float32_t num)
{
    return 1/sqrtf(num);

//    float32_t halfnum = 0.5f * num;
//    float32_t y = num;
//    long i = *(long*)&y;
//    i = 0x5f3759df - (i >> 1);
//    y = *(float32_t*)&i;
//    y = y * (1.5f - (halfnum * y * y));
//    y = y * (1.5f - (halfnum * y * y));
//    return y;
}

/**
 * @brief          sin����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�sinֵ
 */

float32_t AHRS_sinf(float32_t angle)
{
	return sinf(angle);
    //return arm_sin_f32(angle);
	//XXX:don't know if works
}
/**
 * @brief          cos����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�cosֵ
 */


float32_t AHRS_cosf(float32_t angle)
{
	return cosf(angle);
    //return arm_cos_f32(angle);
	//XXX:may not work
}

/**
 * @brief          tan����
 * @author         RM
 * @param[in]      �Ƕ� ��λ rad
 * @retval         ���ض�Ӧ�Ƕȵ�tanֵ
 */

float32_t AHRS_tanf(float32_t angle)
{
    return tanf(angle);
}
/**
 * @brief          ����32λ�������ķ����Ǻ��� asin����
 * @author         RM
 * @param[in]      ����sinֵ�����1.0f����С-1.0f
 * @retval         ���ؽǶ� ��λ����
 */

float32_t AHRS_asinf(float32_t sin)
{

    return asinf(sin);
}

/**
 * @brief          �����Ǻ���acos����
 * @author         RM
 * @param[in]      ����cosֵ�����1.0f����С-1.0f
 * @retval         ���ض�Ӧ�ĽǶ� ��λ����
 */

float32_t AHRS_acosf(float32_t cos)
{

    return acosf(cos);
}

/**
 * @brief          �����Ǻ���atan����
 * @author         RM
 * @param[in]      ����tanֵ�е�yֵ ����������С������
 * @param[in]      ����tanֵ�е�xֵ ����������С������
 * @retval         ���ض�Ӧ�ĽǶ� ��λ����
 */

float32_t AHRS_atan2f(float32_t y, float32_t x)
{
    return atan2f(y, x);
}
