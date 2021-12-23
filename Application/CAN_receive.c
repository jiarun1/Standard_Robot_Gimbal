/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       can_receive.c/h
  * @brief      there is CAN interrupt function  to receive motor data,
  *             and CAN send function to send motor current to control motor.
  * @note       remember to add hook function back soon.
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. support hal lib
  *  V2.0.0     Jan-25-2021     YW              1. modify to fit the project
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "CAN_receive.h"

#include "cmsis_os.h"

#include "main.h"

//#include "detect_task.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//motor data read
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/**
  * motor data
  * motor_chassis[0]: chassis motor1 3508
  * motor_chassis[1]: chassis motor2 3508
  * motor_chassis[2]: chassis motor3 3508
  * motor_chassis[3]: chassis motor4 3508
  * motor_chassis[4]: trigger gimbal motor 2006
  * motor_chassis[5]: pitch gimbal motor 6020
  * motor_chassis[6]: yaw gimbal motor 6020
  * motor_chassis[7]: friction motor1 3508
  * motor_chassis[8]: friction motor2 3508
  */
static motor_measure_t motor_chassis[9];// XXX: The motor ID is different from the standard robot

static CAN_TxHeaderTypeDef  gimbal_tx_message;
static uint8_t              gimbal_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];
static CAN_TxHeaderTypeDef  shoot_tx_message;
static uint8_t              shoot_can_send_data[8];

/**
  * @brief          hal CAN fifo call back, receive motor data
  * @param[in]      hcan, the point to CAN handle
  * @retval         none
  */
//回调函数 当rx信箱中有信息时唤起此函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);  //获得信息

    if (hcan->Instance == hcan1.Instance)
    {
      switch (rx_header.StdId)
      {
        case CAN_FRICTION_MOTOR1:
        case CAN_FRICTION_MOTOR2:
        case CAN_YAW_MOTOR_ID:
        case CAN_PIT_MOTOR_ID:
        case CAN_TRIGGER_MOTOR_ID:
        {
          static uint8_t i = 0;  //对于所有的电机
          i = rx_header.StdId - CAN_3508_M1_ID;  //电机的ID
          if (i == 0 || i == 1) i += 7;
          get_motor_measure(&motor_chassis[i], rx_data);  //获得相应电机的encoder值 转速值rpm 电流值 电机温度
          break;
        }
        default:
        {
          break;
        }
      }
    }
    else if (hcan->Instance == hcan2.Instance)
    {
      switch (rx_header.StdId)
      {
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        {
          static uint8_t i = 0;  //对于所有的电机
          i = rx_header.StdId - CAN_3508_M1_ID;  //电机的ID
          get_motor_measure(&motor_chassis[i], rx_data);  //获得相应电机的encoder值 转速值rpm 电流值 电机温度

          break;
        }

      default:
      {
            break;
      }
    }
  }
}



/**
  * @brief          send control current of motor (0x205, 0x206, 0x207, 0x208)
  * @param[in]      yaw: (0x207) 6020 motor control current, range [-30000,30000]
  * @param[in]      pitch: (0x206) 6020 motor control current, range [-30000,30000]
  * @param[in]      shoot: (0x205) 2006 motor control current, range [-10000,10000]
  * @param[in]      rev: (0x208) reserve motor control current, useless currently
  * @retval         none
  */
//通过CAN发送给平衡环电机的指令
void CAN_cmd_gimbal(int16_t yaw, int16_t pitch, int16_t shoot, int16_t rev)
{
    uint32_t send_mail_box;
    gimbal_tx_message.StdId = CAN_GIMBAL_ALL_ID;
    gimbal_tx_message.IDE = CAN_ID_STD;
    gimbal_tx_message.RTR = CAN_RTR_DATA;
    gimbal_tx_message.DLC = 0x08;

    gimbal_can_send_data[0] = (shoot >> 8);
    gimbal_can_send_data[1] = shoot;
    gimbal_can_send_data[2] = (pitch >> 8);
    gimbal_can_send_data[3] = pitch;
    gimbal_can_send_data[4] = (yaw >> 8);
    gimbal_can_send_data[5] = yaw;
    gimbal_can_send_data[6] = (rev >> 8);
    gimbal_can_send_data[7] = rev;

    HAL_CAN_AddTxMessage(&GIMBAL_CAN, &gimbal_tx_message, gimbal_can_send_data, &send_mail_box);
}

/**
  * @brief          send CAN packet of ID 0x700, it will set chassis motor 3508 to quick ID setting
  *                 (this function needs checking)
  * @param[in]      none
  * @retval         none
  */
void CAN_cmd_chassis_reset_ID(void)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = 0x700;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = 0;
    chassis_can_send_data[1] = 0;
    chassis_can_send_data[2] = 0;
    chassis_can_send_data[3] = 0;
    chassis_can_send_data[4] = 0;
    chassis_can_send_data[5] = 0;
    chassis_can_send_data[6] = 0;
    chassis_can_send_data[7] = 0;

    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}


/**
  * @brief          send control current of motor (0x201, 0x202, 0x203, 0x204)
  * @param[in]      motor1: (0x201) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor2: (0x202) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor3: (0x203) 3508 motor control current, range [-16384,16384] 
  * @param[in]      motor4: (0x204) 3508 motor control current, range [-16384,16384] 
  * @retval         none
  */
//通过CAN发送至四个底盘电机的指令
void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

/**
  * @brief          send control current of the motor (0x201, 0x202)
  * @param[in]      friction motor: (0x201) 3508 motor control current, range [-16384,16384]
  * @param[in]      friction motor: (0x202) 3508 motor control current, range [-16384,16384]
  * @retval          none
  */
void CAN_cmd_shoot(int16_t fri1, int16_t fri2)
{
	uint32_t send_mail_box;
	shoot_tx_message.StdId = CAN_SHOOT_ALL_ID;
	shoot_tx_message.IDE = CAN_ID_STD;
	shoot_tx_message.RTR = CAN_RTR_DATA;
	shoot_tx_message.DLC = 0x08;
	shoot_can_send_data[0] = fri1 >> 8;
	shoot_can_send_data[1] = fri1;
	shoot_can_send_data[2] = fri2 >> 8;
	shoot_can_send_data[3] = fri2;
	shoot_can_send_data[4] = 0;
	shoot_can_send_data[5] = 0;
	shoot_can_send_data[6] = 0;
	shoot_can_send_data[7] = 0;
	HAL_CAN_AddTxMessage(&SHOOT_CAN, &shoot_tx_message, shoot_can_send_data, &send_mail_box);
}

/**
  * @brief          return the yaw 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_yaw_gimbal_motor_measure_point(void)
{
    return &motor_chassis[6];
}

/**
  * @brief          return the pitch 6020 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_pitch_gimbal_motor_measure_point(void)
{
    return &motor_chassis[5];
}


/**
  * @brief          return the trigger 2006 motor data point
  * @param[in]      none
  * @retval         motor data point
  */
const motor_measure_t *get_trigger_motor_measure_point(void)
{
    return &motor_chassis[4];
}

/**
 * @brief           return the friction 3508 motor data point
 * @param[in]       i: motor number, range [0,1]
 * @retval          motor data point
 */
const motor_measure_t *get_friction_motor_measure_point(uint8_t i)
{
	return &motor_chassis[i+7];
}
