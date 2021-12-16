/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       detect_task.c/h
  * @brief      provide task used to detect error, checked by receiving data time.
  *             provide detect hook function and detect data error function.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add oled, gyro accel and mag sensors
  *  V2.0.0     Mar-03-2021     YW              1. modift to fit project
  *
  @verbatim
  ==============================================================================
    How to add a device

    1. in "detect_task.h", add the device name at the end of errorList, for example:

        enum errorList
        {
            ...
            XXX_TOE,    //new device
            ERROR_LIST_LENGHT,
        };

    2.in detect_init function, add the "set_offline_time", "set_online_time", and
      "priority" parameters, for example:

        uint16_t set_item[ERROR_LIST_LENGHT][3] =
        {
            ...
            {n,n,n}, //XX_TOE
        };

    3. if XXX_TOE has "data_is_error_fun", "solve_lost_fun", "solve_data_error_fun"
       functions, assign them to function pointer.

    4. when XXX_TOE device data comes, add the function detect_hook(XXX_TOE) function there.

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
  
#ifndef DETECT_TASK_H
#define DETECT_TASK_H
#include "struct_typedef.h"
#include "main.h"
#include "arm_math.h"


#define DETECT_TASK_INIT_TIME 57
#define DETECT_CONTROL_TIME 10

//错误码以及对应设备顺序
enum errorList
{
    DBUS_TOE = 0,
    YAW_GIMBAL_MOTOR_TOE,
    PITCH_GIMBAL_MOTOR_TOE,
    TRIGGER_MOTOR_TOE,
	FIRCTION_MOTOR1_TOE,
	FRICTION_MOTOR2_TOE,
    BOARD_MPU6500_TOE,
    BOARD_IST8310_TOE,
    //REFEREE_TOE,
    ERROR_LIST_LENGHT,
};

typedef struct
{
    uint32_t new_time;                  //latest timestamp
    uint32_t last_time;                 //last timestamp
    uint32_t lost_time;                 //the timestamp that this device is lost
    uint32_t work_time;                 //the latest timestamp that this device is still working
    uint16_t set_offline_time : 12;     //larger than this timestamp, device is regarded as lost
    uint16_t set_online_time : 12;      //max allowed timestamp of device to startup
    uint8_t enable : 1;                 //1->device is enabled  0->device is disabled
    uint8_t priority : 4;               //the priority of the device
    uint8_t error_exist : 1;            //1->device is works incorrectly (not means lost)    0->device works fine
    uint8_t is_lost : 1;                //1->device is lost     0->device is on-line
    uint8_t data_is_error : 1;          //1->data has error     0->data is correct

    float32_t frequency;
    bool_t (*data_is_error_fun)(void);
    void (*solve_lost_fun)(void);
    void (*solve_data_error_fun)(void);
} __packed error_t;


/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
/**
  * @brief          检测任务
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
extern void detect_task(void *pvParameters);

/**
  * @brief          get toe error status
  * @param[in]      toe: table of equipment
  * @retval         true (eror) or false (no error)
  */
/**
  * @brief          获取设备对应的错误状态
  * @param[in]      toe:设备目录
  * @retval         true(错误) 或者false(没错误)
  */
extern bool_t toe_is_error(uint8_t err);

/**
  * @brief          record the time
  * @param[in]      toe: table of equipment
  * @retval         none
  */
/**
  * @brief          记录时间
  * @param[in]      toe:设备目录
  * @retval         none
  */
extern void detect_hook(uint8_t toe);

/**
  * @brief          get error list
  * @param[in]      none
  * @retval         the point of error_list
  */
/**
  * @brief          得到错误列表
  * @param[in]      none
  * @retval         error_list的指针
  */
extern const error_t *get_error_list_point(void);

#endif
