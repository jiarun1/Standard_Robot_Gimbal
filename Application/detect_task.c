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
  
#include "detect_task.h"
#include "cmsis_os.h"


/**
  * @brief          initialize error_list, assign offline_time, online_time, priority.
  * @param[in]      time: system time
  * @retval         none
  */
static void detect_init(uint32_t time);


error_t error_list[ERROR_LIST_LENGHT + 1];

//时间探头 debug用
uint32_t systim;

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t detect_task_stack;
#endif


/**
  * @brief          detect task
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void detect_task(void *pvParameters)
{
    static uint32_t system_time;
    system_time = xTaskGetTickCount();

    //initialize
    detect_init(system_time);
    //wait for a period
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        static uint8_t error_num_display = 0;
        system_time = xTaskGetTickCount();
        systim = system_time;

        error_num_display = ERROR_LIST_LENGHT;
        error_list[ERROR_LIST_LENGHT].is_lost = 0;
        error_list[ERROR_LIST_LENGHT].error_exist = 0;

        for (int i = 0; i < ERROR_LIST_LENGHT; i++)
        {
            //if device is disabled, continue
            if (error_list[i].enable == 0)
            {
                continue;
            }

            //check if device is off-line
            if (system_time - error_list[i].new_time > error_list[i].set_offline_time)
            {
                if (error_list[i].error_exist == 0)
                {
                    //record error and device lost timestamp
                    error_list[i].is_lost = 1;
                    error_list[i].error_exist = 1;
                    error_list[i].lost_time = system_time;
                }
                //check the priority, save the highest priority code
                if (error_list[i].priority > error_list[error_num_display].priority)
                {
                    error_num_display = i;
                }
                

                error_list[ERROR_LIST_LENGHT].is_lost = 1;
                error_list[ERROR_LIST_LENGHT].error_exist = 1;
                //if solving function is provided
                //(i.e. solve_lost_fun != NULL)
                //run this function
                if (error_list[i].solve_lost_fun != NULL)
                {
                    error_list[i].solve_lost_fun();
                }
            }
            else if (system_time - error_list[i].work_time < error_list[i].set_online_time)
            {
                //device just got online, maybe it is unstable
                //only record there is error, not record it is lost
                error_list[i].is_lost = 0;
                error_list[i].error_exist = 1;
            }
            else
            {
                error_list[i].is_lost = 0;
                //check if data error exists
                if (error_list[i].data_is_error != 0)
                {
                    error_list[i].error_exist = 1;
                }
                else
                {
                    error_list[i].error_exist = 0;
                }

                //calculate frequency
                if (error_list[i].new_time > error_list[i].last_time)
                {
                    error_list[i].frequency = configTICK_RATE_HZ / (float32_t)(error_list[i].new_time - error_list[i].last_time);
                }
            }
        }

        vTaskDelay(DETECT_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        detect_task_stack = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}


/**
  * @brief          get device error status
  * @param[in]      toe: index of device
  * @retval         true (error) or false (no error)
  */
bool_t toe_is_error(uint8_t toe)
{
    return (error_list[toe].error_exist == 1);
}


/**
  * @brief          record the timestamp
  * @param[in]      toe: index of device
  * @retval         none
  */
void detect_hook(uint8_t toe)
{
    error_list[toe].last_time = error_list[toe].new_time;
    error_list[toe].new_time = xTaskGetTickCount();
    
    if (error_list[toe].is_lost)
    {
        error_list[toe].is_lost = 0;
        error_list[toe].work_time = error_list[toe].new_time;
    }
    
    if (error_list[toe].data_is_error_fun != NULL)
    {
        if (error_list[toe].data_is_error_fun())
        {
            error_list[toe].error_exist = 1;
            error_list[toe].data_is_error = 1;

            if (error_list[toe].solve_data_error_fun != NULL)
            {
                error_list[toe].solve_data_error_fun();
            }
        }
        else
        {
            error_list[toe].data_is_error = 0;
        }
    }
    else
    {
        error_list[toe].data_is_error = 0;
    }
}

/**
  * @brief          get error list
  * @param[in]      none
  * @retval         the pointer of error_list
  */
const error_t *get_error_list_point(void)
{
    return error_list;
}

/**
  * @brief          initialize error_list, assign offline_time, online_time, priority.
  * @param[in]      time: system time
  * @retval         none
  */
static void detect_init(uint32_t time)
{
    //{set_offline_time, set_online_time, priority}
    uint16_t set_item[ERROR_LIST_LENGHT][3] =
    {
        {30, 40, 13},   //SBUS
        {2,  3,  12},   //yaw
        {2,  3,  11},   //pitch
        {10, 10, 10},   //trigger
		{5,  5,  9 },   //friction motor 1
		{5,  5,  9 },   //friction motor 2
        {5,  5,  7 },   //board mpu6500
        {40, 200, 7},   //board ist8310
        //{100, 100, 5},  //referee
    };

    for (uint8_t i = 0; i < ERROR_LIST_LENGHT; i++)
    {
        error_list[i].set_offline_time = set_item[i][0];
        error_list[i].set_online_time = set_item[i][1];
        error_list[i].priority = set_item[i][2];
        error_list[i].data_is_error_fun = NULL;
        error_list[i].solve_lost_fun = NULL;
        error_list[i].solve_data_error_fun = NULL;

        error_list[i].enable = 1;
        error_list[i].error_exist = 1;
        error_list[i].is_lost = 1;
        error_list[i].data_is_error = 1;
        error_list[i].frequency = 0.0f;
        error_list[i].new_time = time;
        error_list[i].last_time = time;
        error_list[i].lost_time = time;
        error_list[i].work_time = time;
    }

//    error_list[DBUSTOE].dataIsErrorFun = RC_data_is_error;
//    error_list[DBUSTOE].solveLostFun = slove_RC_lost;
//    error_list[DBUSTOE].solveDataErrorFun = slove_data_error;

}
