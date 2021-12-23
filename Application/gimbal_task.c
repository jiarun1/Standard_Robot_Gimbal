/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      complete gimbal control task. Because gimbal uses the angle
  *             calculated from gyroscope sensor which has a range of (-pi, pi),
  *             the set (target) angle must be in this range. This library has
  *             lots of angle calculation functions. Gimbal generally has two
  *             control mode: gyroscope mode: use eular angle to control; encoder
  *             mode: use encoder on motor to control, and it also has some
  *             special mode: calibration mode; motionless (stop) mode.
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *  V2.0.0     Jan-29-2021     YW              1. modify to fit this project
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */


#include "gimbal_task.h"

#include "main.h"

#include "cmsis_os.h"

#include "math.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot.h"
#include "pid.h"
//extern shoot_control_t shoot_control;

//motor encoder value format, range [0-8191]
#define ecd_format(ecd)                         \
        {                                           \
            if ((ecd) > GIMBAL_ECD_RANGE)           \
                (ecd) -= GIMBAL_ECD_RANGE;          \
                else if ((ecd) < 0)                     \
                (ecd) += GIMBAL_ECD_RANGE;          \
        }

//clear yaw / pitch pid values, gimbal_clear: "gimbal_control_t" type
#define gimbal_total_pid_clear(gimbal_clear)                                                        \
        {                                                                                           \
            gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid);    \
            gimbal_PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_relative_angle_pid);    \
            PID_clear(&(gimbal_clear)->gimbal_yaw_motor.gimbal_motor_gyro_pid);                     \
                                                                                                    \
            gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid);  \
            gimbal_PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_relative_angle_pid);  \
            PID_clear(&(gimbal_clear)->gimbal_pitch_motor.gimbal_motor_gyro_pid);                   \
        }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif


/**
  * @brief          initialize "gimbal_control" variable, include:
  *                 ->pid initialization
  *                 ->remote controller data pointer initialization
  *                 ->gimbal motors data pointer initialization
  *                 ->gyroscope sensor angle data pointer initialization
  * @param[out]     init: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init);


/**
  * @brief          set gimbal control mode, mainly changed in
  *                 "gimbal_behaviour_mode_set" function (in gimbal_behaviour.c/h)
  * @param[out]     gimbal_set_mode: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode);

/**
  * @brief          update gimbal measured data, such as motor encoder, euler angle, motor speed
  * @param[out]     gimbal_feedback_update: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
  * @brief          when gimbal control mode changes, some parameters should be changed,
  *                 such as yaw_set should be yaw angle now.
  * @param[out]     gimbal_mode_change: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
  * @brief          calculate the relative angle between "ecd" and "offset_ecd"
  * @param[in]      ecd: motor encoder value now
  * @param[in]      offset_ecd: gimbal middle encoder value
  * @retval         relative angle, unit: rad
  */
static float32_t motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
  * @brief          set gimbal control set-point, control set-point is set by
  *                 "gimbal_behaviour_control_set" in gimbal_behaviour.c/h.
  * @param[out]     gimbal_set_control: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control);

/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor through CAN bus.
  * @param[out]     gimbal_control_loop: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_GYRO,
  *                 use euler angle calculated by gyroscope sensor to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_ENCONDE,
  *                 use the encoder relative angle to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_RAW,
  *                 current is sent to CAN bus directly.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO,
  *                 use euler angle calculated by gyroscope sensor to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @param[in]      add: new added angle, unit: rad
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_ENCONDE,
  *                 use the encoder relative angle to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @param[in]      add: new added angle, unit: rad
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add);

/**
  * @brief          initialize "gimbal_control" variable, including:
  *                 ->pid initialization
  *                 ->remote controller data pointer initialization
  *                 ->gimbal motor data pointer initialization
  *                 ->gyroscope sensor angle pointer initialization.
  * @param[out]     gimbal_init: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, float32_t maxout, float32_t intergral_limit, float32_t kp, float32_t ki, float32_t kd);

/**
  * @brief          clear gimbal PID data, clear "pid.out" and "pid.iout".
  * @param[out]     gimbal_pid_clear: "gimbal_PID_t" variable pointer
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *pid_clear);

/**
  * @brief          gimbal angle pid calcation,
  *                 because angle is in range (-pi, pi), so can't use common way (in pid.c)
  * @param[out]     pid: gimbal pid data structure pointer
  * @param[in]      get: angle feedback value
  * @param[in]      set: angle set-point value
  * @param[in]      error_delta: rotation speed
  * @retval         pid output
  */
static float32_t gimbal_PID_calc(gimbal_PID_t *pid, float32_t get, float32_t set, float32_t error_delta);


#if GIMBAL_TEST_MODE
//j-scope help to adjust pid parameters
static void J_scope_gimbal_test(void);
#endif




//contains all gimbal control data
gimbal_control_t gimbal_control;

//motor current variables sent to CAN bus
static int16_t yaw_can_set_current = 0, pitch_can_set_current = 0;
static int16_t * shoot_can_set_current;

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
void gimbal_task(void *pvParameters)
{
    //wait a period for gyroscope task update gyroscope values
    vTaskDelay(GIMBAL_TASK_INIT_TIME);

    //gimbal initialize
    gimbal_init(&gimbal_control);

    //shoot initialize
    shoot_init();
    //keep checking whether two gimbal motors are online
 //   while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
 //   {
 //       vTaskDelay(GIMBAL_CONTROL_TIME);
 //       gimbal_feedback_update(&gimbal_control);             //during checking, still keep updating measured data
 //   }

    while (1)
    {

        gimbal_set_mode(&gimbal_control);                    //set gimbal control mode
        gimbal_mode_change_control_transit(&gimbal_control); //control data transit when gimbal mode changes
        gimbal_feedback_update(&gimbal_control);             //update gimbal feedback data
        gimbal_set_control(&gimbal_control);                 //set gimbal control value
        gimbal_control_loop(&gimbal_control);                //pid calculation for gimbal control
        //shoot_can_set_current = shoot_control_loop();        //shoot task control loop
        /*
        if(shoot_control.shoot_mode==SHOOT_BULLET){
        	*(shoot_can_set_current) = (20 * (shoot_control.set_angle - shoot_control.angle) - 80 * shoot_control.speed) * 3;
        }*/

#if YAW_TURN
        yaw_can_set_current = -gimbal_control.gimbal_yaw_motor.given_current;
#else
        yaw_can_set_current = gimbal_control.gimbal_yaw_motor.given_current;
#endif

#if PITCH_TURN
        pitch_can_set_current = -gimbal_control.gimbal_pitch_motor.given_current;
#else
        pitch_can_set_current = gimbal_control.gimbal_pitch_motor.given_current;
#endif

//       if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) && toe_is_error(TRIGGER_MOTOR_TOE)))
//        {
//            if (toe_is_error(DBUS_TOE))
//            {
//                CAN_cmd_gimbal(0, 0, 0, 0);
//            }
//            else
//            {

                CAN_cmd_gimbal(1*yaw_can_set_current, 2*pitch_can_set_current, *(shoot_can_set_current), 0);
                CAN_cmd_shoot(*(shoot_can_set_current+1),*(shoot_can_set_current+2));
                //osDelay(1);
//            }
//        }

#if GIMBAL_TEST_MODE
        J_scope_gimbal_test();
#endif

        vTaskDelay(GIMBAL_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          return yaw motor data pointer
  * @param[in]      none
  * @retval         yaw motor data pointer
  */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
  * @brief          return pitch motor data pointer
  * @param[in]      none
  * @retval         pitch motor data pointer
  */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
  * @brief          initialize "gimbal_control" variable, include:
  *                 ->pid initialization
  *                 ->remote controller data pointer initialization
  *                 ->gimbal motors data pointer initialization
  *                 ->gyroscope sensor angle data pointer initialization
  * @param[out]     init: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_init(gimbal_control_t *init)
{

    static const float32_t Pitch_speed_pid[3] = {PITCH_SPEED_PID_KP, PITCH_SPEED_PID_KI, PITCH_SPEED_PID_KD};
    static const float32_t Yaw_speed_pid[3] = {YAW_SPEED_PID_KP, YAW_SPEED_PID_KI, YAW_SPEED_PID_KD};

    //set encoder middle point value
    init->gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_MOTOR_OFFSET_ECD;
    init->gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_MOTOR_OFFSET_ECD;

    //set max/min relative value
    init->gimbal_yaw_motor.max_relative_angle = GIMBAL_YAW_MAX_RELATIVE_ANGLE;
    init->gimbal_yaw_motor.min_relative_angle = GIMBAL_YAW_MIN_RELATIVE_ANGLE;
    init->gimbal_pitch_motor.max_relative_angle = GIMBAL_PITCH_MAX_RELATIVE_ANGLE;
    init->gimbal_pitch_motor.min_relative_angle = GIMBAL_PITCH_MIN_RELATIVE_ANGLE;

    //get motor data pointer
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();

    //get gyroscope sensor data pointer
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();

    //get remote controller data pointer
    init->gimbal_rc_ctrl = get_remote_control_point();

    //initialize gimbal motor mode
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;

    //initialize yaw motor pid
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_absolute_angle_pid, YAW_GYRO_ABSOLUTE_PID_MAX_OUT, YAW_GYRO_ABSOLUTE_PID_MAX_IOUT, YAW_GYRO_ABSOLUTE_PID_KP, YAW_GYRO_ABSOLUTE_PID_KI, YAW_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_yaw_motor.gimbal_motor_relative_angle_pid, YAW_ENCODE_RELATIVE_PID_MAX_OUT, YAW_ENCODE_RELATIVE_PID_MAX_IOUT, YAW_ENCODE_RELATIVE_PID_KP, YAW_ENCODE_RELATIVE_PID_KI, YAW_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_gyro_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);
    PID_init(&init->gimbal_yaw_motor.gimbal_motor_ecd_speed_pid, PID_POSITION, Yaw_speed_pid, YAW_SPEED_PID_MAX_OUT, YAW_SPEED_PID_MAX_IOUT);

    //initialize pitch motor pid
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_absolute_angle_pid, PITCH_GYRO_ABSOLUTE_PID_MAX_OUT, PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT, PITCH_GYRO_ABSOLUTE_PID_KP, PITCH_GYRO_ABSOLUTE_PID_KI, PITCH_GYRO_ABSOLUTE_PID_KD);
    gimbal_PID_init(&init->gimbal_pitch_motor.gimbal_motor_relative_angle_pid, PITCH_ENCODE_RELATIVE_PID_MAX_OUT, PITCH_ENCODE_RELATIVE_PID_MAX_IOUT, PITCH_ENCODE_RELATIVE_PID_KP, PITCH_ENCODE_RELATIVE_PID_KI, PITCH_ENCODE_RELATIVE_PID_KD);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_gyro_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);
    PID_init(&init->gimbal_pitch_motor.gimbal_motor_ecd_speed_pid, PID_POSITION, Pitch_speed_pid, PITCH_SPEED_PID_MAX_OUT, PITCH_SPEED_PID_MAX_IOUT);

    //clear all PID values (exclude p i d parameters)
    gimbal_total_pid_clear(init);

    //update measured data
    gimbal_feedback_update(init);

    //initialize all set values as measured data now
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    init->gimbal_yaw_motor.motor_ecd_speed_set = init->gimbal_yaw_motor.motor_ecd_speed;


    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;
    init->gimbal_pitch_motor.motor_ecd_speed_set = init->gimbal_pitch_motor.motor_ecd_speed;


}

/**
  * @brief          set gimbal control mode, mainly changed in
  *                 "gimbal_behaviour_mode_set" function (in gimbal_behaviour.c/h)
  * @param[out]     gimbal_set_mode: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}

/**
  * @brief          update gimbal measured data, such as motor encoder, euler angle, motor speed
  * @param[out]     gimbal_feedback_update: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    //update gimbal data
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);

#if PITCH_TURN
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
    feedback_update->gimbal_pitch_motor.motor_ecd_speed = GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
#else

    feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                          feedback_update->gimbal_pitch_motor.offset_ecd);
    feedback_update->gimbal_pitch_motor.motor_ecd_speed = -GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * feedback_update->gimbal_pitch_motor.gimbal_motor_measure->speed_rpm;
#endif

    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);

#if YAW_TURN
    feedback_update->gimbal_yaw_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
    feedback_update->gimbal_yaw_motor.motor_ecd_speed = GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
#else
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd,
                                                                                        feedback_update->gimbal_yaw_motor.offset_ecd);
    feedback_update->gimbal_yaw_motor.motor_ecd_speed = -GIMBAL_MOTOR_RPM_TO_VECTOR_SEN * feedback_update->gimbal_yaw_motor.gimbal_motor_measure->speed_rpm;
#endif


    feedback_update->gimbal_yaw_motor.motor_gyro = cosf(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                            - sinf(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));

/*
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET))
                                                        - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
*/
}

/**
  * @brief          calculate the relative angle between "ecd" and "offset_ecd"
  * @param[in]      ecd: motor encoder value now
  * @param[in]      offset_ecd: gimbal middle encoder value
  * @retval         relative angle, unit: rad
  */
static float32_t motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;
    if (relative_ecd > GIMBAL_HALF_ECD_RANGE)
    {
        relative_ecd -= GIMBAL_ECD_RANGE;
    }
    else if (relative_ecd < -GIMBAL_HALF_ECD_RANGE)
    {
        relative_ecd += GIMBAL_ECD_RANGE;
    }

    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
  * @brief          when gimbal control mode changes, some parameters should be changed,
  *                 such as yaw_set should be yaw angle now.
  * @param[out]     gimbal_mode_change: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    //yaw motor data change when gimbal mode changes
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    //pitch motor data change when gimbal mode changes
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}

/**
  * @brief          set gimbal control set-point, control set-point is set by
  *                 "gimbal_behaviour_control_set" in gimbal_behaviour.c/h.
  * @param[out]     gimbal_set_control: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    float32_t add_yaw_angle = 0.0f;
    float32_t add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    //yaw motor controlled according to gimbal control mode
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //in raw mode, yaw angle send to CAN bus as current directly
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //in gyro mode, gimbal is controlled by gyroscope values
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //in enconde mode gimbal is controlled by encoder values
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    //pitch motor controlled according to gimbal control mode
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        //in raw mode, yaw angle send to CAN bus as current directly
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        //in gyro mode, gimbal is controlled by gyroscope values
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        //in enconde mode gimbal is controlled by encoder values
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}

/**
  * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO,
  *                 use euler angle calculated by gyroscope sensor to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @param[in]      add: new added angle, unit: rad
  * @retval         none
  */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add)
{
    static float32_t bias_angle;
    static float32_t angle_set;
    if (gimbal_motor == NULL)
    {
        return;
    }

    //current angle error
    bias_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    //if relative angle + angle error + added angle > max_relative angle
    if (gimbal_motor->relative_angle + bias_angle + add > gimbal_motor->max_relative_angle)
    {
        //if the added angle is in the direction of max mechanical angle
        if (add > 0.0f)
        {
            //re-calculate max added angle
            add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }//if relative angle + angle error + added angle < min_relative angle
    else if (gimbal_motor->relative_angle + bias_angle + add < gimbal_motor->min_relative_angle)
    {
        //if the added angle is in the direction of min mechanical angle
        if (add < 0.0f)
        {
            //re-calculate max added angle
            add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - bias_angle;
        }
    }
    angle_set = gimbal_motor->absolute_angle_set;
    gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
}

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_ENCONDE,
  *                 use the encoder relative angle to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @param[in]      add: new added angle, unit: rad
  * @retval         none
  */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, float32_t add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    gimbal_motor->relative_angle_set += add;

    //if the new set relative angle > max relative angle
    if (gimbal_motor->relative_angle_set > gimbal_motor->max_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    //if the new set relative angle < min relative angle
    else if (gimbal_motor->relative_angle_set < gimbal_motor->min_relative_angle)
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}


/**
  * @brief          control loop, according to control set-point, calculate motor current, 
  *                 motor current will be sent to motor through CAN bus.
  * @param[out]     gimbal_control_loop: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }
    
    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
}

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_GYRO,
  *                 use euler angle calculated by gyroscope sensor to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @retval         none
  */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //angle and rotation speed series pid control
    gimbal_motor->motor_gyro_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_absolute_angle_pid, gimbal_motor->absolute_angle, gimbal_motor->absolute_angle_set, gimbal_motor->motor_gyro);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_gyro_pid, gimbal_motor->motor_gyro, gimbal_motor->motor_gyro_set);

    //assign control value
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_ENCONDE,
  *                 use the encoder relative angle to control.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @retval         none
  */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    //angle and rotation speed series pid control
    gimbal_motor->motor_ecd_speed_set = gimbal_PID_calc(&gimbal_motor->gimbal_motor_relative_angle_pid, gimbal_motor->relative_angle, gimbal_motor->relative_angle_set, gimbal_motor->motor_ecd_speed);
    gimbal_motor->current_set = PID_calc(&gimbal_motor->gimbal_motor_ecd_speed_pid, gimbal_motor->motor_ecd_speed, gimbal_motor->motor_ecd_speed_set);

    //assign control value
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

/**
  * @brief          gimbal control mode: GIMBAL_MOTOR_RAW,
  *                 current is sent to CAN bus directly.
  * @param[out]     gimbal_motor: yaw or pitch motor pointer
  * @retval         none
  */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif

/**
  * @brief          initialize "gimbal_control" variable, including:
  *                 ->pid initialization
  *                 ->remote controller data pointer initialization
  *                 ->gimbal motor data pointer initialization
  *                 ->gyroscope sensor angle pointer initialization.
  * @param[out]     gimbal_init: "gimbal_control" variable pointer
  * @retval         none
  */
static void gimbal_PID_init(gimbal_PID_t *pid, float32_t maxout, float32_t max_iout, float32_t kp, float32_t ki, float32_t kd)
{
    if (pid == NULL)
    {
        return;
    }
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err = 0.0f;
    pid->get = 0.0f;

    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

/**
  * @brief          gimbal angle pid calcation,
  *                 because angle is in range (-pi, pi), so can't use common way (in pid.c)
  * @param[out]     pid: gimbal pid data structure pointer
  * @param[in]      get: angle feedback value
  * @param[in]      set: angle set-point value
  * @param[in]      error_delta: rotation speed
  * @retval         pid output
  */
static float32_t gimbal_PID_calc(gimbal_PID_t *pid, float32_t get, float32_t set, float32_t error_delta)
{
    float32_t err;
    if (pid == NULL)
    {
        return 0.0f;
    }
    pid->get = get;
    pid->set = set;

    err = set - get;
    pid->err = rad_format(err);
    pid->Pout = pid->kp * pid->err;
    pid->Iout += pid->ki * pid->err;
    pid->Dout = pid->kd * error_delta;
    abs_limit(&pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    abs_limit(&pid->out, pid->max_out);
    return pid->out;
}

/**
  * @brief          clear gimbal PID data, clear "pid.out" and "pid.iout".
  * @param[out]     gimbal_pid_clear: "gimbal_PID_t" variable pointer
  * @retval         none
  */
static void gimbal_PID_clear(gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err = gimbal_pid_clear->set = gimbal_pid_clear->get = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}
