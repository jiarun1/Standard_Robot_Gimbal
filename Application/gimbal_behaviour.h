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
  *  V2.0.0     Jan-28-2021     YW              1. modify to fit the project
  *
  @verbatim
  ==============================================================================
    How to add a new gimbal behavior mode
    1. Firstly, in "gimbal_behaviour.h", add a new behavior name in "gimbal_behaviour_e"

            typedef enum
            {
                ...
                ...
                GIMBAL_XXX_XXX,     // the new one added
            } gimbal_behaviour_e;

    2. Give a new function:

            gimbal_xxx_xxx_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);

       "yaw, pitch" parameters are gimbal movement control input:

            'yaw':   controls yaw axis move, usually means angle increment.
                     positive -> counter-clockwise, negative -> clockwise.
            'pitch': controls pitch axis move, usually means angle increment.
                     positive -> counter-clockwise, negative -> clockwise.

       In this new function, you can assign set-point to "yaw" and "pitch" as your wish.

    3. In "gimbal_behavour_set" function, add new logical judgment to assign "GIMBAL_XXX_XXX" to "gimbal_behaviour" variable.
       And at the end of "gimbal_behaviour_mode_set" function, add:

            else if (gimbal_behaviour == GIMBAL_XXX_XXX)
            {
                gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_XXX;
                gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_XXX;
            }

       , where GIMBAL_MOTOR_XXX is one of following gimbal control mode:

            GIMBAL_MOTOR_RAW : will use 'yaw' and 'pitch' as motor set current, directly sent to can bus.
            GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' are angle increment, control encoder relative angle.
            GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' are angle increment, control gyroscope absolute angle.

    4. At the end of "gimbal_behaviour_control_set" function, add:

            else if (gimbal_behaviour == GIMBAL_XXX_XXX)
            {
                gimbal_xxx_xxx_control(add_yaw, add_pitch, gimbal_control_set);
            }

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */
#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "main.h"

#include "gimbal_task.h"
typedef enum
{
  GIMBAL_ZERO_FORCE=0,
  GIMBAL_INIT,
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,
  GIMBAL_AI,     // aim
} gimbal_behaviour_e;

/**
  * @brief          this function is called by "gimbal_set_mode" function in "gimbal_task.c"
  *                 this function set "gimbal_behaviour" variable, and set motor mode.
  * @param[in]      gimbal_mode_set: gimbal data pointer
  * @retval         none
  */
extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

/**
  * @brief          the function is called by gimbal_set_contorl function in gimbal_task.c
  *                 according to the "gimbal_behaviour" variable, call the corresponding function
  * @param[out]     add_yaw: yaw axis set increment angle, unit: rad
  * @param[out]     add_pitch: pitch axis set increment angle, unit: rad
  * @param[in]      gimbal_mode_set: gimbal data pointer
  * @retval         none
  */
extern void gimbal_behaviour_control_set(float32_t *add_yaw, float32_t *add_pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          in some special gimbal behavior mode, chassis needs keeping no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
extern uint8_t gimbal_cmd_to_chassis_stop(void);

/**
  * @brief          in some special gimbal behavior mode, shoot needs to be stopped
  * @param[in]      none
  * @retval         1: do not shoot 0:normal
  */
extern uint8_t gimbal_cmd_to_shoot_stop(void);

#endif
