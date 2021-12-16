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


#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "math.h"
//#include "bsp_buzzer.h"
#include "detect_task.h"
#include "protocol_def.h"
#include "user_lib.h"
//#include "usb_task.h"

//when gimbal is calibrating, set buzzer frequency and strength
//#define gimbal_warn_buzzer_on() buzzer_on(523.251f, 1.0f)
//#define gimbal_warn_buzzer_off() buzzer_off()

#define int_abs(x) ((x) > 0 ? (x) : (-x))

float overtime= 500;
float attenuation= 0.7;
float scale=100;
float AI_t0=1000;
int AI_t;
float32_t watch[2];

//AI_X &AI_Y is recieved data from nuc, used in "usb_task.c"
//extern
float32_t AI_X;
//extern
float32_t AI_Y;

extern struct VisionCommandFrame frame;

/**
  * @brief          remote control deal zone detect,
  *                 because the value of joy stick is not zero in middle place
  * @param          input:the raw channel value 
  * @param          output: the processed channel value
  * @param          dealine: dead zone limit
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }

/**
  * @brief          set gimbal behavior mode ("gimbal_behaviour" variable).
  * @param[in]      gimbal_mode_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);

  /**
    * @brief          when gimbal behavior mode is GIMBAL_ZERO_FORCE, this function is called
    *                 and gimbal control mode is raw (GIMBAL_MOTOR_RAW). The raw mode means set value
    *                 will be sent to CAN bus directly, and the function will set all values as zero.
    * @param[out]     yaw: yaw motor set current, it will be sent to CAN bus directly.
    * @param[out]     pitch: pitch motor set current, it will be sent to CAN bus directly.
    * @param[in]      gimbal_control_set: gimbal data pointer
    * @retval         none
    */
static void gimbal_zero_force_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behavior mode is GIMBAL_INIT, this function is called
  *                 and gimbal control mode is gyro mode (GIMBAL_MOTOR_GYRO). Gimbal
  *                 will firstly lift the pitch axis and secondly rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit: rad
  * @param[out]     pitch: pitch motor absolute angle increment, unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_init_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behavior mode is GIMBAL_ABSOLUTE_ANGLE, this function is called
  *                 and gimbal control mode is gyro mode (GIMBAL_MOTOR_GYRO).
  * @param[out]     yaw: yaw axis absolute angle increment, unit: rad
  * @param[out]     pitch: pitch axis absolute angle increment,unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_absolute_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behavior mode is GIMBAL_RELATIVE_ANGLE, this function is called
  *                 and gimbal control mode is encoder mode (GIMBAL_MOTOR_ENCONDE).
  * @param[out]     yaw: yaw axis relative angle increment, unit: rad
  * @param[out]     pitch: pitch axis relative angle increment,unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_relative_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          when gimbal behavior mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encoder mode (GIMBAL_MOTOR_ENCONDE).
  * @param[out]     yaw: yaw axis relative angle increment, unit: rad
  * @param[out]     pitch: pitch axis relative angle increment, unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_motionless_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);

//112233
static void gimbal_AI_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set);

static void AI_robotic_calculation(float32_t *p);

//gimbal behavior state
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;

/**
  * @brief          this function is called by "gimbal_set_mode" function in "gimbal_task.c"
  *                 this function set "gimbal_behaviour" variable, and set motor mode.
  * @param[in]      gimbal_mode_set: gimbal data pointer
  * @retval         none
  */
void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    //set gimbal_behaviour mode variable
    gimbal_behavour_set(gimbal_mode_set);

    //according to "gimbal_behaviour", set motor control mode
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
    else if (gimbal_behaviour == GIMBAL_AI)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
                }

}

/**
  * @brief          the function is called by gimbal_set_contorl function in gimbal_task.c
  *                 according to the "gimbal_behaviour" variable, call the corresponding function
  * @param[out]     add_yaw: yaw axis set increment angle, unit: rad
  * @param[out]     add_pitch: pitch axis set increment angle, unit: rad
  * @param[in]      gimbal_mode_set: gimbal data pointer
  * @retval         none
  */
void gimbal_behaviour_control_set(float32_t *add_yaw, float32_t *add_pitch, gimbal_control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_absolute_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_AI)
    {
        gimbal_AI_control(add_yaw, add_pitch, gimbal_control_set);
    }

}

/**
  * @brief          in some special gimbal behavior mode, chassis needs keeping no move
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
uint8_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          in some special gimbal behavior mode, shoot needs to be stopped
  * @param[in]      none
  * @retval         1: do not shoot 0:normal
  */
uint8_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
  * @brief          set gimbal behavior mode ("gimbal_behaviour" variable).
  * @param[in]      gimbal_mode_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    //in initialize mode, judge if gimbal is in middle place
    if (gimbal_behaviour == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
        init_time++;
        
        if ((fabs(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR &&
             fabs(gimbal_mode_set->gimbal_pitch_motor.relative_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR))
        {
            
            if (init_stop_time < GIMBAL_INIT_STOP_TIME)
            {
                init_stop_time++;
            }
        }
        else
        {
            
            if (init_time < GIMBAL_INIT_TIME)
            {
                init_time++;
            }
        }

        //exceed max initialize time / gimbal has kept in middle place for a while
        //state switch is not in up position (GIMBAL_PITCH_MIN_RELATIVE_ANGLE) during initializing / remote controller offline
        //these four conditions can cause initialization exits.
        if (init_time < GIMBAL_INIT_TIME && init_stop_time < GIMBAL_INIT_STOP_TIME &&
            !switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && !toe_is_error(DBUS_TOE))
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
        }
    }

    //remote controller switch controls gimbal behavior mode
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        //gimbal_behaviour = GIMBAL_ZERO_FORCE;
    	//we label "ZERO_FORCE",replaced by our AI
    	gimbal_behaviour = GIMBAL_AI;
    }


    if( toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }


    //check whether go into initialize mode
    //if the last behavior is GIMBAL_ZERO_FORCE, when entering any other mode, gimbal will go into initialize mode
    {
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
        }
        last_gimbal_behaviour = gimbal_behaviour;
    }



}

/**
  * @brief          when gimbal behavior mode is GIMBAL_ZERO_FORCE, this function is called
  *                 and gimbal control mode is raw (GIMBAL_MOTOR_RAW). The raw mode means set value
  *                 will be sent to CAN bus directly, and the function will set all values as zero.
  * @param[out]     yaw: yaw motor set current, it will be sent to CAN bus directly.k
  * @param[out]     pitch: pitch motor set current, it will be sent to CAN bus directly.
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_zero_force_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          when gimbal behavior mode is GIMBAL_INIT, this function is called
  *                 and gimbal control mode is gyro mode (GIMBAL_MOTOR_GYRO). Gimbal
  *                 will firstly lift the pitch axis and secondly rotate yaw axis.
  * @param[out]     yaw: yaw motor relative angle increment, unit: rad
  * @param[out]     pitch: pitch motor absolute angle increment, unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_init_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          when gimbal behavior mode is GIMBAL_ABSOLUTE_ANGLE, this function is called
  *                 and gimbal control mode is gyro mode (GIMBAL_MOTOR_GYRO).
  * @param[out]     yaw: yaw axis absolute angle increment, unit: rad
  * @param[out]     pitch: pitch axis absolute angle increment,unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_absolute_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static int16_t yaw_channel = 0, pitch_channel = 0;

    //dead zone limit
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    //combine joy stick value and keyboard value
    *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;

    //Handle turn round
    {
        static uint16_t last_turn_keyboard = 0;
        static uint8_t gimbal_turn_flag = 0;    //0->not doing turn round   1->doing turn round right now
        static float32_t gimbal_end_angle = 0.0f;

        if ((gimbal_control_set->gimbal_rc_ctrl->key.v & TURN_KEYBOARD) && !(last_turn_keyboard & TURN_KEYBOARD))
        {
            if (gimbal_turn_flag == 0)
            {
                gimbal_turn_flag = 1;
                //save turn round target value (angle)
                gimbal_end_angle = rad_format(gimbal_control_set->gimbal_yaw_motor.absolute_angle + PI);
            }
        }
        last_turn_keyboard = gimbal_control_set->gimbal_rc_ctrl->key.v ;

        if (gimbal_turn_flag)
        {
            //keep controlling to reach the target value (angle), the spinning direction is random
            if (rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle) > 0.0f)
            {
                *yaw += TURN_SPEED;
            }
            else
            {
                *yaw -= TURN_SPEED;
            }
        }
        //when it reaches target value (angle), stop turning round
        if (gimbal_turn_flag && fabs(rad_format(gimbal_end_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle)) < 0.01f)
        {
            gimbal_turn_flag = 0;
        }
    }
}


/**
  * @brief          when gimbal behavior mode is GIMBAL_RELATIVE_ANGLE, this function is called
  *                 and gimbal control mode is encoder mode (GIMBAL_MOTOR_ENCONDE).
  * @param[out]     yaw: yaw axis relative angle increment, unit: rad
  * @param[out]     pitch: pitch axis relative angle increment,unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_relative_angle_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //dead zone limit
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, RC_DEADBAND);

    if(gimbal_control_set->gimbal_rc_ctrl->key.v&KEY_PRESSED_OFFSET_SHIFT)
    {*yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN ;}
    else
    {
    	*yaw = yaw_channel * YAW_RC_SEN;
    }

    *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN ;

}

/**
  * @brief          when gimbal behavior mode is GIMBAL_MOTIONLESS, the function is called
  *                 and gimbal control mode is encoder mode (GIMBAL_MOTOR_ENCONDE).
  * @param[out]     yaw: yaw axis relative angle increment, unit: rad
  * @param[out]     pitch: pitch axis relative angle increment, unit: rad
  * @param[in]      gimbal_control_set: gimbal data pointer
  * @retval         none
  */
static void gimbal_motionless_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

static void gimbal_AI_control(float32_t *yaw, float32_t *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //dead zone limit


    rc_deadband_limit( frame.data.demoCmd.dx, yaw_channel, 1);
    rc_deadband_limit( frame.data.demoCmd.dy, pitch_channel, 1);
    // AI_robotic_calculation(&	 frame.data.demoCmd.dy);
    //AI_robotic_calculation(&frame.data.demoCmd.dx);

    watch[0]= AI_X;
    watch[0]= AI_X;

   // *yaw   =  -frame.data.demoCmd.dx * YAW_AI_SEN;
   // *pitch =  frame.data.demoCmd.dy * PITCH_AI_SEN;
     *yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * 0.000002 - frame.data.demoCmd.dx * YAW_AI_SEN; //yaw_moues_sen is 0.00005f, here we use 0.000002 to largely reduce the influence of mouse.x
     *pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN + frame.data.demoCmd.dy * PITCH_AI_SEN;
}


static void AI_robotic_calculation(float32_t *p)
{
	float step1=attenuation *(AI_t-AI_t0);
	float step2_1=pow(2,step1);
	float step2_2=pow(2,-step1);

	*p = ((1 - (step2_1 - step2_2) / (step2_1 + step2_2))) * scale* (* p) * 0.5;
	AI_t++;

}
