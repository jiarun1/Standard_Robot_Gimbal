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

#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#include "CAN_receive.h"
#include "pid.h"
#include "remote_control.h"

//pitch speed close-loop PID parameters, max output and max integration output
#define PITCH_SPEED_PID_KP        200.0f//45.0f
#define PITCH_SPEED_PID_KI        10.0//10.0f//10.0f
#define PITCH_SPEED_PID_KD        15.0f//15.0f
#define PITCH_SPEED_PID_MAX_OUT   14000.0f      //5000.0f
#define PITCH_SPEED_PID_MAX_IOUT  20.0f      //20.0f

//yaw speed close-loop PID parameters, max output and max integration output
#define YAW_SPEED_PID_KP        800.0f
#define YAW_SPEED_PID_KI        10.0f
#define YAW_SPEED_PID_KD        40.0f
#define YAW_SPEED_PID_MAX_OUT   28000.0f
#define YAW_SPEED_PID_MAX_IOUT  20.0f

//pitch gyroscope angle close-loop PID parameters, max output and max integration output
#define PITCH_GYRO_ABSOLUTE_PID_KP 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KI 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_KD 0.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_OUT 10.0f
#define PITCH_GYRO_ABSOLUTE_PID_MAX_IOUT 0.0f

//yaw gyroscope angle close-loop PID parameters, max output and max integration output
#define YAW_GYRO_ABSOLUTE_PID_KP        0.0f//26.0f
#define YAW_GYRO_ABSOLUTE_PID_KI        0.0f
#define YAW_GYRO_ABSOLUTE_PID_KD        0.3f
#define YAW_GYRO_ABSOLUTE_PID_MAX_OUT   10.0f
#define YAW_GYRO_ABSOLUTE_PID_MAX_IOUT  0.0f

//pitch encoder angle close-loop PID parameters, max output and max integration output
#define PITCH_ENCODE_RELATIVE_PID_KP 455.0f//200.0f
#define PITCH_ENCODE_RELATIVE_PID_KI 4.0f
#define PITCH_ENCODE_RELATIVE_PID_KD 6.0f//5.5f
#define PITCH_ENCODE_RELATIVE_PID_MAX_OUT 200.0f
#define PITCH_ENCODE_RELATIVE_PID_MAX_IOUT 10.0f

//yaw encoder angle close-loop PID parameters, max output and max integration output
#define YAW_ENCODE_RELATIVE_PID_KP        400.0f//100.0f//50.0f
#define YAW_ENCODE_RELATIVE_PID_KI        0.2f
#define YAW_ENCODE_RELATIVE_PID_KD        4.0f//6.5f
#define YAW_ENCODE_RELATIVE_PID_MAX_OUT   300.0f
#define YAW_ENCODE_RELATIVE_PID_MAX_IOUT  5.0f



//the free period after gimbal task starts (initialization)
#define GIMBAL_TASK_INIT_TIME 201

//yaw, pitch, gimbal behavior switch channel index on remote controller
#define YAW_CHANNEL   2
#define PITCH_CHANNEL 3
#define GIMBAL_MODE_CHANNEL 0

//turn 180° (turn around) key on keyboard
#define TURN_KEYBOARD KEY_PRESSED_OFFSET_F
//turn 180° (turn around) speed
#define TURN_SPEED    0.04f

//(test key on keyboard is useless now)
#define TEST_KEYBOARD KEY_PRESSED_OFFSET_R

//joy stick input dead zone, because the value is not 0 when it is in middle
#define RC_DEADBAND   10

//ratio that remote controller value changed into angle increment
#define YAW_RC_SEN    -0.000006f
#define PITCH_RC_SEN  -0.000009f //0.005

//ratio that X,Y data value from NUC changed into angle increment,RC(-660~+660), AI(-1~1)
#define YAW_AI_SEN    0.001f
#define PITCH_AI_SEN  0.001f

//ratio that mouse value changed into angle increment
#define YAW_MOUSE_SEN   0.00005f //0.00005f
#define PITCH_MOUSE_SEN 0.00015f

//(following two parameters are useless now)
#define YAW_ENCODE_SEN    0.01f
#define PITCH_ENCODE_SEN  0.01f

//time period (delay) between each check which checks if gimbal motors are online
//used in "gimbal_task" initializing steps
#define GIMBAL_CONTROL_TIME 1

//gimbal test mode, 0 -> close this mode, 1 -> open this mode
#define GIMBAL_TEST_MODE 0

//adjust these two parameters to inverse gimbal rotation direction
//and make sure the rotation direction is correct
#define PITCH_TURN  0
#define YAW_TURN    0

//half max value of gimbal motor encoder
#define GIMBAL_HALF_ECD_RANGE  4096
//max value of gimbal motor encoder
#define GIMBAL_ECD_RANGE       8192   //YW: it's 8191 originally, but I think it should be 8192
//middle point of gimbal yaw motor encoder
#define GIMBAL_YAW_MOTOR_OFFSET_ECD     7010
//middle point of gimbal pitch motor encoder
#define GIMBAL_PITCH_MOTOR_OFFSET_ECD   5300

//yaw maximum relative angle in unit: rad
//#define GIMBAL_YAW_MAX_RELATIVE_ANGLE       1.5707963267948966192313216916398f
#define GIMBAL_YAW_MAX_RELATIVE_ANGLE       10000.0f
//yaw minimum relative angle in unit: rad
//#define GIMBAL_YAW_MIN_RELATIVE_ANGLE       -1.5707963267948966192313216916398f
#define GIMBAL_YAW_MIN_RELATIVE_ANGLE       -10000.0f
//pitch maximum relative angle in unit: rad
//#define GIMBAL_PITCH_MAX_RELATIVE_ANGLE     1.5707963267948966192313216916398f
#define GIMBAL_PITCH_MAX_RELATIVE_ANGLE     -0.75f
//pitch minimum relative angle in unit: rad
//#define GIMBAL_PITCH_MIN_RELATIVE_ANGLE     -1.5707963267948966192313216916398f
#define GIMBAL_PITCH_MIN_RELATIVE_ANGLE     -1.6f

//gimbal turn back to middle place when initializing
//following parameters are all about initialization
//allowed error range
#define GIMBAL_INIT_ANGLE_ERROR     0.1f
//gimbal stops for this period when it is within error range
#define GIMBAL_INIT_STOP_TIME       100
//max initialization time. After this period, initialization stops
#define GIMBAL_INIT_TIME            6000
//the speed when it come back to middle place
#define GIMBAL_INIT_PITCH_SPEED     0.01f
#define GIMBAL_INIT_YAW_SPEED       0.01f
//the set angle that gimbal comes to (middle place angle?)
#define INIT_YAW_SET    0.0f
#define INIT_PITCH_SET  0.0f

/**
  * used to check if the remote controller's joy stick is in 0 value (no input)
  * and detect how long this condition has been lasting
  * when this condition lasts GIMBAL_MOTIONLESS_TIME_MAX this period
  * gimbal yaw motor will come to middle place to avoid gyroscope drifting
  *
  * (It seems that these parameters are useless because they cannot be found in anywhere)
  */
#define GIMBAL_MOTIONLESS_RC_DEADLINE 10
#define GIMBAL_MOTIONLESS_TIME_MAX    3000

//the ratio that changes motor encoder value into angle value
#ifndef MOTOR_ECD_TO_RAD
#define MOTOR_ECD_TO_RAD 0.000766990394f //      2*  PI  /8192
#endif

//ratio of GM6020 speed (rpm) changing to chassis speed (m/s),
#define GM6020_MOTOR_RPM_TO_VECTOR 0.10471975511965977461542144610932f      //2*pi/60
#define GIMBAL_MOTOR_RPM_TO_VECTOR_SEN GM6020_MOTOR_RPM_TO_VECTOR


typedef enum
{
    GIMBAL_MOTOR_RAW = 0, //gimbal controlled by raw values
    GIMBAL_MOTOR_GYRO,    //gimbal controlled by gyroscope values
    GIMBAL_MOTOR_ENCONDE, //gimbal controlled by motor encoder values
} gimbal_motor_mode_e;

typedef struct
{
    float32_t kp;
    float32_t ki;
    float32_t kd;

    float32_t set;
    float32_t get;
    float32_t err;

    float32_t max_out;
    float32_t max_iout;

    float32_t Pout;
    float32_t Iout;
    float32_t Dout;

    float32_t out;
} gimbal_PID_t;

typedef struct
{
    const motor_measure_t *gimbal_motor_measure;
    gimbal_PID_t gimbal_motor_absolute_angle_pid;
    gimbal_PID_t gimbal_motor_relative_angle_pid;
    pid_type_def gimbal_motor_gyro_pid;
    pid_type_def gimbal_motor_ecd_speed_pid;
    gimbal_motor_mode_e gimbal_motor_mode;
    gimbal_motor_mode_e last_gimbal_motor_mode;
    uint16_t offset_ecd;            //motor middle place encoder value
    float32_t max_relative_angle;        //unit: rad
    float32_t min_relative_angle;        //unit: rad
    float32_t relative_angle;            //unit: rad
    float32_t relative_angle_set;        //unit: rad
    float32_t absolute_angle;            //unit: rad
    float32_t absolute_angle_set;        //unit: rad
    float32_t motor_gyro;                //unit: rad/s (maybe rotation speed?)
    float32_t motor_gyro_set;
    float32_t motor_ecd_speed;
    float32_t motor_ecd_speed_set;
    float32_t motor_speed;
    float32_t raw_cmd_current;
    float32_t current_set;           //this is a temporary given current which will be assigned to "given_current" finally
    int16_t given_current;      //this is the actual given current sent to CAN bus.

} gimbal_motor_t;

typedef struct
{
    float32_t max_yaw;
    float32_t min_yaw;
    float32_t max_pitch;
    float32_t min_pitch;
    uint16_t max_yaw_ecd;
    uint16_t min_yaw_ecd;
    uint16_t max_pitch_ecd;
    uint16_t min_pitch_ecd;
    uint8_t step;
} gimbal_step_cali_t;

typedef struct
{
    const RC_ctrl_t *gimbal_rc_ctrl;
    const float32_t *gimbal_INT_angle_point;
    const float32_t *gimbal_INT_gyro_point;
    gimbal_motor_t gimbal_yaw_motor;
    gimbal_motor_t gimbal_pitch_motor;
    gimbal_step_cali_t gimbal_cali;
} gimbal_control_t;

/**
  * @brief          return yaw motor data pointer
  * @param[in]      none
  * @retval         yaw motor data pointer
  */
extern const gimbal_motor_t *get_yaw_motor_point(void);

/**
  * @brief          return pitch motor data pointer
  * @param[in]      none
  * @retval         pitch motor data pointer
  */
extern const gimbal_motor_t *get_pitch_motor_point(void);

/**
  * @brief          gimbal task, osDelay GIMBAL_CONTROL_TIME (1ms) 
  * @param[in]      pvParameters: null
  * @retval         none
  */
extern void gimbal_task(void *pvParameters);

#endif


