/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      shoot function
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V2.0.0     Feb-05-2021     YW              1. modify to fit the project
  *  V2.1.0     Oct-26-2021     YZ              1. change PWM to CAN control
  *                                             2. change structure of the status machine
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "CAN_receive.h"
#include "remote_control.h"
#include "user_lib.h"

//remote controller shoot-control switch channel index
#define SHOOT_RC_MODE_CHANNEL       1

//the key that turn on/off friction motor and laser
//(go into SHOOT_READY_FRIC/SHOOT_STOP when switch on remote controller switch is in middle position)
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//after one shoot complete, bullet shot out, check the duration period to avoid macro switch triggered incorrectly
#define SHOOT_DONE_KEY_OFF_TIME     15
//mouse button long press threshold
#define PRESS_LONG_TIME             400
//remote controller switch at down position for a while threshold (go into SHOOT_CONTINUE_BULLET, this mode is used to clear bullet)
#define RC_S_LONG_TIME              3500
//friction motor fast speed duration (after this duration, back to slow speed)
#define UP_ADD_TIME                 80

//push bullet motor feedback encoder value range
#define SHOOT_PUSH_HALF_ECD_RANGE              4096
//push bullet motor feedback encoder value range
#define SHOOT_PUSH_ECD_RANGE                   8192    //YW: it's 8191 originally, but I think it should be 8192

//ratio that change rpm into speed
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
//used to change lap of rotor into output axle angle (details check .c file)
#define FULL_COUNT                  18

//push bullet motor speed in different mode
#define TRIGGER_SPEED               9.0f
#define CONTINUE_TRIGGER_SPEED      13.0f
#define READY_TRIGGER_SPEED         5.0f

//bullet stuck judge speed
#define BLOCK_TRIGGER_SPEED         1.0f

//bullet stuck time threshold
#define BLOCK_TIME                  700

//motor reverse rotate duration each time when bullet stuck
#define REVERSE_TIME                500
#define REVERSE_SPEED_LIMIT         13.0f       //useless

//trigger motor shoot bullet pid
#define TRIGGER_ANGLE_PID_KP           500.0f
#define TRIGGER_ANGLE_PID_KI           0.6f
#define TRIGGER_ANGLE_PID_KD           0.2f

//trigger motor stop pid
#define TRIGGER_STOP_PID_KP            300.0f
#define TRIGGER_STOP_PID_KI            0.5f
#define TRIGGER_STOP_PID_KD            35.0f

//friction motor1 pid
#define FRICTION_MOTOR1_PID_KP         50.0f
#define FRICTION_MOTOR1_PID_KI         1.6f
#define FRICTION_MOTOR1_PID_KD         40.0f

//friction motor2 pid
#define FRICTION_MOTOR2_PID_KP         50.0f
#define FRICTION_MOTOR2_PID_KI         1.6f
#define FRICTION_MOTOR2_PID_KD         40.0f

//friction motor speed (fast/slow)
#define FRIC_FAST                      6500.0f
#define FRIC_SLOW                      6000.0f

//friction motor max pid output and intergrate output
#define FRICTION_PID_MAX_OUT           6000.0f
#define FRICTION_PID_MAX_IOUT          500.0f

//trigger motor max pid output and intergrate ouput
#define TRIGGER_BULLET_PID_MAX_OUT     2000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT    4000.0f

#define TRIGGER_READY_PID_MAX_OUT      2000.0f
#define TRIGGER_READY_PID_MAX_IOUT     4000.0f

//(used in referee check)
#define SHOOT_HEAT_REMAIN_VALUE        80


typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_CONTINUE_BULLET,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;                          //shoot mode
    const RC_ctrl_t *shoot_rc;                        //remote control struct

    const motor_measure_t *shoot_motor_measure;       //trigger motor    feedback measurement
    const motor_measure_t *fric_motor1_measure;       //fric motor left  feedback measurement
    const motor_measure_t *fric_motor2_measure;       //fric motor right feedback measurement

    pid_type_def trigger_motor_pid;                   //trigger motor        pid struct
    pid_type_def friction_motor1_pid;                 //friction motor left  pid struct
    pid_type_def friction_motor2_pid;                 //friction motor right pid struct

    float32_t friction_speed_set;                     //friction motor set speed used in pid target value
    float32_t speed;                                  //push bullet motor feedback value
    float32_t speed_set;                              //push bullet motor set speed value used in pid target value
    float32_t angle;                                  //current angle of the trigger motor
    float32_t set_angle;                              //set angle for trigger motor

    int16_t ecd_count;                                //laps / turns
    int16_t ecd;                                      //currently ecd value feedback from motor
    int16_t last_ecd;                                 //last time ecd value

    int16_t bullet_count;                             //number of bullet
    int16_t last_bullet_count;                        //last time number of bullet
    bool_t press_l;                                   //mouse left button click status
    bool_t press_r;                                   //mouse right button click status
    bool_t last_press_l;                              //last time mouse left button click status
    bool_t last_press_r;                              //last time mouse right button click status
    uint16_t press_l_time;                            //time when mouse left button is pressed
    uint16_t press_r_time;                            //time when mouse right button is pressed
    uint16_t rc_s_time;                               //time when the controller is switched down

    uint16_t block_time;                              //time when the motor is blocked
    uint16_t reverse_time;                            //time when the motor is reversing

    uint16_t heat_limit;
    uint16_t heat;
} shoot_control_t;


//because shoot and gimbal use the same CAN id, so shoot task is executed in gimbal task
extern void shoot_init(void);
extern int16_t * shoot_control_loop(void);

#endif
