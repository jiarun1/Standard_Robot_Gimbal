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

//disable referee.c/h
#define REFEREE_LIB_NEED

#include "pid.h"
#include "main.h"
#include "shoot.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "user_lib.h"
#include "bsp_laser.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"

#ifndef REFEREE_LIB_NEED
#include "referee.h"
#endif

//laser settings
#define shoot_laser_on()    laser_on()
#define shoot_laser_off()   laser_off()

//macro switch IO (PB0/J34)
//#define BUTTEN_TRIG_PIN shoot_control.shoot_rc->rc.s[0]

/**
  * @brief          initialize two friction motors.
  * @param[in]      void
  * @retval         void
  */
static void fric_motor_init(void);

/**
  * @brief          set shoot control mode.
  *                 remote controller switch pushed up once -> turn on
  *                 remote controller switch pushed up again -> turn off
  *                 remote controller switch pushed down -> shoot one bullet
  *                 remote controller switch kept down -> keep shooting (used to clean bullet in 3 minutes preparation time)
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);

/**
  * @brief          update shoot related data
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          push bullet motor inverse to handle bullet stuck
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          shoot control. Control push bullet motor angle and complete one shoot
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

/**
 * @brief           get absolute value
 * @param[in]       a number
 * @retval          abs value
 */
static int16_t get_abs_value(int16_t x);


//global variable define
shoot_control_t shoot_control;          //shoot related data
static int16_t can_data[3] = {0,0,0};   //array of data (pointer) would be returned to the gimbal task

/**
 * @brief           get absolute value
 * @param[in]       a number
 * @retval          none
 */
int16_t get_abs_value(int16_t x)
{
	return (x>=0) ? x : -x;
}


int16_t speed1 = 0;
int16_t speed2 = 0;
/**
  * @brief          shoot initialization, initialize:
  *                 ->PID initialization
  *                 ->remote controller data pointer
  *                 ->motor data pointer
  * @param[in]      void
  * @retval         none
  */
void shoot_init(void)
{
	//initialize PID value set
    static const float32_t Trigger_speed_pid[3]   = {TRIGGER_ANGLE_PID_KP, TRIGGER_ANGLE_PID_KI, TRIGGER_ANGLE_PID_KD};
    static const float32_t Friction_speed1_pid[3] = {FRICTION_MOTOR1_PID_KP, FRICTION_MOTOR1_PID_KI, FRICTION_MOTOR1_PID_KD};
    static const float32_t Friction_speed2_pid[3] = {FRICTION_MOTOR2_PID_KP, FRICTION_MOTOR2_PID_KI, FRICTION_MOTOR2_PID_KD};

    //friction motors initialization
    fric_motor_init();

    //set initial status
    shoot_control.shoot_mode = SHOOT_STOP;

    //remote controller data pointer
    shoot_control.shoot_rc = get_remote_control_point();

    //motor feedback data pointer
    shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
    shoot_control.fric_motor1_measure = get_friction_motor_measure_point(0);
    shoot_control.fric_motor2_measure = get_friction_motor_measure_point(1);

    //initialize PID model
    PID_init(&shoot_control.trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    PID_init(&shoot_control.friction_motor1_pid, PID_DELTA, Friction_speed1_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);
    PID_init(&shoot_control.friction_motor2_pid, PID_DELTA, Friction_speed2_pid, FRICTION_PID_MAX_OUT, FRICTION_PID_MAX_IOUT);

    //update data
    shoot_feedback_update();

    //initialize trigger motor settings
    shoot_control.speed = 0.0f;
    shoot_control.ecd_count = 0;
    shoot_control.speed_set = 0.0f;
    shoot_control.bullet_count = 0;
    shoot_control.last_bullet_count = 0;
    shoot_control.angle = (float32_t)shoot_control.shoot_motor_measure->ecd * 360.0 / SHOOT_PUSH_ECD_RANGE / 36.0; //MOTOR_ECD_TO_ANGLE;
    shoot_control.set_angle = shoot_control.angle;
}


/**
  * @brief          initialize two friction motors.
  * @param[in]      void
  * @retval         void
  */
static void fric_motor_init(void)
{
    //let friction motor to rotate 1s to make sure it is successfully controlled
	//int i;
	//for (i=0; i<=500; i++)
	//{
	//  CAN_cmd_shoot(-300, 300), osDelay(1);
	//}
    //CAN_cmd_shoot(0, 0);
}


/**
  * @brief          shoot loop control
  * @param[in]      void
  * @retval         CAN control value pointer [M3508X2 (0x201 0x202), M2006 (0x207)]
  */
int16_t * shoot_control_loop(void)
{
	//update data
	shoot_feedback_update();

	//set control mode
    shoot_set_mode();

    //statue machine control
    if (shoot_control.shoot_mode == SHOOT_STOP)
    {
    	//stop all motor
    	shoot_control.speed_set = 0.0f;
    	shoot_control.friction_speed_set = 0.0f;
    	shoot_control.set_angle = shoot_control.angle;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //set trigger motor speed
        shoot_control.speed_set = 0.0f;
    }
    else if (shoot_control.shoot_mode == SHOOT_READY)
    {
        //set push bullet motor speed
        shoot_control.speed_set = 0; // -0.5 * (shoot_control.set_angle - shoot_control.angle);
    }
    else if (shoot_control.shoot_mode == SHOOT_BULLET)
    {
    	if (shoot_control.set_angle - shoot_control.angle > 30)
    	{
    		shoot_control.speed_set = -TRIGGER_SPEED;
    		shoot_control.trigger_motor_pid.Kp = TRIGGER_ANGLE_PID_KP;
    		shoot_control.trigger_motor_pid.Ki = TRIGGER_ANGLE_PID_KI;
    		shoot_control.trigger_motor_pid.Kd = TRIGGER_ANGLE_PID_KD;
    	}
    	else
    	{
    		shoot_control.speed_set = - 0.3 * (shoot_control.set_angle - shoot_control.angle);
    		shoot_control.trigger_motor_pid.Kp = TRIGGER_STOP_PID_KP;
    		shoot_control.trigger_motor_pid.Ki = TRIGGER_STOP_PID_KI;
    		shoot_control.trigger_motor_pid.Kd = TRIGGER_STOP_PID_KD;
    	}
        shoot_control.trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        shoot_control.trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        shoot_bullet_control();
    }
    else if (shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
    {
        //set push bullet motor's pushing speed, and enable bullet stuck handler
        trigger_motor_turn_back();
    }

    //mouse right button pressed down -> speed up friction motors
    //right button click -> fast shoot 80 times
    static uint16_t up_time = 0;

    if(shoot_control.shoot_mode != SHOOT_STOP)
    {
    	//start laser
        shoot_laser_on();

        //set friction motor speed & speed up time
        if (shoot_control.press_r) up_time = UP_ADD_TIME;

        if (up_time > 0) shoot_control.friction_speed_set = FRIC_FAST, up_time--;
        else shoot_control.friction_speed_set = FRIC_SLOW;
    }
    else
    {
    	//close laser
    	shoot_laser_off();

    	//can_data[0] = 0; can_data[1] = 0; can_data[2] = 0;
    }
    //calculate friction motor and trigger motor PID
    PID_calc(&shoot_control.trigger_motor_pid, shoot_control.speed, shoot_control.speed_set);
    PID_calc(&shoot_control.friction_motor1_pid, shoot_control.fric_motor1_measure->speed_rpm, -1 * shoot_control.friction_speed_set);
    PID_calc(&shoot_control.friction_motor2_pid, shoot_control.fric_motor2_measure->speed_rpm, shoot_control.friction_speed_set);

    //assemble the data pointer value
    can_data[0] = (shoot_control.shoot_mode > SHOOT_READY) ? (int16_t)shoot_control.trigger_motor_pid.out : 0;
    can_data[1] = (int16_t)(shoot_control.friction_motor1_pid.out);
    can_data[2] = (int16_t)(shoot_control.friction_motor2_pid.out);
    return can_data;
}


/**
  * @brief          set shoot control mode.
  *                 remote controller switch pushed up once -> turn on
  *                 remote controller switch pushed up again -> turn off
  *                 remote controller switch pushed down -> shoot one bullet
  *                 remote controller switch kept down -> keep shooting (used to clean bullet in 3 minutes preparation time)
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;    //last switch position
    speed1 = shoot_control.fric_motor1_measure->speed_rpm;
    speed2 = shoot_control.fric_motor2_measure->speed_rpm;
    //using remote controller
    //switch push up check, push up once -> friction motor turned on
    //again push up once                 -> friction motor turned off
    //current status: up, last status: mid, shoot_mode: STOP     -> turn on  friction motors
    //current status: up, last status: mid, shoot_mode: NOT STOP -> turn off friction motors
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //using PC control
    //switch in middle, friction motors can be controlled by keyboard
    //STOP mode        -> press Q to start friction motor
    //NOT IN STOP mode -> press E to stop  friction motor
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //friction motors reach set speed -> READY
    if(shoot_control.shoot_mode == SHOOT_READY_FRIC && get_abs_value(shoot_control.fric_motor1_measure->speed_rpm - (int16_t)shoot_control.friction_speed_set) < 80 && get_abs_value(shoot_control.fric_motor2_measure->speed_rpm - (int16_t)shoot_control.friction_speed_set) < 80)
    {
        //shoot_control.shoot_mode = SHOOT_READY_BULLET;
    	shoot_control.shoot_mode = SHOOT_READY;
    }
    else if(shoot_control.shoot_mode == SHOOT_READY)
    {
        //switch down              -> shoot a bullet
    	//left/right mouse pressed -> shoot a bullet
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l && shoot_control.last_press_l == 0) || (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
            shoot_control.bullet_count += 1;
        }

        //mouse pressed down continuously     -> CONTINUE-BULLET
        //remote controller down continuously -> CONTINUE-BULLET
        //mouse NOT PRESSED & remote controller switch NOT DOWN & IN CONTINUOUS MODE -> READY-BULLET
        if ((shoot_control.press_l_time == PRESS_LONG_TIME) || (shoot_control.press_r_time == PRESS_LONG_TIME) || (shoot_control.rc_s_time == RC_S_LONG_TIME))
        {
            shoot_control.shoot_mode = SHOOT_CONTINUE_BULLET;
        }
        else if(shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
            //shoot_control.shoot_mode = SHOOT_READY_BULLET;
            shoot_control.shoot_mode = SHOOT_READY;
        }
    }
    else if(shoot_control.shoot_mode == SHOOT_BULLET && shoot_control.set_angle - shoot_control.angle < 3)
    {
    	shoot_control.shoot_mode = SHOOT_READY;
    }

    #ifndef REFEREE_LIB_NEED
        get_shoot_heat0_limit_and_heat0(&shoot_control.heat_limit, &shoot_control.heat);
    #endif

    #ifndef REFEREE_LIB_NEED
        if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
        {
            if(shoot_control.shoot_mode == SHOOT_BULLET || shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
            {
                shoot_control.shoot_mode =SHOOT_READY_BULLET;
            }
        }
    #endif

    //in some special gimbal behavior mode, shoot needs to be stopped
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }

    //update last control
    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}

/**
  * @brief          update shoot related data
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{
	//update data from motor
	shoot_control.shoot_motor_measure = get_trigger_motor_measure_point();
	shoot_control.fric_motor1_measure = get_friction_motor_measure_point(0);
	shoot_control.fric_motor2_measure = get_friction_motor_measure_point(1);

	//update ecd value
	shoot_control.ecd = shoot_control.shoot_motor_measure->ecd;

    //push bullet motor speed does a filtering...
    //three past values' buffer
    static float32_t speed_fliter_1 = 0.0f;
    static float32_t speed_fliter_2 = 0.0f;
    static float32_t speed_fliter_3 = 0.0f;

    //filter parameters
    static const float32_t fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};

    //second-order low-pass filter
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (shoot_control.shoot_motor_measure->speed_rpm * MOTOR_RPM_TO_SPEED) * fliter_num[2];
    shoot_control.speed = speed_fliter_3;

    //motor lap count reset.
    //because motor turns 36 laps, output axle turns 1 lap
    //here we convert motor data into output axle data, in order to control output axle angle
    if ((shoot_control.last_ecd > 4096) && (shoot_control.last_ecd < 8192) && (shoot_control.ecd > 0) && (shoot_control.ecd < 4096) && (shoot_control.speed > 0.0001))
    {
        shoot_control.ecd_count--;
    }
    else if ((4096 < shoot_control.ecd) && (shoot_control.ecd < 8192) && (shoot_control.last_ecd > 0) && (shoot_control.last_ecd < 4096) && (shoot_control.speed < -0.0001))
    {
        shoot_control.ecd_count++;
    }

    //calculate output axle angle
    shoot_control.angle = (float32_t)shoot_control.ecd_count * 10.0 + (float32_t)shoot_control.shoot_motor_measure->ecd * 10.0 / SHOOT_PUSH_ECD_RANGE;

    //macro switch
    //shoot_control.key = BUTTEN_TRIG_PIN;

    //mouse button status update
    shoot_control.last_press_l = shoot_control.press_l;
    shoot_control.last_press_r = shoot_control.press_r;

    //mouse button pressed currently
    shoot_control.press_l = shoot_control.shoot_rc->mouse.press_l;
    shoot_control.press_r = shoot_control.shoot_rc->mouse.press_r;

    //mouse button LEFT held time count
    if (shoot_control.press_l)
    {
        if (shoot_control.press_l_time < PRESS_LONG_TIME)
        {
            shoot_control.press_l_time++;
        }
    }
    else
    {
        shoot_control.press_l_time = 0;
    }

    //mouse button RIGHT held time count
    if (shoot_control.press_r)
    {
        if (shoot_control.press_r_time < PRESS_LONG_TIME)
        {
            shoot_control.press_r_time++;
        }
    }
    else
    {
        shoot_control.press_r_time = 0;
    }

    //remote controller switch held at down position time count
    if (shoot_control.shoot_mode != SHOOT_STOP && switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]))
    {
        if (shoot_control.rc_s_time < RC_S_LONG_TIME)
        {
            shoot_control.rc_s_time++;
        }
    }
    else
    {
        shoot_control.rc_s_time = 0;
    }

    //update last ecd
    shoot_control.last_ecd = shoot_control.ecd;
}

/**
  * @brief          push bullet motor inverse to handle bullet stuck
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void)
{
    if(shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.speed_set = -CONTINUE_TRIGGER_SPEED;
    }
    else
    {
        shoot_control.speed_set = CONTINUE_TRIGGER_SPEED;
    }
    /*
    if(fabs(shoot_control.speed) < BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
    */
}

/**
  * @brief          shoot control. Control push bullet motor angle and complete one shoot
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
   // *yaw = - gimbal_control_set->gimbal_rc_ctrl->mouse.x * 0.000002 - frame.data.demoCmd.dx * YAW_AI_SEN; //yaw_moues_sen is 0.00005f, here we use 0.000002 to largely reduce the influence of mouse.x
   // *pitch =  + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN + frame.data.demoCmd.dy * PITCH_AI_SEN;

    if(shoot_control.bullet_count == shoot_control.last_bullet_count) return;
    shoot_control.set_angle += 45;
    shoot_control.last_bullet_count = shoot_control.bullet_count;
}
