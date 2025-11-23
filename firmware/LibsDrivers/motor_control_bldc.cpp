#include <libs_drivers.h>
#include <motor_control_bldc.h>

#include <tmath.h>
#include <sine_table.h>
    
#define MOTOR_TIMER_FREQ             ((uint32_t)2000)
#define MOTOR_CONTROL_MAX_TORQUE     ((float)1.0)
#define MOTOR_CONTROL_MAX_VELOCITY   ((float)1000.0*2.0*PI/60.0)
    
#define MOTOR_POLES             ((int32_t)14)       

#define SQRT3       ((int32_t)1773)      // sqrt(3)     = 1773/1024
#define SQRT3INV    ((int32_t)591)       // 1/sqrt(3)   = 591/1024

#define max2(x,y) (((x) >= (y)) ? (x) : (y)) 
#define min2(x,y) (((x) <= (y)) ? (x) : (y))

#define max3(x, y, z) (max2(max2(x, y), z))
#define min3(x, y, z) (min2(min2(x, y), z))



#ifdef __cplusplus
extern "C" {
#endif

MotorControl *g_motor_control_ptr;

// timer 2 interrupt handler, running velocity control
void TIM2_IRQHandler(void)
{ 
    if (LL_TIM_IsActiveFlag_UPDATE(TIM2))
    {
        LL_TIM_ClearFlag_UPDATE(TIM2);
        g_motor_control_ptr->callback();
    }
} 


#ifdef __cplusplus
}
#endif


//init motor control process
void MotorControl::init()
{
    g_motor_control_ptr = this;

    this->steps = 0;

    this->left_torque         = 0;
    this->right_torque        = 0;

    // encoder reading
    this->left_position_prev    = 0.0f;
    this->right_position_prev   = 0.0f;
    this->left_position         = 0.0f;
    this->right_position        = 0.0f;

    
    left_pwm.init();    
    right_pwm.init();


   
    // set motors to zero position      
    set_torque_from_rotation(PWM_VALUE_MAX/2, 0, true, 0);
    set_torque_from_rotation(PWM_VALUE_MAX/2, 0, true, 1);

    timer.delay_ms(200);    

    // calibrate encoders   
    left_encoder.init(); 
    right_encoder.init();

    // release motors   
    set_torque_from_rotation(0, 0, true, 0);
    set_torque_from_rotation(0, 0, true, 1);

    timer.delay_ms(100);

    /*
    //optimal control init 

    // Q = 1, R = 4*(10**7)
    float a =  0.98147325;
    float b =  5.60662146;

    float k  =  0.00491353;
    float ki =  0.00015595;
    float f  =  0.14402203;

    left_controller.init(a, b, k, ki, f, 1.0);
    right_controller.init(a, b, k, ki, f, 1.0);
    */

    //init timer
    timer_init();
}


// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void MotorControl::set_left_torque(float left_torque)
{
    //->left_ol_mode = true;
    this->left_torque  = left_torque;
}

// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void  MotorControl::set_right_torque(float right_torque)
{
    //this->right_ol_mode = true;
    this->right_torque  = right_torque;
}


/*
// turn ON closed loop control, and set required velocity in rad/s
// max velocity is arround 1000RPM (2PI*1000/60 [rad/s])
void MotorControl::set_left_velocity(float left_velocity)
{
    this->left_req_velocity  = left_velocity;
    this->left_ol_mode       = false;
}

// turn ON closed loop control, and set required velocity in rad/s
// max velocity is arround 1000RPM (2PI*1000/60 [rad/s])
void MotorControl::set_right_velocity(float right_velocity)
{
    this->right_req_velocity  = right_velocity;
    this->right_ol_mode       = false;
}
*/

// force break to both motors
void MotorControl::halt()
{
    //this->set_left_velocity(0);
    //this->set_right_velocity(0);

    set_torque_from_rotation(0, 0, false, 0);
    set_torque_from_rotation(0, 0, false, 1);
}


// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_left_position()
{
    return this->left_position;
}

// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_right_position()
{
    return this->right_position;
}

float MotorControl::get_left_velocity()
{
    return (this->left_position - this->left_position_prev)/(1.0f/MOTOR_TIMER_FREQ);
}

float MotorControl::get_right_velocity()    
{
    return (this->right_position - this->right_position_prev)/(1.0f/MOTOR_TIMER_FREQ);
}   


/*
// read raw left encoder value
int32_t MotorControl::get_left_encoder()
{
    return left_encoder.angle;
}

// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_left_position()
{
    //return -left_filter.position_hat;
    return -left_filter.position_hat;
}

// wheel angular velocity in rad/s, 2PI is equal to one full forward rotation per second, -2PI for backward
float MotorControl::get_left_velocity()
{
    return -left_filter.velocity_hat/(MOTOR_CONTROL_DT*0.000001f);
}

float MotorControl::get_left_position_smooth()
{
    return -left_filter_smooth.position_hat;    
}

float MotorControl::get_left_velocity_smooth()
{
    return -left_filter_smooth.velocity_hat/(MOTOR_CONTROL_DT*0.000001f);    
}


// read raw right encoder value
int32_t MotorControl::get_right_encoder()
{
    return right_encoder.angle;
}

// wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
float MotorControl::get_right_position()
{
    return right_filter.position_hat;
}

// wheel angular velocity in rad/s, 2PI is equal to one full forward rotation per second, -2PI for backward
float MotorControl::get_right_velocity()    
{
    return right_filter.velocity_hat/(MOTOR_CONTROL_DT*0.000001f);
}        
    
float MotorControl::get_right_position_smooth()
{
    return right_filter_smooth.position_hat;    
}

float MotorControl::get_right_velocity_smooth()
{
    return right_filter_smooth.velocity_hat/(MOTOR_CONTROL_DT*0.000001f);    
}
*/

void MotorControl::callback()
{
    // refresh encoders
    left_encoder.update();          
    right_encoder.update();  
    
    // update state
    this->left_position_prev    = this->left_position;
    this->right_position_prev   = this->right_position;
    this->left_position         = -(2.0f*PI*left_encoder.position)/ENCODER_RESOLUTION;
    this->right_position        = (2.0f*PI*right_encoder.position)/ENCODER_RESOLUTION;

   
    /*
    if (this->left_ol_mode)
    {
        left_controller.reset();    
    }
    else
    {
        this->left_torque = left_controller.step(this->left_req_velocity, this->get_left_velocity());
    }

    if (this->right_ol_mode)
    {
        right_controller.reset();
    }
    else
    {
        this->right_torque = right_controller.step(this->right_req_velocity, this->get_right_velocity());
    }  
    */     

    float left_torque  = this->left_torque;
    float right_torque = this->right_torque;

    /*
    float tc = 0.0025;
    float tv = 0.002;   

    left_torque+=  tc*sgn(this->get_left_velocity())  + tv*this->get_left_velocity();
    right_torque+= tc*sgn(this->get_right_velocity()) + tv*this->get_right_velocity();
    */

    // cogging torque compensation 
    /*
    uint32_t theta;
    float k = 0.01;          
    
    theta = (2*12*right_encoder.angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);
    //theta = (2*9*right_encoder.angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);
    right_torque+= -(k*sin_tab(theta))/SINE_TABLE_MAX; 

    theta = (2*12*left_encoder.angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);
    //theta = (2*9*left_encoder.angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);
    left_torque+= -(k*sin_tab(theta))/SINE_TABLE_MAX;   
    */      

    // scale -1...1 range into -MOTOR_CONTROL_MAX ... MOTOR_CONTROL_MAX
    // send torques to motors   
    set_torque_from_rotation(-left_torque*PWM_VALUE_MAX,  left_encoder.angle,  false, 0);
    set_torque_from_rotation(right_torque*PWM_VALUE_MAX, right_encoder.angle, false, 1);

    this->steps++;  
}









void MotorControl::timer_init()
{
    /* Enable TIM2 clock (on APB1) */
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM2);

    /* Disable TIM2 during configuration */
    LL_TIM_DisableCounter(TIM2);

    /* Timer clock = 216 MHz (APB1 doubled) */
    /* Prescaler = 215 → 216 MHz / 216 = 1 MHz timer tick */
    LL_TIM_SetPrescaler(TIM2, 215);

    /* Auto-reload for 4 kHz: 1 MHz / 250 = 4000 Hz */
    LL_TIM_SetAutoReload(TIM2, 1000000/MOTOR_TIMER_FREQ);


    /* Enable update interrupt (UIE) */
    LL_TIM_EnableIT_UPDATE(TIM2);       

    /* NVIC configuration */    
    NVIC_SetPriority(TIM2_IRQn, NVIC_EncodePriority(NVIC_GetPriorityGrouping(), 2, 0));
    NVIC_EnableIRQ(TIM2_IRQn);  

    /* Clear update flag */
    LL_TIM_ClearFlag_UPDATE(TIM2);

    /* Start counter */
    LL_TIM_EnableCounter(TIM2);
}




void MotorControl::set_torque_from_rotation(int32_t torque, uint32_t rotor_angle, bool brake, int motor_id)
{
    torque = clip(torque, -(int32_t)PWM_VALUE_MAX, (int32_t)PWM_VALUE_MAX);

    // convert mechanical angle to electrical angle and index into sine table
    uint32_t table_angle = (rotor_angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);

    if (brake)
    {
        rotor_angle = 0;
        table_angle = 0;
    }  
    else if (torque >= 0)
    {   
        table_angle = (table_angle - (3*SINE_TABLE_SIZE)/4) % SINE_TABLE_SIZE;
    }
    else if (torque < 0)          
    {   
        torque = -torque;
        table_angle = (table_angle + (3*SINE_TABLE_SIZE)/4) % SINE_TABLE_SIZE;
    }    

    

    // produce 3-phase shifted angles
    uint32_t angle_a = table_angle % SINE_TABLE_SIZE;
    uint32_t angle_b = (table_angle + SINE_TABLE_SIZE / 3) % SINE_TABLE_SIZE;
    uint32_t angle_c = (table_angle + 2 * (SINE_TABLE_SIZE / 3)) % SINE_TABLE_SIZE;

    // read sinusoidal values
    int32_t sa = sine_table[angle_a];
    int32_t sb = sine_table[angle_b];   
    int32_t sc = sine_table[angle_c];

    /*
    // center sinusoids around zero
    sa -= SINE_VALUE_MAX/2;
    sb -= SINE_VALUE_MAX/2;
    sc -= SINE_VALUE_MAX/2;    


    // transform into space-vector modulation, to achieve full voltage range
    int32_t min_val = min3(sa, sb, sc);
    int32_t max_val = max3(sa, sb, sc);

    int32_t com_val = (min_val + max_val)/2;

    //normalise into 0..control_max
    int32_t norm_a = ( ((sa - com_val)*2*SQRT3INV)/1024 ) + SINE_VALUE_MAX/2;
    int32_t norm_b = ( ((sb - com_val)*2*SQRT3INV)/1024 ) + SINE_VALUE_MAX/2;
    int32_t norm_c = ( ((sc - com_val)*2*SQRT3INV)/1024 ) + SINE_VALUE_MAX/2;

    //compute PWM value
    int32_t pwm_a = (norm_a*torque)/SINE_VALUE_MAX;
    int32_t pwm_b = (norm_b*torque)/SINE_VALUE_MAX; 
    int32_t pwm_c = (norm_c*torque)/SINE_VALUE_MAX; 
    */  

    int32_t pwm_a = (sa*torque)/SINE_VALUE_MAX;
    int32_t pwm_b = (sb*torque)/SINE_VALUE_MAX; 
    int32_t pwm_c = (sc*torque)/SINE_VALUE_MAX; 


    //pwm values    
    pwm_a = clip(pwm_a, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);
    pwm_b = clip(pwm_b, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);
    pwm_c = clip(pwm_c, (int32_t)(0), ((int32_t)PWM_VALUE_MAX)-1);

    if (motor_id == 0)      
    {
        left_pwm.set(pwm_a, pwm_b, pwm_c);
    }   
    else         
    {
        right_pwm.set(pwm_b, pwm_a, pwm_c);
    }
}
