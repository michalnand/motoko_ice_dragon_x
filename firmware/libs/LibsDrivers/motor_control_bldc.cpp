#include <drivers.h>
#include <motor_control_bldc.h>
#include <gonio.h>
 

    
#define MOTOR_CONTROL_MAX_TORQUE     ((float)1.0)
#define MOTOR_CONTROL_MAX_VELOCITY   ((float)1000.0*2.0*PI/60.0)
    

#define MOTOR_CONTROL_MAX       ((int32_t)1024) 
#define MOTOR_POLES             ((int32_t)14)       

#define SQRT3       ((int32_t)1773)      // sqrt(3)     = 1773/1024
#define SQRT3INV    ((int32_t)591)       // 1/sqrt(3)   = 591/1024

#define max2(x,y) (((x) >= (y)) ? (x) : (y)) 
#define min2(x,y) (((x) <= (y)) ? (x) : (y))

#define max3(x, y, z) (max2(max2(x, y), z))
#define min3(x, y, z) (min2(min2(x, y), z))


MotorControl *g_motor_control_ptr;

#ifdef __cplusplus
extern "C" {
#endif



// timer 2 interrupt handler, running velocity control
void TIM2_IRQHandler(void)
{ 
    //g_motor_control_ptr->callback();     
    //TIM_ClearITPendingBit(TIM2, TIM_IT_Update);

    if (TIM2->SR & TIM_SR_UIF)  // Ensure update interrupt flag is set
    {
        TIM2->SR &= ~TIM_SR_UIF; // Clear the interrupt flag
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


    this->left_ol_mode  = true;
    this->right_ol_mode = true;

    this->left_torque         = 0;
    this->right_torque        = 0;
    this->left_req_velocity   = 0;
    this->right_req_velocity  = 0;

  

    left_pwm.init();    
    right_pwm.init();

    
    float k0 = 0.3;
    float k1 = 1.0;  
    
    float k2 = 0.3;             
    float k3 = 0.3;         

    left_filter.init(k0, k1); //, 0.000001*MOTOR_CONTROL_DT);
    right_filter.init(k0, k1); //,, 0.000001*MOTOR_CONTROL_DT); 

    left_filter_smooth.init(k2, k3); //, 0.000001*MOTOR_CONTROL_DT);
    right_filter_smooth.init(k2, k3); //, 0.000001*MOTOR_CONTROL_DT);

    // set motors to zero position
    set_torque_from_rotation(500, true, 0, 0);
    set_torque_from_rotation(500, true, 0, 1);

    timer.delay_ms(200);    

    // calibrate encoders   
    left_encoder.init(); 
    right_encoder.init();

    // release motors
    set_torque_from_rotation(0, false, 0, 0);
    set_torque_from_rotation(0, false, 0, 1);

    timer.delay_ms(100);

    //optimal control init 

    // Q = 1, R = 4*(10**7)
    float a =  0.98147325;
    float b =  5.60662146;

    float k  =  0.00491353;
    float ki =  0.00015595;
    float f  =  0.14402203;

    left_controller.init(a, b, k, ki, f, 1.0);
    right_controller.init(a, b, k, ki, f, 1.0);


    //init timer
    timer_init();
}


// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void MotorControl::set_left_torque(float left_torque)
{
    this->left_ol_mode = true;
    this->left_torque  = left_torque;
}

// turn OFF closed loop control, and set torque directly
// range : -1, 1, for min and max torque
void  MotorControl::set_right_torque(float right_torque)
{
    this->right_ol_mode = true;
    this->right_torque  = right_torque;
}


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

// force break to both motors
void MotorControl::halt()
{
    //this->set_left_velocity(0);
    //this->set_right_velocity(0);

    set_torque_from_rotation(0, false, 0, 0);
    set_torque_from_rotation(0, false, 0, 1);
}

float MotorControl::get_left_u()
{
    return -1;
}


float MotorControl::get_right_u()
{
    return -1;
}



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


void MotorControl::callback()
{
    // refresh encoders
    left_encoder.update();          
    right_encoder.update();  
    

    float enc_left  = (2.0f*PI*left_encoder.position)/ENCODER_RESOLUTION;
    float enc_right = (2.0f*PI*right_encoder.position)/ENCODER_RESOLUTION;
    
    left_filter.step(enc_left);  
    right_filter.step(enc_right);

    left_filter_smooth.step(enc_left);  
    right_filter_smooth.step(enc_right);
             

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
    int32_t left_torque_  = clamp((int32_t)(left_torque*MOTOR_CONTROL_MAX),  -MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX);
    int32_t right_torque_ = -clamp((int32_t)(right_torque*MOTOR_CONTROL_MAX), -MOTOR_CONTROL_MAX, MOTOR_CONTROL_MAX);

    // send torques to motors
    set_torque_from_rotation(left_torque_,  false, left_encoder.angle, 0);
    set_torque_from_rotation(right_torque_, false, right_encoder.angle, 1);

    this->steps++;
}









void MotorControl::timer_init()
{
    //init timer 2 interrupt for periodic callback calling, 4kHz
    TIM_TimeBaseInitTypeDef     TIM_TimeBaseStructure;
    NVIC_InitTypeDef            NVIC_InitStructure;

    
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);    


    TIM_TimeBaseStructure.TIM_Prescaler         = 0;
    TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
    TIM_TimeBaseStructure.TIM_Period            = ((MOTOR_CONTROL_DT*(APB1Clock/(uint32_t)1000000))/1) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision     = 0; 
    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;   

    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure);
    //TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
    TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);
    TIM_Cmd(TIM2, ENABLE);   

    
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}




void MotorControl::set_torque_from_rotation(int32_t torque, bool brake, uint32_t rotor_angle, int motor_id)
{
    int32_t q, d;

    if (brake == true)
    {   
        q = torque;
        d = 0;
    }
    else 
    {
        if (torque < 0) 
        {
            torque  = -torque;

            q = 0;
            d = -torque;
        }
        else 
        {
            q = 0;
            d = torque;  
        }
    }

    //convert mechanical angle to electrical angle, with respect to sine table size
    uint32_t theta = (rotor_angle*MOTOR_POLES*SINE_TABLE_SIZE)/(2*ENCODER_RESOLUTION);

    //inverse Park transform
    int32_t alpha = (d*cos_tab(theta) - q*sin_tab(theta))/SINE_TABLE_MAX;
    int32_t beta  = (d*sin_tab(theta) + q*cos_tab(theta))/SINE_TABLE_MAX;


    //inverse Clarke transform
    int32_t a = alpha;
    int32_t b = -(alpha/2) + (SQRT3*beta)/(2*1024);
    int32_t c = -(alpha/2) - (SQRT3*beta)/(2*1024);


    //transform into space-vector modulation, to achieve full voltage range
    int32_t min_val = min3(a, b, c);
    int32_t max_val = max3(a, b, c); 

    int32_t com_val = (min_val + max_val)/2;  

    //normalise into 0..MOTOR_CONTROL_MAX
    int32_t a_pwm = ((a - com_val)*SQRT3INV)/1024 + MOTOR_CONTROL_MAX/2;
    int32_t b_pwm = ((b - com_val)*SQRT3INV)/1024 + MOTOR_CONTROL_MAX/2;
    int32_t c_pwm = ((c - com_val)*SQRT3INV)/1024 + MOTOR_CONTROL_MAX/2;
   
     
    a_pwm = clamp((int32_t)(a_pwm*PWM_PERIOD)/MOTOR_CONTROL_MAX, (int32_t)0, (int32_t)PWM_PERIOD-1);
    b_pwm = clamp((int32_t)(b_pwm*PWM_PERIOD)/MOTOR_CONTROL_MAX, (int32_t)0, (int32_t)PWM_PERIOD-1);
    c_pwm = clamp((int32_t)(c_pwm*PWM_PERIOD)/MOTOR_CONTROL_MAX, (int32_t)0, (int32_t)PWM_PERIOD-1);

    if (motor_id == 0)
    {
        left_pwm.set(b_pwm, a_pwm, c_pwm);
    }   
    else    
    {
        right_pwm.set(a_pwm, b_pwm, c_pwm);
    }
}
