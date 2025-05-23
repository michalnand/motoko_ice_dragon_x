#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
 
#include <pwm.h>

#include <motion_kalman.h>
#include <lqr_single.h>
#include <lqg_single.h>

#include <fmath.h>

#include <as5600_t.h>

//dt step in microseconds, 4kHz, 250uS
#define MOTOR_CONTROL_DT    ((uint32_t)250)


class MotorControl     
{   
    public:
        //init motor control process
        void init();

    // main control
    public:
        // turn OFF closed loop control, and set torque directly
        // range : -1, 1, for min and max torque, range MOTOR_CONTROL_MAX_TORQUE
        void set_left_torque(float left_torque);
        void set_right_torque(float right_torque);

        // turn ON closed loop control, and set required velocity in rad/s
        // max velocity is arround 1000RPM (2PI*1000/60 [rad/s]), range : MOTOR_CONTROL_MAX_VELOCITY
        void set_left_velocity(float left_velocity);
        void set_right_velocity(float right_velocity);


        // force break to both motors
        void halt();

        float get_left_u();
        float get_right_u();


    // encoder reading
    public: 
        // raw left encoder steps reading
        int32_t get_left_encoder();

        // wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_left_position();

        // wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_left_velocity();  

        //return more filtered position and velocity
        float get_left_position_smooth();
        float get_left_velocity_smooth();


        // raw right encoder steps reading
        int32_t get_right_encoder();

        // wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_right_position();

        // wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_right_velocity();

        //return more filtered position and velocity
        float get_right_position_smooth();
        float get_right_velocity_smooth();

    public:
        // called by timer interrupt, for closed loop control handling
        void callback();

    private:
        void timer_init();

        void set_torque_from_rotation(int32_t torque, bool brake, uint32_t rotor_angle, int motor_id);



    public:
        bool  left_ol_mode, right_ol_mode;
        float left_torque, right_torque;
        float left_req_velocity, right_req_velocity;


        AS5600T<5, 12, 5,  TGPIOB, TGPIOC> right_encoder;
        AS5600T<11, 10, 5, TGPIOC, TGPIOC> left_encoder;

      
        MotionFilterEMA right_filter, left_filter;    
        MotionFilterEMA right_filter_smooth, left_filter_smooth;    
        
        //motor PWM control
        PWMRightThreePhase    right_pwm;
        PWMLeftThreePhase     left_pwm;

        LQGSingle right_controller;   
        LQGSingle left_controller;


    public:
        uint32_t steps;
};


#endif  