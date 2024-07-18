#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
 
#include <pwm.h>
#include <lqg_single.h>
#include <fmath.h>

//dt step in microseconds, 4kHz, 250uS
#define MOTOR_CONTROL_DT    ((uint32_t)250)

//encoder pulses per rotation
#define ENCODER_RESOLUTION   ((int32_t)180)

    
#define MOTOR_CONTROL_MAX_TORQUE     ((float)1.0)
#define MOTOR_CONTROL_MAX_VELOCITY   ((float)1000.0*2.0*PI/60.0)

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

    // encoder reading
    public: 
        // wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_left_position();

        // wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_left_velocity();

        //return kalman filtered velocity
        float get_left_velocity_fil();


        // wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_right_position();

        // wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_right_velocity();

        //return kalman filtered velocity
        float get_right_velocity_fil();

    public:
        // called by timer interrupt, for closed loop control handling
        void callback();

    private:
        void timer_init();
        void encoder_init();


    private:
        bool  left_ol_mode, right_ol_mode;
        float left_torque, right_torque;
        float left_req_velocity, right_req_velocity;

  
        //motor PWM control
        PWMLeft     left_pwm;
        PWMRight    right_pwm;

        //single input, single output motor speed controller
        LQGSingle left_controller;
        LQGSingle right_controller;
};


#endif