#ifndef _MOTOR_CONTROL_H_
#define _MOTOR_CONTROL_H_
 
#include <pwm.h>
#include <lqg_single.h>

//dt step in microseconds, 4kHz, 250uS
#define MOTOR_CONTROL_DT    ((uint32_t)250)


#define MOTOR_CONTROL_MAX_VELOCITY   ((float)1000.0*2.0*PI/60.0)

class MotorControl      
{   
    public:
        //init motor control process
        void init();

    // main control
    public:
        // turn OFF closed loop control, and set torque directly
        // range : -1, 1, for min and max torque
        void set_left_torque(float left_torque);
        void set_right_torque(float right_torque);

        // turn ON closed loop control, and set required velocity in rad/s
        // max velocity is arround 1000RPM (2PI*1000/60 [rad/s])
        void set_left_velocity(float left_velocity);
        void set_right_velocity(float right_velocity);


    // encoder reading
    public:
        //wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_left_position();

        //wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_left_velocity();

        //wheel position (angle), 2PI is equal to one full forward rotation, -2PI for backward
        float get_right_position();

        //wheel angular velocity, 2PI is equal to one full forward rotation per second, -2PI for backward
        float get_right_velocity();

    private:
        void callback();

    private:
        bool left_ol_mode, right_ol_mode;
        float left_torque, right_torque;
        float left_req_velocity, right_req_velocity;

        float left_position, left_velocity;
        float right_position, right_velocity;

        int64_t left_enc, right_enc;
        
  
        //motor PWM control
        PWMLeft     left_pwm;
        PWMRight    right_pwm;

        //single input, single output motor speed controller
        LQGSingle left_controller;
        LQGSingle right_controller;
};


#endif