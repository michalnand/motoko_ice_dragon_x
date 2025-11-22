#ifndef _MOTOR_CONTROL_BLDC_H_
#define _MOTOR_CONTROL_BLDC_H_

#include <as5600_t.h>


class MotorControl
{

    public:
        void init();

        void set_left_torque(float left_torque);
        void set_right_torque(float right_torque);

        void set_left_velocity(float left_velocity);
        void set_right_velocity(float right_velocity);

        void halt();

        

    public:
        int32_t get_left_encoder();
        float get_left_velocity();
        float get_left_position();
        float get_left_position_smooth();
        float get_left_velocity_smooth();

    public:
        int32_t get_right_encoder();
        float get_right_velocity();
        float get_right_position();
        float get_right_position_smooth();
        float get_right_velocity_smooth();

    public:
        void callback();
        void set_torque_from_rotation(int32_t torque, bool brake, uint32_t rotor_angle, int motor_id);

    private:
        void timer_init();

    private:
        AS5600T<11, 10, 20, 'C', 'C'> left_encoder;
        AS5600T<5, 12,  20,  'B', 'C'> right_encoder;

        PWMLeftThreePhase  left_pwm;
        PWMRightThreePhase right_pwm;

    private:
        uint32_t steps;

        bool left_ol_mode, right_ol_mode;
        float left_torque, right_torque;
        float left_req_velocity, right_req_velocity;
};


#endif

