#include <auvng_controller/pid.h>

double upper_limit_ = 1000, lower_limit_ = -1000;
double windup_limit_ = 1000;

ros::Time prev_time_;
ros::Duration delta_t_;

void do_calculate()
{
    double proportional[6] = {0,0,0,0,0,0}; // proportional term of output
    double integral[6] = {0,0,0,0,0,0};      // integral term of output
    double derivative[6] = {0,0,0,0,0,0};    // derivative term of output

    for (int i = 0; i < 6; i++)
    {
        // calculate the control effort
        proportional[i] = K_p[i] * error[0][i];
        integral[i] = K_i[i]  * int_error[i];
        derivative[i] = K_d[i] * div_error[i];
        control_effort[i] = proportional[i] + integral[i] + derivative[i];

        // Apply saturation limits
        if (control_effort[i] > upper_limit_)
            control_effort[i] = upper_limit_;
        else if (control_effort[i] < lower_limit_)
            control_effort[i] = lower_limit_;

        // Publish the stabilizing control effort if the controller is enabled
        if (!control_enabled)
        {
            control_effort[i] = 0.0;
            int_error[i] = 0.0;
        }
    }

}

void calculate_error()
{

    for (int i = 0; i < 6; i++)
    {
        error[2][i] = error[1][i];
        error[1][i] = error[0][i];
        error[0][i] = set_point[i] - current_state[i];
    }

    for (int i = 3; i < 6; i++)
    {
        while (error[0][i] < -1.0 * ANGLE_WRAP / 2.0)
        {
            error[0][i] += ANGLE_WRAP;
        }
        while (error[0][i] > ANGLE_WRAP / 2.0)
        {
            error[0][i] -= ANGLE_WRAP;
        }

        error[2][i] = 0.0;
        error[1][i] = 0.0;
        int_error[i] = 0.0;
    }

    if (!prev_time_.isZero()) // Not first time through the program
    {
        delta_t_ = ros::Time::now() - prev_time_;
        prev_time_ = ros::Time::now();
        if (0 == delta_t_.toSec())
        {
            ROS_ERROR("delta_t is 0, skipping this loop. Possible overloaded cpu "
                      "at time: %f",
                      ros::Time::now().toSec());
        }
    }
    else
    {
        ROS_INFO("prev_time is 0, doing nothing");
        prev_time_ = ros::Time::now();
    }

    for (int i = 0; i < 6; i++)
    {
        // integrate the error
        int_error[i] += error[0][i] * delta_t_.toSec();

        // Apply windup limit to limit the size of the integral term
        if (int_error[i] > fabsf(windup_limit_))
        {
            int_error[i] = fabsf(windup_limit_);
        }

        if (int_error[i] < -fabsf(windup_limit_))
        {
            int_error[i] = -fabsf(windup_limit_);
        }

        // Take derivative of error
        div_error[i] = (error[0][i] - error[1][i]) / delta_t_.toSec();
    }

}