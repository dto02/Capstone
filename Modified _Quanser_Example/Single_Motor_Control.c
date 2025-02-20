#include "haptic_ultrasound_example.h"
#include "haptic_ultrasound.h"

#include <math.h>
#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"
#include "quanser_thread.h"

#define MOTORS (2) /* # of motors in test*/

struct limiter_state
{
    double mean;
    double time;
    int    count;
    int    state;
};

static int  stop = 0;      /* a flag used to stop the controller */
static char message[512];  /* a buffer used for error messages */

static const char      board_type[] = "q8_usb";                           /* type of board controlling the motor */
static const char      board_identifier[] = "0";                            /* the instance of the board controlling the motor */
static const t_uint32  analog_channel = 0;                              /* analog output channel driving the motor */
static const t_uint32  digital_channel = 0;                              /* digital output channel driving the amplifier enable line */
static const t_double  frequency = 1000;                           /* sampling frequency of the controller */

static void
signal_handler(int signal)
{
    stop = 1;
}

/*
    Limit the motor current such that the thermal rating for the motor is not exceeded
    while continuing to provide the peak torque when necessary.
*/

static void encoder_counts_to_theta(const t_int32 counts[MOTORS], double theta[MOTORS])
{
    static const double factor = 2 * PI / 20000;

    for (int i = 0; i < MOTORS; i++)
    {
        theta[i] = counts[i] * factor;  // Convert encoder counts to radians
    }
}


static void read_encoder(t_card board, t_int32 counts[MOTORS])
{
    hil_read_encoder(board, &analog_channel, 1, counts);
}

//static void read_analog_write_analog(t_card board)
//{
//    t_double analog_input_buffer[1];
//    t_double analog_output_buffer[1];
//
//    /* Read from the analog input */
//    t_error result = hil_read_analog(board, analog_input_channels, 1, analog_input_buffer);
//    if (result < 0)
//    {
//        msg_get_error_message(NULL, result, message, sizeof(message));
//        printf("Error reading analog input: %s Error %d.\n", message, -result);
//        return;
//    }
//
//    /* Process or map input to output (e.g., simple passthrough) */
//    analog_output_buffer[0] = analog_input_buffer[0];
//
//    /* Write to the analog output */
//    result = hil_write_analog(board, analog_output_channels, 1, analog_output_buffer);
//    if (result < 0)
//    {
//        msg_get_error_message(NULL, result, message, sizeof(message));
//        printf("Error writing analog output: %s Error %d.\n", message, -result);
//    }
//    else
//    {
//        printf("Analog Input: %f  -> Analog Output: %f\n", analog_input_buffer[0], analog_output_buffer[0]);
//    }
//}



static void
limit_current(struct limiter_state* current_limiter, double dt, double* motor_current)
{
    static const double current_limit_1 = LIMIT_1_SMALL;
    static const double current_limit_2 = LIMIT_2_SMALL;
    static const double timeout_1 = TIMEOUT_1_SMALL;
    static const double timeout_2 = TIMEOUT_2_SMALL;

    double value = fabs(*motor_current);

    switch (current_limiter->state)
    {
    case 0: // Limiting to the upper limits and waiting for input to exceed lower limits
        if (value > current_limit_2)
        {
            current_limiter->state = 1;
            current_limiter->time = 0;
            current_limiter->count = 0;
            current_limiter->mean = value;

            if (value > current_limit_1)
                *motor_current = (*motor_current > 0) ? current_limit_1 : -current_limit_1;
        }
        break;

    case 1: // Limiting to the upper limits and waiting for peak time limit
        current_limiter->mean += value;
        current_limiter->time += dt;
        current_limiter->count++;

        if (current_limiter->time >= timeout_1)
        {
            current_limiter->mean /= current_limiter->count + 1;
            if (current_limiter->mean > current_limit_2)
            {
                current_limiter->state = 2;
                current_limiter->time = 0;
                current_limiter->count = 0;
                *motor_current = (*motor_current > 0) ? current_limit_2 : -current_limit_2;
            }
            else
            {
                current_limiter->state = 0;
            }
        }
        else if (value > current_limit_1)
            *motor_current = (*motor_current > 0) ? current_limit_1 : -current_limit_1;
        break;

    case 2: // Limiting to the lower limits and waiting for motor to recover
        current_limiter->time += dt;
        current_limiter->count++;

        if (current_limiter->time >= timeout_2)
        {
            if (value > current_limit_2)
            {
                current_limiter->state = 1;
                current_limiter->time = 0;
                current_limiter->count = 0;
                current_limiter->mean = value;

                if (value > current_limit_1)
                    *motor_current = (*motor_current > 0) ? current_limit_1 : -current_limit_1;
            }
            else
            {
                current_limiter->state = 0;
            }
        }
        else if (value > current_limit_2)
            *motor_current = (*motor_current > 0) ? current_limit_2 : -current_limit_2;
        break;
    }
}

/*
    Determine the motor current in amps needed to produce the specified joint torque in N-m.
*/
static double
joint_torque_to_motor_current(double joint_torque)
{
    static const double torque_constant = KT_SMALL;
    return joint_torque / torque_constant; // convert N-m to amps
}

/*
    Determine the voltage with which we need to drive the current amplifier
    to get the motor current desired.
*/
static double
motor_current_to_output_voltage(double motor_current)
{
    return 0.5 * motor_current;
}

/*
    Disable the motor amplifier and drive the motor with zero current.
*/
static void
disable_motor(t_card board)
{
    static const t_double  zero_torque = 0;
    static const t_boolean disable_amplifier = 0;

    /* Make sure the amplifier is disabled and the motor programmed for zero torque */
    hil_set_digital_directions(board, NULL, 0, &digital_channel, 1);   /* program digital I/O line as output */
    hil_write_digital(board, &digital_channel, 1, &disable_amplifier); /* disable amplifier */
    //hil_write_analog(board, &analog_channel, 1, &zero_torque);         /* drive the motor with 0 V */

}

/*
    Enable the motor amplifier.
*/
static void
enable_motor(t_card board)
{
    static const t_boolean enable_amplifier = 1;

    /* Enable the amplifier */
    hil_write_digital(board, &digital_channel, 1, &enable_amplifier); /* enable amplifier */
}

/*
    Stop the controller for the motor.
*/
static void
stop_controller(t_card board, t_task task)
{
    /* Stop the controller */
    hil_task_stop(task);

    /* Turn off motor */
    disable_motor(board);

    /* Delete the task */
    hil_task_delete(task);

    printf("The motor has been disabled and the controller has been stopped.\n");
}

/*
    Start the controller for the motor.
*/
static int
start_controller(t_card board, t_task* task_pointer)
{
    static const t_uint32 samples = -1; /* read continuously */
    const t_uint32        samples_in_buffer = (t_uint32)(0.1 * frequency);

    qsched_param_t scheduling_parameters;
    int result;

    /* Create a task to read the encoder at regular intervals */
    result = hil_task_create_encoder_reader(board, samples_in_buffer, &analog_channel, 1, task_pointer);
    if (result == 0)
    {
        enable_motor(board);

        printf("The motor has been enabled. Starting the controller...\n");

        /* Bump up the priority of our thread to minimize latencies */
        scheduling_parameters.sched_priority = qsched_get_priority_max(QSCHED_FIFO);
        qthread_setschedparam(qthread_self(), QSCHED_FIFO, &scheduling_parameters);

        /* Start the controller at the chosen sampling rate. */
        result = hil_task_start(*task_pointer, SYSTEM_CLOCK_1, frequency, samples);
        if (result == 0)
            return 1;
        else
        {
            msg_get_error_message(NULL, result, message, ARRAY_LENGTH(message));
            printf("Unable to start task. %s Error %d.\n", message, -result);
        }

        /* Disable the amplifier and delete the task */
        stop_controller(board, *task_pointer);
    }
    else
    {
        msg_get_error_message(NULL, result, message, ARRAY_LENGTH(message));
        printf("Unable to create a task. %s Error %d.\n", message, -result);
    }

    return 0;
}

/*
    Drive the motor to produce the desired torque.
    Motor current is dynamically limited heuristically to prevent overheating of the motor
    while continuing to provide peak torque.
*/
static void
generate_torque(t_card board, struct limiter_state* current_limiter, double period, double desired_torque)
{
    double motor_current;      /* motor current in amps */
    double output_voltage;     /* output voltage in volts */

    /* Compute the motor current needed to produce the desired torque */
    motor_current = joint_torque_to_motor_current(desired_torque);

    /* Limit motor current to prevent overheating */
    limit_current(current_limiter, period, &motor_current);

    /* Compute output voltage required to produce the motor current */
    output_voltage = motor_current_to_output_voltage(motor_current);

    /* Write the voltage to the output */
    //hil_write_analog(board, &analog_channel, 1, &output_voltage);
}

/*
    The main entry point for the program.
*/
int main(int argc, char* argv[])
{
    qsigaction_t action;
    t_card board;
    t_int result;

    action.sa_handler = signal_handler;
    action.sa_flags = 0;
    qsigemptyset(&action.sa_mask);
    qsigaction(SIGINT, &action, NULL);

    printf("This example reads encoder values at %g Hz.\n", frequency);

    result = hil_open(board_type, board_identifier, &board);
    if (result == 0)
    {
        printf("Press CTRL-C to stop the controller.\n\n");

        const t_double period = 1.0 / frequency;
        t_task task;

        if (start_controller(board, &task))
        {
            t_int32 counts[MOTORS];
            double theta[MOTORS];

            while (stop == 0)
            {
                read_encoder(board, counts);
                encoder_counts_to_theta(counts, theta);
                printf("Encoder Counts: [%d, %d]  Theta: [%.6f, %.6f] rad\n",
                counts[0], counts[1], theta[0], theta[1]);
            }

            stop_controller(board, task);
            printf("\nController has been stopped. Press Enter to continue.\n");
            (void)getchar();
        }

        hil_close(board);
    }
    else
    {
        msg_get_error_message(NULL, result, message, sizeof(message));
        printf("Unable to open board. %s Error %d.\n", message, -result);
    }

    return 0;
}
