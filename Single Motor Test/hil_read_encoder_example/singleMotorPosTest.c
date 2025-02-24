//////////////////////////////////////////////////////////////////
//
// singleMotorPosTest.c - Improved C file
//
// C file for single motor control by writing a signal to the analog
// output channel for the LCAM
//
// This example demonstrates the use of the following functions:
//    hil_open
//    hil_write_analog
//    hil_read_encoder
//    hil_close
//
// Copyright (C) 2008 Quanser Inc.
//////////////////////////////////////////////////////////////////

#include "hil_read_encoder_example.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"
#include "quanser_thread.h"

#define BOARD_TYPE "q8_usb"
#define BOARD_ID "0"
#define NUM_CHANNELS 1

int main(int argc, char* argv[])
{
    t_card board;
    t_int result;
    char message[512];

    const t_uint32 channels[NUM_CHANNELS] = { 0 }; // analog channel 0 
    t_double voltages[NUM_CHANNELS];
    t_int32 counts[NUM_CHANNELS];
    t_int32 initial_count;
    int first_reading = 1;

    // Proportional control parameters
    const t_uint32 samples = -1; /* read continuously */
    const t_uint32 analog_channel = 0;
    const t_uint32 encoder_channel = 0;
    const t_double frequency = 1000;
    const t_double sine_frequency = 0.5; /* frequency of command signal */
    const t_uint32 samples_in_buffer = (t_uint32)(0.1 * frequency);
    const t_double period = 1.0 / frequency;

    t_int32  count;
    t_double voltage;
    t_int    samples_read;
    t_task   task;

    // Prevent Ctrl+C from stopping the application so hil_close gets called
    qsigaction_t action;
    action.sa_handler = SIG_IGN;
    action.sa_flags = 0;
    qsigemptyset(&action.sa_mask);
    qsigaction(SIGINT, &action, NULL);

    // Open the board
    result = hil_open(BOARD_TYPE, BOARD_ID, &board);
    if (result < 0) {
        msg_get_error_message(NULL, result, message, sizeof(message));
        printf("Error: Unable to open board. %s (Error %d)\n", message, -result);
        return 1;
    }

    printf("Single motor position control.\n");

    count = 0;
    result = hil_set_encoder_counts(board, &encoder_channel, 1, &count);


    // set output voltage to send to analog out
    voltages[NUM_CHANNELS - 1] = 1.0; // set channel 0 to 1V

    // write voltage to analog out
    result = hil_write_analog(board, channels, NUM_CHANNELS, voltages);
    if (result < 0)
    {
        msg_get_error_message(NULL, result, message, sizeof(message));
        printf("Unable to write channels. %s Error %d.\n", message, -result);
    }

    // Read encoder value in a loop
    while (1) {
        result = hil_read_encoder(board, channels, NUM_CHANNELS, counts);
        if (result < 0) {
            msg_get_error_message(NULL, result, message, sizeof(message));
            printf("Error: Unable to read channel 0. %s (Error %d)\n", message, -result);
            break;
        }

        if (first_reading) {
            initial_count = counts[0];
            first_reading = 0;
        }

        printf("ENC #0: %5d (Relative: %5d)\n", counts[0], counts[0] - initial_count);

        // Exit when 'q' is pressed
        if (getchar() == 'q') break;
    }

    // Close the board
    hil_close(board);
    printf("\nPress Enter to exit.\n");
    getchar();

    return 0;
}
