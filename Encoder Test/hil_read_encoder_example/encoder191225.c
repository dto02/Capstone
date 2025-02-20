//////////////////////////////////////////////////////////////////
//
// hil_read_encoder_example.c - Improved C file
//
// This example reads one sample immediately from encoder input channel 0.
//
// This example demonstrates the use of the following functions:
//    hil_open
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
    const t_uint32 channels[NUM_CHANNELS] = { 0 };
    t_int32 counts[NUM_CHANNELS];
    t_int32 initial_count;
    int first_reading = 1;

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

    printf("Reading encoder value from channel 0. Press Ctrl+C to exit.\n");

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
    }

    // Close the board
    hil_close(board);
    printf("\nPress Enter to exit.\n");
    getchar();
    return 0;
}
