#include "hil_read_encoder_example.h"
#include <iostream>
#include <string>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <conio.h> // For kbhit() and getch() on Windows
#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"
#include "quanser_thread.h"

constexpr const char* BOARD_TYPE = "q8_usb";
constexpr const char* BOARD_ID = "0";
constexpr int NUM_CHANNELS = 1;

// Global variables for cleanup
t_card board;
t_double voltages[NUM_CHANNELS] = { 0.0 }; // Initialize voltage to 0V

// Signal handler to set voltage to 0V on program termination
void signalHandler(int signal) {
    std::cout << "\nCaught signal " << signal << ". Setting voltage to 0V and closing board.\n";
    hil_write_analog(board, nullptr, NUM_CHANNELS, voltages); // Write 0V to analog output
    hil_close(board); // Close the board
    std::exit(signal); // Exit the program
}

int main(int argc, char* argv[])
{
    t_int result;
    std::string message(512, '\0');

    const t_uint32 channels[NUM_CHANNELS] = { 0 }; // Analog channel 0
    t_int32 counts[NUM_CHANNELS];
    t_int32 initial_count = 0;
    bool first_reading = true;
    t_double degrees[NUM_CHANNELS];

    // Proportional control parameters
    constexpr t_uint32 samples = -1; // Read continuously
    constexpr t_uint32 analog_channel = 0;
    constexpr t_uint32 encoder_channel = 0;
    constexpr t_double frequency = 1000;
    constexpr t_double sine_frequency = 0.5; // Frequency of command signal
    constexpr t_uint32 samples_in_buffer = static_cast<t_uint32>(0.1 * frequency);
    constexpr t_double period = 1.0 / frequency;

    t_int32 count = 0;
    t_double voltage = 0.0;
    t_int samples_read = 0;
    t_task task;

    // Register signal handlers for clean exit
    std::signal(SIGINT, signalHandler);  // Ctrl+C
    std::signal(SIGTERM, signalHandler); // Termination signal

    // Open the board
    result = hil_open(BOARD_TYPE, BOARD_ID, &board);
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Error: Unable to open board. " << message << " (Error " << -result << ")\n";
        return 1;
    }

    std::cout << "Single motor position control.\n";

    // Set encoder count to 0
    result = hil_set_encoder_counts(board, &encoder_channel, 1, &count);
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Error: Unable to set encoder counts. " << message << " (Error " << -result << ")\n";
        hil_close(board);
        return 1;
    }

    // Set output voltage to send to analog out
    voltages[NUM_CHANNELS - 1] = 1.0; // Set channel 0 to 1V

    // Write voltage to analog out
    result = hil_write_analog(board, channels, NUM_CHANNELS, voltages);
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Unable to write channels. " << message << " Error " << -result << ".\n";
    }

    // Read encoder value in a loop
    while (true) {
        result = hil_read_encoder(board, channels, NUM_CHANNELS, counts);
        if (result < 0) {
            msg_get_error_message(nullptr, result, &message[0], message.size());
            std::cerr << "Error: Unable to read channel 0. " << message << " (Error " << -result << ")\n";
            break;
        }

        if (first_reading) {
            initial_count = counts[0];
            first_reading = false;
        }

        degrees[0] = static_cast<double>(counts[0]) * (360.0 / (5000.0 * 4.0));
        std::cout << "ENC #0: " << degrees[0] << " degrees, " << counts[0] << " counts\n";

        // Check for 'q' key press (non-blocking)
        if (_kbhit()) { // Check if a key is pressed
            char key = _getch(); // Get the pressed key
            if (key == 'q' || key == 'Q') {
                break; // Exit the loop if 'q' is pressed
            }
        }
    }

    // Cleanup: Set voltage to 0V and close the board
    voltages[NUM_CHANNELS - 1] = 0.0; // Set channel 0 to 0V
    result = hil_write_analog(board, channels, NUM_CHANNELS, voltages);
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Error: Unable to write 0V to channels. " << message << " (Error " << -result << ")\n";
    }

    hil_close(board);
    std::cout << "\nPress Enter to exit.\n";
    std::cin.ignore();
    std::cin.get();

    return 0;
}