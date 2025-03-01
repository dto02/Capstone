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
#include <tuple>
#include <chrono>
#include <thread>

// kinematics header file
#include "transformations.h"

using t_int32 = int;
using namespace std::chrono_literals;

constexpr const char* BOARD_TYPE = "q8_usb";
constexpr const char* BOARD_ID = "0";
constexpr int NUM_CHANNELS = 6;

// Global variables for cleanup
t_card board;
t_double voltages[NUM_CHANNELS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Initialize voltage to 0V

// Signal handler to set voltage to 0V on program termination
void signalHandler(int signal) {
    std::cout << "\nCaught signal " << signal << ". Setting voltage to 0V and closing board.\n";
    hil_write_analog(board, nullptr, NUM_CHANNELS, voltages); // Write 0V to analog output
    hil_close(board); // Close the board
    std::exit(signal); // Exit the program
}

std::tuple<int, int, int> fKinematics(const t_double degrees[NUM_CHANNELS]) {
    int x = 1;
    int y = 2;
    int z = 3;

    // Return tuple
    return std::make_tuple(x, y, z);
}

int main(int argc, char* argv[])
{
    t_int result;
    std::string message(512, '\0');

    const t_uint32 channels[NUM_CHANNELS] = { 0, 1, 2, 3, 4, 5};
    /*t_int32 counts[NUM_CHANNELS];*/
    t_int32 initial_count = 0;
    bool first_reading = true;
    t_double degrees[NUM_CHANNELS];

    std::vector<uint32_t> encoder_channels = { 0, 1, 2, 3, 4, 5};  // uint32_t for channels
    std::vector<int32_t> counts(encoder_channels.size(), 0);        // int32_t for counts

    t_double voltage = 0.0;
    t_int samples_read = 0;
    t_task task;

    // Register signal handlers for clean exit
    std::signal(SIGINT, signalHandler);  // Ctrl+C
    std::signal(SIGTERM, signalHandler); // Termination signal

    //// Open the board
    //result = hil_open(BOARD_TYPE, BOARD_ID, &board);
    //if (result < 0) {
    //    msg_get_error_message(nullptr, result, &message[0], message.size());
    //    std::cerr << "Error: Unable to open board. " << message << " (Error " << -result << ")\n";
    //    return 1;
    //}

    //std::cout << "Position Sensing.\n";

    // //Set encoder count to 0
    //result = hil_set_encoder_counts(board, encoder_channels.data(), encoder_channels.size(), counts.data());
    //if (result < 0) {
    //    msg_get_error_message(nullptr, result, &message[0], message.size());
    //    std::cerr << "Error: Unable to set encoder counts. " << message << " (Error " << -result << ")\n";
    //    hil_close(board);
    //    return 1;
    //}

    //// Set output voltage to send to analog out
    //for (int i = 0; i < NUM_CHANNELS; i++) {
    //    voltages[NUM_CHANNELS - i] = 1.0; // Set channel 0 to 1V
    //}

    //// Write voltage to analog out
    //result = hil_write_analog(board, channels, NUM_CHANNELS, voltages);
    //if (result < 0) {
    //    msg_get_error_message(nullptr, result, &message[0], message.size());
    //    std::cerr << "Unable to write channels. " << message << " Error " << -result << ".\n";
    //}

    std::vector<double> transformedXYZ = computeTransformedXYZ(
        90.0, 45.0, 135.0, 90.0, 45.0, 135.0, 0.0);

    // Print the transformed coordinates
    std::cout << "Transformed XYZ: [";
    for (size_t i = 0; i < transformedXYZ.size(); ++i) {
        std::cout << transformedXYZ[i];
        if (i < transformedXYZ.size() - 1) std::cout << ", ";
    }
    std::cout << "]\n";

    // Read encoder value in a loop
    while (true) {
 /*       result = hil_read_encoder(board, channels, NUM_CHANNELS, counts.data());
        if (result < 0) {
            msg_get_error_message(nullptr, result, &message[0], message.size());
            std::cerr << "Error: Unable to read channel 0. " << message << " (Error " << -result << ")\n";
            break;
        }

        if (first_reading) {
            initial_count = counts[0];
            first_reading = false;
        }

        for (int i = 0; i < NUM_CHANNELS; i++) {
            degrees[i] = static_cast<double>(counts[i]) * (360.0 / (5000.0 * 4.0));
            std::cout << "ENC #" << i << ": " << degrees[i] << " degrees, " << counts[i] << " counts\n";
        }*/

        //
        // Call computeTransformedXYZ with 6 encoder angles and 1 EE encoder (set to 0)
        //std::vector<double> transformedXYZ = computeTransformedXYZ(
        //    90.0, 0.0, 180.0
        //    //90.0, 45.0, 135.0, 90.0, 45.0, 135.0, 0.0);

        //// Print the transformed coordinates
        //std::cout << "Transformed XYZ: [";
        //for (size_t i = 0; i < transformedXYZ.size(); ++i) {
        //    std::cout << transformedXYZ[i];
        //    if (i < transformedXYZ.size() - 1) std::cout << ", ";
        //}
        //std::cout << "]\n";
        //std::this_thread::sleep_for(1s);


        ////Position Control Forward Kinematics Function
        //std::tuple<int, int, int> result = fKinematics(degrees);

        //// Get values from the tuple
        //int x, y, z;
        //std::tie(x, y, z) = result;
        //std::cout << "x: " << x << ", y: " << y << ", z: " << z << std::endl;

        //std::cout << "]\n";
        //// Check for 'q' key press (non-blocking)
        //if (_kbhit()) { // Check if a key is pressed
        //    char key = _getch(); // Get the pressed key
        //    if (key == 'q' || key == 'Q') {
        //        break; // Exit the loop if 'q' is pressed
        //    }
        //}
    }


    // Cleanup: Set voltage to 0V and close the board
    for (int i = 0; i < NUM_CHANNELS; i++) {
        voltages[i] = 0.0;
    }

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

