#include <iostream>
#include <string>
#include <cstdlib>   // For system()
#include <sstream>   // For parsing output from Python
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
#include <fcntl.h>
#include <cstring>
#include <chrono>
#include <thread>
#include <windows.h>

#define ENCODER_SHM_NAME "encoder_shm"  // Shared memory for encoder positions (Python writes, C++ reads)
#define TORQUE_SHM_NAME "torque_shm"    // Shared memory for torque commands (C++ writes, Python reads)
#define SHM_SIZE 4  // 4 bytes for float

// kinematics header file
#include "transformations.h"

using t_int32 = int;
using namespace std::chrono_literals;

constexpr const char* BOARD_TYPE = "q8_usb";
constexpr const char* BOARD_ID = "0";
constexpr int NUM_CHANNELS = 6;

// Global variables for cleanup
t_card board;
t_double voltages[NUM_CHANNELS] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }; // Initialize voltage to 0V

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

    const t_uint32 channels[NUM_CHANNELS] = { 0, 1, 2, 3, 4, 5 };
    /*t_int32 counts[NUM_CHANNELS];*/
    t_int32 initial_count = 0;
    bool first_reading = true;
    t_double degrees[NUM_CHANNELS];

    std::vector<uint32_t> encoder_channels = { 0, 1, 2, 3, 4, 5 };  // uint32_t for channels
    std::vector<int32_t> counts(encoder_channels.size(), 0);        // int32_t for counts

    t_double voltage = 0.0;
    t_int samples_read = 0;
    t_task task;

    // Create shared memory for torque commands (C++ writes)
    HANDLE hTorqueMapFile = CreateFileMapping(
        INVALID_HANDLE_VALUE,  // Use the paging file
        nullptr,              // Default security
        PAGE_READWRITE,       // Read/write access
        0,                    // Maximum object size (high-order DWORD)
        SHM_SIZE,             // Maximum object size (low-order DWORD)
        TORQUE_SHM_NAME       // Name of mapping object
    );

    if (hTorqueMapFile == nullptr) {
        std::cerr << "Failed to create torque shared memory: " << GetLastError() << std::endl;
        return 1;
    }

    // Map the torque shared memory
    void* pTorqueBuf = MapViewOfFile(
        hTorqueMapFile,       // Handle to map object
        FILE_MAP_ALL_ACCESS,  // Read/write permission
        0,
        0,
        SHM_SIZE
    );

    if (pTorqueBuf == nullptr) {
        std::cerr << "Failed to map torque shared memory: " << GetLastError() << std::endl;
        CloseHandle(hTorqueMapFile);
        return 1;
    }

    // Open shared memory for encoder positions (Python writes, C++ reads)
    HANDLE hEncoderMapFile = nullptr;
    while (hEncoderMapFile == nullptr) {
        hEncoderMapFile = OpenFileMappingA(FILE_MAP_READ, FALSE, ENCODER_SHM_NAME);
        if (!hEncoderMapFile) {
            std::cerr << "Waiting for encoder shared memory to be created..." << std::endl;
            std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Retry every 500ms
        }
    }

    std::cout << "Encoder shared memory opened!" << std::endl;

    // Map the encoder shared memory
    void* pEncoderBuf = MapViewOfFile(
        hEncoderMapFile,      // Handle to map object
        FILE_MAP_READ,        // Read-only permission
        0,
        0,
        SHM_SIZE
    );

    if (pEncoderBuf == nullptr) {
        std::cerr << "Failed to map encoder shared memory: " << GetLastError() << std::endl;
        CloseHandle(hEncoderMapFile);
        CloseHandle(hTorqueMapFile);
        return 1;
    }

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

    // Set encoder count to 0
    result = hil_set_encoder_counts(board, encoder_channels.data(), encoder_channels.size(), counts.data());
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Error: Unable to set encoder counts. " << message << " (Error " << -result << ")\n";
        hil_close(board);
        return 1;
    }

    // Main loop
    while (true) {
        // Read encoder values
        result = hil_read_encoder(board, channels, NUM_CHANNELS, counts.data());
        if (result < 0) {
            msg_get_error_message(nullptr, result, &message[0], message.size());
            std::cerr << "Error: Unable to read encoder values. " << message << " (Error " << -result << ")\n";
            break;
        }

        // Convert encoder counts to degrees
        for (int i = 0; i < NUM_CHANNELS; i++) {
            degrees[i] = static_cast<double>(counts[i]) * (360.0 / (5000.0 * 4.0));
        }

        // Print encoder values
        for (int i = 0; i < NUM_CHANNELS; i++) {
            std::cout << "ENC #" << i << ": " << degrees[i] << " degrees, " << counts[i] << " counts\n";
        }

        // Read the encoder position from shared memory
        float encoder_position;
        std::memcpy(&encoder_position, pEncoderBuf, sizeof(float));
        std::cout << "Encoder Position: " << encoder_position << " degrees" << std::endl;

        // Call computeTransformedXYZ with 6 encoder angles and 1 EE encoder (set to 0)
        std::vector<double> transformedXYZ = computeTransformedXYZ(
            degrees[0] + 90, degrees[1] + 45, degrees[2] + 135, degrees[3] + 90, degrees[4] + 45, degrees[5] + 135, encoder_position);

        // Print the transformed coordinates
        std::cout << "Transformed XYZ: [";
        for (size_t i = 0; i < transformedXYZ.size(); ++i) {
            std::cout << transformedXYZ[i];
            if (i < transformedXYZ.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n";
        std::this_thread::sleep_for(1s);

        // Check for 't' key press (non-blocking)
        if (_kbhit()) {
            char key = _getch(); // Get the pressed key
            if (key == 't' || key == 'T') {
                // Prompt for torque input
                float torque_command;
                std::cout << "Enter torque command (keep under 0.038Nm): ";
                std::cin >> torque_command;9

                // Write the torque command to shared memory
                std::memcpy(pTorqueBuf, &torque_command, sizeof(float));
                std::cout << "Torque command written to shared memory: " << torque_command << std::endl;
            }
        }

        // Sleep for a short time to avoid busy-waiting
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Cleanup
    UnmapViewOfFile(pTorqueBuf);
    UnmapViewOfFile(pEncoderBuf);
    CloseHandle(hTorqueMapFile);
    CloseHandle(hEncoderMapFile);

    // Set voltage to 0V and close the board
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