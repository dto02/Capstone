#include "hil.h"
#include "quanser_signal.h"
#include "quanser_messages.h"
#include "quanser_thread.h"
#include "hil_read_encoder_example.h"
#include <conio.h> // For kbhit() and getch() on Windows
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <mutex>

// kinematics header file
#include "transformations.h"

using t_int32 = int;
using namespace std::chrono_literals;

// Constants
constexpr const char* BOARD_TYPE = "q8_usb";
constexpr const char* BOARD_ID = "0";
constexpr int NUM_CHANNELS = 6;

#define ENCODER_SHM_NAME "encoder_shm"  // Shared memory for encoder positions (Python writes, C++ reads)
#define TORQUE_SHM_NAME "torque_shm"    // Shared memory for torque commands (C++ writes, Python reads)
#define SHM_SIZE 4                      // 4 bytes for float
#define GRID_SIZE 10                    // Grid size for OpenGL rendering
#define VERTICAL_GRID_SIZE 5            // Vertical grid lines

// Global variables
t_card board;                           // Quanser board handle
t_double voltages[NUM_CHANNELS] = { 0.0 }; // Initialize voltage to 0V
volatile bool running = true;           // Flag for graceful exit
unsigned int boxVAO, boxVBO, boxEBO;    // OpenGL objects for the cube
unsigned int gridVAO, gridVBO;          // OpenGL objects for the grid
glm::vec3 cubePosition(0.0f, 0.0f, 0.0f); // Cube position in 3D space
glm::mat4 cubeRotation = glm::mat4(1.0f); // Cube rotation matrix (identity initially)
std::mutex cubeMutex;                   // Mutex to protect cubePosition and cubeRotation

// Vertex Shader
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
out vec3 fragColor;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
    fragColor = aColor; // Pass color to fragment shader
}
)";

// Fragment Shader
const char* gridFragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;
void main() {
    FragColor = vec4(1.0, 1.0, 1.0, 1.0); // Force white color for the grid
}
)";

const char* cubeFragmentShaderSource = R"(
#version 330 core
in vec3 fragColor;
out vec4 FragColor;
void main() {
    FragColor = vec4(fragColor, 1.0); // Use interpolated color for the cube
}
)";

// Compile OpenGL shader
unsigned int compileShader(unsigned int type, const char* source) {
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);

    int success;
    char infoLog[512];
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(shader, 512, nullptr, infoLog);
        std::cerr << "Shader Compilation Error: " << infoLog << std::endl;
    }

    return shader;
}

void generateGrid(float* gridVertices) {
    int index = 0;
    float length = static_cast<float>(GRID_SIZE);

    // Horizontal grid lines (X and Z axes)
    for (int i = -GRID_SIZE; i <= GRID_SIZE; i++) {
        gridVertices[index++] = static_cast<float>(i); gridVertices[index++] = 1.0f; gridVertices[index++] = -length;
        gridVertices[index++] = static_cast<float>(i); gridVertices[index++] = 1.0f; gridVertices[index++] = length;

        gridVertices[index++] = -length; gridVertices[index++] = 1.0f; gridVertices[index++] = static_cast<float>(i);
        gridVertices[index++] = length; gridVertices[index++] = 1.0f; gridVertices[index++] = static_cast<float>(i);
    }

    // Vertical grid lines (Y-axis)
    for (int i = 1; i <= VERTICAL_GRID_SIZE; i++) {
        float y = -static_cast<float>(i); // Extend downward
        gridVertices[index++] = -length; gridVertices[index++] = y; gridVertices[index++] = -length;
        gridVertices[index++] = length; gridVertices[index++] = y; gridVertices[index++] = -length;

        gridVertices[index++] = -length; gridVertices[index++] = y; gridVertices[index++] = length;
        gridVertices[index++] = length; gridVertices[index++] = y; gridVertices[index++] = length;

        gridVertices[index++] = -length; gridVertices[index++] = y; gridVertices[index++] = -length;
        gridVertices[index++] = -length; gridVertices[index++] = y; gridVertices[index++] = length;

        gridVertices[index++] = length; gridVertices[index++] = y; gridVertices[index++] = -length;
        gridVertices[index++] = length; gridVertices[index++] = y; gridVertices[index++] = length;
    }
}

// Render Cube
void renderCube(const glm::mat4& model, unsigned int shaderProgram) {
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glBindVertexArray(boxVAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}

// Signal handler to set voltage to 0V on program termination
void signalHandler(int signal) {
    std::cout << "\nCaught signal " << signal << ". Setting voltage to 0V and closing board.\n";
    hil_write_analog(board, nullptr, NUM_CHANNELS, voltages); // Write 0V to analog output
    hil_close(board); // Close the board
    running = false;  // Signal threads to stop
    std::exit(signal); // Exit the program
}

// Encoder reading loop (runs in a separate thread)
void readEncoderLoop(t_card board, const t_uint32* channels, std::vector<int32_t>& counts, glm::vec3& cubePosition, glm::mat4& cubeRotation, std::mutex& cubeMutex) {
    t_double degrees[NUM_CHANNELS];
    bool first_reading = true;
    t_int32 initial_count = 0;

    while (running) {
        t_int result = hil_read_encoder(board, channels, NUM_CHANNELS, counts.data());
        if (result < 0) {
            std::cerr << "Error: Unable to read encoder values.\n";
            break;
        }

        if (first_reading) {
            initial_count = counts[0];
            first_reading = false;
        }

        for (int i = 0; i < NUM_CHANNELS; i++) {
            degrees[i] = static_cast<double>(counts[i]) * (360.0 / (5000.0 * 4.0));
        }

        degrees[0] = degrees[0] + 90;
        degrees[1] = degrees[1];
        degrees[2] = degrees[2] + 180;
        degrees[3] = 90 - degrees[3];
        degrees[4] = degrees[4];
        degrees[5] = degrees[5] + 180;

        // Call computeTransformedXYZ with 6 encoder angles and 1 EE encoder (set to 0)
        std::vector<double> transformedXYZ = computeTransformedXYZ(
            degrees[0], degrees[1], degrees[2], degrees[3], degrees[4], degrees[5], -58.5);

        // Print the transformed XYZ coordinates
        std::cout << "Transformed XYZ: [";
        for (size_t i = 0; i < transformedXYZ.size(); ++i) {
            std::cout << transformedXYZ[i];
            if (i < transformedXYZ.size() - 1) std::cout << ", ";
        }
        std::cout << "]\n";

        // Update cubePosition and cubeRotation
        if (transformedXYZ.size() >= 3) {
            std::lock_guard<std::mutex> lock(cubeMutex); // Lock the mutex

            // Update position
            cubePosition.x = (float)(-16.4 / 42 * transformedXYZ[1] - 3.5);
            cubePosition.y = (float)(7.5 / 26 * (transformedXYZ[2] - 7.5));
            cubePosition.z = (float)(-16.4 / 42 * transformedXYZ[0] - 3.5);

            // Update rotation for X, Y, and Z axes
            float rotationAngleX = glm::radians(static_cast<float>(transformedXYZ[4])); // Rotate around X-axis
            float rotationAngleY = glm::radians(static_cast<float>(transformedXYZ[5])); // Rotate around Y-axis
            float rotationAngleZ = glm::radians(static_cast<float>(-1.0 * transformedXYZ[3])); // Rotate around Z-axis

            // Combine rotations into a single rotation matrix
            cubeRotation = glm::rotate(glm::mat4(1.0f), rotationAngleX, glm::vec3(1.0f, 0.0f, 0.0f)) * // X-axis
                glm::rotate(glm::mat4(1.0f), rotationAngleY, glm::vec3(0.0f, 1.0f, 0.0f)) * // Y-axis
                glm::rotate(glm::mat4(1.0f), rotationAngleZ, glm::vec3(0.0f, 0.0f, 1.0f));  // Z-axis
        }

        // Simulate delay
        std::this_thread::sleep_for(std::chrono::milliseconds(25)); // Adjust delay as needed
    }
}

int main(int argc, char* argv[]) {
    t_int result;
    std::string message(512, '\0');

    const t_uint32 channels[NUM_CHANNELS] = { 0, 1, 2, 3, 4, 5 };
    std::vector<int32_t> counts(NUM_CHANNELS, 0); // Initialize counts to 0

    // Register signal handlers for clean exit
    std::signal(SIGINT, signalHandler);  // Ctrl+C
    std::signal(SIGTERM, signalHandler); // Termination signal

    // Initialize GLFW
    if (!glfwInit()) {
        std::cerr << "Failed to initialize GLFW\n";
        return 1;
    }

    // Create OpenGL window
    GLFWwindow* window = glfwCreateWindow(1600, 1200, "Moving Box with Isometric Grid", nullptr, nullptr);
    if (!window) {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return 1;
    }
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Compile and link grid shader program
    unsigned int gridVertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    unsigned int gridFragmentShader = compileShader(GL_FRAGMENT_SHADER, gridFragmentShaderSource);
    unsigned int gridShaderProgram = glCreateProgram();
    glAttachShader(gridShaderProgram, gridVertexShader);
    glAttachShader(gridShaderProgram, gridFragmentShader);
    glLinkProgram(gridShaderProgram);

    // Compile and link cube shader program
    unsigned int cubeVertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    unsigned int cubeFragmentShader = compileShader(GL_FRAGMENT_SHADER, cubeFragmentShaderSource);
    unsigned int cubeShaderProgram = glCreateProgram();
    glAttachShader(cubeShaderProgram, cubeVertexShader);
    glAttachShader(cubeShaderProgram, cubeFragmentShader);
    glLinkProgram(cubeShaderProgram);

    // Generate grid vertices
    float gridVertices[(GRID_SIZE * 4 + 4 + VERTICAL_GRID_SIZE * 4) * 3];
    generateGrid(gridVertices);

    // Create grid VAO/VBO
    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gridVertices), gridVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Cube vertices and indices
    float boxVertices[] = {
        // Positions          // Colors (RGB)
        // Bottom face (Green)
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 1.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  0.0f, 1.0f, 0.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 1.0f, 0.0f,

        // Top face (Yellow)
        -0.5f,  0.5f, -0.5f,  1.0f, 1.0f, 0.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 1.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 1.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  1.0f, 1.0f, 0.0f,

        // Front face (Red)
        -0.5f, -0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.0f,
        -0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 0.0f,

        // Back face (Blue)
        -0.5f, -0.5f, -0.5f,  0.0f, 0.0f, 1.0f,
         0.5f, -0.5f, -0.5f,  0.0f, 0.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  0.0f, 0.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 0.0f, 1.0f,

        // Left face (Cyan)
        -0.5f, -0.5f, -0.5f,  0.0f, 1.0f, 1.0f,
        -0.5f, -0.5f,  0.5f,  0.0f, 1.0f, 1.0f,
        -0.5f,  0.5f,  0.5f,  0.0f, 1.0f, 1.0f,
        -0.5f,  0.5f, -0.5f,  0.0f, 1.0f, 1.0f,

        // Right face (Magenta)
         0.5f, -0.5f, -0.5f,  1.0f, 0.0f, 1.0f,
         0.5f, -0.5f,  0.5f,  1.0f, 0.0f, 1.0f,
         0.5f,  0.5f,  0.5f,  1.0f, 0.0f, 1.0f,
         0.5f,  0.5f, -0.5f,  1.0f, 0.0f, 1.0f
    };
    unsigned int boxIndices[] = {
        // Bottom face
        0, 1, 2, 2, 3, 0,
        // Top face
        4, 5, 6, 6, 7, 4,
        // Front face
        8, 9, 10, 10, 11, 8,
        // Back face
        12, 13, 14, 14, 15, 12,
        // Left face
        16, 17, 18, 18, 19, 16,
        // Right face
        20, 21, 22, 22, 23, 20
    };

    // Create cube VAO/VBO/EBO
    glGenVertexArrays(1, &boxVAO);
    glGenBuffers(1, &boxVBO);
    glGenBuffers(1, &boxEBO);
    glBindVertexArray(boxVAO);

    // Bind and set vertex buffer
    glBindBuffer(GL_ARRAY_BUFFER, boxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(boxVertices), boxVertices, GL_STATIC_DRAW);

    // Bind and set element buffer
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boxEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(boxIndices), boxIndices, GL_STATIC_DRAW);

    // Position attribute
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Color attribute
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);

    // Camera setup
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1600.0f / 1200.0f, 0.1f, 100.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(10.0f, 10.0f, 10.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    // Open the board
    result = hil_open(BOARD_TYPE, BOARD_ID, &board);
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Error: Unable to open board. " << message << " (Error " << -result << ")\n";
        return 1;
    }

    std::cout << "Position Sensing.\n";

    // Set encoder count to 0
    result = hil_set_encoder_counts(board, channels, NUM_CHANNELS, counts.data());
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Error: Unable to set encoder counts. " << message << " (Error " << -result << ")\n";
        hil_close(board);
        return 1;
    }

    // Launch the encoder reading thread
    std::thread encoderThread(readEncoderLoop, board, channels, std::ref(counts), std::ref(cubePosition), std::ref(cubeRotation), std::ref(cubeMutex));

    // Main OpenGL rendering loop
    while (running && !glfwWindowShouldClose(window)) {
        // Clear screen and depth buffer
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Black background
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        // Draw grid (X-Z plane)
        glUseProgram(gridShaderProgram); // Use grid shader program
        glm::mat4 gridModel = glm::mat4(1.0f); // Identity matrix for the grid
        glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(gridModel));
        glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(gridShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, (GRID_SIZE * 4 + 4 + VERTICAL_GRID_SIZE * 4)); // Draw grid lines

        // Draw cube
        glUseProgram(cubeShaderProgram); // Use cube shader program
        {
            std::lock_guard<std::mutex> lock(cubeMutex); // Lock the mutex to safely access cubePosition and cubeRotation

            // Combine position and rotation into a single transformation matrix
            glm::mat4 cubeModel = glm::translate(glm::mat4(1.0f), cubePosition) * cubeRotation;

            // Set the model matrix and render the cube
            glUniformMatrix4fv(glGetUniformLocation(cubeShaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(cubeModel));
            glUniformMatrix4fv(glGetUniformLocation(cubeShaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
            glUniformMatrix4fv(glGetUniformLocation(cubeShaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));
            glBindVertexArray(boxVAO);
            glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
        }

        // Swap buffers and poll events
        glfwSwapBuffers(window);
        glfwPollEvents();

        // Simulate delay
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Clean up
    running = false; // Signal the thread to stop
    encoderThread.join(); // Wait for the thread to finish

    // Cleanup: Set voltage to 0V and close the board
    for (int i = 0; i < NUM_CHANNELS; i++) {
        voltages[i] = 0.0;
    }

    result = hil_write_analog(board, channels, NUM_CHANNELS, voltages);
    if (result < 0) {
        msg_get_error_message(nullptr, result, &message[0], message.size());
        std::cerr << "Error: Unable to write 0V to channels. " << message << " (Error " << -result << ")\n";
    }

    // Cleanup OpenGL resources
    glDeleteVertexArrays(1, &gridVAO);
    glDeleteBuffers(1, &gridVBO);
    glDeleteVertexArrays(1, &boxVAO);
    glDeleteBuffers(1, &boxVBO);
    glDeleteBuffers(1, &boxEBO);
    glDeleteProgram(cubeShaderProgram);
    glDeleteProgram(gridShaderProgram);
    glfwDestroyWindow(window);
    glfwTerminate();

    hil_close(board);
    std::cout << "\nPress Enter to exit.\n";
    std::cin.ignore();
    std::cin.get();

    return 0;
}