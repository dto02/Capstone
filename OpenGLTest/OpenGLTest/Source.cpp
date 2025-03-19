#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include <iostream>
#include <cmath> // For sin() function
#include <vector> // To store positions

// *Vertex Shader*
const char* vertexShaderSource = R"(
#version 330 core
layout (location = 0) in vec3 aPos;
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;
void main() {
    gl_Position = projection * view * model * vec4(aPos, 1.0);
}
)";

// *Fragment Shader*
const char* fragmentShaderSource = R"(
#version 330 core
out vec4 FragColor;
uniform vec3 color;
void main() {
    FragColor = vec4(color, 1.0);
}
)";

// *Grid Settings*
const int GRID_SIZE = 10; // Reduced grid size for cleaner top area
const int VERTICAL_GRID_SIZE = 5; // Reduced vertical grid lines
float gridVertices[(GRID_SIZE * 4 + 4 + VERTICAL_GRID_SIZE * 4) * 3]; // Adjusted size for vertical lines

void generateGrid() {
    int index = 0;
    float length = (float)GRID_SIZE;

    // Horizontal grid lines (X and Z axes)
    for (int i = -GRID_SIZE; i <= GRID_SIZE; i++) {
        // Lines along X-axis
        gridVertices[index++] = (float)i; gridVertices[index++] = 0.0f; gridVertices[index++] = -length;
        gridVertices[index++] = (float)i; gridVertices[index++] = 0.0f; gridVertices[index++] = length;

        // Lines along Z-axis
        gridVertices[index++] = -length; gridVertices[index++] = 0.0f; gridVertices[index++] = (float)i;
        gridVertices[index++] = length; gridVertices[index++] = 0.0f; gridVertices[index++] = (float)i;
    }

    // Vertical grid lines (Y-axis)
    for (int i = 1; i <= VERTICAL_GRID_SIZE; i++) {
        float y = -(float)i; // Extend downward
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

// *Cube (Box) Vertices*
float boxVertices[] = {
    -0.5f, -0.5f, -0.5f,
     0.5f, -0.5f, -0.5f,
     0.5f, -0.5f,  0.5f,
    -0.5f, -0.5f,  0.5f,

    -0.5f,  0.5f, -0.5f,
     0.5f,  0.5f, -0.5f,
     0.5f,  0.5f,  0.5f,
    -0.5f,  0.5f,  0.5f
};

// *Cube Face Indices (for solid box)*
unsigned int boxIndices[] = {
    0, 1, 2, 2, 3, 0,
    4, 5, 6, 6, 7, 4,
    0, 1, 5, 5, 4, 0,
    2, 3, 7, 7, 6, 2,
    0, 3, 7, 7, 4, 0,
    1, 2, 6, 6, 5, 1
};

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

// *Keyboard Input Variables*
glm::vec3 boxPosition(0.0f, 0.0f, 0.0f); // Initial position
glm::vec3 boxRotation(0.0f, 0.0f, 0.0f); // Initial rotation (in degrees)
float movementSpeed = 0.1f; // Speed of movement
float rotationSpeed = 1.0f; // Speed of rotation

// *Key Callback Function*
void keyCallback(GLFWwindow* window, int key, int scancode, int action, int mods) {
    if (action == GLFW_PRESS || action == GLFW_REPEAT) {
        bool shiftPressed = (mods & GLFW_MOD_SHIFT); // Check if Shift key is pressed

        if (shiftPressed) {
            // Rotational movement
            switch (key) {
            case GLFW_KEY_UP:
                boxRotation.x += rotationSpeed; // Rotate around X-axis
                break;
            case GLFW_KEY_DOWN:
                boxRotation.x -= rotationSpeed; // Rotate around X-axis
                break;
            case GLFW_KEY_LEFT:
                boxRotation.y += rotationSpeed; // Rotate around Y-axis
                break;
            case GLFW_KEY_RIGHT:
                boxRotation.y -= rotationSpeed; // Rotate around Y-axis
                break;
            case GLFW_KEY_W:
                boxRotation.z += rotationSpeed; // Rotate around Z-axis
                break;
            case GLFW_KEY_S:
                boxRotation.z -= rotationSpeed; // Rotate around Z-axis
                break;
            }
        }
        else {
            // Translational movement
            switch (key) {
            case GLFW_KEY_UP:
                boxPosition.z -= movementSpeed; // Move forward along Z-axis
                break;
            case GLFW_KEY_DOWN:
                boxPosition.z += movementSpeed; // Move backward along Z-axis
                break;
            case GLFW_KEY_LEFT:
                boxPosition.x -= movementSpeed; // Move left along X-axis
                break;
            case GLFW_KEY_RIGHT:
                boxPosition.x += movementSpeed; // Move right along X-axis
                break;
            case GLFW_KEY_W:
                boxPosition.y += movementSpeed; // Move up along Y-axis
                break;
            case GLFW_KEY_S:
                boxPosition.y -= movementSpeed; // Move down along Y-axis
                break;
            }
        }
    }
}

int main() {
    glfwInit();
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    GLFWwindow* window = glfwCreateWindow(1600, 1200, "Moving Box with Isometric Grid", nullptr, nullptr);
    glfwMakeContextCurrent(window);
    gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0f);

    // *Set Key Callback*
    glfwSetKeyCallback(window, keyCallback);

    // *Compile Shaders*
    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

    // *Shader Program*
    unsigned int shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);

    // *Generate Grid*
    generateGrid();

    // *Grid VAO/VBO*
    unsigned int gridVAO, gridVBO;
    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gridVertices), gridVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // *Box VAO/VBO/EBO*
    unsigned int boxVAO, boxVBO, boxEBO;
    glGenVertexArrays(1, &boxVAO);
    glGenBuffers(1, &boxVBO);
    glGenBuffers(1, &boxEBO);

    glBindVertexArray(boxVAO);
    glBindBuffer(GL_ARRAY_BUFFER, boxVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(boxVertices), boxVertices, GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, boxEBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(boxIndices), boxIndices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // *Camera Setup*
    glm::mat4 projection = glm::perspective(glm::radians(45.0f), 1600.0f / 1200.0f, 0.1f, 100.0f);
    glm::mat4 view = glm::lookAt(glm::vec3(10.0f, 10.0f, 10.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    while (!glfwWindowShouldClose(window)) {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(shaderProgram);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

        // *Draw Grid*
        glm::mat4 gridModel = glm::mat4(1.0f);
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(gridModel));
        glUniform3f(glGetUniformLocation(shaderProgram, "color"), 0.6f, 0.6f, 0.6f);
        glBindVertexArray(gridVAO);
        glDrawArrays(GL_LINES, 0, (GRID_SIZE * 4 + 4 + VERTICAL_GRID_SIZE * 4)); // Adjusted vertex count

        // *Update Box Model Matrix*
        glm::mat4 model = glm::mat4(1.0f);
        model = glm::translate(model, boxPosition); // Apply translation
        model = glm::rotate(model, glm::radians(boxRotation.x), glm::vec3(1.0f, 0.0f, 0.0f)); // Rotate around X-axis
        model = glm::rotate(model, glm::radians(boxRotation.y), glm::vec3(0.0f, 1.0f, 0.0f)); // Rotate around Y-axis
        model = glm::rotate(model, glm::radians(boxRotation.z), glm::vec3(0.0f, 0.0f, 1.0f)); // Rotate around Z-axis

        // *Draw Box*
        glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
        glUniform3f(glGetUniformLocation(shaderProgram, "color"), 1.0f, 0.0f, 0.0f);
        glBindVertexArray(boxVAO);
        glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);

        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    glfwTerminate();
    return 0;
}