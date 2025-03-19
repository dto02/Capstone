#include "scene.h"

// *Global Variables*
glm::vec3 boxPosition(0.0f, 0.0f, 0.0f);
glm::vec3 boxRotation(0.0f, 0.0f, 0.0f);
float movementSpeed = 0.1f;
float rotationSpeed = 1.0f;

unsigned int shaderProgram, gridVAO, gridVBO, boxVAO, boxVBO, boxEBO;
glm::mat4 projection, view;

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

// *Compile Shader Function*
unsigned int compileShader(unsigned int type, const char* source) {
    unsigned int shader = glCreateShader(type);
    glShaderSource(shader, 1, &source, nullptr);
    glCompileShader(shader);
    return shader;
}

// *Setup OpenGL*
void setupOpenGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LINE_SMOOTH);
    glLineWidth(2.0f);
}

// *Setup Shaders*
void setupShaders() {
    unsigned int vertexShader = compileShader(GL_VERTEX_SHADER, vertexShaderSource);
    unsigned int fragmentShader = compileShader(GL_FRAGMENT_SHADER, fragmentShaderSource);

    shaderProgram = glCreateProgram();
    glAttachShader(shaderProgram, vertexShader);
    glAttachShader(shaderProgram, fragmentShader);
    glLinkProgram(shaderProgram);
    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
}

// *Grid Setup*
const int GRID_SIZE = 10;
float gridVertices[(GRID_SIZE * 4 + 4) * 3];

void setupGrid() {
    int index = 0;
    float length = (float)GRID_SIZE;

    for (int i = -GRID_SIZE; i <= GRID_SIZE; i++) {
        gridVertices[index++] = (float)i; gridVertices[index++] = 0.0f; gridVertices[index++] = -length;
        gridVertices[index++] = (float)i; gridVertices[index++] = 0.0f; gridVertices[index++] = length;
    }

    glGenVertexArrays(1, &gridVAO);
    glGenBuffers(1, &gridVBO);
    glBindVertexArray(gridVAO);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(gridVertices), gridVertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
}

// *Cube Setup*
float boxVertices[] = {
    -0.5f, -0.5f, -0.5f,  0.5f, -0.5f, -0.5f,
     0.5f, -0.5f,  0.5f, -0.5f, -0.5f,  0.5f,
    -0.5f,  0.5f, -0.5f,  0.5f,  0.5f, -0.5f,
     0.5f,  0.5f,  0.5f, -0.5f,  0.5f,  0.5f
};

unsigned int boxIndices[] = {
    0, 1, 2, 2, 3, 0,
    4, 5, 6, 6, 7, 4,
    0, 1, 5, 5, 4, 0,
    2, 3, 7, 7, 6, 2,
    0, 3, 7, 7, 4, 0,
    1, 2, 6, 6, 5, 1
};

void setupCube() {
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
}

// *Render Scene*
void renderScene(GLFWwindow* window) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(shaderProgram);

    projection = glm::perspective(glm::radians(45.0f), 1600.0f / 1200.0f, 0.1f, 100.0f);
    view = glm::lookAt(glm::vec3(10.0f, 10.0f, 10.0f), glm::vec3(0.0f), glm::vec3(0.0f, 1.0f, 0.0f));

    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "view"), 1, GL_FALSE, glm::value_ptr(view));
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "projection"), 1, GL_FALSE, glm::value_ptr(projection));

    glm::mat4 model = glm::translate(glm::mat4(1.0f), boxPosition);
    glUniformMatrix4fv(glGetUniformLocation(shaderProgram, "model"), 1, GL_FALSE, glm::value_ptr(model));
    glUniform3f(glGetUniformLocation(shaderProgram, "color"), 1.0f, 1.0f, 1.0f);

    glBindVertexArray(gridVAO);
    glDrawArrays(GL_LINES, 0, (GRID_SIZE * 4 + 4));

    glBindVertexArray(boxVAO);
    glDrawElements(GL_TRIANGLES, 36, GL_UNSIGNED_INT, 0);
}
