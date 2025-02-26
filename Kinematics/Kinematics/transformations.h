#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <string>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Round very small values to zero
void roundSmallValues(std::vector<std::vector<double>>& matrix, double threshold = 1e-10) {
    for (auto& row : matrix) {
        for (auto& value : row) {
            if (std::abs(value) < threshold) {
                value = 0.0;
            }
        }
    }
}

// Print any matrix
void printMatrix(const std::vector<std::vector<double>>& matrix, const std::string& name) {
    std::cout << name << ":" << std::endl;
    for (const auto& row : matrix) {
        for (const auto& val : row) {
            std::cout << std::setw(10) << val << " ";
        }
        std::cout << std::endl;
    }
    std::cout << std::endl;
}

// Function to multiply two matrices (handles both matrix × matrix and matrix × vector)
std::vector<std::vector<double>> multiplyMatrices(
    const std::vector<std::vector<double>>& A,
    const std::vector<std::vector<double>>& B
) {
    size_t rowsA = A.size();
    size_t colsA = A[0].size();
    size_t rowsB = B.size();
    size_t colsB = B[0].size();

    if (colsA != rowsB) {
        throw std::invalid_argument("Incompatible dimensions for matrix multiplication");
    }

    std::vector<std::vector<double>> result(rowsA, std::vector<double>(colsB, 0.0));

    for (size_t i = 0; i < rowsA; ++i) {
        for (size_t j = 0; j < colsB; ++j) {
            for (size_t k = 0; k < colsA; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    roundSmallValues(result);
    return result;
}

// Generate a rotation matrix from Euler angles (equivalent to MATLAB's eul2rotm)
std::vector<std::vector<double>> eul2rotm(double rx, double ry, double rz) {
    double rad_rx = rx * M_PI / 180.0;
    double rad_ry = ry * M_PI / 180.0;
    double rad_rz = rz * M_PI / 180.0;

    // Rotation matrix around X-axis
    std::vector<std::vector<double>> Rx = {
        {1, 0, 0},
        {0, cos(rad_rx), -sin(rad_rx)},
        {0, sin(rad_rx), cos(rad_rx)}
    };

    // Rotation matrix around Y-axis
    std::vector<std::vector<double>> Ry = {
        {cos(rad_ry), 0, sin(rad_ry)},
        {0, 1, 0},
        {-sin(rad_ry), 0, cos(rad_ry)}
    };

    // Rotation matrix around Z-axis
    std::vector<std::vector<double>> Rz = {
        {cos(rad_rz), -sin(rad_rz), 0},
        {sin(rad_rz), cos(rad_rz), 0},
        {0, 0, 1}
    };

    // Combined rotation matrix: Rz * Ry * Rx
    auto temp = multiplyMatrices(Ry, Rx);
    auto R = multiplyMatrices(Rz, temp);

    roundSmallValues(R);
    return R;
}

// Generate a 4x4 transformation matrix from a 3x3 rotation matrix and a 3x1 translation vector
void createTransformationMatrix(
    const std::vector<std::vector<double>>& R,
    const std::vector<std::vector<double>>& T,
    std::vector<std::vector<double>>& transformation
) {
    transformation = {
        {R[0][0], R[0][1], R[0][2], T[0][0]},
        {R[1][0], R[1][1], R[1][2], T[1][0]},
        {R[2][0], R[2][1], R[2][2], T[2][0]},
        {0, 0, 0, 1}
    };

    roundSmallValues(transformation);
}


// Helper function for cross product of two 3D vectors
std::vector<std::vector<double>> crossProduct(const std::vector<std::vector<double>>& a, const std::vector<std::vector<double>>& b) {
    return {
        { a[1][0] * b[2][0] - a[2][0] * b[1][0] },
        { a[2][0] * b[0][0] - a[0][0] * b[2][0] },
        { a[0][0] * b[1][0] - a[1][0] * b[0][0] }
    };
}

// Helper function for dot product of two 3D column vectors
double dotProduct(const std::vector<std::vector<double>>& a, const std::vector<std::vector<double>>& b) {
    return a[0][0] * b[0][0] + a[1][0] * b[1][0] + a[2][0] * b[2][0];
}

// Helper function for vector normalization
std::vector<std::vector<double>> normalizeVector(const std::vector<std::vector<double>>& v) {
    double norm = std::sqrt(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0]);
    return { { v[0][0] / norm }, { v[1][0] / norm }, { v[2][0] / norm } };
}

// Function to compute inverse of a 3x3 matrix (basic implementation)
std::vector<std::vector<double>> inverseMatrix3x3(const std::vector<std::vector<double>>& M) {
    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

    if (fabs(det) < 1e-9) {
        std::cerr << "Matrix inversion error: Singular matrix" << std::endl;
        return {};
    }

    std::vector<std::vector<double>> inv(3, std::vector<double>(3));
    inv[0][0] = (M[1][1] * M[2][2] - M[1][2] * M[2][1]) / det;
    inv[0][1] = (M[0][2] * M[2][1] - M[0][1] * M[2][2]) / det;
    inv[0][2] = (M[0][1] * M[1][2] - M[0][2] * M[1][1]) / det;
    inv[1][0] = (M[1][2] * M[2][0] - M[1][0] * M[2][2]) / det;
    inv[1][1] = (M[0][0] * M[2][2] - M[0][2] * M[2][0]) / det;
    inv[1][2] = (M[0][2] * M[1][0] - M[0][0] * M[1][2]) / det;
    inv[2][0] = (M[1][0] * M[2][1] - M[1][1] * M[2][0]) / det;
    inv[2][1] = (M[0][1] * M[2][0] - M[0][0] * M[2][1]) / det;
    inv[2][2] = (M[0][0] * M[1][1] - M[0][1] * M[1][0]) / det;

    return inv;
}

// Function to print the coordinate frame (translation vector)
void printCoordinateFrame(const std::string& name, const std::vector<std::vector<double>>& transform) {
    std::cout << name << " Coordinate Frame: ("
        << transform[0][3] << ", "
        << transform[1][3] << ", "
        << transform[2][3] << ")"
        << std::endl;
}

#endif // TRANSFORMATIONS_H
