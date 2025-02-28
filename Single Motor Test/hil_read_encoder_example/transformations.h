#pragma

#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <iostream>
#include <cmath>
#include <iomanip>
#include <vector>
#include <string>
#include <tuple>

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
    /*std::cout << name << " Coordinate Frame: ("
        << transform[0][3] << ", "
        << transform[1][3] << ", "
        << transform[2][3] << ")"
        << std::endl;*/
}

// From Kinematics.cpp

// Length Parameters
double l1 = 4.45 / 2.0;  // Half of distance between panto motors
double l2 = 14.3;        // Motor to elbow linkage length
double l5 = 14.3;        // Motor to elbow linkage length (duplicate value)
double l3 = 19.3;        // Elbow to tip linkage length
double l4 = 19.3;        // Elbow to tip linkage length (duplicate value)
double l6 = 17.7 / 2.0;        // Half of distance between shoulder motors
double l8 = 17.7 / 2.0;

double computeTheta3(double theta2, double q2) {
    theta2 = theta2 * M_PI / 180.0;
    q2 = q2 * M_PI / 180.0;

    // Compute the components of vector q1_2
    double q1_2x = l2 * cos(theta2);
    double q1_2y = l2 * sin(theta2);

    // Compute the components of vector q4_2 (vector from q2 to q4)
    double q4_2x = q1_2x + 2 * l1 - l5 * cos(q2);
    double q4_2y = q1_2y - l5 * sin(q2);

    // Compute the angle of the q4_2 vector relative to the x-axis
    double angle_q4_2 = atan2(q4_2y, q4_2x);

    // Calculate gamma3 as the difference between theta2 and the angle of q4_2
    double gamma3 = theta2 - angle_q4_2;
    double gamma3deg = gamma3 * 360.0 / (2 * M_PI);

    // Compute the norm (magnitude) of the q4_2 vector
    double norm_q4_2 = sqrt(q4_2x * q4_2x + q4_2y * q4_2y);

    // Calculate gamma4 using the provided relationship.
    double numerator = pow(norm_q4_2 + l3, 2) - pow(l4, 2);
    double denominator = pow(l4, 2) - pow(norm_q4_2 - l3, 2);
    double ratio = numerator / denominator;

    double gamma4 = M_PI - 2 * atan(sqrt(ratio));
    double gamma4deg = gamma4 * 360.0 / (2 * M_PI);

    // Finally, compute theta3 based on the computed gamma values.
    double theta3 = M_PI - gamma3 - gamma4;
    double theta3deg = theta3 * 360.0 / (2 * M_PI);

    return theta3deg;
}

// Function to compute the transformation from b to 3 (based on theta1, theta2, theta3)
// Returns a tuple containing Transformb0, Transformb1, Transformb2, and Transformb3
std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>,
    std::vector<std::vector<double>>, std::vector<std::vector<double>>>
    computeBto3Transformation(double theta1, double theta2, double theta3, double side)
{
    // Define the rotation matrices based on the provided angles
    auto Cb_0 = eul2rotm(0, 90, 0);
    auto C0_1 = eul2rotm(0, -90, theta1);
    auto C1_2 = eul2rotm(0, 0, theta2);
    auto C2_3 = eul2rotm(0, 0, theta3);

    // Define translation vectors
    std::vector<std::vector<double>> Tb0 = { {l1}, {-l6}, {0.0} };
    std::vector<std::vector<double>> T01 = { {0.0}, {0.0}, {0.0} };
    std::vector<std::vector<double>> T12 = multiplyMatrices(C1_2, { {l2}, {0.0}, {0.0} });
    std::vector<std::vector<double>> T23 = multiplyMatrices(C2_3, { {l3}, {0.0}, {0.0} });

    // Create transformation matrices
    std::vector<std::vector<double>> Transformb0, Transform01, Transform12, Transform23;
    createTransformationMatrix(Cb_0, Tb0, Transformb0);
    createTransformationMatrix(C0_1, T01, Transform01);
    createTransformationMatrix(C1_2, T12, Transform12);
    createTransformationMatrix(C2_3, T23, Transform23);

    // Multiply the transformations to get the full transformation from b to 3
    auto Transformb1 = multiplyMatrices(Transformb0, Transform01);
    auto Transformb2 = multiplyMatrices(Transformb1, Transform12);
    auto Transformb3 = multiplyMatrices(Transformb2, Transform23);

    return { Transformb0, Transformb1, Transformb2, Transformb3 };
}


// Function that performs the transformation.
// The 7 parameters can later be substituted into your calculations as needed.
std::vector<double> computeTransformedXYZ(double theta1, double theta2, double q2,
    double theta1p, double theta2p, double q2p, double theta4)
{
    double theta3 = computeTheta3(theta2, q2);
    double theta3p = computeTheta3(theta2p, q2p);

    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>,
        std::vector<std::vector<double>>, std::vector<std::vector<double>>>
        transform5bar = computeBto3Transformation(theta1, theta2, theta3, -1);

    auto Transformb0 = std::get<0>(transform5bar);
    auto Transformb1 = std::get<1>(transform5bar);
    auto Transformb2 = std::get<2>(transform5bar);
    auto Transformb3 = std::get<3>(transform5bar);

    // Print each transformation step
    printCoordinateFrame("Transformb0", Transformb0);
    printCoordinateFrame("Transformb1", Transformb1);
    printCoordinateFrame("Transformb2", Transformb2);
    printCoordinateFrame("Transformb3", Transformb3);

    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>,
        std::vector<std::vector<double>>, std::vector<std::vector<double>>>
        transform5barP = computeBto3Transformation(theta1p, theta2p, theta3p, 1);

    auto Transformb0p = std::get<0>(transform5barP);
    auto Transformb1p = std::get<1>(transform5barP);
    auto Transformb2p = std::get<2>(transform5barP);
    auto Transformb3p = std::get<3>(transform5barP);

    // Now continue with the rest of the transformations
    auto Cb_prime = eul2rotm(0, 0, 180);
    std::vector<std::vector<double>> Tbp = { {0.0}, {0.0}, {0.0} };

    std::vector<std::vector<double>> Transformbp;
    createTransformationMatrix(Cb_prime, Tbp, Transformbp);
    Transformb0p = multiplyMatrices(Transformbp, Transformb0p);
    Transformb1p = multiplyMatrices(Transformbp, Transformb1p);
    Transformb2p = multiplyMatrices(Transformbp, Transformb2p);
    Transformb3p = multiplyMatrices(Transformbp, Transformb3p);

    // Print each transformation step
    printCoordinateFrame("Transformb0p", Transformb0p);
    printCoordinateFrame("Transformb1p", Transformb1p);
    printCoordinateFrame("Transformb2p", Transformb2p);
    printCoordinateFrame("Transformb3p", Transformb3p);

    // Now continue with the rest of the transformations
    auto C3_4 = eul2rotm(-90, 0, theta4);
    std::vector<std::vector<double>> Tb4 = { {0.0}, {0.0}, {0.0} };

    std::vector<std::vector<double>> Transform34;
    createTransformationMatrix(C3_4, Tb4, Transform34);
    auto Transformb4 = multiplyMatrices(Transformb3, Transform34); // Just an example

    printCoordinateFrame("Transformb4", Transformb4);


    std::vector<double> frame3pXYZ = { Transformb3p[0][3], Transformb3p[1][3], Transformb3p[2][3] };
    std::vector<double> frame4XYZ = { Transformb4[0][3], Transformb4[1][3], Transformb4[2][3] };

    std::vector<std::vector<double>> j6 = normalizeVector({
        { frame3pXYZ[0] - frame4XYZ[0] },
        { frame3pXYZ[1] - frame4XYZ[1] },
        { frame3pXYZ[2] - frame4XYZ[2] }
        });

    std::vector<std::vector<double>> j4 = { { Transformb4[0][1] }, { Transformb4[1][1] }, { Transformb4[2][1] } };
    std::vector<std::vector<double>> k4 = { { Transformb4[0][2] }, { Transformb4[1][2] }, { Transformb4[2][2] } };
    std::vector<std::vector<double>> i4 = { { Transformb4[0][0] }, { Transformb4[1][0] }, { Transformb4[2][0] } };

    std::vector<std::vector<double>> k5 = normalizeVector(crossProduct(j6, k4));

    double norm_k5_minus_i4 = std::sqrt(
        std::pow(k5[0][0] - i4[0][0], 2) +
        std::pow(k5[1][0] - i4[1][0], 2) +
        std::pow(k5[2][0] - i4[2][0], 2)
    );
    double norm_k5_plus_i4 = std::sqrt(
        std::pow(k5[0][0] + i4[0][0], 2) +
        std::pow(k5[1][0] + i4[1][0], 2) +
        std::pow(k5[2][0] + i4[2][0], 2)
    );
    double sign_k5_j4 = (dotProduct(k5, j4) >= 0) ? 1.0 : -1.0;
    double theta5rad = 2.0 * atan(norm_k5_minus_i4 / norm_k5_plus_i4) * sign_k5_j4;
    double theta5 = theta5rad * 360.0 / (2 * M_PI);

    auto C4_5 = eul2rotm(0, 90, theta5);
    std::vector<std::vector<double>> Transform45;
    createTransformationMatrix(C4_5, { {0.0}, {0.0}, {0.0} }, Transform45);
    auto Transformb5 = multiplyMatrices(Transformb4, Transform45);
    printCoordinateFrame("Transformb5", Transformb5);

    std::vector<std::vector<double>> j5 = { { Transformb5[0][1] }, { Transformb5[1][1] }, { Transformb5[2][1] } };
    std::vector<std::vector<double>> i5 = { { Transformb5[0][0] }, { Transformb5[1][0] }, { Transformb5[2][0] } };

    double norm_j6_minus_j5 = std::sqrt(
        std::pow(j6[0][0] - j5[0][0], 2) +
        std::pow(j6[1][0] - j5[1][0], 2) +
        std::pow(j6[2][0] - j5[2][0], 2)
    );
    double norm_j6_plus_j5 = std::sqrt(
        std::pow(j6[0][0] + j5[0][0], 2) +
        std::pow(j6[1][0] + j5[1][0], 2) +
        std::pow(j6[2][0] + j5[2][0], 2)
    );
    double sign_j6_i5 = (dotProduct(j6, i5) >= 0) ? 1.0 : -1.0;
    double theta6rad = -2.0 * atan(norm_j6_minus_j5 / norm_j6_plus_j5) * sign_j6_i5;
    double theta6 = theta6rad * 360.0 / (2 * M_PI);

    auto C5_6 = eul2rotm(0, 0, theta6);
    std::vector<std::vector<double>> Transform56;
    createTransformationMatrix(C5_6, multiplyMatrices(C5_6, { {0.0}, {l8}, {0.0} }), Transform56);
    auto Transformb6 = multiplyMatrices(Transformb5, Transform56);
    printCoordinateFrame("Transformb6", Transformb6);

    return { Transformb6[0][3], Transformb6[1][3], Transformb6[2][3] };
}

#endif // TRANSFORMATIONS_H
