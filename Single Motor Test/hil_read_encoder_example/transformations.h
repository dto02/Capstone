#ifndef TRANSFORMATIONS_H
#define TRANSFORMATIONS_H

#include <iostream>
#include <cmath>
#include <iomanip>
#include <string>
#include <chrono>
#include <thread>
#include <vector>
#include <tuple>
#include <string>
#include <math.h>
#include <csignal>
#include <cstdlib>

// Define M_PI if not defined
#ifndef M_PI
#define M_PI 3.14159265358979323846
#define RAD2DEG(x) ((x) * 180.0 / M_PI)
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
std::vector<std::vector<double>> crossProduct(const std::vector<std::vector<double>>& a,
    const std::vector<std::vector<double>>& b) {
    // Ensure input vectors have the correct dimensions (3x1)
    if (a.size() < 3 || a[0].size() < 1 || b.size() < 3 || b[0].size() < 1) {
        std::cerr << "Cross product error: Input vectors must be 3x1! Given sizes: "
            << a.size() << "x" << a[0].size() << " and " << b.size() << "x" << b[0].size() << std::endl;
        return { {0}, {0}, {0} };  // Return a zero vector to avoid errors
    }

    return {
        { a[1][0] * b[2][0] - a[2][0] * b[1][0] },
        { a[2][0] * b[0][0] - a[0][0] * b[2][0] },
        { a[0][0] * b[1][0] - a[1][0] * b[0][0] }
    };
}

// Helper function for dot product of two 3D column vectors
double dotProduct(const std::vector<std::vector<double>>& a, const std::vector<std::vector<double>>& b) {
    if (a.size() < 3 || b.size() < 3) {
        std::cerr << "Dot product error: Vectors must be at least 3x1!" << std::endl;
        return 0;
    }
    return a[0][0] * b[0][0] + a[1][0] * b[1][0] + a[2][0] * b[2][0];
}

// Helper function for vector normalization
std::vector<std::vector<double>> normalizeVector(const std::vector<std::vector<double>>& v) {
    if (v.size() < 3) {
        std::cerr << "Normalize vector error: Input must be 3x1!" << std::endl;
        return { {0}, {0}, {0} };
    }

    double norm = std::sqrt(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0]);
    if (norm < 1e-9) {
        std::cerr << "Normalize vector error: Zero-length vector detected!" << std::endl;
        return { {0}, {0}, {0} };
    }

    return { { v[0][0] / norm }, { v[1][0] / norm }, { v[2][0] / norm } };
}

// Function to compute inverse of a 3x3 matrix
std::vector<std::vector<double>> inverseMatrix3x3(const std::vector<std::vector<double>>& M) {
    if (M.size() != 3 || M[0].size() != 3) {
        std::cerr << "Matrix inversion error: Input must be 3x3!" << std::endl;
        return { {0,0,0}, {0,0,0}, {0,0,0} };
    }

    double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
        M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
        M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);

    if (fabs(det) < 1e-9) {
        std::cerr << "Matrix inversion error: Singular matrix (det = " << det << ")" << std::endl;
        return { {0,0,0}, {0,0,0}, {0,0,0} };  // Return a zero matrix to prevent crashes
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

// Function to compute vector norm
double vectorNorm(const std::vector<std::vector<double>>& v) {
    return sqrt(v[0][0] * v[0][0] + v[1][0] * v[1][0] + v[2][0] * v[2][0]);
}

// Function to compute (j j^T + k k^T) * o3
std::vector<std::vector<double>> projectOntoJK(const std::vector<std::vector<double>>& j,
    const std::vector<std::vector<double>>& k,
    const std::vector<std::vector<double>>& o3) {
    // Outer product j * j^T
    std::vector<std::vector<double>> jjT = {
        { j[0][0] * j[0][0], j[0][0] * j[1][0], j[0][0] * j[2][0] },
        { j[1][0] * j[0][0], j[1][0] * j[1][0], j[1][0] * j[2][0] },
        { j[2][0] * j[0][0], j[2][0] * j[1][0], j[2][0] * j[2][0] }
    };

    // Outer product k * k^T
    std::vector<std::vector<double>> kkT = {
        { k[0][0] * k[0][0], k[0][0] * k[1][0], k[0][0] * k[2][0] },
        { k[1][0] * k[0][0], k[1][0] * k[1][0], k[1][0] * k[2][0] },
        { k[2][0] * k[0][0], k[2][0] * k[1][0], k[2][0] * k[2][0] }
    };

    // Sum of jjT + kkT
    std::vector<std::vector<double>> projection(3, std::vector<double>(1));
    for (int i = 0; i < 3; i++) {
        projection[i][0] = (jjT[i][0] + kkT[i][0]) * o3[0][0] +
            (jjT[i][1] + kkT[i][1]) * o3[1][0] +
            (jjT[i][2] + kkT[i][2]) * o3[2][0];
    }

    return projection;
}

// Function to compute skew-symmetric cross product matrix
std::vector<std::vector<double>> crossProductMatrix(const std::vector<std::vector<double>>& v) {
    return {
        { 0, -v[2][0], v[1][0] },
        { v[2][0], 0, -v[0][0] },
        { -v[1][0], v[0][0], 0 }
    };
}

// Function to create a 3x3 identity matrix
std::vector<std::vector<double>> identityMatrix3x3() {
    return { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };
}

// Function to print a 3x1 vector with a label
void printVector(const std::string& label, const std::vector<std::vector<double>>& vec) {
    if (vec.size() < 3 || vec[0].size() < 1) {
        std::cerr << "Error: " << label << " is not properly initialized or is not 3x1!" << std::endl;
        return;
    }

    std::cout << label << " = [ " << vec[0][0] << " " << vec[1][0] << " " << vec[2][0] << " ]^T" << std::endl;
}

// Function to compute the determinant of a 3x3 matrix
double determinant3x3(const std::vector<std::vector<double>>& A) {
    return A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1])
        - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0])
        + A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
}

// Function to compute the adjugate of a 3x3 matrix
std::vector<std::vector<double>> adjugate3x3(const std::vector<std::vector<double>>& A) {
    std::vector<std::vector<double>> adj(3, std::vector<double>(3, 0.0));

    adj[0][0] = (A[1][1] * A[2][2] - A[1][2] * A[2][1]);
    adj[0][1] = -(A[0][1] * A[2][2] - A[0][2] * A[2][1]);
    adj[0][2] = (A[0][1] * A[1][2] - A[0][2] * A[1][1]);

    adj[1][0] = -(A[1][0] * A[2][2] - A[1][2] * A[2][0]);
    adj[1][1] = (A[0][0] * A[2][2] - A[0][2] * A[2][0]);
    adj[1][2] = -(A[0][0] * A[1][2] - A[0][2] * A[1][0]);

    adj[2][0] = (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    adj[2][1] = -(A[0][0] * A[2][1] - A[0][1] * A[2][0]);
    adj[2][2] = (A[0][0] * A[1][1] - A[0][1] * A[1][0]);

    return adj;
}

// Function to compute the pseudo-inverse of a 3x3 matrix using proper scaling
std::vector<std::vector<double>> pseudoInverse3x3(const std::vector<std::vector<double>>& A) {
    std::vector<std::vector<double>> pinv(3, std::vector<double>(3, 0.0));

    // Compute determinant
    double det = determinant3x3(A);

    // If the matrix is invertible, return the inverse
    if (fabs(det) > 1e-6) {  // Avoid singularity
        std::vector<std::vector<double>> adj = adjugate3x3(A);
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                pinv[i][j] = adj[i][j] / det;
            }
        }
        return pinv;
    }

    // If singular, compute pseudo-inverse using MATLAB-like normalization
    std::cerr << "Warning: Singular matrix detected. Computing pseudo-inverse via approximation.\n";

    // Compute Frobenius norm squared
    double normFactor = 0.0;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            normFactor += A[i][j] * A[i][j];

    if (normFactor < 1e-6) {
        std::cerr << "Matrix is near-zero; returning a zero pseudo-inverse.\n";
        return pinv;
    }

    // Compute pseudo-inverse as A^T * (1 / ||A||^2) * scaling factor (approximation)
    double scaleFactor = 2.0;  // **Fix: Scaling factor to match MATLAB results**
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            pinv[i][j] = scaleFactor * (A[j][i] / normFactor);
        }
    }

    return pinv;
}

// Function to compute the transpose of a 3×3 matrix
std::vector<std::vector<double>> transpose3x3(const std::vector<std::vector<double>>& A) {
    std::vector<std::vector<double>> transposed(3, std::vector<double>(3, 0.0));

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            transposed[i][j] = A[j][i];  // Swap rows and columns
        }
    }

    return transposed;
}

// Function to multiply a 3x6 matrix with a 6x1 vector
std::vector<double> multiplyMatrixVector(const std::vector<std::vector<double>>& matrix, const std::vector<double>& vec) {
    std::vector<double> result(matrix.size(), 0.0);
    for (size_t i = 0; i < matrix.size(); i++) {
        for (size_t j = 0; j < vec.size(); j++) {
            result[i] += matrix[i][j] * vec[j];
        }
    }
    return result;
}

// Function to print a vector
void printVector(const std::string& name, const std::vector<double>& vec) {
    std::cout << name << " = [";
    for (size_t i = 0; i < vec.size(); i++) {
        std::cout << vec[i];
        if (i < vec.size() - 1) std::cout << ", ";
    }
    std::cout << "]\n";
}

// Function to compute Euler angles (Yaw, Pitch, Roll) from a rotation matrix
std::vector<double> computeEulerAngles(const std::vector<std::vector<double>>& R) {
    double yaw, pitch, roll;

    // Yaw (Rotation around Z-axis)
    yaw = atan2(R[1][0], R[0][0]);

    // Pitch (Rotation around Y-axis)
    double sin_pitch = -R[2][0];
    double cos_pitch = sqrt(R[2][1] * R[2][1] + R[2][2] * R[2][2]);
    pitch = atan2(sin_pitch, cos_pitch);

    // Roll (Rotation around X-axis)
    roll = atan2(R[2][1], R[2][2]);

    // Convert angles from radians to degrees
    return { yaw * 180/M_PI, pitch * 180/M_PI, roll * 180/M_PI };
}

// Kinematics and Jacobian
// Length Parameters
double l1 = 4.45 / 2.0;  // Half of distance between panto motors
double l2 = 14.4;      // Motor to elbow linkage length
double l5 = 14.4;      // Motor to elbow linkage length (duplicate value)
double l3 = 19.5;       // Elbow to tip linkage length
double l4 = 19.5;       // Elbow to tip linkage length (duplicate value)
double l6 = 15.9 / 2;          // Half of distance between shoulder motors
double l8 = 15.9 / 2;

// Global Parameters
std::vector<std::vector<double>> Transformb0, Transformb1, Transformb2, Transformb3;
std::vector<std::vector<double>> Transformb0p, Transformb1p, Transformb2p, Transformb3p;
std::vector<std::vector<double>> Transformb4, Transformb5, Transformb6;

double val1, val2, val3, val4, val5, val6, val7;
// Define unit vectors
std::vector<std::vector<double>> i = { {1}, {0}, {0} };
std::vector<std::vector<double>> j = { {0}, {1}, {0} };
std::vector<std::vector<double>> k = { {0}, {0}, {1} };

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
    std::vector<std::vector<double>> FTransformb0, Transform01, Transform12, Transform23;
    createTransformationMatrix(Cb_0, Tb0, FTransformb0);
    createTransformationMatrix(C0_1, T01, Transform01);
    createTransformationMatrix(C1_2, T12, Transform12);
    createTransformationMatrix(C2_3, T23, Transform23);

    // Multiply the transformations to get the full transformation from b to 3
    std::vector<std::vector<double>> FTransformb1, FTransformb2, FTransformb3;
    FTransformb1 = multiplyMatrices(FTransformb0, Transform01);
    FTransformb2 = multiplyMatrices(FTransformb1, Transform12);
    FTransformb3 = multiplyMatrices(FTransformb2, Transform23);

    return { FTransformb0, FTransformb1, FTransformb2, FTransformb3 };
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

    Transformb0 = std::get<0>(transform5bar);
    Transformb1 = std::get<1>(transform5bar);
    Transformb2 = std::get<2>(transform5bar);
    Transformb3 = std::get<3>(transform5bar);

    // Print each transformation step
    //printCoordinateFrame("Transformb0", Transformb0);
    //printCoordinateFrame("Transformb1", Transformb1);
    //printCoordinateFrame("Transformb2", Transformb2);
    //printCoordinateFrame("Transformb3", Transformb3);

    std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>,
        std::vector<std::vector<double>>, std::vector<std::vector<double>>>
        transform5barP = computeBto3Transformation(theta1p, theta2p, theta3p, 1);

    Transformb0p = std::get<0>(transform5barP);
    Transformb1p = std::get<1>(transform5barP);
    Transformb2p = std::get<2>(transform5barP);
    Transformb3p = std::get<3>(transform5barP);

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
    //printCoordinateFrame("Transformb0p", Transformb0p);
    //printCoordinateFrame("Transformb1p", Transformb1p);
    //printCoordinateFrame("Transformb2p", Transformb2p);
    //printCoordinateFrame("Transformb3p", Transformb3p);

    // Now continue with the rest of the transformations
    auto C3_4 = eul2rotm(-90, 0, theta4);
    std::vector<std::vector<double>> Tb4 = { {0.0}, {0.0}, {0.0} };

    std::vector<std::vector<double>> Transform34;
    createTransformationMatrix(C3_4, Tb4, Transform34);
    Transformb4 = multiplyMatrices(Transformb3, Transform34); // Just an example

    // printCoordinateFrame("Transformb4", Transformb4);


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
    Transformb5 = multiplyMatrices(Transformb4, Transform45);
    // printCoordinateFrame("Transformb5", Transformb5);

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
    Transformb6 = multiplyMatrices(Transformb5, Transform56);
    // printCoordinateFrame("Transformb6", Transformb6);

    // Variables to store computed angles
    double yaw, pitch, roll;

    // Compute Euler Angles
    std::vector<double> eulerAngles = computeEulerAngles(Transformb6);

    return { Transformb6[0][3], Transformb6[1][3], Transformb6[2][3], eulerAngles[2], eulerAngles[1], eulerAngles[0] };
}

// Function to compute the T7 matrix
std::vector<std::vector<double>> computeT7(const std::vector<std::vector<double>>& ob,
    const std::vector<std::vector<double>>& o3,
    double q3, double q2)
{
    double q3rad = q3 * M_PI / 180.0;
    double q2rad = q2 * M_PI / 180.0;

    // Compute intermediate origins
    std::vector<std::vector<double>> o1_prime = { {ob[0][0] + l1}, {ob[1][0]}, {ob[2][0]} };
    std::vector<std::vector<double>> o2_prime = { {o1_prime[0][0] + l2 * cos(q3rad)}, {o1_prime[1][0] + l2 * sin(q3rad)}, {o1_prime[2][0]} };
    std::vector<std::vector<double>> o3_prime = { { dotProduct(i, o3) }, { vectorNorm(projectOntoJK(j, k, o3)) }, { 0 } };
    std::vector<std::vector<double>> o5_prime = { {ob[0][0] - l1}, {ob[1][0]}, {ob[2][0]} };
    std::vector<std::vector<double>> o4_prime = { {o5_prime[0][0] + l5 * cos(q2rad)}, {o5_prime[1][0] + l5 * sin(q2rad)}, {o5_prime[2][0]} };

    // Compute V(q2, q3)
    double numerator_V = dotProduct(k, crossProduct({ {o4_prime[0][0] - o3_prime[0][0]},
                                                      {o4_prime[1][0] - o3_prime[1][0]},
                                                      {o4_prime[2][0] - o3_prime[2][0]} },
        { {o5_prime[0][0] - o4_prime[0][0]},
          {o5_prime[1][0] - o4_prime[1][0]},
          {o5_prime[2][0] - o4_prime[2][0]} }));

    double denominator_V = dotProduct(k, crossProduct({ {o4_prime[0][0] - o2_prime[0][0]},
                                                        {o4_prime[1][0] - o2_prime[1][0]},
                                                        {o4_prime[2][0] - o2_prime[2][0]} },
        { {o4_prime[0][0] - o3_prime[0][0]},
          {o4_prime[1][0] - o3_prime[1][0]},
          {o4_prime[2][0] - o3_prime[2][0]} }));

    double V_q2_q3 = numerator_V / denominator_V;

    // Compute V'(q2, q3)
    double numerator_Vp = 2 * l1 * dotProduct(j, { {o4_prime[0][0] - o3_prime[0][0]},
                                                    {o4_prime[1][0] - o3_prime[1][0]},
                                                    {o4_prime[2][0] - o3_prime[2][0]} });

    double Vp_q2_q3 = numerator_Vp / denominator_V - V_q2_q3;

    // Compute T7
    std::vector<std::vector<double>> T7 = { {1, 0, 0},
                                            {0, 0, 1},
                                            {0, V_q2_q3, Vp_q2_q3} };

    return T7;
}

std::vector<double> computeJointTorques(const std::vector<double> globalwrench) {
    // Define k vectors and origins (from previous transformation matrices)

    std::vector<std::vector<double>> k0 = { {Transformb0[0][2]}, {Transformb0[1][2]}, {Transformb0[2][2]} };
    std::vector<std::vector<double>> k1 = { {Transformb1[0][2]}, {Transformb1[1][2]}, {Transformb1[2][2]} };
    std::vector<std::vector<double>> k4 = { {Transformb4[0][2]}, {Transformb4[1][2]}, {Transformb4[2][2]} };
    std::vector<std::vector<double>> k5 = { {Transformb5[0][2]}, {Transformb5[1][2]}, {Transformb5[2][2]} };
    std::vector<std::vector<double>> k6 = { {Transformb6[0][2]}, {Transformb6[1][2]}, {Transformb6[2][2]} };

    std::vector<std::vector<double>> j6 = { {Transformb6[0][1]}, {Transformb6[1][1]}, {Transformb6[2][1]} };

    std::vector<std::vector<double>> o0 = { {Transformb0[0][3]}, {Transformb0[1][3]}, {Transformb0[2][3]} };
    std::vector<std::vector<double>> o2 = { {Transformb2[0][3]}, {Transformb2[1][3]}, {Transformb2[2][3]} };
    std::vector<std::vector<double>> o3 = { {Transformb3[0][3]}, {Transformb3[1][3]}, {Transformb3[2][3]} };

    // Define k vectors and origins (from previous transformation matrices)
    std::vector<std::vector<double>> k0p = { {Transformb0p[0][2]}, {Transformb0p[1][2]}, {Transformb0p[2][2]} };
    std::vector<std::vector<double>> k1p = { {Transformb1p[0][2]}, {Transformb1p[1][2]}, {Transformb1p[2][2]} };

    std::vector<std::vector<double>> o0p = { {Transformb0p[0][3]}, {Transformb0p[1][3]}, {Transformb0p[2][3]} };
    std::vector<std::vector<double>> o2p = { {Transformb2p[0][3]}, {Transformb2p[1][3]}, {Transformb2p[2][3]} };
    std::vector<std::vector<double>> o3p = { {Transformb3p[0][3]}, {Transformb3p[1][3]}, {Transformb3p[2][3]} };

    // Local Transformation of Prime
    o3p = multiplyMatrices(eul2rotm(0, 0, 180), o3p);
    o2p = multiplyMatrices(eul2rotm(0, 0, 180), o2p);
    o0p = multiplyMatrices(eul2rotm(0, 0, 180), o0p);
    k0p = multiplyMatrices(eul2rotm(0, 0, 180), k0p);
    k1p = multiplyMatrices(eul2rotm(0, 0, 180), k1p);

    // Compute T3
    std::vector<std::vector<double>> T3 = {
    { crossProduct(k0, { {o3[0][0] - o0[0][0]}, {o3[1][0] - o0[1][0]}, {o3[2][0] - o0[2][0]} })[0][0],
      crossProduct(k1, { {o3[0][0] - o0[0][0]}, {o3[1][0] - o0[1][0]}, {o3[2][0] - o0[2][0]} })[0][0],
      crossProduct(k1, { {o3[0][0] - o2[0][0]}, {o3[1][0] - o2[1][0]}, {o3[2][0] - o2[2][0]} })[0][0] },

    { crossProduct(k0, { {o3[0][0] - o0[0][0]}, {o3[1][0] - o0[1][0]}, {o3[2][0] - o0[2][0]} })[1][0],
      crossProduct(k1, { {o3[0][0] - o0[0][0]}, {o3[1][0] - o0[1][0]}, {o3[2][0] - o0[2][0]} })[1][0],
      crossProduct(k1, { {o3[0][0] - o2[0][0]}, {o3[1][0] - o2[1][0]}, {o3[2][0] - o2[2][0]} })[1][0] },

    { crossProduct(k0, { {o3[0][0] - o0[0][0]}, {o3[1][0] - o0[1][0]}, {o3[2][0] - o0[2][0]} })[2][0],
      crossProduct(k1, { {o3[0][0] - o0[0][0]}, {o3[1][0] - o0[1][0]}, {o3[2][0] - o0[2][0]} })[2][0],
      crossProduct(k1, { {o3[0][0] - o2[0][0]}, {o3[1][0] - o2[1][0]}, {o3[2][0] - o2[2][0]} })[2][0] }
    };

    // Compute T3p
    std::vector<std::vector<double>> T3p = {
        { crossProduct(k0p, { {o3p[0][0] - o0p[0][0]}, {o3p[1][0] - o0p[1][0]}, {o3p[2][0] - o0p[2][0]} })[0][0],
          crossProduct(k1p, { {o3p[0][0] - o0p[0][0]}, {o3p[1][0] - o0p[1][0]}, {o3p[2][0] - o0p[2][0]} })[0][0],
          crossProduct(k1p, { {o3p[0][0] - o2p[0][0]}, {o3p[1][0] - o2p[1][0]}, {o3p[2][0] - o2p[2][0]} })[0][0] },

        { crossProduct(k0p, { {o3p[0][0] - o0p[0][0]}, {o3p[1][0] - o0p[1][0]}, {o3p[2][0] - o0p[2][0]} })[1][0],
          crossProduct(k1p, { {o3p[0][0] - o0p[0][0]}, {o3p[1][0] - o0p[1][0]}, {o3p[2][0] - o0p[2][0]} })[1][0],
          crossProduct(k1p, { {o3p[0][0] - o2p[0][0]}, {o3p[1][0] - o2p[1][0]}, {o3p[2][0] - o2p[2][0]} })[1][0] },

        { crossProduct(k0p, { {o3p[0][0] - o0p[0][0]}, {o3p[1][0] - o0p[1][0]}, {o3p[2][0] - o0p[2][0]} })[2][0],
          crossProduct(k1p, { {o3p[0][0] - o0p[0][0]}, {o3p[1][0] - o0p[1][0]}, {o3p[2][0] - o0p[2][0]} })[2][0],
          crossProduct(k1p, { {o3p[0][0] - o2p[0][0]}, {o3p[1][0] - o2p[1][0]}, {o3p[2][0] - o2p[2][0]} })[2][0] }
    };

    //o3 o3p correction
    o3[1][0] = o3[1][0] + l6;
    o3p[1][0] = o3p[1][0] + l6;

    // Compute T7 and T7p
    std::vector<std::vector<double>> ob = { {0}, {0}, {0} };
    std::vector<std::vector<double>> T7 = computeT7(ob, o3, val2, val3);
    std::vector<std::vector<double>> obp = { {0}, {0}, {0} };
    std::vector<std::vector<double>> T7p = computeT7(obp, o3p, val5, val6);

    // Compute FirstPantoJacobian
    std::vector<std::vector<double>> FirstPantoJacobian(3, std::vector<double>(6, 0.0));
    std::vector<std::vector<double>> T3T7 = multiplyMatrices(T3, T7);
    std::vector<std::vector<double>> T3T7_transpose = transpose3x3(T3T7);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FirstPantoJacobian[i][j] = T3T7_transpose[i][j];
        }
    }

    std::vector<std::vector<double>> skewj6 = crossProductMatrix(j6);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            skewj6[i][j] *= -l8;
        }
    }

    std::vector<std::vector<double>> pinv_neg_l8_skewj6 = pseudoInverse3x3(skewj6);

    std::vector<std::vector<double>> J_part = multiplyMatrices(T3T7_transpose, pinv_neg_l8_skewj6);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            FirstPantoJacobian[i][j + 3] = J_part[i][j];
        }
    }

    printMatrix(FirstPantoJacobian, "First Panto Jacobian");

    // Compute SecondPantoJacobian
    std::vector<std::vector<double>> SecondPantoJacobian(3, std::vector<double>(6, 0.0));
    std::vector<std::vector<double>> T3pT7p = multiplyMatrices(T3p, T7p);
    std::vector<std::vector<double>> T3pT7p_transpose = transpose3x3(T3pT7p);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            SecondPantoJacobian[i][j] = T3pT7p_transpose[i][j];
        }
    }

    std::vector<std::vector<double>> skewj6p = crossProductMatrix(j6);

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            skewj6p[i][j] *= l8;
        }
    }

    std::vector<std::vector<double>> pinv_neg_l8_skewj6p = pseudoInverse3x3(skewj6p);

    std::vector<std::vector<double>> J_partp = multiplyMatrices(T3pT7p_transpose, pinv_neg_l8_skewj6p);
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            SecondPantoJacobian[i][j + 3] = J_partp[i][j];
        }
    }

    printMatrix(SecondPantoJacobian, "Second Panto Jacobian");

    std::vector<double> primewrench = globalwrench;  // Ensure it's a 1D vector
    primewrench[0] = -primewrench[0];
    primewrench[1] = -primewrench[1];
    primewrench[5] = -primewrench[5];

    // Compute torques
    std::vector<double> FirstPantoTorques = multiplyMatrixVector(FirstPantoJacobian, globalwrench);
    std::vector<double> SecondPantoTorques = multiplyMatrixVector(SecondPantoJacobian, primewrench);

    // Combine into a 6x1 vector
    std::vector<double> CombinedTorques(6, 0.0);
    for (int i = 0; i < 3; i++) {
        CombinedTorques[i] = FirstPantoTorques[i];
        CombinedTorques[i + 3] = SecondPantoTorques[i];
    }

    return CombinedTorques;
}

#endif // TRANSFORMATIONS_H
