#include <iostream>
#include <vector>
#include "transformations.h"

// Length Parameters
double l1 = 4.04 / 2.0;  // Half of distance between panto motors
double l2 = 16.820;      // Motor to elbow linkage length
double l5 = 16.820;      // Motor to elbow linkage length (duplicate value)
double l3 = 17.09;       // Elbow to tip linkage length
double l4 = 17.09;       // Elbow to tip linkage length (duplicate value)
double l6 = 20;          // Half of distance between shoulder motors
double l8 = 20;

double computeTheta3(double theta2, double q2) {
    const double PI = 3.14159265358979323846;
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
    double gamma3deg = gamma3 * 360.0 / (2 * PI);

    // Compute the norm (magnitude) of the q4_2 vector
    double norm_q4_2 = sqrt(q4_2x * q4_2x + q4_2y * q4_2y);

    // Calculate gamma4 using the provided relationship.
    double numerator = pow(norm_q4_2 + l3, 2) - pow(l4, 2);
    double denominator = pow(l4, 2) - pow(norm_q4_2 - l3, 2);
    double ratio = numerator / denominator;

    double gamma4 = PI - 2 * atan(sqrt(ratio));
    double gamma4deg = gamma4 * 360.0 / (2 * PI);

    // Finally, compute theta3 based on the computed gamma values.
    double theta3 = PI - gamma3 - gamma4;
    double theta3deg = theta3 * 360.0 / (2 * PI);

    return theta3;
}


// Function that performs the transformation.
// The 7 parameters can later be substituted into your calculations as needed.
std::vector<double> computeTransformedXYZ(double theta1, double theta2, double q2,
    double theta1p, double theta2p, double q2p,
    double theta4)
{
    // theta3 calculation
    double theta3 = computeTheta3(theta2, q2);

    // Define 3x3 rotation matrices (currently hardcoded, adjust later to use the parameters)
    auto Cb_0 = eul2rotm(90, 0, 0);
    auto C0_1 = eul2rotm(0, -90, theta1);
    auto C1_2 = eul2rotm(0, 0, theta2);
    auto C2_3 = eul2rotm(0, 0, theta3);
    auto C3_4 = eul2rotm(-90, 0, theta4);


    // Define length vectors
    std::vector<std::vector<double>> l2vec = { {l2}, {0.0}, {0.0} };
    std::vector<std::vector<double>> l3vec = { {l3}, {3.0}, {0.0} };

    // Define 3x1 translation vectors (currently hardcoded)
    std::vector<std::vector<double>> Tb0 = { {l1}, {0.0}, {0.0} };
    std::vector<std::vector<double>> T01 = { {0.0}, {0.0}, {0.0} };
    std::vector<std::vector<double>> T12 = multiplyMatrices(C1_2, l2vec);
    std::vector<std::vector<double>> T23 = multiplyMatrices(C2_3, l3vec);
    std::vector<std::vector<double>> T34 = { {0.0}, {0.0}, {0.0} };

    auto Cb_actual = eul2rotm(0, 0, 0);
    auto Cb_prime = eul2rotm(0, 0, 180);

    std::vector<std::vector<double>> TbActual = { {0.0}, {-l6}, {0.0} };
    std::vector<std::vector<double>> TbPrime = { {0.0}, {0}, {0.0} };

    // Create transformation matrices
    std::vector<std::vector<double>> TransformbActual, TransformbPrime, Transformb0, Transform01, Transform12, Transform23, Transform34, Transform45, Transform56;
    createTransformationMatrix(Cb_actual, TbActual, TransformbActual);
    createTransformationMatrix(Cb_prime, TbPrime, TransformbPrime);

    createTransformationMatrix(Cb_0, Tb0, Transformb0);
    createTransformationMatrix(C0_1, T01, Transform01);
    createTransformationMatrix(C1_2, T12, Transform12);
    createTransformationMatrix(C2_3, T23, Transform23);
    createTransformationMatrix(C3_4, T34, Transform34);

    // Multiply the transformations
    Transformb0 = multiplyMatrices(TransformbActual, Transformb0);
    auto Transformb1 = multiplyMatrices(Transformb0, Transform01);
    auto Transformb2 = multiplyMatrices(Transformb1, Transform12);
    auto Transformb3 = multiplyMatrices(Transformb2, Transform23);
    auto Transformb4 = multiplyMatrices(Transformb3, Transform34);
    auto Transformb3p = multiplyMatrices(TransformbPrime, Transformb3);

    // Extract translation vectors
    std::vector<double> frame3pXYZ = { Transformb3p[0][3], Transformb3p[1][3], Transformb3p[2][3] };
    std::vector<double> frame4XYZ = { Transformb4[0][3], Transformb4[1][3], Transformb4[2][3] };

    // Compute unit vector j6
    std::vector<std::vector<double>> j6 = normalizeVector({
        { frame3pXYZ[0] - frame4XYZ[0] },
        { frame3pXYZ[1] - frame4XYZ[1] },
        { frame3pXYZ[2] - frame4XYZ[2] }
        });

    // Extract rotation matrix columns as column vectors
    std::vector<std::vector<double>> j4 = { { Transformb4[0][1] }, { Transformb4[1][1] }, { Transformb4[2][1] } };
    std::vector<std::vector<double>> k4 = { { Transformb4[0][2] }, { Transformb4[1][2] }, { Transformb4[2][2] } };
    std::vector<std::vector<double>> i4 = { { Transformb4[0][0] }, { Transformb4[1][0] }, { Transformb4[2][0] } };

    // Compute k5
    std::vector<std::vector<double>> cross_j6_k4 = crossProduct(j6, k4);
    std::vector<std::vector<double>> k5 = normalizeVector(cross_j6_k4);

    // Compute theta5
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
    double theta5 = 2.0 * atan(norm_k5_minus_i4 / norm_k5_plus_i4) * sign_k5_j4;
    double theta5_degrees = theta5 * (180.0 / M_PI);

    auto C4_5 = eul2rotm(0, 90, theta5);
    std::vector<std::vector<double>> T45 = { {0.0}, {0.0}, {0.0} };
    createTransformationMatrix(C4_5, T45, Transform45);
    auto Transformb5 = multiplyMatrices(Transformb4, Transform45);

    // Extract j5 and i5 as column vectors
    std::vector<std::vector<double>> j5 = { { Transformb5[0][1] }, { Transformb5[1][1] }, { Transformb5[2][1] } };
    std::vector<std::vector<double>> i5 = { { Transformb5[0][0] }, { Transformb5[1][0] }, { Transformb5[2][0] } };

    // Compute theta6
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
    double theta6 = -2.0 * atan(norm_j6_minus_j5 / norm_j6_plus_j5) * sign_j6_i5;

    auto C5_6 = eul2rotm(0, 0, theta6);
    std::vector<std::vector<double>> l8vec = { {0.0}, {l8}, {0.0} };
    std::vector<std::vector<double>> T56 = multiplyMatrices(C5_6, l8vec);
    createTransformationMatrix(C5_6, T56, Transform56);
    auto Transformb6 = multiplyMatrices(Transformb5, Transform56);

    // Extract x, y, z (from rows 1 to 3 of the resulting)
    std::vector<double> xyz = {
        Transformb6[0][3],
        Transformb6[1][3],
        Transformb6[2][3]
    };

    return xyz;
}

int main()
{
    // Example input parameters (you can change these as needed)
    double val1 = 0, val2 = 45, val3 = 150, val4 = 180 / 8, val5 = 30, val6 = 180 - 45, val7 = 45;


    //double theta1, double theta2, double q2, double theta1p, double theta2p, double q2p,  double theta4

    // Call the function and obtain the transformed coordinates
    std::vector<double> result = computeTransformedXYZ(val1, val2, val3, val4, val5, val6, val7);

    // Output the resulting x, y, and z values
    std::cout << "Transformed Coordinates:" << std::endl;
    std::cout << "X: " << result[0] << std::endl;
    std::cout << "Y: " << result[1] << std::endl;
    std::cout << "Z: " << result[2] << std::endl;

    return 0;
}