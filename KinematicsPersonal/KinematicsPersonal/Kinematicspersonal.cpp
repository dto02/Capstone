#include <iostream>
#include <vector>
#include "transformations.h"
#include <tuple>

// Length Parameters
double l1 = 4.04 / 2.0;  // Half of distance between panto motors
double l2 = 16.820;      // Motor to elbow linkage length
double l5 = 16.820;      // Motor to elbow linkage length (duplicate value)
double l3 = 17.09;       // Elbow to tip linkage length
double l4 = 17.09;       // Elbow to tip linkage length (duplicate value)
double l6 = 20;          // Half of distance between shoulder motors
double l8 = 20;

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

// Function to print the coordinate frame (translation vector)
void printCoordinateFrame(const std::string& name, const std::vector<std::vector<double>>& transform) {
    std::cout << name << " Coordinate Frame: ("
        << transform[0][3] << ", "
        << transform[1][3] << ", "
        << transform[2][3] << ")"
        << std::endl;
}

// Function to compute the transformation from b to 3 (based on theta1, theta2, theta3)
// Returns a tuple containing Transformb0, Transformb1, Transformb2, and Transformb3
std::tuple<std::vector<std::vector<double>>, std::vector<std::vector<double>>,
    std::vector<std::vector<double>>, std::vector<std::vector<double>>>
    computeBto3Transformation(double theta1, double theta2, double theta3, double side)
{
    // Define the rotation matrices based on the provided angles
    auto Cb_0 = eul2rotm(90, 0, 0);
    auto C0_1 = eul2rotm(0, -90, theta1);
    auto C1_2 = eul2rotm(0, 0, theta2);
    auto C2_3 = eul2rotm(0, 0, theta3);

    // Define translation vectors
    std::vector<std::vector<double>> Tb0 = { {side*l6}, {-l1}, {0.0} };
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
        transform5bar = computeBto3Transformation(theta1, theta2, theta3,-1);

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

int main()
{
    // Example input parameters (you can change these as needed)
    double val1 = 0, val2 = 45, val3 = 150, val4 = 22.5, val5 = 30, val6 = 135, val7 = 45;

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