#include <iostream>
#include <vector>
#include "transformations.h"
#include <tuple>
#include <cmath>

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
    printCoordinateFrame("Transformb0", Transformb0);
    printCoordinateFrame("Transformb1", Transformb1);
    printCoordinateFrame("Transformb2", Transformb2);
    printCoordinateFrame("Transformb3", Transformb3);

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
    printCoordinateFrame("Transformb0p", Transformb0p);
    printCoordinateFrame("Transformb1p", Transformb1p);
    printCoordinateFrame("Transformb2p", Transformb2p);
    printCoordinateFrame("Transformb3p", Transformb3p);

    // Now continue with the rest of the transformations
    auto C3_4 = eul2rotm(-90, 0, theta4);
    std::vector<std::vector<double>> Tb4 = { {0.0}, {0.0}, {0.0} };

    std::vector<std::vector<double>> Transform34;
    createTransformationMatrix(C3_4, Tb4, Transform34);
    Transformb4 = multiplyMatrices(Transformb3, Transform34); // Just an example

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
    Transformb5 = multiplyMatrices(Transformb4, Transform45);
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
    Transformb6 = multiplyMatrices(Transformb5, Transform56);
    printCoordinateFrame("Transformb6", Transformb6);

    // Variables to store computed angles
    double yaw, pitch, roll;

    // Compute Euler Angles
    std::vector<double> eulerAngles = computeEulerAngles(Transformb6);

    return { Transformb6[0][3], Transformb6[1][3], Transformb6[2][3], eulerAngles[0], eulerAngles[1], eulerAngles[2] };
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

int main()
{
    // Example input parameters (you can change these as needed)
    val1 = 90, val2 = 30, val3 = 180 - 60, val4 = 90, val5 = 30, val6 = 180 - 60, val7 = -58.5 + 10;

    //double theta1, double theta2, double q2, double theta1p, double theta2p, double q2p,  double theta4

    // Call the function and obtain the transformed coordinates
    std::vector<double> result = computeTransformedXYZ(val1, val2, val3, val4, val5, val6, val7);

    // Output the resulting x, y, and z values
    std::cout << "\n" << "Transformed Coordinates:" << std::endl;
    std::cout << "X: " << result[0] << std::endl;
    std::cout << "Y: " << result[1] << std::endl;
    std::cout << "Z: " << result[2] << "\n" << std::endl;

    // Display results
    std::cout << "Rotation Angles:" << std::endl;
    std::cout << "Yaw (Z-axis): " << result[3] << " degrees" << std::endl;
    std::cout << "Pitch (Y-axis): " << result[4] << " degrees" << std::endl;
    std::cout << "Roll (X-axis): " << result[5] << " degrees" << "\n" << std::endl;

    // Define a 6x1 wrench vector (Fx, Fy, Fz, Tx, Ty, Tz)
    std::vector<double> W = { 2, 3, 0, 0, 0, 0.2 };

    // Construct J matrix (7x6)
    auto JT = computeJointTorques(W);
    printVector("JT", JT);

    return 0;
}