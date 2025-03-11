#include "log/log.hpp"
#include <cmath>
#include <limits>

#define __clamp__(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

/*
 * Inverse kinematics 3DOF formula:
 * A = sqrt(z^2 + y^2)
 * B = sqrt(A^2 - L1^2)
 * C = sqrt(x^2 + B^2)
 * q1 = atan2(y, z) + atan2(B, L1)
 * q2 = atan2(x, B) + asin(L3 * sin(q3) / C)
 * q3 = acos((L2^2 + L3^2 - C^2) / (2 * L2 * L3))
 */

namespace cerberus_iksolver
{
    std::tuple<double, double, double> iksolver3d(double x, double y, double z, double L1, double L2, double L3)
    {
        double A = std::sqrt(y * y + z * z);

        if (A < L1)
            return {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};

        double B = std::sqrt(A * A - L1 * L1);
        double C = std::sqrt(x * x + B * B);

        if (C > L2 + L3 || C < std::abs(L2 - L3))
            return {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};

        double q1 = std::atan2(y, z) + std::atan2(B, L1);
        double q3 = std::acos(__clamp__((L2 * L2 + L3 * L3 - C * C) / (2 * L2 * L3), -1.0, 1.0));
        double q2 = std::atan2(x, B) + std::asin(__clamp__(L3 * std::sin(q3) / C, -1.0, 1.0));

        return {q1, q2, q3};
    }
} // namespace cerberus_iksolver
