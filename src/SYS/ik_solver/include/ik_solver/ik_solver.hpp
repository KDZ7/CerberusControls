#include <cmath>
#include <vector>
#include <limits>
#include <iostream>

#define __clamp__(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))

namespace cerberus_solver
{
    std::vector<double> solver(double x, double y, double z, double L1, double L2, double L3)
    {
        std::vector<double> q(3, 0.0);
        double A = std::sqrt(__clamp__(z * z + y * y, 1e-6, 1e6));
        double B = std::sqrt(__clamp__(A * A - L1 * L1, 1e-6, 1e6));
        double C = std::sqrt(__clamp__(x * x + B * B, 1e-6, 1e6));

        if (C > L2 + L3 || C < std::abs(L2 - L3))
        {
            std::cerr << "[ERROR IK Solver]: Target [" << x << ", " << y << ", " << z << "] is out of reach." << std::endl;
            return {std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()};
        }

        q[0] = std::atan2(y, z) + std::atan2(B, L1);
        q[2] = std::acos(__clamp__((L2 * L2 + L3 * L3 - C * C) / (2 * L2 * L3), -1.0, 1.0));
        q[1] = std::atan2(x, B) + std::asin(__clamp__(L3 * std::sin(q[2]) / C, -1.0, 1.0));

        return q;
    }

} // namespace cerberus_solver