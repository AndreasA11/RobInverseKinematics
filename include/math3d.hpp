#ifndef PROJECT3_MATH3D_HPP
#define PROJECT3_MATH3D_HPP

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <utility>

namespace math3d {

using Vector3 = std::array<double, 3>;

Vector3 add(const Vector3& a, const Vector3& b);
Vector3 subtract(const Vector3& a, const Vector3& b);
Vector3 scale(const Vector3& v, double factor);
double dot(const Vector3& a, const Vector3& b);
double norm(const Vector3& v);
Vector3 normalized(const Vector3& v);
// *CHANGES*: Add 3D vector cross product for Project 4 Jacobian calculations.
Vector3 cross(const Vector3& a, const Vector3& b);

struct Quaternion {
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double w{1.0};
};

Quaternion normalized(const Quaternion& q);
Quaternion multiply(const Quaternion& a, const Quaternion& b);
Quaternion from_rpy(const Vector3& rpy);
Quaternion from_axis_angle(const Vector3& axis, double angle);

// Matrices are column-major: element(row, column) is data[4 * column + row].
struct Matrix4 {
    std::array<double, 16> data{
        1.0, 0.0, 0.0, 0.0,
        0.0, 1.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 1.0};

    double operator()(std::size_t row, std::size_t column) const {
        return data[4 * column + row];
    }
    double& operator()(std::size_t row, std::size_t column) {
        return data[4 * column + row];
    }
};

Matrix4 from_translation_rotation(const Vector3& translation, const Quaternion& rotation);
Matrix4 from_origin(const Vector3& xyz, const Vector3& rpy);
Matrix4 multiply(const Matrix4& parent, const Matrix4& child);
Vector3 translation_of(const Matrix4& transform);
Quaternion rotation_of(const Matrix4& transform);
Vector3 transform_point(const Matrix4& transform, const Vector3& point);
// *CHANGES*: Add vector rotation using upper-left 3x3 block of Matrix4 for Jacobian joint axes.
Vector3 rotate_vector(const Matrix4& transform, const Vector3& v);

// *CHANGES*: Add 3x3 matrix and rotation utilities for Project 4 full-pose orientation IK.
struct Matrix3 {
    std::array<double, 9> data{
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };

    double operator()(std::size_t row, std::size_t column) const {
        return data[3 * row + column];
    }
    double& operator()(std::size_t row, std::size_t column) {
        return data[3 * row + column];
    }
};

Matrix3 from_quaternion(const Quaternion& q);
Matrix3 rotation_matrix_of(const Matrix4& m);
Matrix3 multiply(const Matrix3& a, const Matrix3& b);
Matrix3 transpose(const Matrix3& m);
double rotation_angle(const Matrix3& r);
Vector3 rotation_vector(const Matrix3& r);

// *CHANGES*: Gaussian elimination with partial pivoting for small linear systems (e.g. 3x3 and 6x6).
template <std::size_t N>
bool solve_linear_system(std::array<std::array<double, N>, N> A,
                         std::array<double, N> b,
                         std::array<double, N>& x) {
    for (std::size_t col = 0; col < N; ++col) {
        std::size_t pivot_row = col;
        double max_val = std::abs(A[col][col]);
        for (std::size_t row = col + 1; row < N; ++row) {
            double val = std::abs(A[row][col]);
            if (val > max_val) {
                max_val = val;
                pivot_row = row;
            }
        }
        if (max_val < 1e-12) {
            return false;
        }
        if (pivot_row != col) {
            std::swap(A[col], A[pivot_row]);
            std::swap(b[col], b[pivot_row]);
        }
        double pivot = A[col][col];
        for (std::size_t row = col + 1; row < N; ++row) {
            double factor = A[row][col] / pivot;
            A[row][col] = 0.0;
            for (std::size_t c = col + 1; c < N; ++c) {
                A[row][c] -= factor * A[col][c];
            }
            b[row] -= factor * b[col];
        }
    }
    for (int row = static_cast<int>(N) - 1; row >= 0; --row) {
        double sum = b[row];
        for (std::size_t c = static_cast<std::size_t>(row + 1); c < N; ++c) {
            sum -= A[row][c] * x[c];
        }
        x[row] = sum / A[row][row];
    }
    return true;
}

} // namespace math3d

#endif
