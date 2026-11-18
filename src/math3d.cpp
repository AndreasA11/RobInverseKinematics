#include "math3d.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif

namespace math3d {

Vector3 add(const Vector3& a, const Vector3& b) {
    return {a[0] + b[0], a[1] + b[1], a[2] + b[2]};
}

Vector3 subtract(const Vector3& a, const Vector3& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

Vector3 scale(const Vector3& v, double factor) {
    return {v[0] * factor, v[1] * factor, v[2] * factor};
}

double dot(const Vector3& a, const Vector3& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

double norm(const Vector3& v) {
    return std::hypot(v[0], v[1], v[2]);
}

Vector3 normalized(const Vector3& v) {
    double length = norm(v);
    if (!(length > 0.0) || !std::isfinite(length)) {
        throw std::invalid_argument("cannot normalize a zero or nonfinite vector");
    }
    return scale(v, 1.0 / length);
}

// *CHANGES*: Implement standard 3D right-handed cross product for Project 4.
Vector3 cross(const Vector3& a, const Vector3& b) {
    return {
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]
    };
}

Quaternion normalized(const Quaternion& q) {
    double length = std::hypot(std::hypot(q.x, q.y), std::hypot(q.z, q.w));
    if (!(length > 0.0) || !std::isfinite(length)) {
        throw std::invalid_argument("cannot normalize a zero or nonfinite quaternion");
    }
    return {q.x / length, q.y / length, q.z / length, q.w / length};
}

Quaternion multiply(const Quaternion& a, const Quaternion& b) {
    // Hamilton product: a rotation is applied after b's rotation.
    return normalized({
        a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
        a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
        a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
        a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z});
}

Quaternion from_axis_angle(const Vector3& axis, double angle) {
    Vector3 unit = normalized(axis);
    double sine = std::sin(angle / 2.0);
    return normalized({unit[0] * sine, unit[1] * sine, unit[2] * sine,
                       std::cos(angle / 2.0)});
}

Quaternion from_rpy(const Vector3& rpy) {
    Quaternion qx = from_axis_angle({1.0, 0.0, 0.0}, rpy[0]);
    Quaternion qy = from_axis_angle({0.0, 1.0, 0.0}, rpy[1]);
    Quaternion qz = from_axis_angle({0.0, 0.0, 1.0}, rpy[2]);
    return multiply(multiply(qz, qy), qx);
}

Matrix4 from_translation_rotation(const Vector3& translation, const Quaternion& rotation) {
    Quaternion q = normalized(rotation);
    double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    double xw = q.x * q.w, yw = q.y * q.w, zw = q.z * q.w;

    Matrix4 result;
    result(0, 0) = 1.0 - 2.0 * (yy + zz);
    result(1, 0) = 2.0 * (xy + zw);
    result(2, 0) = 2.0 * (xz - yw);
    result(0, 1) = 2.0 * (xy - zw);
    result(1, 1) = 1.0 - 2.0 * (xx + zz);
    result(2, 1) = 2.0 * (yz + xw);
    result(0, 2) = 2.0 * (xz + yw);
    result(1, 2) = 2.0 * (yz - xw);
    result(2, 2) = 1.0 - 2.0 * (xx + yy);
    result(0, 3) = translation[0];
    result(1, 3) = translation[1];
    result(2, 3) = translation[2];
    return result;
}

Matrix4 from_origin(const Vector3& xyz, const Vector3& rpy) {
    return from_translation_rotation(xyz, from_rpy(rpy));
}

Matrix4 multiply(const Matrix4& parent, const Matrix4& child) {
    Matrix4 result;
    for (std::size_t column = 0; column < 4; ++column) {
        for (std::size_t row = 0; row < 4; ++row) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 4; ++k) {
                sum += parent(row, k) * child(k, column);
            }
            result(row, column) = sum;
        }
    }
    return result;
}

Vector3 translation_of(const Matrix4& transform) {
    return {transform(0, 3), transform(1, 3), transform(2, 3)};
}

Quaternion rotation_of(const Matrix4& m) {
    // Choose the largest component first to avoid a tiny denominator near pi.
    double trace = m(0, 0) + m(1, 1) + m(2, 2);
    Quaternion q;
    if (trace > 0.0) {
        double s = 2.0 * std::sqrt(trace + 1.0);
        q.w = s / 4.0;
        q.x = (m(2, 1) - m(1, 2)) / s;
        q.y = (m(0, 2) - m(2, 0)) / s;
        q.z = (m(1, 0) - m(0, 1)) / s;
    } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
        double s = 2.0 * std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2));
        q.w = (m(2, 1) - m(1, 2)) / s;
        q.x = s / 4.0;
        q.y = (m(0, 1) + m(1, 0)) / s;
        q.z = (m(0, 2) + m(2, 0)) / s;
    } else if (m(1, 1) > m(2, 2)) {
        double s = 2.0 * std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2));
        q.w = (m(0, 2) - m(2, 0)) / s;
        q.x = (m(0, 1) + m(1, 0)) / s;
        q.y = s / 4.0;
        q.z = (m(1, 2) + m(2, 1)) / s;
    } else {
        double s = 2.0 * std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1));
        q.w = (m(1, 0) - m(0, 1)) / s;
        q.x = (m(0, 2) + m(2, 0)) / s;
        q.y = (m(1, 2) + m(2, 1)) / s;
        q.z = s / 4.0;
    }
    return normalized(q);
}

Vector3 transform_point(const Matrix4& transform, const Vector3& point) {
    return {
        transform(0, 0) * point[0] + transform(0, 1) * point[1] +
            transform(0, 2) * point[2] + transform(0, 3),
        transform(1, 0) * point[0] + transform(1, 1) * point[1] +
            transform(1, 2) * point[2] + transform(1, 3),
        transform(2, 0) * point[0] + transform(2, 1) * point[1] +
            transform(2, 2) * point[2] + transform(2, 3)};
}

// *CHANGES*: Implement 3D vector rotation using 3x3 rotation submatrix.
Vector3 rotate_vector(const Matrix4& transform, const Vector3& v) {
    return {
        transform(0, 0) * v[0] + transform(0, 1) * v[1] + transform(0, 2) * v[2],
        transform(1, 0) * v[0] + transform(1, 1) * v[1] + transform(1, 2) * v[2],
        transform(2, 0) * v[0] + transform(2, 1) * v[1] + transform(2, 2) * v[2]
    };
}

// *CHANGES*: Convert unit quaternion (x, y, z, w) to 3x3 rotation matrix according to spec/IK_API.md.
Matrix3 from_quaternion(const Quaternion& q_raw) {
    Quaternion q = normalized(q_raw);
    double xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    double xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    double xw = q.x * q.w, yw = q.y * q.w, zw = q.z * q.w;

    Matrix3 r;
    r(0, 0) = 1.0 - 2.0 * (yy + zz);
    r(0, 1) = 2.0 * (xy - zw);
    r(0, 2) = 2.0 * (xz + yw);

    r(1, 0) = 2.0 * (xy + zw);
    r(1, 1) = 1.0 - 2.0 * (xx + zz);
    r(1, 2) = 2.0 * (yz - xw);

    r(2, 0) = 2.0 * (xz - yw);
    r(2, 1) = 2.0 * (yz + xw);
    r(2, 2) = 1.0 - 2.0 * (xx + yy);
    return r;
}

// *CHANGES*: Extract 3x3 rotation matrix from Matrix4 transform.
Matrix3 rotation_matrix_of(const Matrix4& m) {
    Matrix3 r;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            r(row, col) = m(row, col);
        }
    }
    return r;
}

// *CHANGES*: 3x3 matrix multiplication.
Matrix3 multiply(const Matrix3& a, const Matrix3& b) {
    Matrix3 result;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            double sum = 0.0;
            for (std::size_t k = 0; k < 3; ++k) {
                sum += a(row, k) * b(k, col);
            }
            result(row, col) = sum;
        }
    }
    return result;
}

// *CHANGES*: 3x3 matrix transpose.
Matrix3 transpose(const Matrix3& m) {
    Matrix3 result;
    for (std::size_t row = 0; row < 3; ++row) {
        for (std::size_t col = 0; col < 3; ++col) {
            result(row, col) = m(col, row);
        }
    }
    return result;
}

// *CHANGES*: Calculate geodesic rotation angle in [0, pi] from 3x3 rotation matrix.
double rotation_angle(const Matrix3& r) {
    Vector3 v{
        r(2, 1) - r(1, 2),
        r(0, 2) - r(2, 0),
        r(1, 0) - r(0, 1)
    };
    double s = 0.5 * norm(v);
    double trace = r(0, 0) + r(1, 1) + r(2, 2);
    double c = 0.5 * (trace - 1.0);
    c = std::clamp(c, -1.0, 1.0);
    return std::atan2(s, c);
}

// *CHANGES*: Compute rotation vector (axis * angle) in root frame for full-pose IK step error.
Vector3 rotation_vector(const Matrix3& r) {
    Vector3 v{
        r(2, 1) - r(1, 2),
        r(0, 2) - r(2, 0),
        r(1, 0) - r(0, 1)
    };
    double v_norm = norm(v);
    double trace = r(0, 0) + r(1, 1) + r(2, 2);
    double c = std::clamp(0.5 * (trace - 1.0), -1.0, 1.0);
    double theta = std::atan2(0.5 * v_norm, c);

    if (theta < 1e-12) {
        return {0.0, 0.0, 0.0};
    }
    if (M_PI - theta < 1e-6) {
        // Angle near pi: R = 2 u u^T - I => 2 u_i u_j = R_ij + delta_ij
        std::size_t k = 0;
        double max_diag = r(0, 0);
        if (r(1, 1) > max_diag) { max_diag = r(1, 1); k = 1; }
        if (r(2, 2) > max_diag) { max_diag = r(2, 2); k = 2; }
        Vector3 u{0.0, 0.0, 0.0};
        u[k] = std::sqrt(std::max(0.0, (max_diag + 1.0) / 2.0));
        if (u[k] > 1e-6) {
            for (std::size_t j = 0; j < 3; ++j) {
                if (j != k) {
                    u[j] = r(k, j) / (2.0 * u[k]);
                }
            }
        }
        double u_len = norm(u);
        if (u_len > 0.0) {
            u = scale(u, 1.0 / u_len);
        }
        return scale(u, theta);
    }
    if (theta < 1e-4) {
        // Taylor expansion for small angle: theta / (2 * sin(theta)) ~ 0.5 * (1 + theta^2 / 6)
        double factor = 0.5 * (1.0 + theta * theta / 6.0);
        return scale(v, factor);
    }
    return scale(v, theta / v_norm);
}

} // namespace math3d
