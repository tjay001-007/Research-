// SPDX-License-Identifier: BSD-3-Clause
// PX4 Attitude Control Library - Math Types
// Lightweight vector, matrix, and quaternion types for flight control

#pragma once

#include <cmath>
#include <cstring>
#include <algorithm>

namespace px4_att_control {

// ---------------------------------------------------------------------------
// Vec3: 3-element vector
// ---------------------------------------------------------------------------
struct Vec3 {
    float x{0.f}, y{0.f}, z{0.f};

    Vec3() = default;
    Vec3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}

    float  operator()(int i) const { return (&x)[i]; }
    float& operator()(int i)       { return (&x)[i]; }

    Vec3 operator+(const Vec3& v) const { return {x + v.x, y + v.y, z + v.z}; }
    Vec3 operator-(const Vec3& v) const { return {x - v.x, y - v.y, z - v.z}; }
    Vec3 operator*(float s)       const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s)       const { float inv = 1.f / s; return *this * inv; }
    Vec3 operator-()              const { return {-x, -y, -z}; }

    Vec3& operator+=(const Vec3& v) { x += v.x; y += v.y; z += v.z; return *this; }
    Vec3& operator-=(const Vec3& v) { x -= v.x; y -= v.y; z -= v.z; return *this; }
    Vec3& operator*=(float s)       { x *= s; y *= s; z *= s; return *this; }

    float dot(const Vec3& v)   const { return x * v.x + y * v.y + z * v.z; }
    float norm_squared()       const { return dot(*this); }
    float norm()               const { return std::sqrt(norm_squared()); }

    Vec3 cross(const Vec3& v) const {
        return {y * v.z - z * v.y,
                z * v.x - x * v.z,
                x * v.y - y * v.x};
    }

    Vec3 normalized() const {
        float n = norm();
        if (n < 1e-10f) return {0.f, 0.f, 0.f};
        return *this / n;
    }

    // Element-wise clamp
    Vec3 clamped(float lo, float hi) const {
        return {std::clamp(x, lo, hi),
                std::clamp(y, lo, hi),
                std::clamp(z, lo, hi)};
    }

    // Element-wise operations
    static Vec3 element_min(const Vec3& a, const Vec3& b) {
        return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
    }
    static Vec3 element_max(const Vec3& a, const Vec3& b) {
        return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
    }

    static Vec3 zero() { return {0.f, 0.f, 0.f}; }
};

inline Vec3 operator*(float s, const Vec3& v) { return v * s; }

// ---------------------------------------------------------------------------
// Mat33: 3x3 matrix (row-major)
// ---------------------------------------------------------------------------
struct Mat33 {
    float data[3][3]{};

    Mat33() = default;

    float  operator()(int r, int c) const { return data[r][c]; }
    float& operator()(int r, int c)       { return data[r][c]; }

    Vec3 row(int r) const { return {data[r][0], data[r][1], data[r][2]}; }
    Vec3 col(int c) const { return {data[0][c], data[1][c], data[2][c]}; }

    Vec3 operator*(const Vec3& v) const {
        return {row(0).dot(v), row(1).dot(v), row(2).dot(v)};
    }

    Mat33 operator*(const Mat33& m) const {
        Mat33 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r.data[i][j] = row(i).dot(m.col(j));
        return r;
    }

    Mat33 transposed() const {
        Mat33 r;
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                r.data[i][j] = data[j][i];
        return r;
    }

    // Determinant
    float det() const {
        return data[0][0] * (data[1][1] * data[2][2] - data[1][2] * data[2][1])
             - data[0][1] * (data[1][0] * data[2][2] - data[1][2] * data[2][0])
             + data[0][2] * (data[1][0] * data[2][1] - data[1][1] * data[2][0]);
    }

    // Inverse (returns identity on singular)
    Mat33 inversed() const {
        float d = det();
        if (std::fabs(d) < 1e-10f) return identity();
        float inv_d = 1.f / d;
        Mat33 r;
        r.data[0][0] =  (data[1][1] * data[2][2] - data[1][2] * data[2][1]) * inv_d;
        r.data[0][1] = -(data[0][1] * data[2][2] - data[0][2] * data[2][1]) * inv_d;
        r.data[0][2] =  (data[0][1] * data[1][2] - data[0][2] * data[1][1]) * inv_d;
        r.data[1][0] = -(data[1][0] * data[2][2] - data[1][2] * data[2][0]) * inv_d;
        r.data[1][1] =  (data[0][0] * data[2][2] - data[0][2] * data[2][0]) * inv_d;
        r.data[1][2] = -(data[0][0] * data[1][2] - data[0][2] * data[1][0]) * inv_d;
        r.data[2][0] =  (data[1][0] * data[2][1] - data[1][1] * data[2][0]) * inv_d;
        r.data[2][1] = -(data[0][0] * data[2][1] - data[0][1] * data[2][0]) * inv_d;
        r.data[2][2] =  (data[0][0] * data[1][1] - data[0][1] * data[1][0]) * inv_d;
        return r;
    }

    static Mat33 identity() {
        Mat33 r;
        r.data[0][0] = r.data[1][1] = r.data[2][2] = 1.f;
        return r;
    }

    static Mat33 diag(float a, float b, float c) {
        Mat33 r;
        r.data[0][0] = a; r.data[1][1] = b; r.data[2][2] = c;
        return r;
    }
};

// ---------------------------------------------------------------------------
// Quatf: Hamilton quaternion (w, x, y, z) for attitude representation
// Follows PX4 convention: q = w + xi + yj + zk
// ---------------------------------------------------------------------------
struct Quatf {
    float w{1.f}, x{0.f}, y{0.f}, z{0.f};

    Quatf() = default;
    Quatf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    // Construct from axis-angle
    static Quatf from_axis_angle(const Vec3& axis, float angle) {
        float ha = angle * 0.5f;
        float sa = std::sin(ha);
        Vec3 a = axis.normalized();
        return {std::cos(ha), a.x * sa, a.y * sa, a.z * sa};
    }

    // Construct from Euler angles (ZYX convention: yaw, pitch, roll)
    static Quatf from_euler(float roll, float pitch, float yaw) {
        float cr = std::cos(roll  * 0.5f), sr = std::sin(roll  * 0.5f);
        float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
        float cy = std::cos(yaw   * 0.5f), sy = std::sin(yaw   * 0.5f);
        return {cr * cp * cy + sr * sp * sy,
                sr * cp * cy - cr * sp * sy,
                cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy};
    }

    // Construct from rotation matrix
    static Quatf from_dcm(const Mat33& R) {
        Quatf q;
        float tr = R(0,0) + R(1,1) + R(2,2);
        if (tr > 0.f) {
            float s = 2.f * std::sqrt(tr + 1.f);
            q.w = 0.25f * s;
            q.x = (R(2,1) - R(1,2)) / s;
            q.y = (R(0,2) - R(2,0)) / s;
            q.z = (R(1,0) - R(0,1)) / s;
        } else if (R(0,0) > R(1,1) && R(0,0) > R(2,2)) {
            float s = 2.f * std::sqrt(1.f + R(0,0) - R(1,1) - R(2,2));
            q.w = (R(2,1) - R(1,2)) / s;
            q.x = 0.25f * s;
            q.y = (R(0,1) + R(1,0)) / s;
            q.z = (R(0,2) + R(2,0)) / s;
        } else if (R(1,1) > R(2,2)) {
            float s = 2.f * std::sqrt(1.f + R(1,1) - R(0,0) - R(2,2));
            q.w = (R(0,2) - R(2,0)) / s;
            q.x = (R(0,1) + R(1,0)) / s;
            q.y = 0.25f * s;
            q.z = (R(1,2) + R(2,1)) / s;
        } else {
            float s = 2.f * std::sqrt(1.f + R(2,2) - R(0,0) - R(1,1));
            q.w = (R(1,0) - R(0,1)) / s;
            q.x = (R(0,2) + R(2,0)) / s;
            q.y = (R(1,2) + R(2,1)) / s;
            q.z = 0.25f * s;
        }
        return q.normalized();
    }

    // Hamilton product: this * r
    Quatf operator*(const Quatf& r) const {
        return {w * r.w - x * r.x - y * r.y - z * r.z,
                w * r.x + x * r.w + y * r.z - z * r.y,
                w * r.y - x * r.z + y * r.w + z * r.x,
                w * r.z + x * r.y - y * r.x + z * r.w};
    }

    Quatf conjugate() const { return {w, -x, -y, -z}; }
    Quatf inversed() const  { return conjugate() / norm_squared(); }

    Quatf operator*(float s) const { return {w * s, x * s, y * s, z * s}; }
    Quatf operator/(float s) const { float inv = 1.f / s; return *this * inv; }

    float norm_squared() const { return w * w + x * x + y * y + z * z; }
    float norm()         const { return std::sqrt(norm_squared()); }

    Quatf normalized() const {
        float n = norm();
        if (n < 1e-10f) return Quatf{};
        return *this / n;
    }

    // Ensure canonical form (w >= 0) for unique representation
    Quatf canonical() const {
        return (w < 0.f) ? Quatf{-w, -x, -y, -z} : *this;
    }

    // Convert to rotation matrix (DCM)
    Mat33 to_dcm() const {
        Mat33 R;
        float xx = x * x, yy = y * y, zz = z * z;
        float xy = x * y, xz = x * z, yz = y * z;
        float wx = w * x, wy = w * y, wz = w * z;

        R(0,0) = 1.f - 2.f * (yy + zz);
        R(0,1) = 2.f * (xy - wz);
        R(0,2) = 2.f * (xz + wy);
        R(1,0) = 2.f * (xy + wz);
        R(1,1) = 1.f - 2.f * (xx + zz);
        R(1,2) = 2.f * (yz - wx);
        R(2,0) = 2.f * (xz - wy);
        R(2,1) = 2.f * (yz + wx);
        R(2,2) = 1.f - 2.f * (xx + yy);
        return R;
    }

    // Extract Euler angles (roll, pitch, yaw) from quaternion
    Vec3 to_euler() const {
        Mat33 R = to_dcm();
        float pitch = std::asin(-std::clamp(R(2,0), -1.f, 1.f));
        float roll, yaw;
        if (std::fabs(R(2,0)) < 0.999f) {
            roll = std::atan2(R(2,1), R(2,2));
            yaw  = std::atan2(R(1,0), R(0,0));
        } else {
            roll = std::atan2(-R(1,2), R(1,1));
            yaw  = 0.f;
        }
        return {roll, pitch, yaw};
    }

    // Imaginary (vector) part
    Vec3 imag() const { return {x, y, z}; }

    // Rotate a vector by this quaternion: q * v * q^{-1}
    Vec3 rotate(const Vec3& v) const {
        return to_dcm() * v;
    }

    // Reduced attitude error vector (used by PX4 attitude controller)
    // Returns the rotation vector from this to target, expressed in body frame
    // eq_reduced = 2 * sign(q_error.w) * q_error.xyz
    static Vec3 reduced_error(const Quatf& q, const Quatf& q_sp) {
        Quatf q_err = q.inversed() * q_sp;
        // Ensure shortest path
        if (q_err.w < 0.f) {
            q_err = q_err * -1.f;
        }
        return q_err.imag() * 2.f;
    }
};

// ---------------------------------------------------------------------------
// Utility functions
// ---------------------------------------------------------------------------

constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.f;
constexpr float RAD_TO_DEG = 180.f / 3.14159265358979323846f;

inline float constrain(float val, float lo, float hi) {
    return std::clamp(val, lo, hi);
}

inline float wrap_pi(float angle) {
    while (angle >  3.14159265f) angle -= 2.f * 3.14159265f;
    while (angle < -3.14159265f) angle += 2.f * 3.14159265f;
    return angle;
}

} // namespace px4_att_control
