/**
 * @file math_utils.hpp
 *
 * Lightweight math utilities for the PX4 fixed-wing controller stack.
 *
 * In the real PX4 firmware, these come from the `matrix` library (a header-only
 * Eigen-like library) and from `mathlib`. This standalone version provides the
 * minimum required to compile and run the controllers outside PX4.
 *
 * Conventions:
 *   - NED (North-East-Down) coordinate frame
 *   - Angles in radians
 *   - SI units throughout (metres, seconds, kg, etc.)
 */

#pragma once

#include <cmath>
#include <cstring>
#include <algorithm>

namespace math {

/* ------------------------------------------------------------------ */
/* Constants                                                           */
/* ------------------------------------------------------------------ */

static constexpr float GRAVITY     = 9.80665f;   ///< Standard gravity [m/s^2]
static constexpr float PI          = 3.14159265358979323846f;
static constexpr float TWO_PI      = 2.0f * PI;
static constexpr float DEG_TO_RAD  = PI / 180.0f;
static constexpr float RAD_TO_DEG  = 180.0f / PI;
static constexpr float FLT_EPSILON = 1.192092896e-07f;

/* ------------------------------------------------------------------ */
/* Scalar utilities                                                    */
/* ------------------------------------------------------------------ */

/** Clamp a value to [lo, hi] */
template <typename T>
inline T constrain(T val, T lo, T hi) {
    return (val < lo) ? lo : (val > hi) ? hi : val;
}

/** Linear interpolation: returns a + t*(b-a) */
inline float lerp(float a, float b, float t) {
    return a + t * (b - a);
}

/**
 * Interpolate y between two (x,y) anchor points.
 *
 * Given x1 < x2, maps:
 *   x <= x1  ->  y1
 *   x >= x2  ->  y2
 *   x in between -> linear blend
 *
 * Used for airspeed-dependent trim scheduling.
 */
inline float interpolate(float x, float x1, float x2, float y1, float y2) {
    if (x <= x1) return y1;
    if (x >= x2) return y2;
    return y1 + (y2 - y1) * (x - x1) / (x2 - x1);
}

/** Wrap an angle to [-pi, pi) */
inline float wrap_pi(float angle) {
    while (angle >  PI) angle -= TWO_PI;
    while (angle < -PI) angle += TWO_PI;
    return angle;
}

/** Sign function: returns -1, 0, or +1 */
inline float sign(float x) {
    if (x > 0.0f) return 1.0f;
    if (x < 0.0f) return -1.0f;
    return 0.0f;
}

/* ------------------------------------------------------------------ */
/* 3D Vector                                                           */
/* ------------------------------------------------------------------ */

/**
 * Simple 3-element vector (replaces matrix::Vector3f from PX4).
 *
 * Provides element access by index and by named axes (x,y,z),
 * plus basic arithmetic operators.
 */
struct Vector3f {
    float data[3]{0.0f, 0.0f, 0.0f};

    Vector3f() = default;
    Vector3f(float x, float y, float z) { data[0] = x; data[1] = y; data[2] = z; }

    float& operator()(int i)       { return data[i]; }
    float  operator()(int i) const { return data[i]; }

    float& x()       { return data[0]; }
    float  x() const { return data[0]; }
    float& y()       { return data[1]; }
    float  y() const { return data[1]; }
    float& z()       { return data[2]; }
    float  z() const { return data[2]; }

    Vector3f operator+(const Vector3f& v) const {
        return {data[0]+v.data[0], data[1]+v.data[1], data[2]+v.data[2]};
    }
    Vector3f operator-(const Vector3f& v) const {
        return {data[0]-v.data[0], data[1]-v.data[1], data[2]-v.data[2]};
    }
    Vector3f operator*(float s) const {
        return {data[0]*s, data[1]*s, data[2]*s};
    }
    Vector3f& operator+=(const Vector3f& v) {
        data[0]+=v.data[0]; data[1]+=v.data[1]; data[2]+=v.data[2]; return *this;
    }

    float norm() const { return std::sqrt(data[0]*data[0]+data[1]*data[1]+data[2]*data[2]); }
    float norm_squared() const { return data[0]*data[0]+data[1]*data[1]+data[2]*data[2]; }

    void zero() { data[0]=0; data[1]=0; data[2]=0; }
};

/* ------------------------------------------------------------------ */
/* 3x3 Rotation Matrix (DCM)                                           */
/* ------------------------------------------------------------------ */

/**
 * 3x3 Direction Cosine Matrix (rotation matrix).
 *
 * Stored row-major: m[row][col].
 * In PX4 this is matrix::Dcmf.
 */
struct Matrix3f {
    float m[3][3]{};

    Matrix3f() { std::memset(m, 0, sizeof(m)); }

    /**
     * Construct from Euler angles (3-2-1 / ZYX convention).
     *
     * This is the rotation from NED (world) frame to body frame:
     *   R = Rx(phi) * Ry(theta) * Rz(psi)
     *
     * @param phi   Roll angle [rad]
     * @param theta Pitch angle [rad]
     * @param psi   Yaw angle [rad]
     */
    static Matrix3f from_euler(float phi, float theta, float psi) {
        Matrix3f R;
        float cp = std::cos(phi),   sp = std::sin(phi);
        float ct = std::cos(theta), st = std::sin(theta);
        float cy = std::cos(psi),   sy = std::sin(psi);

        R.m[0][0] = ct*cy;           R.m[0][1] = ct*sy;           R.m[0][2] = -st;
        R.m[1][0] = sp*st*cy-cp*sy;  R.m[1][1] = sp*st*sy+cp*cy; R.m[1][2] = sp*ct;
        R.m[2][0] = cp*st*cy+sp*sy;  R.m[2][1] = cp*st*sy-sp*cy; R.m[2][2] = cp*ct;
        return R;
    }

    /**
     * Extract Euler angles from the rotation matrix.
     * Returns (roll, pitch, yaw) in radians.
     */
    void to_euler(float& roll, float& pitch, float& yaw) const {
        pitch = std::asin(-m[0][2]);
        roll  = std::atan2(m[1][2], m[2][2]);
        yaw   = std::atan2(m[0][1], m[0][0]);
    }

    float& operator()(int r, int c)       { return m[r][c]; }
    float  operator()(int r, int c) const { return m[r][c]; }
};

/* ------------------------------------------------------------------ */
/* Quaternion                                                          */
/* ------------------------------------------------------------------ */

/**
 * Unit quaternion for attitude representation.
 *
 * Convention: q = [w, x, y, z] where w is the scalar part.
 * Represents the rotation from NED to body frame.
 */
struct Quatf {
    float w{1.0f}, x{0.0f}, y{0.0f}, z{0.0f};

    Quatf() = default;
    Quatf(float w_, float x_, float y_, float z_) : w(w_), x(x_), y(y_), z(z_) {}

    /** Construct from Euler angles (ZYX convention) */
    static Quatf from_euler(float roll, float pitch, float yaw) {
        float cr = std::cos(roll*0.5f),  sr = std::sin(roll*0.5f);
        float cp = std::cos(pitch*0.5f), sp = std::sin(pitch*0.5f);
        float cy = std::cos(yaw*0.5f),   sy = std::sin(yaw*0.5f);

        Quatf q;
        q.w = cr*cp*cy + sr*sp*sy;
        q.x = sr*cp*cy - cr*sp*sy;
        q.y = cr*sp*cy + sr*cp*sy;
        q.z = cr*cp*sy - sr*sp*cy;
        return q;
    }

    /** Convert to Euler angles */
    void to_euler(float& roll, float& pitch, float& yaw) const {
        // Roll (x-axis rotation)
        float sinr_cosp = 2.0f * (w*x + y*z);
        float cosr_cosp = 1.0f - 2.0f * (x*x + y*y);
        roll = std::atan2(sinr_cosp, cosr_cosp);

        // Pitch (y-axis rotation)
        float sinp = 2.0f * (w*y - z*x);
        if (std::fabs(sinp) >= 1.0f)
            pitch = std::copysign(PI / 2.0f, sinp);
        else
            pitch = std::asin(sinp);

        // Yaw (z-axis rotation)
        float siny_cosp = 2.0f * (w*z + x*y);
        float cosy_cosp = 1.0f - 2.0f * (y*y + z*z);
        yaw = std::atan2(siny_cosp, cosy_cosp);
    }

    /** Convert to rotation matrix */
    Matrix3f to_dcm() const {
        float roll, pitch, yaw;
        to_euler(roll, pitch, yaw);
        return Matrix3f::from_euler(roll, pitch, yaw);
    }
};

} // namespace math
