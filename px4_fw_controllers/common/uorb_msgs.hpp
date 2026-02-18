/**
 * @file uorb_msgs.hpp
 *
 * Lightweight stand-in for PX4 uORB message types.
 *
 * In the real PX4 firmware, these are auto-generated from .msg files and
 * communicated via the uORB publish/subscribe middleware. Each message is
 * a plain-old-data struct with a timestamp and typed fields.
 *
 * This file defines the key message types used by the fixed-wing controller
 * stack, allowing the code to be compiled and tested outside PX4.
 *
 * Message naming follows PX4 convention: snake_case with _s suffix.
 */

#pragma once

#include <cstdint>

namespace uorb {

/* ------------------------------------------------------------------ */
/* Timestamps                                                          */
/* ------------------------------------------------------------------ */

using hrt_abstime = uint64_t;  ///< Microseconds since boot (PX4 hrt_absolute_time)

/* ------------------------------------------------------------------ */
/* Core vehicle state messages                                         */
/* ------------------------------------------------------------------ */

/**
 * vehicle_attitude_s — Current vehicle attitude from the EKF.
 *
 * Published by the attitude estimator at ~250 Hz.
 * The quaternion represents the rotation from NED to body frame.
 */
struct vehicle_attitude_s {
    hrt_abstime timestamp{0};
    float q[4]{1, 0, 0, 0};   ///< Quaternion [w, x, y, z]
};

/**
 * vehicle_angular_velocity_s — Gyroscope measurements (body rates).
 *
 * Published by the IMU driver. The rates are in body frame.
 */
struct vehicle_angular_velocity_s {
    hrt_abstime timestamp{0};
    float xyz[3]{0, 0, 0};    ///< [roll_rate, pitch_rate, yaw_rate] in rad/s
};

/**
 * vehicle_local_position_s — Local NED position and velocity.
 *
 * Published by the position estimator. Origin is the home position.
 */
struct vehicle_local_position_s {
    hrt_abstime timestamp{0};
    float x{0}, y{0}, z{0};         ///< Position NED [m] (z negative = above ground)
    float vx{0}, vy{0}, vz{0};      ///< Velocity NED [m/s]
    float heading{0};                 ///< Heading (yaw) [rad]
    bool xy_valid{false};
    bool z_valid{false};
    bool v_xy_valid{false};
    bool v_z_valid{false};
};

/**
 * airspeed_validated_s — Validated airspeed measurement.
 *
 * Fuses pitot tube, wind estimate, and GPS-based airspeed.
 */
struct airspeed_validated_s {
    hrt_abstime timestamp{0};
    float calibrated_airspeed_m_s{0};   ///< CAS [m/s]
    float true_airspeed_m_s{0};         ///< TAS [m/s]
};

/**
 * wind_s — Wind estimate from the EKF.
 */
struct wind_s {
    hrt_abstime timestamp{0};
    float windspeed_north{0};   ///< [m/s]
    float windspeed_east{0};    ///< [m/s]
};

/* ------------------------------------------------------------------ */
/* Setpoint / command messages                                         */
/* ------------------------------------------------------------------ */

/**
 * vehicle_attitude_setpoint_s — Desired attitude.
 *
 * Published by the path controller (fw_lateral_longitudinal_control)
 * and consumed by the attitude controller (fw_att_control).
 */
struct vehicle_attitude_setpoint_s {
    hrt_abstime timestamp{0};
    float q_d[4]{1, 0, 0, 0};    ///< Desired quaternion [w, x, y, z]
    float roll_body{0};            ///< Desired roll [rad]
    float pitch_body{0};           ///< Desired pitch [rad]
    float yaw_body{0};             ///< Desired yaw [rad]
    float thrust_body[3]{0,0,0};   ///< Desired thrust (FW: x=forward, y=0, z=0)
    float fw_control_yaw_wheel{0}; ///< Wheel steering command for ground ops
    bool fw_control_yaw{false};     ///< True if yaw should be controlled (vs coord turn)
};

/**
 * vehicle_rates_setpoint_s — Desired body angular rates.
 *
 * Published by the attitude controller and consumed by the rate controller.
 */
struct vehicle_rates_setpoint_s {
    hrt_abstime timestamp{0};
    float roll{0};     ///< Desired roll rate [rad/s]
    float pitch{0};    ///< Desired pitch rate [rad/s]
    float yaw{0};      ///< Desired yaw rate [rad/s]
    float thrust_body[3]{0,0,0};
};

/**
 * vehicle_torque_setpoint_s — Torque commands for the control allocator.
 *
 * Published by the rate controller. Normalized [-1, +1].
 * xyz = [roll_torque, pitch_torque, yaw_torque]
 */
struct vehicle_torque_setpoint_s {
    hrt_abstime timestamp{0};
    float xyz[3]{0, 0, 0};
};

/**
 * vehicle_thrust_setpoint_s — Thrust command for the control allocator.
 *
 * For fixed-wing: x = forward thrust [0,1], y = 0, z = 0.
 */
struct vehicle_thrust_setpoint_s {
    hrt_abstime timestamp{0};
    float xyz[3]{0, 0, 0};
};

/* ------------------------------------------------------------------ */
/* Path controller setpoints                                           */
/* ------------------------------------------------------------------ */

/**
 * fixed_wing_lateral_setpoint_s — Lateral guidance command.
 *
 * Published by the mode manager, consumed by the lateral controller.
 */
struct fixed_wing_lateral_setpoint_s {
    hrt_abstime timestamp{0};
    float course_setpoint{0};           ///< Desired course angle [rad]
    float lateral_acceleration{0};      ///< Desired lateral accel [m/s^2]
    float heading_setpoint{0};          ///< Desired heading [rad]
    bool  course_valid{false};
};

/**
 * fixed_wing_longitudinal_setpoint_s — Longitudinal guidance command.
 *
 * Published by the mode manager, consumed by TECS.
 */
struct fixed_wing_longitudinal_setpoint_s {
    hrt_abstime timestamp{0};
    float altitude{0};                   ///< Desired altitude [m] (positive up)
    float height_rate{0};                ///< Desired climb rate [m/s]
    float equivalent_airspeed{0};        ///< Desired EAS [m/s]
    float pitch_sp{0};                   ///< Direct pitch setpoint (manual modes)
    float thrust_sp{0};                  ///< Direct thrust setpoint (manual modes)
    bool  altitude_valid{false};
    bool  height_rate_valid{false};
    bool  airspeed_valid{false};
    bool  pitch_sp_valid{false};
    bool  thrust_sp_valid{false};
};

/* ------------------------------------------------------------------ */
/* Navigation messages                                                 */
/* ------------------------------------------------------------------ */

/**
 * position_setpoint_triplet_s — Previous / current / next waypoints.
 *
 * Published by the navigator, consumed by the mode manager.
 */
struct position_setpoint_s {
    double lat{0};             ///< Latitude [deg]
    double lon{0};             ///< Longitude [deg]
    float  alt{0};             ///< Altitude AMSL [m]
    float  loiter_radius{50};  ///< Loiter radius [m]
    int8_t loiter_direction{1}; ///< 1=CW, -1=CCW
    bool   valid{false};
    uint8_t type{0};           ///< 0=position, 1=loiter, 2=takeoff, 3=land
};

struct position_setpoint_triplet_s {
    hrt_abstime timestamp{0};
    position_setpoint_s previous;
    position_setpoint_s current;
    position_setpoint_s next;
};

/* ------------------------------------------------------------------ */
/* Vehicle status                                                      */
/* ------------------------------------------------------------------ */

struct vehicle_status_s {
    hrt_abstime timestamp{0};
    uint8_t nav_state{0};
    bool is_vtol{false};
    bool is_vtol_tailsitter{false};
    bool in_transition_mode{false};
    bool condition_landed{false};
};

/* ------------------------------------------------------------------ */
/* Control allocator feedback                                          */
/* ------------------------------------------------------------------ */

/**
 * control_allocator_status_s — Feedback from the mixer.
 *
 * Reports how much torque/thrust was "unallocated" (demanded but
 * impossible to achieve due to actuator saturation).
 * Used for anti-windup in the rate controller.
 */
struct control_allocator_status_s {
    hrt_abstime timestamp{0};
    float unallocated_torque[3]{0, 0, 0};
    float unallocated_thrust[3]{0, 0, 0};
};

/* ------------------------------------------------------------------ */
/* Autotune messages                                                   */
/* ------------------------------------------------------------------ */

struct autotune_attitude_control_status_s {
    hrt_abstime timestamp{0};
    uint8_t state{0};          ///< Current autotune state
    float   rate_sp[3]{0,0,0}; ///< Excitation signal to inject
};

/* ------------------------------------------------------------------ */
/* Manual control                                                      */
/* ------------------------------------------------------------------ */

struct manual_control_setpoint_s {
    hrt_abstime timestamp{0};
    float roll{0};      ///< [-1, +1]
    float pitch{0};     ///< [-1, +1]
    float yaw{0};       ///< [-1, +1]
    float throttle{0};  ///< [-1, +1] or [0, +1]
    float flaps{0};     ///< [0, 1]
    float aux1{0};      ///< Spoiler/aux channel
};

/* ------------------------------------------------------------------ */
/* Battery status                                                      */
/* ------------------------------------------------------------------ */

struct battery_status_s {
    hrt_abstime timestamp{0};
    float voltage_v{0};
    float scale{1.0f};  ///< Thrust scale factor for voltage compensation
};

} // namespace uorb
