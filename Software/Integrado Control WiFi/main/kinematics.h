/**
 * @file kinematics.h
 * @brief Pure vehicle kinematics (no hardware access): Ackermann steering
 *        with an electronic differential on the two rear wheels.
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    float wheelbase_m;      /* front axle to rear axle */
    float track_m;          /* rear wheel separation */
    float ediff_gain;       /* 0 = same speed both wheels, 1 = full geometric differential */
    float min_inner_ratio;  /* inner wheel never slower than this fraction of the outer wheel */
    float max_speed_pct;    /* output |duty| cap */
    float max_steer_deg;    /* steering limit each side */
} kinematics_cfg_t;

typedef struct {
    float left_pct;         /* rear-left wheel, -100..100 % (+ = forward) */
    float right_pct;        /* rear-right wheel */
    float steer_deg;        /* front steering, + = right */
} wheel_cmd_t;

/**
 * @brief Map a vehicle command (speed, steering) to rear wheel speeds + steering angle.
 *
 * Electronic differential (turn radius R = L / tan(delta), measured at the rear axle):
 *     v_outer = v,   v_inner = v * ratio,   ratio = (1 - k) / (1 + k),   k = gain * (T/2) * tan|delta| / L
 * i.e. ratio = (R - T/2) / (R + T/2). The outer wheel keeps the commanded speed and
 * the inner wheel is only slowed down; ratio is clamped to [min_inner_ratio, 1], so
 * the inner wheel never stops or reverses.
 *
 * @param speed_pct Vehicle speed, -100..100 % (+ = forward).
 * @param steer_deg Steering angle, + = right.
 */
void kinematics_ackermann(float speed_pct, float steer_deg,
                          const kinematics_cfg_t *cfg, wheel_cmd_t *out);

#ifdef __cplusplus
}
#endif
