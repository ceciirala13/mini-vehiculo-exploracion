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
 *     v_left  = v * (1 + k),   v_right = v * (1 - k),   k = gain * (T/2) * tan(delta) / L
 * so the inner wheel slows down and the outer speeds up. k is clamped to [-1, 1]
 * (the inner wheel never reverses) and both outputs are scaled together so neither
 * exceeds max_speed_pct.
 *
 * @param speed_pct Vehicle speed, -100..100 % (+ = forward).
 * @param steer_deg Steering angle, + = right.
 */
void kinematics_ackermann(float speed_pct, float steer_deg,
                          const kinematics_cfg_t *cfg, wheel_cmd_t *out);

#ifdef __cplusplus
}
#endif
