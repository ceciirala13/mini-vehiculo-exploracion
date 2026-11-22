/**
 * @file kinematics.c
 * @brief Ackermann + electronic differential (pure math).
 */

#include "kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : (v > hi) ? hi : v;
}

void kinematics_ackermann(float speed_pct, float steer_deg,
                          const kinematics_cfg_t *cfg, wheel_cmd_t *out)
{
    speed_pct = clampf(speed_pct, -cfg->max_speed_pct, cfg->max_speed_pct);
    steer_deg = clampf(steer_deg, -cfg->max_steer_deg, cfg->max_steer_deg);

    float ratio = 1.0f;
    if (cfg->wheelbase_m > 0.0f) {
        float delta = fabsf(steer_deg) * (float)M_PI / 180.0f;
        float k = cfg->ediff_gain * (cfg->track_m * 0.5f) * tanf(delta) / cfg->wheelbase_m;
        k = clampf(k, 0.0f, 1.0f);
        ratio = (1.0f - k) / (1.0f + k);
        ratio = clampf(ratio, clampf(cfg->min_inner_ratio, 0.0f, 1.0f), 1.0f);
    }

    /* Outer wheel = commanded speed, inner wheel slower.
     * Steering right (+) -> right is the inner wheel. */
    float left  = speed_pct;
    float right = speed_pct;
    if (steer_deg > 0.0f) {
        right *= ratio;
    } else if (steer_deg < 0.0f) {
        left *= ratio;
    }

    out->left_pct  = left;
    out->right_pct = right;
    out->steer_deg = steer_deg;
}
