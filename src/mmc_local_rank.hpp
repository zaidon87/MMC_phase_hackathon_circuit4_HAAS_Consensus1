/*
 * Local-rank / neighbor-consensus MMC balancing helpers.
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdint.h>
#include <math.h>
#include "mmc_config.hpp"

static inline float mmc_clamp01(float value)
{
    if (value < 0.0F) return 0.0F;
    if (value > 1.0F) return 1.0F;
    return value;
}

static inline float mmc_smooth_current_direction(float arm_current)
{
    return tanhf(arm_current / MMC_CURRENT_SCALE_A);
}

static inline float mmc_neighbor_error(float vc_i, float vc_prev, float vc_next)
{
    float err = 0.5F * (vc_prev + vc_next) - vc_i;
    if (fabsf(err) < MMC_VOLTAGE_DEADBAND_V) err = 0.0F;
    return err;
}

static inline float mmc_local_rank_priority_from_neighbors(float vc_i,
                                                           float vc_prev,
                                                           float vc_next,
                                                           float arm_current)
{
    const float err = mmc_neighbor_error(vc_i, vc_prev, vc_next);

    float rank_score = 0.0F;
    rank_score += (vc_i > vc_prev) ? 1.0F : 0.0F;
    rank_score += (vc_i > vc_next) ? 1.0F : 0.0F;

    const float rank_centered = rank_score - 1.0F;
    const float dir = mmc_smooth_current_direction(arm_current);

    return (MMC_K_V * err * dir) - (MMC_K_RANK * rank_centered * dir);
}

static inline float mmc_local_rank_priority(uint8_t local_index,
                                            const float *vc,
                                            uint8_t n,
                                            float arm_current)
{
    const uint8_t prev_index = static_cast<uint8_t>((local_index + n - 1U) % n);
    const uint8_t next_index = static_cast<uint8_t>((local_index + 1U) % n);
    return mmc_local_rank_priority_from_neighbors(vc[local_index],
                                                  vc[prev_index],
                                                  vc[next_index],
                                                  arm_current);
}

static inline void mmc_clear_gates(uint8_t *gates, uint8_t n)
{
    for (uint8_t i = 0; i < n; ++i) gates[i] = 0U;
}

static inline void mmc_select_top_priorities(const float *priority,
                                             uint8_t n,
                                             uint8_t n_insert,
                                             uint8_t *gates)
{
    mmc_clear_gates(gates, n);

    if (n_insert > n) n_insert = n;

    for (uint8_t selected = 0; selected < n_insert; ++selected)
    {
        float best_value = -1.0e30F;
        uint8_t best_index = 0U;

        for (uint8_t i = 0; i < n; ++i)
        {
            if ((gates[i] == 0U) && (priority[i] > best_value))
            {
                best_value = priority[i];
                best_index = i;
            }
        }

        gates[best_index] = 1U;
    }
}
